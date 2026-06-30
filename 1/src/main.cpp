#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <math.h>
#include <stdio.h>
#include "params.h"

class Motor {
public:
    Motor(int in1, int in2, int ch1, int ch2, bool reverse = false)
        : in1Pin(in1), in2Pin(in2), pwmCh1(ch1), pwmCh2(ch2), reversed(reverse) {}

    void begin() {
        ledcSetup(pwmCh1, PWM_FREQ, PWM_RESOLUTION);
        ledcSetup(pwmCh2, PWM_FREQ, PWM_RESOLUTION);
        ledcAttachPin(in1Pin, pwmCh1);
        ledcAttachPin(in2Pin, pwmCh2);
        stop();
    }

    void setSpeed(int speed) {
        speed = constrain(speed, -100, 100);

        if (reversed) {
            speed = -speed;
        }

        int absSpeed = abs(speed);
        int duty = 0;

        if (absSpeed > 0) {
            const int MIN_DUTY = 580;
            duty = map(absSpeed, 1, 100, MIN_DUTY, PWM_MAX);
            duty = constrain(duty, 0, PWM_MAX);
        }

        if (speed > 0) {
            ledcWrite(pwmCh1, duty);
            ledcWrite(pwmCh2, 0);
        } else if (speed < 0) {
            ledcWrite(pwmCh1, 0);
            ledcWrite(pwmCh2, duty);
        } else {
            stop();
        }
    }

    void stop() {
        ledcWrite(pwmCh1, 0);
        ledcWrite(pwmCh2, 0);
    }

private:
    int in1Pin;
    int in2Pin;
    int pwmCh1;
    int pwmCh2;
    bool reversed;
};

class Encoder {
public:
    Encoder(int a, int b, bool reverse = false)
        : pinA(a), pinB(b), reversed(reverse) {}

    void begin() {
        pinMode(pinA, INPUT_PULLUP);
        pinMode(pinB, INPUT_PULLUP);
        lastA = digitalRead(pinA);
    }

    void update() {
        int currentA = digitalRead(pinA);
        int currentB = digitalRead(pinB);

        if (currentA != lastA && currentA == HIGH) {
            int step = (currentB == LOW) ? -1 : 1;
            count += reversed ? -step : step;
        }

        lastA = currentA;
    }

    long read() {
        return count;
    }

    void reset() {
        count = 0;
    }

private:
    int pinA;
    int pinB;
    bool reversed;
    long count = 0;
    int lastA = 0;
};

class BreathingLed {
public:
    BreathingLed(int pin, int ch) : ledPin(pin), pwmCh(ch) {}

    void begin() {
        ledcSetup(pwmCh, LED_PWM_FREQ, PWM_RESOLUTION);
        ledcAttachPin(ledPin, pwmCh);
        ledcWrite(pwmCh, 0);
    }

    void update() {
        duty += step;

        if (duty >= PWM_MAX) {
            duty = PWM_MAX;
            step = -LED_BREATH_STEP;
        } else if (duty <= 0) {
            duty = 0;
            step = LED_BREATH_STEP;
        }

        ledcWrite(pwmCh, duty);
    }

    void off() {
        ledcWrite(pwmCh, 0);
    }

private:
    int ledPin;
    int pwmCh;
    int duty = 0;
    int step = LED_BREATH_STEP;
};

class GraySensorArray {
public:
    void begin() {
        analogReadResolution(12);

        for (int i = 0; i < 5; i++) {
            pinMode(GRAY_SENSOR_PINS[i], INPUT);
            analogSetPinAttenuation(GRAY_SENSOR_PINS[i], ADC_11db);
        }
    }

    void readRaw(int values[5]) {
        for (int i = 0; i < 5; i++) {
            values[i] = analogRead(GRAY_SENSOR_PINS[i]);
        }
    }

    int readTurnWeight(int values[5], bool active[5], int &activeCount) {
        readRaw(values);

        int weightSum = 0;
        activeCount = 0;

        for (int i = 0; i < 5; i++) {
            active[i] = values[i] > GRAY_THRESHOLD;

            if (active[i]) {
                weightSum += GRAY_SENSOR_WEIGHTS[i];
                activeCount++;
            }
        }

        if (activeCount == 0) {
            return lastTurnWeight;
        }

        // 多个传感器同时压线时取平均
        // 防止 S 弯、宽线时突然猛甩
        int turnWeight = weightSum / activeCount;

        lastTurnWeight = turnWeight;
        return turnWeight;
    }

private:
    int lastTurnWeight = 0;
};

struct MpuTelemetry {
    bool valid = false;
    float gyroZDps = 0.0f;
    float confidence = 0.0f;
    float prediction = 0.0f;
    float nearOffset = 0.0f;
    float farOffset = 0.0f;
    int lostCount = 0;
    uint32_t lastUpdateMs = 0;
    uint32_t packetCount = 0;
    uint32_t parseFailCount = 0;
};

Motor leftMotor(LEFT_MOTOR_PINS[0], LEFT_MOTOR_PINS[1], 0, 1, true);
Motor rightMotor(RIGHT_MOTOR_PINS[0], RIGHT_MOTOR_PINS[1], 2, 3, false);

Encoder leftEncoder(LEFT_ENCODER_PINS[0], LEFT_ENCODER_PINS[1]);
Encoder rightEncoder(RIGHT_ENCODER_PINS[0], RIGHT_ENCODER_PINS[1]);

BreathingLed led(LED_PIN, 4);
GraySensorArray gray;
WiFiUDP telemetryUdp;
MpuTelemetry mpuTelemetry;
bool wifiConnected = false;
uint32_t lastWifiRetryMs = 0;

// 记录上一次转向方向
// -1 = 上次偏左
//  1 = 上次偏右
int lastTurnDir = 1;

// 直角弯 / 丢线确认计数
int leftHardCount = 0;
int rightHardCount = 0;
int lostLineCount = 0;
int lastGrayDWeight = 0;
bool hasGrayDWeight = false;

static void connectTelemetryLink()
{
    if (!ENABLE_MPU_TELEMETRY) {
        wifiConnected = false;
        WiFi.disconnect(true);
        WiFi.mode(WIFI_OFF);
        return;
    }

    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.begin(TELEMETRY_AP_SSID, TELEMETRY_AP_PASSWORD);

    wifiConnected = (WiFi.status() == WL_CONNECTED);

    if (wifiConnected) {
        telemetryUdp.begin(TELEMETRY_UDP_PORT);
        Serial.printf("Telemetry WiFi connected, IP=%s\r\n", WiFi.localIP().toString().c_str());
    } else {
        Serial.println("Telemetry WiFi connecting...");
    }
}

static void updateTelemetryLink()
{
    if (!ENABLE_MPU_TELEMETRY) {
        return;
    }

    if (WiFi.status() != WL_CONNECTED) {
        wifiConnected = false;
    }

    if (!wifiConnected) {
        uint32_t now = millis();

        if (now - lastWifiRetryMs >= 3000) {
            lastWifiRetryMs = now;
            connectTelemetryLink();
        }
        return;
    }

    int packetSize = telemetryUdp.parsePacket();

    while (packetSize > 0) {
        char packet[128];
        int len = telemetryUdp.read(packet, sizeof(packet) - 1);

        if (len > 0) {
            packet[len] = '\0';

            char tag = '\0';
            float prediction = 0.0f;
            float nearOffset = 0.0f;
            float farOffset = 0.0f;
            float gyroZ = 0.0f;
            int valid = 0;
            int lostCount = 0;
            float confidence = 0.0f;

            int matched = sscanf(
                packet,
                " %c,%f,%f,%f,%f,%d,%d,%f",
                &tag,
                &prediction,
                &nearOffset,
                &farOffset,
                &gyroZ,
                &valid,
                &lostCount,
                &confidence
            );

            if (matched >= 5 && tag == 'P') {
                mpuTelemetry.valid = (valid != 0);
                mpuTelemetry.gyroZDps = gyroZ;
                mpuTelemetry.confidence = confidence;
                mpuTelemetry.prediction = prediction;
                mpuTelemetry.nearOffset = nearOffset;
                mpuTelemetry.farOffset = farOffset;
                mpuTelemetry.lostCount = lostCount;
                mpuTelemetry.lastUpdateMs = millis();
                mpuTelemetry.packetCount++;
            } else {
                mpuTelemetry.parseFailCount++;
            }
        }

        packetSize = telemetryUdp.parsePacket();
    }
}

static bool mpuTelemetryFresh()
{
    if (!mpuTelemetry.valid) {
        return false;
    }

    return (millis() - mpuTelemetry.lastUpdateMs) <= TELEMETRY_TIMEOUT_MS;
}

void setup() {
    Serial.begin(115200);
    delay(300);

    leftMotor.begin();
    rightMotor.begin();

    leftEncoder.begin();
    rightEncoder.begin();

    led.begin();
    gray.begin();

    connectTelemetryLink();

    Serial.printf(
        "Gray pins: %d,%d,%d,%d,%d. MPU telemetry: %s\r\n",
        GRAY_SENSOR_PINS[0],
        GRAY_SENSOR_PINS[1],
        GRAY_SENSOR_PINS[2],
        GRAY_SENSOR_PINS[3],
        GRAY_SENSOR_PINS[4],
        ENABLE_MPU_TELEMETRY ? "on" : "off"
    );
}

void loop() {
    updateTelemetryLink();

    leftEncoder.update();
    rightEncoder.update();

    int values[5];
    bool active[5];
    int activeCount = 0;

    int turnWeight = gray.readTurnWeight(values, active, activeCount);

    bool lineSeen = activeCount > 0;

    // =========================
    // 丢线确认
    // =========================
    if (!lineSeen) {
        lostLineCount++;
    } else {
        lostLineCount = 0;
    }

    // =========================
    // 直角弯防误判
    // =========================
    // 左直角弯：
    // 左外侧连续压线，并且中间、右侧没有压线
    // 允许左内侧同时压线
    if (active[0] && !active[2] && !active[3] && !active[4]) {
        leftHardCount++;
    } else {
        leftHardCount = 0;
    }

    // 右直角弯：
    // 右外侧连续压线，并且中间、左侧没有压线
    // 允许右内侧同时压线
    if (active[4] && !active[2] && !active[1] && !active[0]) {
        rightHardCount++;
    } else {
        rightHardCount = 0;
    }

    bool hardLeft = leftHardCount >= HARD_CONFIRM_COUNT;
    bool hardRight = rightHardCount >= HARD_CONFIRM_COUNT;
    bool realLost = lostLineCount >= LOST_CONFIRM_COUNT;

    // 只在当前确实看到线时，更新“上一次偏左/偏右”的记忆
    if (lineSeen) {
        if (hardLeft) {
            lastTurnDir = -1;
        } else if (hardRight) {
            lastTurnDir = 1;
        } else if (turnWeight < 0) {
            lastTurnDir = -1;
        } else if (turnWeight > 0) {
            lastTurnDir = 1;
        }
    }

    int finalTurnWeight = turnWeight;
    float controlTurnWeight = static_cast<float>(turnWeight);
    float grayDCorrection = 0.0f;
    float gyroCorrection = 0.0f;

    if (realLost) {
        // 真丢线时，按上一次偏左/偏右的方向去找回黑线
        if (lastTurnDir > 0) {
            finalTurnWeight = LOST_TURN_WEIGHT;
        } else {
            finalTurnWeight = -LOST_TURN_WEIGHT;
        }
        controlTurnWeight = static_cast<float>(finalTurnWeight);

    } else if (hardLeft) {
        // 左直角弯：直接给一个固定的强转向权重
        finalTurnWeight = -HARD_TURN_WEIGHT;
        controlTurnWeight = static_cast<float>(finalTurnWeight);

    } else if (hardRight) {
        // 右直角弯：直接给一个固定的强转向权重
        finalTurnWeight = HARD_TURN_WEIGHT;
        controlTurnWeight = static_cast<float>(finalTurnWeight);

    } else {
        // 普通循迹：直接使用灰度传感器算出来的转向权重
        finalTurnWeight = turnWeight;
        controlTurnWeight = static_cast<float>(finalTurnWeight);

        if (ENABLE_GRAY_D_CORRECTION && lineSeen) {
            if (hasGrayDWeight) {
                int deltaWeight = turnWeight - lastGrayDWeight;
                grayDCorrection = GRAY_D_GAIN * static_cast<float>(deltaWeight);
                grayDCorrection = constrain(
                    grayDCorrection,
                    -GRAY_D_CORRECTION_CLAMP,
                    GRAY_D_CORRECTION_CLAMP
                );
                controlTurnWeight += grayDCorrection;
            }

            lastGrayDWeight = turnWeight;
            hasGrayDWeight = true;
        } else {
            hasGrayDWeight = false;
            lastGrayDWeight = turnWeight;
        }
    }

    // MPU 只在正常循迹时作为 D 项使用。
    // 丢线/直角弯时优先让规则逻辑接管，避免 MPU 把车拉偏。
    if (ENABLE_MPU_D_CORRECTION &&
        !realLost &&
        !hardLeft &&
        !hardRight &&
        abs(finalTurnWeight) <= MPU_D_MAX_TURN_WEIGHT &&
        mpuTelemetryFresh()) {
        gyroCorrection = MPU_GYRO_D_SIGN * MPU_GYRO_D_GAIN * mpuTelemetry.gyroZDps;
        gyroCorrection = constrain(gyroCorrection, -MPU_D_CORRECTION_CLAMP, MPU_D_CORRECTION_CLAMP);
        controlTurnWeight += gyroCorrection;
    }

    finalTurnWeight = static_cast<int>(lroundf(controlTurnWeight));
    int speedDelta = static_cast<int>(lroundf(TURN_SPEED_STEP * controlTurnWeight));

    // 速度差公式：
    // turnWeight > 0 时，右轮更快，车身向右修正
    // turnWeight < 0 时，左轮更快，车身向左修正
    int leftSpeed = BASE_SPEED - speedDelta;
    int rightSpeed = BASE_SPEED + speedDelta;

    // 左电机通常会比右电机略快一点，这里做整体补偿
    leftSpeed += LEFT_SPEED_TRIM;

    // 限制输出范围，避免速度超出电机允许值
    leftSpeed = -constrain(leftSpeed, -MAX_SPEED, MAX_SPEED);
    rightSpeed = -constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);

    leftMotor.setSpeed(leftSpeed);
    rightMotor.setSpeed(rightSpeed);

    led.update();

    static uint32_t lastPrintMs = 0;
    static uint32_t lastTraceMs = 0;
    uint32_t now = millis();

    if (now - lastPrintMs >= PRINT_INTERVAL_MS) {
        lastPrintMs = now;
        Serial.printf(
            "t=%lu gray=%d line=%d hardL=%d hardR=%d lost=%d base=%d grayD=%.2f gyro=%.2f corr=%.2f wifi=%s pkt=%lu age=%lu valid=%d conf=%.2f turn=%d ctrl=%.2f L=%d R=%d\r\n",
            static_cast<unsigned long>(now),
            turnWeight,
            lineSeen ? 1 : 0,
            hardLeft ? 1 : 0,
            hardRight ? 1 : 0,
            realLost ? 1 : 0,
            BASE_SPEED,
            grayDCorrection,
            mpuTelemetryFresh() ? mpuTelemetry.gyroZDps : 0.0f,
            gyroCorrection,
            wifiConnected ? "ok" : "fail",
            static_cast<unsigned long>(mpuTelemetry.packetCount),
            mpuTelemetry.valid ? static_cast<unsigned long>(now - mpuTelemetry.lastUpdateMs) : 0UL,
            mpuTelemetry.valid ? 1 : 0,
            mpuTelemetry.confidence,
            finalTurnWeight,
            controlTurnWeight,
            leftSpeed,
            rightSpeed
        );
    }

    if (ENABLE_SERIAL_TRACE && now - lastTraceMs >= SERIAL_TRACE_INTERVAL_MS) {
        lastTraceMs = now;
        Serial.printf(
            "mpu raw: pred=%.3f near=%.3f far=%.3f gyro=%.2f valid=%d age=%lu packets=%lu parseFail=%lu\n",
            mpuTelemetry.prediction,
            mpuTelemetry.nearOffset,
            mpuTelemetry.farOffset,
            mpuTelemetry.gyroZDps,
            mpuTelemetry.valid ? 1 : 0,
            mpuTelemetry.valid ? static_cast<unsigned long>(now - mpuTelemetry.lastUpdateMs) : 0UL,
            static_cast<unsigned long>(mpuTelemetry.packetCount),
            static_cast<unsigned long>(mpuTelemetry.parseFailCount)
        );
    }

    delay(LOOP_DELAY_MS);
}
