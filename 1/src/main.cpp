#include <Arduino.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
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
        if (ledPin < 0) {
            return;
        }
        ledcSetup(pwmCh, LED_PWM_FREQ, PWM_RESOLUTION);
        ledcAttachPin(ledPin, pwmCh);
        ledcWrite(pwmCh, 0);
    }

    void update() {
        if (ledPin < 0) {
            return;
        }
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
        if (ledPin < 0) {
            return;
        }
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
    uint32_t lastImuUpdateMs = 0;
    uint32_t lastCamUpdateMs = 0;
    uint32_t packetCount = 0;
    uint32_t parseFailCount = 0;
    uint32_t byteCount = 0;
    int imuSeq = 0;
    int camSeq = 0;
};

struct CameraFeedForwardSample {
    bool valid = false;
    uint32_t timeMs = 0;
    int seq = 0;
    float centerOffset = 0.0f;
    float nearOffset = 0.0f;
    float farOffset = 0.0f;
    float lineQuality = 0.0f;
};

constexpr size_t CAMERA_FF_BUFFER_SIZE = 24;

Motor leftMotor(LEFT_MOTOR_PINS[0], LEFT_MOTOR_PINS[1], 0, 1, true);
Motor rightMotor(RIGHT_MOTOR_PINS[0], RIGHT_MOTOR_PINS[1], 2, 3, false);

Encoder leftEncoder(LEFT_ENCODER_PINS[0], LEFT_ENCODER_PINS[1]);
Encoder rightEncoder(RIGHT_ENCODER_PINS[0], RIGHT_ENCODER_PINS[1]);

BreathingLed led(LED_PIN, 4);
GraySensorArray gray;
MpuTelemetry mpuTelemetry;
CameraFeedForwardSample cameraFeedForwardBuffer[CAMERA_FF_BUFFER_SIZE];
size_t cameraFeedForwardWriteIndex = 0;
HardwareSerial telemetrySerial(2);
char telemetryLine[128];
size_t telemetryLineLen = 0;

static void pushCameraFeedForwardSample(
    int seq,
    float nearOffset,
    float farOffset,
    float centerOffset,
    float lineQuality,
    uint32_t timeMs
)
{
    CameraFeedForwardSample &sample = cameraFeedForwardBuffer[cameraFeedForwardWriteIndex];

    sample.valid = true;
    sample.timeMs = timeMs;
    sample.seq = seq;
    sample.centerOffset = constrain(centerOffset, -1.0f, 1.0f);
    sample.nearOffset = constrain(nearOffset, -1.0f, 1.0f);
    sample.farOffset = constrain(farOffset, -1.0f, 1.0f);
    sample.lineQuality = constrain(lineQuality, 0.0f, 1.0f);

    cameraFeedForwardWriteIndex = (cameraFeedForwardWriteIndex + 1) % CAMERA_FF_BUFFER_SIZE;
}

static bool getDelayedCameraFeedForwardSample(uint32_t now, CameraFeedForwardSample &sample)
{
    if (!ENABLE_CAMERA_SPEED_FEED_FORWARD) {
        return false;
    }

    bool found = false;
    uint32_t bestTime = 0;

    for (size_t i = 0; i < CAMERA_FF_BUFFER_SIZE; i++) {
        const CameraFeedForwardSample &candidate = cameraFeedForwardBuffer[i];

        if (!candidate.valid || candidate.lineQuality < CAMERA_SPEED_MIN_LINE_QUALITY) {
            continue;
        }

        uint32_t ageMs = now - candidate.timeMs;

        if (ageMs < CAMERA_SPEED_DELAY_MS || ageMs > TELEMETRY_TIMEOUT_MS) {
            continue;
        }

        if (!found || candidate.timeMs > bestTime) {
            sample = candidate;
            bestTime = candidate.timeMs;
            found = true;
        }
    }

    return found;
}

static bool cameraSampleLooksCurved(const CameraFeedForwardSample &sample)
{
    float endDiff = fabsf(sample.nearOffset - sample.farOffset);
    float centerAbs = fabsf(sample.centerOffset);
    float nearAbs = fabsf(sample.nearOffset);
    float farAbs = fabsf(sample.farOffset);

    return endDiff >= CAMERA_CURVE_OFFSET_THRESHOLD ||
        centerAbs >= CAMERA_CURVE_OFFSET_THRESHOLD ||
        nearAbs >= CAMERA_CURVE_OFFSET_THRESHOLD ||
        farAbs >= CAMERA_CURVE_OFFSET_THRESHOLD;
}

static uint8_t telemetryChecksum(const char *text)
{
    uint8_t value = 0;

    while (*text != '\0') {
        value ^= static_cast<uint8_t>(*text);
        text++;
    }

    return value;
}

static bool splitTelemetryPacket(const char *packet, char *body, size_t bodySize)
{
    if (bodySize == 0) {
        return false;
    }

    const char *star = strrchr(packet, '*');

    if (star == nullptr) {
        strncpy(body, packet, bodySize);
        body[bodySize - 1] = '\0';
        return true;
    }

    size_t bodyLen = static_cast<size_t>(star - packet);

    if (bodyLen >= bodySize) {
        mpuTelemetry.parseFailCount++;
        return false;
    }

    memcpy(body, packet, bodyLen);
    body[bodyLen] = '\0';

    char *end = nullptr;
    unsigned long received = strtoul(star + 1, &end, 16);

    if (end == star + 1 || received > 0xFF) {
        mpuTelemetry.parseFailCount++;
        return false;
    }

    uint8_t expected = telemetryChecksum(body);

    if (static_cast<uint8_t>(received) != expected) {
        mpuTelemetry.parseFailCount++;
        return false;
    }

    return true;
}

static bool parseS3CamTelemetryBody(const char *body)
{
    int seq = 0;
    float pitch = 0.0f;
    float roll = 0.0f;
    float yawOrGyro = 0.0f;

    if (sscanf(body, "IMU,%d,%f,%f,%f", &seq, &pitch, &roll, &yawOrGyro) == 4) {
        if (mpuTelemetry.imuSeq > 0 && seq > mpuTelemetry.imuSeq + 1) {
            mpuTelemetry.lostCount += seq - mpuTelemetry.imuSeq - 1;
        }

        mpuTelemetry.imuSeq = seq;
        mpuTelemetry.valid = true;
        mpuTelemetry.gyroZDps = yawOrGyro;
        mpuTelemetry.confidence = 1.0f;
        mpuTelemetry.prediction = yawOrGyro;
        mpuTelemetry.nearOffset = 0.0f;
        mpuTelemetry.farOffset = 0.0f;
        mpuTelemetry.lastUpdateMs = millis();
        mpuTelemetry.lastImuUpdateMs = mpuTelemetry.lastUpdateMs;
        mpuTelemetry.packetCount++;
        return true;
    }

    int camSeq = 0;
    float nearOffset = 0.0f;
    float farOffset = 0.0f;
    float curve = 0.0f;
    float quality = 0.0f;

    if (sscanf(body, "CAM,%d,%f,%f,%f,%f", &camSeq, &nearOffset, &farOffset, &curve, &quality) == 5) {
        if (mpuTelemetry.camSeq > 0 && camSeq > mpuTelemetry.camSeq + 1) {
            mpuTelemetry.lostCount += camSeq - mpuTelemetry.camSeq - 1;
        }

        mpuTelemetry.camSeq = camSeq;
        mpuTelemetry.valid = quality > 0.0f;
        mpuTelemetry.gyroZDps = 0.0f;
        mpuTelemetry.confidence = constrain(quality, 0.0f, 1.0f);
        mpuTelemetry.prediction = constrain(curve, -1.0f, 1.0f);
        mpuTelemetry.nearOffset = constrain(nearOffset, -1.0f, 1.0f);
        mpuTelemetry.farOffset = constrain(farOffset, -1.0f, 1.0f);
        mpuTelemetry.lastUpdateMs = millis();
        mpuTelemetry.lastCamUpdateMs = mpuTelemetry.lastUpdateMs;
        pushCameraFeedForwardSample(
            camSeq,
            mpuTelemetry.nearOffset,
            mpuTelemetry.farOffset,
            mpuTelemetry.prediction,
            mpuTelemetry.confidence,
            mpuTelemetry.lastCamUpdateMs
        );
        mpuTelemetry.packetCount++;
        return true;
    }

    return false;
}

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

static void parseTelemetryLine(const char *packet)
{
    if (!ENABLE_MPU_TELEMETRY) {
        return;
    }

    char body[128];

    if (!splitTelemetryPacket(packet, body, sizeof(body))) {
        return;
    }

    if (parseS3CamTelemetryBody(body)) {
        return;
    }

    mpuTelemetry.parseFailCount++;

    static uint32_t lastBadPacketPrintMs = 0;
    uint32_t now = millis();

    if (now - lastBadPacketPrintMs >= 1000) {
        lastBadPacketPrintMs = now;
        Serial.printf("Telemetry parse fail: '%s'\r\n", packet);
    }
}

static void beginTelemetryLink()
{
    if (!ENABLE_MPU_TELEMETRY) {
        return;
    }

    if (TELEMETRY_UART_RX_PIN < 0 ||
        TELEMETRY_UART_RX_PIN > 39 ||
        TELEMETRY_UART_TX_PIN < 0 ||
        TELEMETRY_UART_TX_PIN > 39) {
        Serial.printf(
            "Telemetry UART disabled: invalid ESP32 pins RX=%d TX=%d\r\n",
            TELEMETRY_UART_RX_PIN,
            TELEMETRY_UART_TX_PIN
        );
        return;
    }

    telemetrySerial.begin(
        TELEMETRY_UART_BAUD,
        SERIAL_8N1,
        TELEMETRY_UART_RX_PIN,
        TELEMETRY_UART_TX_PIN
    );

    Serial.printf(
        "Telemetry UART2 enabled: RX=%d TX=%d baud=%lu\r\n",
        TELEMETRY_UART_RX_PIN,
        TELEMETRY_UART_TX_PIN,
        static_cast<unsigned long>(TELEMETRY_UART_BAUD)
    );
    Serial.println("Wire: S3 GPIO45 TX -> V1 GPIO22 RX, S3 GPIO46 RX <- V1 GPIO23 TX, common GND.");
}

static void updateTelemetryLink()
{
    if (!ENABLE_MPU_TELEMETRY) {
        return;
    }

    while (telemetrySerial.available() > 0) {
        char c = static_cast<char>(telemetrySerial.read());
        mpuTelemetry.byteCount++;

        if (c == '\r') {
            continue;
        }

        if (c == '\n') {
            telemetryLine[telemetryLineLen] = '\0';

            if (telemetryLineLen > 0) {
                parseTelemetryLine(telemetryLine);
            }

            telemetryLineLen = 0;
            continue;
        }

        if (telemetryLineLen < sizeof(telemetryLine) - 1) {
            telemetryLine[telemetryLineLen++] = c;
        } else {
            telemetryLineLen = 0;
            mpuTelemetry.parseFailCount++;
        }
    }
}

static bool mpuTelemetryFresh()
{
    if (mpuTelemetry.lastImuUpdateMs == 0) {
        return false;
    }

    return (millis() - mpuTelemetry.lastImuUpdateMs) <= TELEMETRY_TIMEOUT_MS;
}

static bool cameraTelemetryFresh()
{
    if (mpuTelemetry.lastCamUpdateMs == 0) {
        return false;
    }

    return (millis() - mpuTelemetry.lastCamUpdateMs) <= TELEMETRY_TIMEOUT_MS;
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

    beginTelemetryLink();

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
    uint32_t now = millis();

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
    CameraFeedForwardSample delayedCameraSample;
    bool cameraSpeedActive = getDelayedCameraFeedForwardSample(now, delayedCameraSample);
    bool cameraCurveAhead = cameraSpeedActive && cameraSampleLooksCurved(delayedCameraSample);
    int dynamicBaseSpeed = BASE_SPEED;

    if (cameraSpeedActive) {
        dynamicBaseSpeed = cameraCurveAhead ? CAMERA_CURVE_SPEED : CAMERA_STRAIGHT_SPEED;
    }

    if (realLost) {
        // 真丢线时，按上一次偏左/偏右的方向去找回黑线
        if (lastTurnDir > 0) {
            finalTurnWeight = LOST_TURN_WEIGHT;
        } else {
            finalTurnWeight = -LOST_TURN_WEIGHT;
        }
        controlTurnWeight = static_cast<float>(finalTurnWeight);
        dynamicBaseSpeed = CAMERA_CURVE_SPEED;

    } else if (hardLeft) {
        // 左直角弯：直接给一个固定的强转向权重
        finalTurnWeight = -HARD_TURN_WEIGHT;
        controlTurnWeight = static_cast<float>(finalTurnWeight);
        dynamicBaseSpeed = CAMERA_CURVE_SPEED;

    } else if (hardRight) {
        // 右直角弯：直接给一个固定的强转向权重
        finalTurnWeight = HARD_TURN_WEIGHT;
        controlTurnWeight = static_cast<float>(finalTurnWeight);
        dynamicBaseSpeed = CAMERA_CURVE_SPEED;

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
    int leftSpeed = dynamicBaseSpeed - speedDelta;
    int rightSpeed = dynamicBaseSpeed + speedDelta;

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
    bool imuFresh = mpuTelemetryFresh();
    bool camFresh = cameraTelemetryFresh();
    uint32_t imuAge = mpuTelemetry.lastImuUpdateMs > 0
        ? now - mpuTelemetry.lastImuUpdateMs
        : 0;
    uint32_t camAge = mpuTelemetry.lastCamUpdateMs > 0
        ? now - mpuTelemetry.lastCamUpdateMs
        : 0;
    uint32_t cameraSpeedAge = cameraSpeedActive
        ? now - delayedCameraSample.timeMs
        : 0;

    if (now - lastPrintMs >= PRINT_INTERVAL_MS) {
        lastPrintMs = now;
        const char *lineState = lineSeen ? "ON" : "LOST";
        const char *turnHint = "MID";

        if (finalTurnWeight < 0) {
            turnHint = "LEFT";
        } else if (finalTurnWeight > 0) {
            turnHint = "RIGHT";
        }

        Serial.printf(
            "LINE %s gray=%d turn=%s hardL=%d hardR=%d lost=%d | MOTOR L=%d R=%d base=%d ctrl=%.2f grayD=%.2f camCurve=%d\r\n",
            lineState,
            turnWeight,
            turnHint,
            hardLeft ? 1 : 0,
            hardRight ? 1 : 0,
            realLost ? 1 : 0,
            leftSpeed,
            rightSpeed,
            dynamicBaseSpeed,
            controlTurnWeight,
            grayDCorrection,
            cameraCurveAhead ? 1 : 0
        );
        Serial.printf(
            "CAM %s age=%lums speedAge=%lums speedSeq=%d near=%.2f far=%.2f center=%.2f lineQ=%.2f\r\n",
            camFresh ? "OK" : "WAIT",
            static_cast<unsigned long>(camAge),
            static_cast<unsigned long>(cameraSpeedAge),
            cameraSpeedActive ? delayedCameraSample.seq : 0,
            mpuTelemetry.nearOffset,
            mpuTelemetry.farOffset,
            mpuTelemetry.prediction,
            mpuTelemetry.confidence
        );
        Serial.printf(
            "MPU %s age=%lums seq=%d gyroZ=%.2f corr=%.2f | UART bytes=%lu packets=%lu bad=%lu lost=%d\r\n",
            imuFresh ? "OK" : "WAIT",
            static_cast<unsigned long>(imuAge),
            mpuTelemetry.imuSeq,
            imuFresh ? mpuTelemetry.gyroZDps : 0.0f,
            gyroCorrection,
            static_cast<unsigned long>(mpuTelemetry.byteCount),
            static_cast<unsigned long>(mpuTelemetry.packetCount),
            static_cast<unsigned long>(mpuTelemetry.parseFailCount),
            mpuTelemetry.lostCount
        );
        Serial.printf(
            "RAW t=%lu rx=%d uart=%s base=%d lineQ=%.2f\r\n",
            static_cast<unsigned long>(now),
            TELEMETRY_UART_RX_PIN,
            (imuFresh || camFresh) ? "OK" : "WAIT",
            dynamicBaseSpeed,
            mpuTelemetry.confidence
        );
    }

    if (ENABLE_SERIAL_TRACE && now - lastTraceMs >= SERIAL_TRACE_INTERVAL_MS) {
        lastTraceMs = now;
        Serial.printf(
            "VISION_RAW center=%.3f near=%.3f far=%.3f lineQ=%.2f camCurve=%d speedAge=%lu base=%d camAge=%lu imuGyro=%.2f imuAge=%lu\r\n",
            mpuTelemetry.prediction,
            mpuTelemetry.nearOffset,
            mpuTelemetry.farOffset,
            mpuTelemetry.confidence,
            cameraCurveAhead ? 1 : 0,
            static_cast<unsigned long>(cameraSpeedAge),
            dynamicBaseSpeed,
            static_cast<unsigned long>(camAge),
            mpuTelemetry.gyroZDps,
            static_cast<unsigned long>(imuAge)
        );
    }

    delay(LOOP_DELAY_MS);
}
