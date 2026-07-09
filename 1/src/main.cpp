#include <Arduino.h>
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

    float readTurnWeight(int values[5], bool active[5], int &activeCount) {
        readRaw(values);

        float weightedSum = 0.0f;
        float signalSum = 0.0f;
        activeCount = 0;

        for (int i = 0; i < 5; i++) {
            active[i] = values[i] > GRAY_THRESHOLD;

            if (active[i]) {
                float signal = static_cast<float>(values[i] - GRAY_THRESHOLD);
                weightedSum += static_cast<float>(GRAY_SENSOR_WEIGHTS[i]) * signal;
                signalSum += signal;
                activeCount++;
            }
        }

        if (signalSum <= 0.0f) {
            return lastTurnWeight;
        }

        // 使用模拟量强度拟合目标线位置，保留连续误差，减少二值跳变带来的抖动。
        float turnWeight = weightedSum / signalSum;

        lastTurnWeight = turnWeight;
        return turnWeight;
    }

private:
    float lastTurnWeight = 0.0f;
};

Motor leftMotor(LEFT_MOTOR_PINS[0], LEFT_MOTOR_PINS[1], 0, 1, true);
Motor rightMotor(RIGHT_MOTOR_PINS[0], RIGHT_MOTOR_PINS[1], 2, 3, false);

Encoder leftEncoder(LEFT_ENCODER_PINS[0], LEFT_ENCODER_PINS[1]);
Encoder rightEncoder(RIGHT_ENCODER_PINS[0], RIGHT_ENCODER_PINS[1]);

BreathingLed led(LED_PIN, 4);
GraySensorArray gray;

// 记录上一次转向方向
// -1 = 上次偏左
//  1 = 上次偏右
int lastTurnDir = 1;

// 直角弯 / 丢线确认计数
int leftHardCount = 0;
int rightHardCount = 0;
int lostLineCount = 0;
float lastGrayDWeight = 0.0f;
bool hasGrayDWeight = false;
int straightLockCount = 0;

void setup() {
    Serial.begin(115200);
    delay(300);

    leftMotor.begin();
    rightMotor.begin();

    leftEncoder.begin();
    rightEncoder.begin();

    led.begin();
    gray.begin();

    Serial.printf(
        "Gray pins: %d,%d,%d,%d,%d\r\n",
        GRAY_SENSOR_PINS[0],
        GRAY_SENSOR_PINS[1],
        GRAY_SENSOR_PINS[2],
        GRAY_SENSOR_PINS[3],
        GRAY_SENSOR_PINS[4]
    );
}

void loop() {
    leftEncoder.update();
    rightEncoder.update();

    int values[5];
    bool active[5];
    int activeCount = 0;

    float turnWeight = gray.readTurnWeight(values, active, activeCount);

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
    // 左外侧压线，并且右侧没有压线；允许中间/左内侧仍在线上，提前触发。
    if (active[0] && !active[3] && !active[4]) {
        leftHardCount++;
    } else {
        leftHardCount = 0;
    }

    // 右直角弯：
    // 右外侧压线，并且左侧没有压线；允许中间/右内侧仍在线上，提前触发。
    if (active[4] && !active[1] && !active[0]) {
        rightHardCount++;
    } else {
        rightHardCount = 0;
    }

    bool hardLeft = leftHardCount >= HARD_CONFIRM_COUNT;
    bool hardRight = rightHardCount >= HARD_CONFIRM_COUNT;
    bool realLost = lostLineCount >= LOST_CONFIRM_COUNT;

    bool straightCandidate = active[2] && !active[0] && !active[1] && !active[3] && !active[4] && !hardLeft && !hardRight;
    if (straightCandidate) {
        straightLockCount++;
    } else {
        straightLockCount = 0;
    }

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

    int finalTurnWeight = static_cast<int>(lroundf(turnWeight));
    float controlTurnWeight = turnWeight;
    float grayDCorrection = 0.0f;

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
        finalTurnWeight = static_cast<int>(lroundf(turnWeight));
        controlTurnWeight = turnWeight;

        bool centerLocked = ENABLE_STRAIGHT_CENTER_LOCK && straightLockCount >= STRAIGHT_LOCK_CONFIRM_COUNT;
        if (centerLocked) {
            controlTurnWeight = 0.0f;
            lastGrayDWeight = turnWeight;
            hasGrayDWeight = true;

        } else if (straightCandidate) {
            lastGrayDWeight = turnWeight;
            hasGrayDWeight = true;

        } else if (ENABLE_GRAY_D_CORRECTION && lineSeen) {
            if (hasGrayDWeight) {
                float deltaWeight = turnWeight - lastGrayDWeight;
                grayDCorrection = GRAY_D_GAIN * deltaWeight;
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
    uint32_t now = millis();

    if (now - lastPrintMs >= PRINT_INTERVAL_MS) {
        lastPrintMs = now;
        Serial.printf(
            "t=%lu gray=%.2f line=%d hardL=%d hardR=%d lost=%d base=%d grayD=%.2f turn=%d ctrl=%.2f L=%d R=%d\r\n",
            static_cast<unsigned long>(now),
            turnWeight,
            lineSeen ? 1 : 0,
            hardLeft ? 1 : 0,
            hardRight ? 1 : 0,
            realLost ? 1 : 0,
            BASE_SPEED,
            grayDCorrection,
            finalTurnWeight,
            controlTurnWeight,
            leftSpeed,
            rightSpeed
        );
    }

    delay(LOOP_DELAY_MS);
}
