#include <Arduino.h>

const int PWM_FREQ = 20000;
const int PWM_RESOLUTION = 10;
const int PWM_MAX = 1023;

// =========================
// =========================
// 调参区
// =========================

// 左轮整体补偿：
// 左轮快：填负数，例如 -2、-3
// 左轮慢：填正数，例如 2、3
int LEFT_SPEED_TRIM = -2;

// 直线速度
// 直角弯冲出去就降到 34；太慢就升到 38
int BASE_SPEED = 36;

int MAX_SPEED = 80;

// 普通循迹转向力度
// 抖就降到 6；普通弯转不过就升到 8
int TURN_SPEED_STEP = 8;

// 直角弯时使用的强制转向权重
// 直角弯响应慢就升到 7；甩头就降到 5
int HARD_TURN_WEIGHT = 15;

// 丢线找线时使用的转向权重
// 丢线后找不回来就升到 5；乱甩就降到 3
int LOST_TURN_WEIGHT = 4;

// 连续检测到几次外侧压线，才认为是直角弯
// 响应慢就用 2；S弯误判就用 3 或 4
int HARD_CONFIRM_COUNT = 1;

// 连续丢线几次，才进入丢线找线
// 误触发 LOST 就升到 4；丢线救不回来就降到 2
int LOST_CONFIRM_COUNT = 4;

// 主循环延时
// 5ms 反应更快；10ms 稳一点但滞后更大
int LOOP_DELAY_MS = 3;

// 串口打印间隔
int PRINT_INTERVAL_MS = 200;

const int GRAY_SENSOR_PINS[5] = {33, 32, 35, 34, 27};
int GRAY_THRESHOLD = 600;

// 左外、左内、中、右内、右外
// 比 {-10,-2,0,2,10} 温和，减少 S 弯乱甩
const int GRAY_SENSOR_WEIGHTS[5] = {-4, -2, 0, 2, 4};

const int LED_PIN = 22;
const int LED_PWM_FREQ = 1000;
const int LED_BREATH_STEP = 24;

const int LEFT_MOTOR_PINS[2] = {14, 25};
const int RIGHT_MOTOR_PINS[2] = {13, 15};

const int LEFT_ENCODER_PINS[2] = {18, 19};
const int RIGHT_ENCODER_PINS[2] = {16, 17};

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

Motor leftMotor(LEFT_MOTOR_PINS[0], LEFT_MOTOR_PINS[1], 0, 1, true);
Motor rightMotor(RIGHT_MOTOR_PINS[0], RIGHT_MOTOR_PINS[1], 2, 3, false);

Encoder leftEncoder(LEFT_ENCODER_PINS[0], LEFT_ENCODER_PINS[1]);
Encoder rightEncoder(RIGHT_ENCODER_PINS[0], RIGHT_ENCODER_PINS[1]);

BreathingLed led(LED_PIN, 4);
GraySensorArray gray;

unsigned long lastPrintTime = 0;

// 记录上一次转向方向
// -1 = 上次偏左
//  1 = 上次偏右
int lastTurnDir = 1;

// 直角弯 / 丢线确认计数
int leftHardCount = 0;
int rightHardCount = 0;
int lostLineCount = 0;

void setup() {
    Serial.begin(115200);
    delay(500);

    leftMotor.begin();
    rightMotor.begin();

    leftEncoder.begin();
    rightEncoder.begin();

    led.begin();
    gray.begin();

    Serial.println("ESP32 Line Follower Start");
}

void loop() {
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

    // 只在真的看到线时更新上一次方向
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

    const char *mode = "NORMAL";

    int finalTurnWeight = turnWeight;

    if (realLost) {
        // =========================
        // 真丢线找线
        // 不直接改左右轮方向，只给一个找线 turnWeight
        // =========================
        mode = "LOST";

        if (lastTurnDir > 0) {
            finalTurnWeight = LOST_TURN_WEIGHT;
        } else {
            finalTurnWeight = -LOST_TURN_WEIGHT;
        }

    } else if (hardLeft) {
        // =========================
        // 左直角弯
        // 不乱改方向，只把 turnWeight 加大
        // =========================
        mode = "HARD_LEFT";
        finalTurnWeight = -HARD_TURN_WEIGHT;

    } else if (hardRight) {
        // =========================
        // 右直角弯
        // 不乱改方向，只把 turnWeight 加大
        // =========================
        mode = "HARD_RIGHT";
        finalTurnWeight = HARD_TURN_WEIGHT;

    } else {
        mode = "NORMAL";
        finalTurnWeight = turnWeight;
    }

    int speedDelta = TURN_SPEED_STEP * finalTurnWeight;

    // =========================
    // 关键：回到你原来的方向公式
    // 不再乱改成别的方向
    // =========================
    int leftSpeed = BASE_SPEED - speedDelta;
    int rightSpeed = BASE_SPEED + speedDelta;

    // 左轮整体速度补偿
    leftSpeed += LEFT_SPEED_TRIM;

    // 防止强转时一边算成太离谱
    leftSpeed = -constrain(leftSpeed, -MAX_SPEED, MAX_SPEED);
    rightSpeed = -constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);

    leftMotor.setSpeed(leftSpeed);
    rightMotor.setSpeed(rightSpeed);

    led.update();

    unsigned long now = millis();

    if (now - lastPrintTime >= PRINT_INTERVAL_MS) {
        Serial.print("mode=");
        Serial.print(mode);

        Serial.print(", gray=[");

        for (int i = 0; i < 5; i++) {
            Serial.print(values[i]);
            if (i < 4) Serial.print(", ");
        }

        Serial.print("], active=[");

        for (int i = 0; i < 5; i++) {
            Serial.print(active[i] ? 1 : 0);
            if (i < 4) Serial.print(", ");
        }

        Serial.print("], count=");
        Serial.print(activeCount);

        Serial.print(", rawTurn=");
        Serial.print(turnWeight);

        Serial.print(", finalTurn=");
        Serial.print(finalTurnWeight);

        Serial.print(", delta=");
        Serial.print(speedDelta);

        Serial.print(", leftHardCount=");
        Serial.print(leftHardCount);

        Serial.print(", rightHardCount=");
        Serial.print(rightHardCount);

        Serial.print(", lostCount=");
        Serial.print(lostLineCount);

        Serial.print(", lastDir=");
        Serial.print(lastTurnDir);

        Serial.print(", L=");
        Serial.print(leftSpeed);

        Serial.print(", R=");
        Serial.print(rightSpeed);

        Serial.print(", encL=");
        Serial.print(leftEncoder.read());

        Serial.print(", encR=");
        Serial.println(rightEncoder.read());

        lastPrintTime = now;
    }

    delay(LOOP_DELAY_MS);
}