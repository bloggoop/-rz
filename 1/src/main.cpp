#include <Arduino.h>
#include <Wire.h>
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
    uint32_t packetCount = 0;
    uint32_t parseFailCount = 0;
    uint32_t byteCount = 0;
    int imuSeq = 0;
    int camSeq = 0;
};

Motor leftMotor(LEFT_MOTOR_PINS[0], LEFT_MOTOR_PINS[1], 0, 1, true);
Motor rightMotor(RIGHT_MOTOR_PINS[0], RIGHT_MOTOR_PINS[1], 2, 3, false);

Encoder leftEncoder(LEFT_ENCODER_PINS[0], LEFT_ENCODER_PINS[1]);
Encoder rightEncoder(RIGHT_ENCODER_PINS[0], RIGHT_ENCODER_PINS[1]);

BreathingLed led(LED_PIN, 4);
GraySensorArray gray;
MpuTelemetry mpuTelemetry;
HardwareSerial telemetrySerial(1);
char telemetryLine[128];
size_t telemetryLineLen = 0;
uint32_t lastTelemetryByteMs = 0;
int activeTelemetryRxPin = TELEMETRY_UART_RX_PIN;
uint32_t lastTelemetryProbeMs = 0;
volatile bool hasI2CTelemetryLine = false;
char i2cTelemetryLine[128];
bool i2cPinSwap = false;
uint32_t lastI2CReprobeMs = 0;
uint8_t telemetryRawSample[48];
size_t telemetryRawSampleLen = 0;
int lastPulseLevel = HIGH;
uint32_t pulseLowStartMs = 0;
uint32_t lastPulseEdgeMs = 0;
int pulseState = 0;
int pulseSign = 1;
int pulseCount = 0;

static void resetTelemetryRawSample()
{
    telemetryRawSampleLen = 0;
}

static void recordTelemetryRawByte(uint8_t value)
{
    if (telemetryRawSampleLen < sizeof(telemetryRawSample)) {
        telemetryRawSample[telemetryRawSampleLen++] = value;
    }
}

static void printTelemetryRawSample(const char *prefix)
{
    if (telemetryRawSampleLen == 0) {
        return;
    }

    Serial.printf("%s RX=%d raw:", prefix, activeTelemetryRxPin);

    for (size_t i = 0; i < telemetryRawSampleLen; i++) {
        Serial.printf(" %02X", telemetryRawSample[i]);
    }

    Serial.print(" ascii='");

    for (size_t i = 0; i < telemetryRawSampleLen; i++) {
        char c = static_cast<char>(telemetryRawSample[i]);
        Serial.print((c >= 32 && c <= 126) ? c : '.');
    }

    Serial.println("'");
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

        static uint32_t lastCrcPrintMs = 0;
        uint32_t now = millis();

        if (now - lastCrcPrintMs >= 1000) {
            lastCrcPrintMs = now;
            Serial.printf("Telemetry CRC fail: '%s' expected=%02X\r\n", packet, expected);
        }

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

static void updateUartRxPinScan()
{
    if (!ENABLE_UART_RX_PIN_SCAN) {
        return;
    }

    constexpr size_t PIN_COUNT = sizeof(UART_RX_SCAN_PINS) / sizeof(UART_RX_SCAN_PINS[0]);
    static bool initialized = false;
    static int lastLevels[PIN_COUNT] = {};
    static uint32_t edgeCounts[PIN_COUNT] = {};
    static uint32_t lastReportMs = 0;

    if (!initialized) {
        for (size_t i = 0; i < PIN_COUNT; i++) {
            pinMode(UART_RX_SCAN_PINS[i], INPUT);
            lastLevels[i] = digitalRead(UART_RX_SCAN_PINS[i]);
            edgeCounts[i] = 0;
        }

        initialized = true;
        lastReportMs = millis();
    }

    for (size_t i = 0; i < PIN_COUNT; i++) {
        int level = digitalRead(UART_RX_SCAN_PINS[i]);

        if (level != lastLevels[i]) {
            lastLevels[i] = level;
            edgeCounts[i]++;
        }
    }

    uint32_t now = millis();

    if (now - lastReportMs < UART_RX_SCAN_INTERVAL_MS) {
        return;
    }

    lastReportMs = now;
    Serial.print("uart rx scan edges:");

    for (size_t i = 0; i < PIN_COUNT; i++) {
        Serial.printf(" GPIO%d=%lu", UART_RX_SCAN_PINS[i], static_cast<unsigned long>(edgeCounts[i]));
        edgeCounts[i] = 0;
    }

    Serial.println();
}

static void parseTelemetryLine(const char *packet)
{
    if (!ENABLE_MPU_TELEMETRY) {
        return;
    }

    char body[128];

    if (!splitTelemetryPacket(packet, body, sizeof(body))) {
        return;
    }

    if (strncmp(body, "IMU,", 4) == 0 || strncmp(body, "CAM,", 4) == 0) {
        if (parseS3CamTelemetryBody(body)) {
            return;
        }

        mpuTelemetry.parseFailCount++;
        return;
    }

    char tag = '\0';
    float prediction = 0.0f;
    float nearOffset = 0.0f;
    float farOffset = 0.0f;
    float gyroZ = 0.0f;
    int valid = 0;
    int lostCount = 0;
    float confidence = 0.0f;

    int matched = sscanf(
        body,
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

    if (tag == 'G') {
        int gyroTenths = 0;
        int compactValid = 0;

        matched = sscanf(body, " G,%d,%d", &gyroTenths, &compactValid);

        if (matched >= 2) {
            mpuTelemetry.valid = (compactValid != 0);
            mpuTelemetry.gyroZDps = static_cast<float>(gyroTenths) / 10.0f;
            mpuTelemetry.confidence = compactValid != 0 ? 1.0f : 0.0f;
            mpuTelemetry.prediction = 0.0f;
            mpuTelemetry.nearOffset = 0.0f;
            mpuTelemetry.farOffset = 0.0f;
            mpuTelemetry.lostCount = 0;
            mpuTelemetry.lastUpdateMs = millis();
            mpuTelemetry.packetCount++;
            return;
        }
    }

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

        static uint32_t lastBadPacketPrintMs = 0;
        uint32_t now = millis();

        if (now - lastBadPacketPrintMs >= 1000) {
            lastBadPacketPrintMs = now;
            Serial.printf("Telemetry parse fail on RX=%d: '", activeTelemetryRxPin);

            for (size_t i = 0; packet[i] != '\0' && i < 80; i++) {
                char c = packet[i];
                if (c >= 32 && c <= 126) {
                    Serial.print(c);
                } else {
                    Serial.printf("\\x%02X", static_cast<unsigned char>(c));
                }
            }

            Serial.println("'");
        }
    }
}

static void configureI2CTelemetrySlave()
{
    int sdaPin = i2cPinSwap ? TELEMETRY_I2C_SCL_PIN : TELEMETRY_I2C_SDA_PIN;
    int sclPin = i2cPinSwap ? TELEMETRY_I2C_SDA_PIN : TELEMETRY_I2C_SCL_PIN;

    Wire.end();
    Wire.setBufferSize(128);
    bool started = Wire.begin(
        TELEMETRY_I2C_ADDR,
        sdaPin,
        sclPin,
        TELEMETRY_I2C_CLOCK
    );
    Wire.setTimeOut(50);

    Wire.onReceive([](int len) {
        size_t index = 0;

        while (Wire.available() > 0 && index < sizeof(i2cTelemetryLine) - 1) {
            char c = static_cast<char>(Wire.read());

            if (c == '\r' || c == '\n') {
                continue;
            }

            i2cTelemetryLine[index++] = c;
        }

        while (Wire.available() > 0) {
            Wire.read();
        }

        i2cTelemetryLine[index] = '\0';
        hasI2CTelemetryLine = index > 0;
        mpuTelemetry.byteCount += len;
    });

    Serial.printf(
        "Telemetry I2C slave %s: addr=0x%02X SDA=%d SCL=%d%s\r\n",
        started ? "enabled" : "FAILED",
        TELEMETRY_I2C_ADDR,
        sdaPin,
        sclPin,
        i2cPinSwap ? " swapped" : ""
    );
}

static void beginTelemetryLink()
{
    if (!ENABLE_MPU_TELEMETRY) {
        return;
    }

    if (ENABLE_PULSE_TELEMETRY) {
        pinMode(TELEMETRY_UART_RX_PIN, INPUT_PULLUP);
        lastPulseLevel = digitalRead(TELEMETRY_UART_RX_PIN);
        Serial.printf(
            "Telemetry pulse RX enabled: RX=%d\r\n",
            TELEMETRY_UART_RX_PIN
        );
        return;
    }

    if (ENABLE_I2C_TELEMETRY) {
        configureI2CTelemetrySlave();
        return;
    }

    activeTelemetryRxPin = ENABLE_UART_RX_AUTO_PROBE ? UART_RX_PROBE_PINS[0] : TELEMETRY_UART_RX_PIN;

    if (activeTelemetryRxPin < 0 ||
        activeTelemetryRxPin > 39 ||
        TELEMETRY_UART_TX_PIN < 0 ||
        TELEMETRY_UART_TX_PIN > 39) {
        Serial.printf(
            "Telemetry UART disabled: invalid ESP32 pins RX=%d TX=%d\r\n",
            activeTelemetryRxPin,
            TELEMETRY_UART_TX_PIN
        );
        return;
    }

    telemetrySerial.begin(
        TELEMETRY_UART_BAUD,
        SERIAL_8N1,
        activeTelemetryRxPin,
        TELEMETRY_UART_TX_PIN
    );
    resetTelemetryRawSample();

    Serial.printf(
        "Telemetry UART1 enabled: RX=%d TX=%d baud=%lu%s\r\n",
        activeTelemetryRxPin,
        TELEMETRY_UART_TX_PIN,
        static_cast<unsigned long>(TELEMETRY_UART_BAUD),
        ENABLE_UART_RX_AUTO_PROBE ? " auto-probe" : ""
    );
}

static void completePulseTelemetryPacket()
{
    float gyro = static_cast<float>(pulseSign * max(0, pulseCount - 1));

    mpuTelemetry.valid = true;
    mpuTelemetry.gyroZDps = gyro;
    mpuTelemetry.confidence = 1.0f;
    mpuTelemetry.prediction = 0.0f;
    mpuTelemetry.nearOffset = 0.0f;
    mpuTelemetry.farOffset = 0.0f;
    mpuTelemetry.lostCount = 0;
    mpuTelemetry.lastUpdateMs = millis();
    mpuTelemetry.packetCount++;

    pulseState = 0;
    pulseCount = 0;
    pulseSign = 1;
}

static void updatePulseTelemetryLink()
{
    uint32_t now = millis();
    int level = digitalRead(TELEMETRY_UART_RX_PIN);

    if (level != lastPulseLevel) {
        if (level == LOW) {
            pulseLowStartMs = now;
        } else {
            uint32_t lowMs = now - pulseLowStartMs;

            if (lowMs >= 220) {
                pulseState = 1;
                pulseCount = 0;
                pulseSign = 1;
                mpuTelemetry.byteCount++;
            } else if (pulseState == 1 && lowMs >= 80) {
                pulseSign = (lowMs >= 150) ? -1 : 1;
                pulseState = 2;
                lastPulseEdgeMs = now;
                mpuTelemetry.byteCount++;
            } else if (pulseState == 2 && lowMs >= 70) {
                pulseCount++;
                lastPulseEdgeMs = now;
                mpuTelemetry.byteCount++;
            }
        }

        lastPulseLevel = level;
    }

    if (pulseState == 2 && pulseCount > 0 && now - lastPulseEdgeMs >= 500) {
        completePulseTelemetryPacket();
    }
}

static void updateTelemetryRxProbe()
{
    if (!ENABLE_MPU_TELEMETRY || !ENABLE_UART_RX_AUTO_PROBE || mpuTelemetry.packetCount > 0) {
        return;
    }

    uint32_t now = millis();

    if (now - lastTelemetryProbeMs < UART_RX_PROBE_INTERVAL_MS) {
        return;
    }

    lastTelemetryProbeMs = now;

    constexpr size_t PIN_COUNT = sizeof(UART_RX_PROBE_PINS) / sizeof(UART_RX_PROBE_PINS[0]);
    static size_t probeIndex = 0;

    probeIndex = (probeIndex + 1) % PIN_COUNT;
    printTelemetryRawSample("Telemetry probe sample");
    activeTelemetryRxPin = UART_RX_PROBE_PINS[probeIndex];

    if (activeTelemetryRxPin < 0 || activeTelemetryRxPin > 39) {
        Serial.printf("Telemetry UART probe skipped invalid RX=%d\r\n", activeTelemetryRxPin);
        return;
    }

    telemetryLineLen = 0;
    resetTelemetryRawSample();
    telemetrySerial.end();
    telemetrySerial.begin(
        TELEMETRY_UART_BAUD,
        SERIAL_8N1,
        activeTelemetryRxPin,
        TELEMETRY_UART_TX_PIN
    );

    Serial.printf(
        "Telemetry UART probing RX=%d bytes=%lu packets=%lu parseFail=%lu\r\n",
        activeTelemetryRxPin,
        static_cast<unsigned long>(mpuTelemetry.byteCount),
        static_cast<unsigned long>(mpuTelemetry.packetCount),
        static_cast<unsigned long>(mpuTelemetry.parseFailCount)
    );
}

static void updateTelemetryLink()
{
    if (!ENABLE_MPU_TELEMETRY) {
        return;
    }

    if (ENABLE_PULSE_TELEMETRY) {
        updatePulseTelemetryLink();
        return;
    }

    if (ENABLE_I2C_TELEMETRY) {
        uint32_t now = millis();

        if (mpuTelemetry.byteCount == 0 &&
            mpuTelemetry.packetCount == 0 &&
            now - lastI2CReprobeMs >= 2500) {
            lastI2CReprobeMs = now;
            i2cPinSwap = !i2cPinSwap;
            configureI2CTelemetrySlave();
        }

        if (hasI2CTelemetryLine) {
            char packet[128];

            noInterrupts();
            strncpy(packet, i2cTelemetryLine, sizeof(packet));
            packet[sizeof(packet) - 1] = '\0';
            hasI2CTelemetryLine = false;
            interrupts();

            parseTelemetryLine(packet);
        }

        return;
    }

    while (telemetrySerial.available() > 0) {
        char c = static_cast<char>(telemetrySerial.read());
        mpuTelemetry.byteCount++;
        lastTelemetryByteMs = millis();
        recordTelemetryRawByte(static_cast<uint8_t>(c));

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

    if (telemetryLineLen >= 5 &&
        (telemetryLine[0] == 'P' || telemetryLine[0] == 'G') &&
        telemetryLine[1] == ',' &&
        millis() - lastTelemetryByteMs >= 1000) {
        telemetryLine[telemetryLineLen] = '\0';
        parseTelemetryLine(telemetryLine);
        telemetryLineLen = 0;
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
    updateTelemetryRxProbe();
    updateUartRxPinScan();

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
    // int speedDelta = static_cast<int>(lroundf(TURN_SPEED_STEP * controlTurnWeight));

    int effectiveBaseSpeed = STRAIGHT_SPEED;

    if (realLost || hardLeft || hardRight) {
        effectiveBaseSpeed = HARD_OR_LOST_SPEED;
    } else if (lineSeen && abs(turnWeight) >= BIG_CURVE_TURN_WEIGHT) {
        effectiveBaseSpeed = BIG_CURVE_SPEED;
    } else if (lineSeen && abs(turnWeight) >= SMALL_CURVE_TURN_WEIGHT) {
        effectiveBaseSpeed = SMALL_CURVE_SPEED;
    }

    // ===== 转向死区：消除小幅度抖动 =====

// 原始转向指令（已包含灰度、MPU等修正）
    float controlTurnWeightRaw = controlTurnWeight;  // 即您之前计算出的最终 controlTurnWeight

    // 应用死区
    if (fabs(controlTurnWeightRaw) < TURN_DEADZONE) {
        controlTurnWeight = 0.0f;   // 强制归零，相当于直道
    } else {
        controlTurnWeight = controlTurnWeightRaw;   // 保持原值
    }
    // =====================================

    // 然后计算 speedDelta（基于修正后的 controlTurnWeight）
    int speedDelta = static_cast<int>(lroundf(TURN_SPEED_STEP * controlTurnWeight));
    int maxSpeedDelta = effectiveBaseSpeed + NORMAL_TURN_DELTA_MARGIN;

    if (realLost || hardLeft || hardRight) {
        maxSpeedDelta = HARD_OR_LOST_MAX_SPEED_DELTA;
    }

    speedDelta = constrain(speedDelta, -maxSpeedDelta, maxSpeedDelta);
    int leftSpeed = effectiveBaseSpeed - speedDelta;
    int rightSpeed = effectiveBaseSpeed + speedDelta;

    leftSpeed += LEFT_SPEED_TRIM;
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
            "t=%lu gray=%d line=%d hardL=%d hardR=%d lost=%d base=%d grayD=%.2f gyro=%.2f corr=%.2f %s=%s rx=%d bytes=%lu pkt=%lu age=%lu valid=%d conf=%.2f turn=%d ctrl=%.2f L=%d R=%d\r\n",
            static_cast<unsigned long>(now),
            turnWeight,
            lineSeen ? 1 : 0,
            hardLeft ? 1 : 0,
            hardRight ? 1 : 0,
            realLost ? 1 : 0,
            effectiveBaseSpeed,
            grayDCorrection,
            mpuTelemetryFresh() ? mpuTelemetry.gyroZDps : 0.0f,
            gyroCorrection,
            ENABLE_I2C_TELEMETRY ? "i2c" : "uart",
            mpuTelemetryFresh() ? "ok" : "wait",
            activeTelemetryRxPin,
            static_cast<unsigned long>(mpuTelemetry.byteCount),
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
            "mpu raw: pred=%.3f near=%.3f far=%.3f gyro=%.2f valid=%d age=%lu rx=%d bytes=%lu packets=%lu parseFail=%lu\n",
            mpuTelemetry.prediction,
            mpuTelemetry.nearOffset,
            mpuTelemetry.farOffset,
            mpuTelemetry.gyroZDps,
            mpuTelemetry.valid ? 1 : 0,
            mpuTelemetry.valid ? static_cast<unsigned long>(now - mpuTelemetry.lastUpdateMs) : 0UL,
            activeTelemetryRxPin,
            static_cast<unsigned long>(mpuTelemetry.byteCount),
            static_cast<unsigned long>(mpuTelemetry.packetCount),
            static_cast<unsigned long>(mpuTelemetry.parseFailCount)
        );
    }

    delay(LOOP_DELAY_MS);
}
