#include <Arduino.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

constexpr int UART_RX_PIN = 22;
constexpr int UART_TX_PIN = 23;
constexpr uint32_t UART_BAUDRATE = 115200;
constexpr uint32_t UART_TIMEOUT_MS = 500;
constexpr bool UART_DEBUG_PRINT = true;

class AttitudeLink {
public:
    AttitudeLink(int rxPin, int txPin, uint32_t baudrate, uint32_t timeoutMs,
                 bool debugPrint)
        : rxPin(rxPin),
          txPin(txPin),
          baudrate(baudrate),
          timeoutMs(timeoutMs),
          debugPrint(debugPrint),
          uart(2) {}

    void begin() {
        uart.begin(baudrate, SERIAL_8N1, rxPin, txPin);
        Serial.printf(
            "UART(2) attitude link: rx=GPIO%d tx=GPIO%d baud=%lu\r\n",
            rxPin,
            txPin,
            static_cast<unsigned long>(baudrate)
        );
    }

    bool update() {
        bool updated = false;

        while (uart.available() > 0) {
            char c = static_cast<char>(uart.read());

            if (c == '\r') {
                continue;
            }

            if (c == '\n') {
                line[lineLen] = '\0';

                if (lineLen > 0) {
                    updated = parseLine(line) || updated;
                }

                lineLen = 0;
                continue;
            }

            if (lineLen < sizeof(line) - 1) {
                line[lineLen++] = c;
            } else {
                lineLen = 0;
                badPackets++;
            }
        }

        return updated;
    }

    bool isFresh() const {
        return lastUpdateMs != 0 && (millis() - lastUpdateMs) <= timeoutMs;
    }

    bool isCameraFresh() const {
        return lastCamUpdateMs != 0 && (millis() - lastCamUpdateMs) <= timeoutMs;
    }

    void printSnapshot() const {
        Serial.printf(
            "imu=%d,%d,%.2f,%.2f,%.2f cam=%d,%d,%.2f,%.2f,%.2f,%.2f lost=%lu,%lu bad=%lu\r\n",
            isFresh() ? 1 : 0,
            seq,
            pitch,
            roll,
            yaw,
            isCameraFresh() ? 1 : 0,
            camSeq,
            camNear,
            camFar,
            camCurve,
            camQuality,
            static_cast<unsigned long>(lostPackets),
            static_cast<unsigned long>(camLostPackets),
            static_cast<unsigned long>(badPackets)
        );
    }

private:
    int rxPin;
    int txPin;
    uint32_t baudrate;
    uint32_t timeoutMs;
    bool debugPrint;
    HardwareSerial uart;

    char line[128] = {};
    size_t lineLen = 0;

    int seq = 0;
    float pitch = 0.0f;
    float roll = 0.0f;
    float yaw = 0.0f;
    uint32_t lastUpdateMs = 0;

    int camSeq = 0;
    float camNear = 0.0f;
    float camFar = 0.0f;
    float camCurve = 0.0f;
    float camQuality = 0.0f;
    uint32_t lastCamUpdateMs = 0;

    uint32_t lostPackets = 0;
    uint32_t camLostPackets = 0;
    uint32_t badPackets = 0;

    uint8_t checksum(const char *text) const {
        uint8_t value = 0;

        while (*text != '\0') {
            value ^= static_cast<uint8_t>(*text);
            text++;
        }

        return value;
    }

    bool splitAndCheck(const char *raw, char *body, size_t bodySize) {
        const char *star = strrchr(raw, '*');

        if (star == nullptr) {
            strncpy(body, raw, bodySize);
            body[bodySize - 1] = '\0';
            return true;
        }

        size_t bodyLen = static_cast<size_t>(star - raw);

        if (bodyLen >= bodySize) {
            badPackets++;
            return false;
        }

        memcpy(body, raw, bodyLen);
        body[bodyLen] = '\0';

        char *end = nullptr;
        unsigned long received = strtoul(star + 1, &end, 16);

        if (end == star + 1 || received > 0xFF) {
            badPackets++;
            return false;
        }

        uint8_t expected = checksum(body);

        if (static_cast<uint8_t>(received) != expected) {
            badPackets++;

            if (debugPrint) {
                Serial.printf("UART_BAD_CRC,%s,%02X\r\n", raw, expected);
            }

            return false;
        }

        return true;
    }

    bool parseLine(const char *raw) {
        if (strncmp(raw, "IMU,", 4) != 0 && strncmp(raw, "CAM,", 4) != 0) {
            return false;
        }

        char body[128];

        if (!splitAndCheck(raw, body, sizeof(body))) {
            return false;
        }

        if (strncmp(body, "CAM,", 4) == 0) {
            return parseCameraBody(body);
        }

        return parseImuBody(body);
    }

    bool parseImuBody(const char *body) {
        int nextSeq = 0;
        float nextPitch = 0.0f;
        float nextRoll = 0.0f;
        float nextYaw = 0.0f;

        if (sscanf(body, "IMU,%d,%f,%f,%f",
                   &nextSeq, &nextPitch, &nextRoll, &nextYaw) != 4) {
            badPackets++;
            return false;
        }

        if (seq > 0 && nextSeq > seq + 1) {
            lostPackets += static_cast<uint32_t>(nextSeq - seq - 1);

            if (debugPrint) {
                Serial.printf("UART_LOST,%d,%d\r\n", seq + 1, nextSeq - 1);
            }
        }

        seq = nextSeq;
        pitch = nextPitch;
        roll = nextRoll;
        yaw = nextYaw;
        lastUpdateMs = millis();

        if (debugPrint) {
            Serial.printf("UART_IMU,%d,%.2f,%.2f,%.2f\r\n",
                          seq, pitch, roll, yaw);
        }

        return true;
    }

    bool parseCameraBody(const char *body) {
        int nextSeq = 0;
        float nearValue = 0.0f;
        float farValue = 0.0f;
        float curveValue = 0.0f;
        float qualityValue = 0.0f;

        if (sscanf(body, "CAM,%d,%f,%f,%f,%f",
                   &nextSeq, &nearValue, &farValue,
                   &curveValue, &qualityValue) != 5) {
            badPackets++;
            return false;
        }

        if (camSeq > 0 && nextSeq > camSeq + 1) {
            camLostPackets += static_cast<uint32_t>(nextSeq - camSeq - 1);

            if (debugPrint) {
                Serial.printf("CAM_LOST,%d,%d\r\n", camSeq + 1, nextSeq - 1);
            }
        }

        camSeq = nextSeq;
        camNear = constrain(nearValue, -1.0f, 1.0f);
        camFar = constrain(farValue, -1.0f, 1.0f);
        camCurve = constrain(curveValue, -1.0f, 1.0f);
        camQuality = constrain(qualityValue, 0.0f, 1.0f);
        lastCamUpdateMs = millis();

        if (debugPrint) {
            Serial.printf("UART_CAM,%d,%.2f,%.2f,%.2f,%.2f\r\n",
                          camSeq, camNear, camFar, camCurve, camQuality);
        }

        return true;
    }
};

AttitudeLink attitudeLink(
    UART_RX_PIN,
    UART_TX_PIN,
    UART_BAUDRATE,
    UART_TIMEOUT_MS,
    UART_DEBUG_PRINT
);

void setup() {
    Serial.begin(115200);
    delay(300);
    attitudeLink.begin();
    Serial.println("Communication test started.");
    Serial.println("Wire: S3 GPIO45 TX -> V1 GPIO22 RX, S3 GPIO46 RX <- V1 GPIO23 TX, common GND.");
}

void loop() {
    attitudeLink.update();

    static uint32_t lastPrintMs = 0;
    uint32_t now = millis();

    if (now - lastPrintMs >= 500) {
        lastPrintMs = now;
        attitudeLink.printSnapshot();
    }
}
