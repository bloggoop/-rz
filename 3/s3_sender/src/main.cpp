#include <Arduino.h>
#include <string.h>

constexpr int UART_TX_PIN = 45;
constexpr int UART_RX_PIN = 46;
constexpr uint32_t UART_BAUDRATE = 115200;
constexpr uint32_t SEND_INTERVAL_MS = 300;

HardwareSerial linkUart(1);
int seq = 0;

static uint8_t checksum(const char *text) {
    uint8_t value = 0;

    while (*text != '\0') {
        value ^= static_cast<uint8_t>(*text);
        text++;
    }

    return value;
}

static void sendPacket(const char *body) {
    uint8_t crc = checksum(body);
    linkUart.printf("%s*%02X\n", body, crc);
    Serial.printf("TX:%s*%02X\r\n", body, crc);
}

static void sendImuPacket() {
    char body[64];
    seq++;

    float yaw = (seq % 20) * 0.10f;

    snprintf(
        body,
        sizeof(body),
        "IMU,%d,%.2f,%.2f,%.2f",
        seq,
        0.0f,
        0.0f,
        yaw
    );

    sendPacket(body);
}

void setup() {
    Serial.begin(115200);
    delay(300);

    linkUart.begin(UART_BAUDRATE, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

    Serial.printf(
        "S3 sender started: UART1 tx=GPIO%d rx=GPIO%d baud=%lu\r\n",
        UART_TX_PIN,
        UART_RX_PIN,
        static_cast<unsigned long>(UART_BAUDRATE)
    );
    Serial.println("Wire: S3 GPIO45 TX -> V1 GPIO22 RX, S3 GPIO46 RX <- V1 GPIO23 TX, common GND.");
}

void loop() {
    static uint32_t lastSendMs = 0;
    uint32_t now = millis();

    if (now - lastSendMs >= SEND_INTERVAL_MS) {
        lastSendMs = now;
        sendImuPacket();
    }
}
