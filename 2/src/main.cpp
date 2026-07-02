#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <Wire.h>
#include <math.h>
#include "esp_camera.h"

// Camera pins for the board that already worked in the serial JPEG test.
#define PWDN_GPIO_NUM -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 15
#define SIOD_GPIO_NUM 4
#define SIOC_GPIO_NUM 5
#define Y9_GPIO_NUM 16
#define Y8_GPIO_NUM 17
#define Y7_GPIO_NUM 18
#define Y6_GPIO_NUM 12
#define Y5_GPIO_NUM 10
#define Y4_GPIO_NUM 8
#define Y3_GPIO_NUM 9
#define Y2_GPIO_NUM 11
#define VSYNC_GPIO_NUM 6
#define HREF_GPIO_NUM 7
#define PCLK_GPIO_NUM 13

constexpr char AP_SSID[] = "ESP32S3-CAM";
constexpr char AP_PASSWORD[] = "12345678";

constexpr uint8_t BINARY_THRESHOLD = 128;
constexpr bool ENABLE_CAMERA = false;

// =========================
// UDP 发给 V1 主控板
// =========================

constexpr uint16_t CAMERA_LINK_PORT = 3333;
constexpr uint32_t TELEMETRY_SEND_INTERVAL_MS = 300;

IPAddress TELEMETRY_BROADCAST_IP(192, 168, 4, 255);
WiFiUDP telemetryUdp;

// =========================
// UART 发给 V1 主控板
// =========================

constexpr bool ENABLE_V1_UART_LINK = true;
constexpr int V1_UART_TX_PIN = 45;
constexpr int V1_UART_RX_PIN = 46;
constexpr uint32_t V1_UART_BAUD = 115200;

// =========================
// MPU6050
// =========================

constexpr int MPU_SDA_PIN = 47;
constexpr int MPU_SCL_PIN = 21;

constexpr uint8_t MPU6050_ADDR = 0x68;

// 如果 V1 上阻尼方向反了，优先改 V1 的 GYRO_Z_SIGN。
// 这里一般保持 1.0。
constexpr float MPU_GYRO_Z_SIGN = 1.0f;

constexpr float MPU_GYRO_Z_DEADBAND = 1.2f;
constexpr float MPU_GYRO_Z_FILTER = 0.75f;

WebServer server(80);

static bool cameraReady = false;
static bool mpuReady = false;

static uint32_t captureCount = 0;
static uint32_t telemetryCount = 0;
static uint32_t lastTelemetryMs = 0;
static uint32_t uartTelemetrySeq = 0;

static float gyroZBiasDps = 0.0f;
static float gyroZDps = 0.0f;
static float filteredGyroZDps = 0.0f;
HardwareSerial v1Uart(1);

// =========================
// MPU 底层
// =========================

static bool mpuWriteReg(uint8_t reg, uint8_t value)
{
    Wire.begin(MPU_SDA_PIN, MPU_SCL_PIN, 50000);
    Wire.setClock(50000);
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

static bool mpuReadRegs(uint8_t reg, uint8_t *buf, size_t len)
{
    Wire.begin(MPU_SDA_PIN, MPU_SCL_PIN, 50000);
    Wire.setClock(50000);
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(reg);

    if (Wire.endTransmission(false) != 0) {
        return false;
    }

    size_t got = Wire.requestFrom(
        static_cast<int>(MPU6050_ADDR),
        static_cast<int>(len)
    );

    if (got != len) {
        return false;
    }

    for (size_t i = 0; i < len; i++) {
        buf[i] = Wire.read();
    }

    return true;
}

static int16_t makeInt16(uint8_t high, uint8_t low)
{
    return static_cast<int16_t>((static_cast<uint16_t>(high) << 8) | low);
}

static bool readGyroZRaw(int16_t &gzRaw)
{
    uint8_t buf[6];

    if (!mpuReadRegs(0x43, buf, sizeof(buf))) {
        return false;
    }

    gzRaw = makeInt16(buf[4], buf[5]);
    return true;
}

static bool initMPU()
{
    Wire.begin(MPU_SDA_PIN, MPU_SCL_PIN, 50000);
    Wire.setClock(50000);
    delay(100);

    uint8_t who = 0;

    if (!mpuReadRegs(0x75, &who, 1)) {
        Serial.println("MPU6050 not found: I2C read failed");
        return false;
    }

    Serial.printf("MPU6050 WHO_AM_I = 0x%02X\r\n", who);

    if (who != 0x68 && who != 0x70) {
        Serial.println("MPU6050 WHO_AM_I unexpected");
        return false;
    }

    mpuWriteReg(0x6B, 0x00);
    delay(50);

    // DLPF 约 44Hz
    mpuWriteReg(0x1A, 0x03);

    // gyro ±250 deg/s
    mpuWriteReg(0x1B, 0x00);

    // accel ±2g
    mpuWriteReg(0x1C, 0x00);

    Serial.println("Calibrating gyroZ, keep car still...");

    constexpr int CALIB_SAMPLES = 500;

    float sum = 0.0f;
    int validSamples = 0;

    for (int i = 0; i < CALIB_SAMPLES; i++) {
        int16_t gzRaw = 0;

        if (readGyroZRaw(gzRaw)) {
            sum += static_cast<float>(gzRaw) / 131.0f;
            validSamples++;
        }

        delay(3);
    }

    if (validSamples < 100) {
        Serial.println("MPU6050 calibration failed");
        return false;
    }

    gyroZBiasDps = sum / static_cast<float>(validSamples);
    gyroZDps = 0.0f;
    filteredGyroZDps = 0.0f;

    Serial.printf("MPU6050 init OK, gyroZ bias = %.3f deg/s\r\n", gyroZBiasDps);

    return true;
}

static void updateMPU()
{
    if (!mpuReady) {
        return;
    }

    int16_t gzRaw = 0;

    if (!readGyroZRaw(gzRaw)) {
        return;
    }

    float gz = static_cast<float>(gzRaw) / 131.0f;
    gz -= gyroZBiasDps;
    gz *= MPU_GYRO_Z_SIGN;

    if (fabsf(gz) < MPU_GYRO_Z_DEADBAND) {
        gz = 0.0f;
    }

    gyroZDps = gz;

    filteredGyroZDps =
        MPU_GYRO_Z_FILTER * filteredGyroZDps +
        (1.0f - MPU_GYRO_Z_FILTER) * gyroZDps;
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

static void sendV1Packet(const char *body)
{
    uint8_t crc = telemetryChecksum(body);
    v1Uart.printf("%s*%02X\n", body, crc);
}

static void sendV1ImuTelemetry(float gyroDps, int valid)
{
    char body[80];
    uartTelemetrySeq++;

    if (valid == 0) {
        gyroDps = 0.0f;
    }

    snprintf(
        body,
        sizeof(body),
        "IMU,%lu,%.2f,%.2f,%.2f",
        static_cast<unsigned long>(uartTelemetrySeq),
        0.0f,
        0.0f,
        gyroDps
    );

    sendV1Packet(body);
}

// =========================
// UDP
// =========================

static void sendTelemetryPacket()
{
    uint32_t now = millis();

    if (now - lastTelemetryMs < TELEMETRY_SEND_INTERVAL_MS) {
        return;
    }

    lastTelemetryMs = now;
    Wire.setClock(50000);

    int valid = mpuReady ? 1 : 0;

    if (ENABLE_V1_UART_LINK) {
        sendV1ImuTelemetry(filteredGyroZDps, valid);
    }

    static uint32_t lastUartDebugMs = 0;
    if (now - lastUartDebugMs >= 1000) {
        lastUartDebugMs = now;
        Serial.printf(
            "S3 -> V1 UART TX GPIO%d: gyro=%.1f valid=%d\r\n",
            V1_UART_TX_PIN,
            filteredGyroZDps,
            valid
        );
    }

    telemetryUdp.beginPacket(TELEMETRY_BROADCAST_IP, CAMERA_LINK_PORT);
    telemetryUdp.printf("G,%.1f,%d", filteredGyroZDps, valid);
    telemetryUdp.endPacket();

    telemetryCount++;
}

// =========================
// Camera
// =========================

static bool initCamera()
{
    if (!ENABLE_CAMERA) {
        Serial.println("Camera disabled by code switch");
        return false;
    }

    if (cameraReady) {
        return true;
    }

    camera_config_t config = {};

    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;

    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;

    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;

    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;

    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;

    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_GRAYSCALE;
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 12;

    config.fb_count = psramFound() ? 2 : 1;
    config.fb_location = psramFound() ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;
    config.grab_mode = CAMERA_GRAB_LATEST;

    Serial.printf("PSRAM: %s\r\n", psramFound() ? "yes" : "no");

    esp_err_t err = esp_camera_init(&config);

    if (err != ESP_OK) {
        Serial.printf("Camera init failed: 0x%X\r\n", err);
        return false;
    }

    sensor_t *sensor = esp_camera_sensor_get();

    if (sensor != nullptr) {
        sensor->set_framesize(sensor, config.frame_size);
    }

    Serial.println("Camera init OK");

    return true;
}

static void handleRoot()
{
    const char html[] PROGMEM = R"rawliteral(
<!doctype html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>ESP32-S3 Camera + MPU</title>
  <style>
    body { margin: 0; font-family: Arial, sans-serif; background: #111; color: #eee; }
    header { padding: 12px 16px; background: #202020; font-weight: 700; }
    main { padding: 16px; }
    img { width: 100%; max-width: 900px; height: auto; background: #000; display: block; }
    .status { margin: 10px 0 0; color: #9fd18b; }
    .disabled { color: #ffca6b; }
    a { color: #7cc7ff; }
  </style>
</head>
<body>
  <header>ESP32-S3 Camera + MPU6050</header>
  <main>
    <img id="frame" src="/binary" alt="binary camera frame">
    <p id="status" class="status">Loading frames...</p>
    <p id="imu" class="status">Loading IMU...</p>
    <p><a href="/imu">/imu</a> | <a href="/binary">/binary</a></p>
  </main>
  <script>
    const frame = document.getElementById('frame');
    const statusEl = document.getElementById('status');
    const imuEl = document.getElementById('imu');
    let count = 0;
    let cameraEnabled = false;
    let frameLoopStarted = false;

    function nextFrame() {
      frame.src = '/binary?t=' + Date.now();
    }

    async function updateImu() {
      try {
        const r = await fetch('/imu?t=' + Date.now());
        const j = await r.json();
        cameraEnabled = j.cameraEnabled;
        imuEl.textContent =
          'MPU: ' + (j.mpuReady ? 'ok' : 'fail') +
          ' | gyroZ: ' + j.gyroZ.toFixed(2) + ' deg/s' +
          ' | camera: ' + (j.cameraReady ? 'ok' : 'off') +
          ' | udp: ' + j.telemetryCount;

        if (cameraEnabled && !frameLoopStarted) {
          frameLoopStarted = true;
          statusEl.classList.remove('disabled');
          nextFrame();
        } else if (!cameraEnabled) {
          frame.removeAttribute('src');
          statusEl.textContent = 'Camera disabled by code switch';
          statusEl.classList.add('disabled');
          frameLoopStarted = false;
        }
      } catch (e) {
        imuEl.textContent = 'IMU read failed';
      }

      setTimeout(updateImu, 200);
    }

    frame.onload = () => {
      count++;
      statusEl.textContent = 'Frames loaded: ' + count;
      if (cameraEnabled) {
        setTimeout(nextFrame, 120);
      }
    };

    frame.onerror = () => {
      statusEl.textContent = 'Frame failed, retrying...';
      if (cameraEnabled) {
        setTimeout(nextFrame, 1000);
      }
    };

    if (cameraEnabled) {
      setTimeout(nextFrame, 200);
    } else {
      frame.alt = 'camera disabled';
      frame.removeAttribute('src');
      statusEl.textContent = 'Camera disabled by code switch';
      statusEl.classList.add('disabled');
    }

    updateImu();
  </script>
</body>
</html>
)rawliteral";

    server.send_P(200, "text/html", html);
}

static void handleImu()
{
    char json[256];

    snprintf(
        json,
        sizeof(json),
        "{\"mpuReady\":%s,\"gyroZ\":%.3f,\"rawGyroZ\":%.3f,\"bias\":%.3f,\"telemetryCount\":%lu,\"cameraEnabled\":%s,\"cameraReady\":%s}",
        mpuReady ? "true" : "false",
        filteredGyroZDps,
        gyroZDps,
        gyroZBiasDps,
        static_cast<unsigned long>(telemetryCount),
        ENABLE_CAMERA ? "true" : "false",
        cameraReady ? "true" : "false"
    );

    server.send(200, "application/json", json);
}

static void handleCapture()
{
    if (!ENABLE_CAMERA || !cameraReady) {
        server.send(503, "text/plain", "Camera is not ready");
        return;
    }

    camera_fb_t *fb = esp_camera_fb_get();

    if (fb == nullptr) {
        server.send(503, "text/plain", "Camera capture failed");
        return;
    }

    if (fb->format != PIXFORMAT_GRAYSCALE) {
        esp_camera_fb_return(fb);
        server.send(500, "text/plain", "Camera is not in grayscale mode");
        return;
    }

    captureCount++;

    WiFiClient client = server.client();

    const uint32_t width = fb->width;
    const uint32_t height = fb->height;
    const uint32_t rowSize = ((width * 3) + 3) & ~3U;
    const uint32_t pixelBytes = rowSize * height;
    const uint32_t fileSize = 54 + pixelBytes;

    client.print("HTTP/1.1 200 OK\r\n");
    client.print("Content-Type: image/bmp\r\n");
    client.print("Cache-Control: no-store\r\n");
    client.printf("Content-Length: %lu\r\n\r\n", static_cast<unsigned long>(fileSize));

    auto writeU16 = [&client](uint16_t value) {
        uint8_t bytes[2] = {
            static_cast<uint8_t>(value & 0xFF),
            static_cast<uint8_t>((value >> 8) & 0xFF),
        };

        client.write(bytes, sizeof(bytes));
    };

    auto writeU32 = [&client](uint32_t value) {
        uint8_t bytes[4] = {
            static_cast<uint8_t>(value & 0xFF),
            static_cast<uint8_t>((value >> 8) & 0xFF),
            static_cast<uint8_t>((value >> 16) & 0xFF),
            static_cast<uint8_t>((value >> 24) & 0xFF),
        };

        client.write(bytes, sizeof(bytes));
    };

    client.write('B');
    client.write('M');

    writeU32(fileSize);
    writeU16(0);
    writeU16(0);
    writeU32(54);
    writeU32(40);
    writeU32(width);
    writeU32(height);
    writeU16(1);
    writeU16(24);
    writeU32(0);
    writeU32(pixelBytes);
    writeU32(2835);
    writeU32(2835);
    writeU32(0);
    writeU32(0);

    uint8_t row[960] = {};

    for (int32_t y = static_cast<int32_t>(height) - 1; y >= 0; --y) {
        const uint8_t *src = fb->buf + static_cast<uint32_t>(y) * width;

        for (uint32_t x = 0; x < width; ++x) {
            uint8_t value = src[x] >= BINARY_THRESHOLD ? 255 : 0;
            uint32_t offset = x * 3;

            row[offset + 0] = value;
            row[offset + 1] = value;
            row[offset + 2] = value;
        }

        for (uint32_t i = width * 3; i < rowSize; ++i) {
            row[i] = 0;
        }

        client.write(row, rowSize);
    }

    esp_camera_fb_return(fb);
}

static void startCameraServer()
{
    server.on("/", HTTP_GET, handleRoot);
    server.on("/capture", HTTP_GET, handleCapture);
    server.on("/binary", HTTP_GET, handleCapture);
    server.on("/imu", HTTP_GET, handleImu);

    server.on("/stream", HTTP_GET, []() {
        server.send(200, "text/plain", "MJPEG stream disabled. Use /binary.");
    });

    server.begin();

    Serial.println("HTTP server started");
}

void setup()
{
    Serial.begin(115200);
    delay(2000);

    Serial.println();
    Serial.println("ESP32-S3 CAM + MPU6050 + UDP telemetry");

    cameraReady = initCamera();

    if (!ENABLE_CAMERA) {
        Serial.println("Camera intentionally disabled.");
    } else if (!cameraReady) {
        Serial.println("Camera failed. Check ribbon cable and pin mapping.");
    }

    mpuReady = initMPU();

    if (ENABLE_V1_UART_LINK) {
        v1Uart.begin(V1_UART_BAUD, SERIAL_8N1, V1_UART_RX_PIN, V1_UART_TX_PIN);
        Serial.printf(
            "V1 UART telemetry: TX=GPIO%d RX=GPIO%d baud=%lu\r\n",
            V1_UART_TX_PIN,
            V1_UART_RX_PIN,
            static_cast<unsigned long>(V1_UART_BAUD)
        );
    }

    if (!mpuReady) {
        Serial.println("MPU failed. Camera still runs.");
    }

    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASSWORD);

    telemetryUdp.begin(CAMERA_LINK_PORT);

    Serial.printf("WiFi AP: %s\r\n", AP_SSID);
    Serial.printf("Password: %s\r\n", AP_PASSWORD);
    Serial.printf("Open: http://%s/\r\n", WiFi.softAPIP().toString().c_str());
    Serial.printf("UDP broadcast: 192.168.4.255:%u\r\n", CAMERA_LINK_PORT);
    Serial.printf("Camera enabled: %s\r\n", ENABLE_CAMERA ? "yes" : "no");

    startCameraServer();
}

void loop()
{
    updateMPU();
    sendTelemetryPacket();

    server.handleClient();

    static uint32_t lastStatusMs = 0;
    uint32_t now = millis();

    if (now - lastStatusMs >= 2000) {
        lastStatusMs = now;

        Serial.printf(
            "alive camera=%s mpu=%s gyroZ=%.2f ip=%s clients=%u captures=%lu udp=%lu\r\n",
            cameraReady ? "ok" : "fail",
            mpuReady ? "ok" : "fail",
            filteredGyroZDps,
            WiFi.softAPIP().toString().c_str(),
            WiFi.softAPgetStationNum(),
            static_cast<unsigned long>(captureCount),
            static_cast<unsigned long>(telemetryCount)
        );
    }
}
