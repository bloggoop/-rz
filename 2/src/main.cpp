#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
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

WebServer server(80);
static bool cameraReady = false;
static uint32_t captureCount = 0;

static bool initCamera()
{
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

    const esp_err_t err = esp_camera_init(&config);
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
  <title>ESP32-S3 Camera</title>
  <style>
    body { margin: 0; font-family: Arial, sans-serif; background: #111; color: #eee; }
    header { padding: 12px 16px; background: #202020; font-weight: 700; }
    main { padding: 16px; }
    img { width: 100%; max-width: 900px; height: auto; background: #000; display: block; }
    .status { margin: 10px 0 0; color: #9fd18b; }
    .links { color: #aaa; }
    a { color: #7cc7ff; }
  </style>
</head>
<body>
  <header>ESP32-S3 Camera</header>
  <main>
    <img id="frame" src="/binary" alt="binary camera frame">
    <p id="status" class="status">Loading frames...</p>
    <p class="links">Binary image: <a href="/binary">/binary</a> | threshold: 128</p>
  </main>
  <script>
    const frame = document.getElementById('frame');
    const statusEl = document.getElementById('status');
    let count = 0;

    function nextFrame() {
      frame.src = '/binary?t=' + Date.now();
    }

    frame.onload = () => {
      count += 1;
      statusEl.textContent = 'Frames loaded: ' + count;
      setTimeout(nextFrame, 120);
    };

    frame.onerror = () => {
      statusEl.textContent = 'Frame load failed. Refreshing...';
      setTimeout(nextFrame, 1000);
    };

    setTimeout(nextFrame, 200);
  </script>
</body>
</html>
)rawliteral";

    server.send_P(200, "text/html", html);
}

static void handleCapture()
{
    if (!cameraReady) {
        server.send(503, "text/plain", "Camera is not ready");
        return;
    }

    camera_fb_t *fb = esp_camera_fb_get();
    if (fb == nullptr) {
        Serial.println("Capture failed");
        server.send(503, "text/plain", "Camera capture failed");
        return;
    }

    if (fb->format != PIXFORMAT_GRAYSCALE) {
        esp_camera_fb_return(fb);
        server.send(500, "text/plain", "Camera is not in grayscale mode");
        return;
    }

    captureCount++;
    Serial.printf("Binary #%lu: %ux%u gray_len=%u threshold=%u\r\n",
                  static_cast<unsigned long>(captureCount),
                  fb->width,
                  fb->height,
                  fb->len,
                  BINARY_THRESHOLD);

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
        const uint8_t *src = fb->buf + (static_cast<uint32_t>(y) * width);
        for (uint32_t x = 0; x < width; ++x) {
            const uint8_t value = src[x] >= BINARY_THRESHOLD ? 255 : 0;
            const uint32_t offset = x * 3;
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

static void handleStream()
{
    WiFiClient client = server.client();
    client.print("HTTP/1.1 200 OK\r\n");
    client.print("Content-Type: multipart/x-mixed-replace; boundary=frame\r\n");
    client.print("Cache-Control: no-cache\r\n");
    client.print("Connection: close\r\n\r\n");

    while (client.connected()) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (fb == nullptr) {
            break;
        }

        client.print("--frame\r\n");
        client.print("Content-Type: image/jpeg\r\n");
        client.printf("Content-Length: %u\r\n\r\n", fb->len);
        client.write(fb->buf, fb->len);
        client.print("\r\n");

        esp_camera_fb_return(fb);
        delay(40);
    }
}

static void startCameraServer()
{
    server.on("/", HTTP_GET, handleRoot);
    server.on("/capture", HTTP_GET, handleCapture);
    server.on("/binary", HTTP_GET, handleCapture);
    server.on("/stream", HTTP_GET, []() {
        server.send(200, "text/plain", "MJPEG stream is disabled. Open /binary for thresholded output.");
    });
    server.begin();
    Serial.println("HTTP server started");
}

void setup()
{
    Serial.begin(115200);
    delay(2000);

    Serial.println();
    Serial.println("ESP32-S3 camera web preview");

    cameraReady = initCamera();
    if (!cameraReady) {
        Serial.println("Check camera ribbon cable and pin mapping.");
        return;
    }

    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASSWORD);

    Serial.printf("WiFi AP: %s\r\n", AP_SSID);
    Serial.printf("Password: %s\r\n", AP_PASSWORD);
    Serial.printf("Open: http://%s/\r\n", WiFi.softAPIP().toString().c_str());

    startCameraServer();
}

void loop()
{
    server.handleClient();

    static uint32_t lastStatusMs = 0;
    const uint32_t now = millis();
    if (now - lastStatusMs >= 2000) {
        lastStatusMs = now;
        Serial.printf("alive camera=%s ap=%s ip=%s clients=%u captures=%lu\r\n",
                      cameraReady ? "ok" : "fail",
                      AP_SSID,
                      WiFi.softAPIP().toString().c_str(),
                      WiFi.softAPgetStationNum(),
                      static_cast<unsigned long>(captureCount));
    }
}
