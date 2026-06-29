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
    config.pixel_format = PIXFORMAT_JPEG;
    config.frame_size = psramFound() ? FRAMESIZE_VGA : FRAMESIZE_QVGA;
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
        sensor->set_quality(sensor, config.jpeg_quality);
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
    <img id="frame" src="/capture" alt="camera frame">
    <p id="status" class="status">Loading frames...</p>
    <p class="links">Still image: <a href="/capture">/capture</a> | MJPEG: <a href="/stream">/stream</a></p>
  </main>
  <script>
    const frame = document.getElementById('frame');
    const statusEl = document.getElementById('status');
    let count = 0;

    function nextFrame() {
      frame.src = '/capture?t=' + Date.now();
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

    captureCount++;
    Serial.printf("Capture #%lu: %ux%u len=%u\r\n",
                  static_cast<unsigned long>(captureCount),
                  fb->width,
                  fb->height,
                  fb->len);

    WiFiClient client = server.client();
    client.print("HTTP/1.1 200 OK\r\n");
    client.print("Content-Type: image/jpeg\r\n");
    client.print("Cache-Control: no-store\r\n");
    client.printf("Content-Length: %u\r\n\r\n", fb->len);
    client.write(fb->buf, fb->len);
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
    server.on("/stream", HTTP_GET, handleStream);
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

