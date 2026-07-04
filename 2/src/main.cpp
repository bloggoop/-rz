#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <math.h>
#include <string.h>
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
constexpr bool TRACK_LINE_IS_DARK = true;
constexpr bool ENABLE_CAMERA = true;
constexpr framesize_t CAMERA_FRAME_SIZE = FRAMESIZE_QQVGA;
constexpr uint32_t CAMERA_XCLK_HZ = 24000000;

constexpr uint32_t TELEMETRY_SEND_INTERVAL_MS = 300;
constexpr uint32_t VISION_PROCESS_INTERVAL_MS = 30;
constexpr uint32_t VISION_UART_SEND_INTERVAL_MS = 30;

constexpr uint8_t VISION_SAMPLE_ROWS = 9;
constexpr uint8_t VISION_BLACK_MIN_PIXELS = 12;
constexpr uint8_t VISION_MIN_ROW_CONTRAST = 24;
constexpr uint8_t VISION_MIN_SEGMENT_PIXELS = 10;
constexpr float VISION_SEARCH_LEFT_RATIO = 0.18f;
constexpr float VISION_SEARCH_RIGHT_RATIO = 0.82f;
constexpr float VISION_EXPECTED_LINE_WIDTH_RATIO = 0.18f;
constexpr float VISION_CENTER_STABILITY_FULL_SCALE = 0.28f;
constexpr uint8_t VISION_STRAIGHT_MIN_ROWS = 7;
constexpr float VISION_STRAIGHT_MAX_END_DIFF = 0.16f;
constexpr float VISION_STRAIGHT_MAX_RESIDUAL = 0.055f;
constexpr float VISION_STRAIGHT_MAX_ROW_JUMP = 0.14f;
constexpr float VISION_LINE_FOUND_MIN_QUALITY = 0.35f;
constexpr float VISION_RIGHT_ANGLE_NEAR_OFFSET = 0.38f;
constexpr float VISION_RIGHT_ANGLE_END_DIFF = 0.42f;
constexpr float VISION_RIGHT_ANGLE_MAX_JUMP = 0.32f;
constexpr float VISION_S_CURVE_END_DIFF = 0.34f;
constexpr float VISION_S_CURVE_MID_SMALL = 0.18f;
constexpr float VISION_S_CURVE_SLOPE_MIN = 0.12f;

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

constexpr uint8_t MPU6050_ADDR_LOW = 0x68;
constexpr uint8_t MPU6050_ADDR_HIGH = 0x69;

// 如果 V1 上阻尼方向反了，优先改 V1 的 GYRO_Z_SIGN。
// 这里一般保持 1.0。
constexpr float MPU_GYRO_Z_SIGN = 1.0f;

constexpr float MPU_GYRO_Z_DEADBAND = 1.2f;
constexpr float MPU_GYRO_Z_FILTER = 0.75f;

WebServer server(80);

enum class RoadType : uint8_t {
    Unknown = 0,
    Straight = 1,
    SCurve = 2,
    RightAngle = 3,
    NormalCurve = 4,
};

static bool cameraReady = false;
static bool mpuReady = false;

static uint32_t captureCount = 0;
static uint32_t visionCount = 0;
static uint32_t telemetryCount = 0;
static uint32_t lastTelemetryMs = 0;
static uint32_t lastVisionMs = 0;
static uint32_t lastVisionUartMs = 0;
static uint32_t uartTelemetrySeq = 0;
static uint32_t latestGrayFrameId = 0;
static uint32_t latestGrayFrameWidth = 0;
static uint32_t latestGrayFrameHeight = 0;

static float gyroZBiasDps = 0.0f;
static float gyroZDps = 0.0f;
static float filteredGyroZDps = 0.0f;
HardwareSerial linkUart(1);
static uint8_t mpuAddr = MPU6050_ADDR_LOW;
static float visionFps = 0.0f;
static uint8_t latestGrayFrame[160 * 120] = {};

struct VisionResult {
    RoadType type = RoadType::Unknown;
    int validRows = 0;
    float centerOffset = 0.0f;
    float topOffset = 0.0f;
    float bottomOffset = 0.0f;
    float meanResidual = 0.0f;
    float maxJump = 0.0f;
    float confidence = 0.0f;
    uint32_t frameId = 0;
    uint32_t lastUpdateMs = 0;
};

static VisionResult vision;

static const char *roadTypeName(RoadType type)
{
    switch (type) {
    case RoadType::Straight:
        return "straight";
    case RoadType::SCurve:
        return "s_curve";
    case RoadType::RightAngle:
        return "right_angle";
    case RoadType::NormalCurve:
        return "normal_curve";
    default:
        return "unknown";
    }
}

// =========================
// MPU 底层
// =========================

static bool mpuWriteReg(uint8_t reg, uint8_t value)
{
    Wire.setClock(50000);
    Wire.beginTransmission(mpuAddr);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

static bool mpuReadRegs(uint8_t reg, uint8_t *buf, size_t len)
{
    Wire.setClock(50000);
    Wire.beginTransmission(mpuAddr);
    Wire.write(reg);

    if (Wire.endTransmission() != 0) {
        return false;
    }

    delay(2);

    size_t got = Wire.requestFrom(
        static_cast<int>(mpuAddr),
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
    Wire.setTimeOut(50);
    delay(100);

    uint8_t who = 0;
    const uint8_t candidates[] = {MPU6050_ADDR_LOW, MPU6050_ADDR_HIGH};
    bool found = false;

    for (size_t i = 0; i < sizeof(candidates); i++) {
        mpuAddr = candidates[i];
        who = 0;

        bool ok = mpuReadRegs(0x75, &who, 1);
        Serial.printf(
            "MPU6050 probe addr=0x%02X read=%s WHO_AM_I=0x%02X\r\n",
            mpuAddr,
            ok ? "ok" : "fail",
            who
        );

        if (ok && (who == 0x68 || who == 0x70)) {
            found = true;
            break;
        }
    }

    if (!found) {
        Serial.println("MPU6050 not found at 0x68 or 0x69");
        return false;
    }

    Serial.printf("MPU6050 attached at addr=0x%02X WHO_AM_I=0x%02X\r\n", mpuAddr, who);

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
    linkUart.printf("%s*%02X\n", body, crc);
}

static void beginV1UartLink()
{
    if (!ENABLE_V1_UART_LINK) {
        return;
    }

    linkUart.begin(V1_UART_BAUD, SERIAL_8N1, V1_UART_RX_PIN, V1_UART_TX_PIN);

    Serial.printf(
        "V1 UART telemetry: uart=1 TX=GPIO%d RX=GPIO%d baud=%lu\r\n",
        V1_UART_TX_PIN,
        V1_UART_RX_PIN,
        static_cast<unsigned long>(V1_UART_BAUD)
    );
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

static void sendV1VisionTelemetry(const VisionResult &result)
{
    char body[80];
    uartTelemetrySeq++;

    // CAM: near/far are image-center-relative offsets; curve carries the center-error target.
    snprintf(
        body,
        sizeof(body),
        "CAM,%lu,%.3f,%.3f,%.3f,%.2f,%u",
        static_cast<unsigned long>(uartTelemetrySeq),
        result.bottomOffset,
        result.topOffset,
        result.centerOffset,
        result.confidence,
        static_cast<unsigned>(result.type)
    );

    sendV1Packet(body);
}

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

    telemetryCount++;
}

// =========================
// Vision
// =========================

static float absf(float value)
{
    return value < 0.0f ? -value : value;
}

static bool isTrackPixel(uint8_t gray, uint8_t threshold)
{
    return TRACK_LINE_IS_DARK ? gray < threshold : gray >= threshold;
}

static bool findLineCenterInRow(
    const uint8_t *row,
    uint32_t width,
    float &normalizedCenter,
    int &centerX,
    uint16_t &segmentPixels
)
{
    uint8_t minGray = 255;
    uint8_t maxGray = 0;

    for (uint32_t x = 0; x < width; x++) {
        uint8_t gray = row[x];

        if (gray < minGray) {
            minGray = gray;
        }

        if (gray > maxGray) {
            maxGray = gray;
        }
    }

    uint8_t threshold = BINARY_THRESHOLD;

    if (maxGray - minGray >= VISION_MIN_ROW_CONTRAST) {
        threshold = static_cast<uint8_t>((static_cast<uint16_t>(minGray) + maxGray) / 2);
    }

    const int searchStart = constrain(
        static_cast<int>(width * VISION_SEARCH_LEFT_RATIO),
        0,
        static_cast<int>(width) - 1
    );
    const int searchEnd = constrain(
        static_cast<int>(width * VISION_SEARCH_RIGHT_RATIO),
        searchStart,
        static_cast<int>(width) - 1
    );
    int bestStart = -1;
    int bestEnd = -1;
    int bestScore = -1000000;
    int runStart = -1;
    const int imageCenterX = static_cast<int>(width / 2);

    for (int x = searchStart; x <= searchEnd; x++) {
        int blackVotes = 0;
        int samples = 0;

        for (int dx = -2; dx <= 2; dx++) {
            int nx = x + dx;

            if (nx >= 0 && nx < static_cast<int>(width)) {
                samples++;

                if (isTrackPixel(row[nx], threshold)) {
                    blackVotes++;
                }
            }
        }

        bool denoisedBlack = blackVotes >= 3 && samples >= 3;

        if (denoisedBlack) {
            if (runStart < 0) {
                runStart = x;
            }
        } else if (runStart >= 0) {
            int runEnd = x - 1;
            int runLen = runEnd - runStart + 1;

            if (runLen >= VISION_MIN_SEGMENT_PIXELS) {
                int runCenter = (runStart + runEnd) / 2;
                int distancePenalty = abs(runCenter - imageCenterX);
                int score = runLen * 12 - distancePenalty * 2;

                if (score > bestScore) {
                    bestScore = score;
                    bestStart = runStart;
                    bestEnd = runEnd;
                }
            }

            runStart = -1;
        }
    }

    if (runStart >= 0) {
        int runEnd = searchEnd;
        int runLen = runEnd - runStart + 1;

        if (runLen >= VISION_MIN_SEGMENT_PIXELS) {
            int runCenter = (runStart + runEnd) / 2;
            int distancePenalty = abs(runCenter - imageCenterX);
            int score = runLen * 12 - distancePenalty * 2;

            if (score > bestScore) {
                bestStart = runStart;
                bestEnd = runEnd;
            }
        }
    }

    if (bestStart < 0 || bestEnd < bestStart) {
        return false;
    }

    segmentPixels = static_cast<uint16_t>(bestEnd - bestStart + 1);

    if (segmentPixels < VISION_BLACK_MIN_PIXELS) {
        return false;
    }

    centerX = (bestStart + bestEnd) / 2;
    float center = static_cast<float>(centerX);
    float halfWidth = static_cast<float>(width - 1) * 0.5f;
    normalizedCenter = (center - halfWidth) / halfWidth;

    return true;
}

static RoadType classifyRoadShape(
    const float *offsets,
    int count,
    const VisionResult &result
)
{
    if (count < 5 || result.confidence < VISION_LINE_FOUND_MIN_QUALITY) {
        return RoadType::Unknown;
    }

    float farOffset = result.topOffset;
    float nearOffset = result.bottomOffset;
    float midOffset = offsets[count / 2];
    float endDiff = absf(nearOffset - farOffset);
    float farSlope = midOffset - farOffset;
    float nearSlope = nearOffset - midOffset;

    bool straightLine =
        count >= VISION_STRAIGHT_MIN_ROWS &&
        endDiff <= VISION_STRAIGHT_MAX_END_DIFF &&
        result.meanResidual <= VISION_STRAIGHT_MAX_RESIDUAL &&
        result.maxJump <= VISION_STRAIGHT_MAX_ROW_JUMP;

    if (straightLine) {
        return RoadType::Straight;
    }

    bool rightAngle =
        absf(nearOffset) >= VISION_RIGHT_ANGLE_NEAR_OFFSET &&
        (endDiff >= VISION_RIGHT_ANGLE_END_DIFF ||
         result.maxJump >= VISION_RIGHT_ANGLE_MAX_JUMP);

    if (rightAngle) {
        return RoadType::RightAngle;
    }

    bool oppositeEnds =
        (farOffset > VISION_S_CURVE_SLOPE_MIN && nearOffset < -VISION_S_CURVE_SLOPE_MIN) ||
        (farOffset < -VISION_S_CURVE_SLOPE_MIN && nearOffset > VISION_S_CURVE_SLOPE_MIN);
    bool oppositeSlopes =
        (farSlope > VISION_S_CURVE_SLOPE_MIN && nearSlope < -VISION_S_CURVE_SLOPE_MIN) ||
        (farSlope < -VISION_S_CURVE_SLOPE_MIN && nearSlope > VISION_S_CURVE_SLOPE_MIN);
    bool middleReturns =
        absf(midOffset) <= VISION_S_CURVE_MID_SMALL &&
        endDiff >= VISION_S_CURVE_END_DIFF;

    if ((oppositeEnds && middleReturns) || oppositeSlopes) {
        return RoadType::SCurve;
    }

    return RoadType::NormalCurve;
}

static VisionResult analyzeBinaryRoad(camera_fb_t *fb)
{
    VisionResult result;
    result.frameId = visionCount + 1;
    result.lastUpdateMs = millis();

    if (fb == nullptr || fb->format != PIXFORMAT_GRAYSCALE || fb->width < 2 || fb->height < 2) {
        return result;
    }

    float offsets[VISION_SAMPLE_ROWS] = {};
    uint16_t widths[VISION_SAMPLE_ROWS] = {};
    int count = 0;

    const uint32_t width = fb->width;
    const uint32_t height = fb->height;
    const uint32_t yStart = height / 2;
    const uint32_t yEnd = height - 1;

    for (uint8_t i = 0; i < VISION_SAMPLE_ROWS; i++) {
        uint32_t y = yEnd - ((yEnd - yStart) * i) / (VISION_SAMPLE_ROWS - 1);
        const uint8_t *row = fb->buf + y * width;
        float center = 0.0f;
        int centerX = 0;
        uint16_t segmentPixels = 0;

        if (findLineCenterInRow(row, width, center, centerX, segmentPixels)) {
            uint8_t offsetIndex = VISION_SAMPLE_ROWS - 1 - i;
            offsets[offsetIndex] = center;
            widths[offsetIndex] = segmentPixels;
            count++;
        }
    }

    result.validRows = count;

    if (count > 0) {
        int compactIndex = 0;

        for (uint8_t i = 0; i < VISION_SAMPLE_ROWS; i++) {
            if (widths[i] > 0) {
                offsets[compactIndex] = offsets[i];
                widths[compactIndex] = widths[i];
                compactIndex++;
            }
        }

        count = compactIndex;
        result.validRows = count;
    }

    if (count > 0) {
        float sum = 0.0f;
        float widthSum = 0.0f;

        result.topOffset = offsets[0];
        result.bottomOffset = offsets[count - 1];

        for (int i = 0; i < count; i++) {
            sum += offsets[i];
            widthSum += widths[i];
        }

        result.centerOffset = sum / static_cast<float>(count);

        float residualSum = 0.0f;
        result.maxJump = 0.0f;

        for (int i = 0; i < count; i++) {
            residualSum += absf(offsets[i] - result.centerOffset);

            if (i > 0) {
                float jump = absf(offsets[i] - offsets[i - 1]);

                if (jump > result.maxJump) {
                    result.maxJump = jump;
                }
            }
        }

        result.meanResidual = residualSum / static_cast<float>(count);

        float averageWidth = widthSum / static_cast<float>(count);
        float expectedWidth = static_cast<float>(width) * VISION_EXPECTED_LINE_WIDTH_RATIO;
        float widthQuality = constrain(averageWidth / expectedWidth, 0.0f, 1.0f);
        float rowQuality = static_cast<float>(count) / static_cast<float>(VISION_SAMPLE_ROWS);
        float stabilityQuality = 1.0f - constrain(
            result.meanResidual / VISION_CENTER_STABILITY_FULL_SCALE,
            0.0f,
            1.0f
        );

        result.confidence = constrain(
            rowQuality * 0.45f + widthQuality * 0.35f + stabilityQuality * 0.20f,
            0.0f,
            1.0f
        );
        result.type = classifyRoadShape(offsets, count, result);
    }

    return result;
}

static void cacheGrayFrame(camera_fb_t *fb, uint32_t frameId)
{
    if (fb == nullptr ||
        fb->format != PIXFORMAT_GRAYSCALE ||
        fb->width > 160 ||
        fb->height > 120) {
        return;
    }

    const uint32_t bytes = fb->width * fb->height;
    memcpy(latestGrayFrame, fb->buf, bytes);
    latestGrayFrameWidth = fb->width;
    latestGrayFrameHeight = fb->height;
    latestGrayFrameId = frameId;
}

static void updateVision()
{
    if (!ENABLE_CAMERA || !cameraReady) {
        return;
    }

    uint32_t now = millis();

    if (now - lastVisionMs < VISION_PROCESS_INTERVAL_MS) {
        return;
    }

    camera_fb_t *fb = esp_camera_fb_get();

    if (fb == nullptr) {
        lastVisionMs = now;
        return;
    }

    uint32_t previousUpdateMs = vision.lastUpdateMs;

    VisionResult rawVision = analyzeBinaryRoad(fb);
    vision = rawVision;
    visionCount++;
    cacheGrayFrame(fb, vision.frameId);
    lastVisionMs = millis();

    if (previousUpdateMs != 0 && vision.lastUpdateMs > previousUpdateMs) {
        uint32_t dt = vision.lastUpdateMs - previousUpdateMs;
        float measuredFps = 1000.0f / static_cast<float>(dt);
        visionFps = 0.7f * visionFps + 0.3f * measuredFps;
    }

    esp_camera_fb_return(fb);

    if (ENABLE_V1_UART_LINK && now - lastVisionUartMs >= VISION_UART_SEND_INTERVAL_MS) {
        lastVisionUartMs = now;
        sendV1VisionTelemetry(vision);
    }
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

    config.xclk_freq_hz = CAMERA_XCLK_HZ;
    config.pixel_format = PIXFORMAT_GRAYSCALE;
    config.frame_size = CAMERA_FRAME_SIZE;
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
        sensor->set_framesize(sensor, CAMERA_FRAME_SIZE);
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
    <p id="vision" class="status">Loading vision...</p>
    <p><a href="/imu">/imu</a> | <a href="/vision">/vision</a> | <a href="/binary">/binary</a></p>
  </main>
  <script>
    const frame = document.getElementById('frame');
    const statusEl = document.getElementById('status');
    const imuEl = document.getElementById('imu');
    const visionEl = document.getElementById('vision');
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
          ' | uart: ' + j.telemetryCount;

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

    async function updateVision() {
      try {
        const r = await fetch('/vision?t=' + Date.now());
        const j = await r.json();
        visionEl.textContent =
          'Vision: ' + j.type +
          ' | offset: ' + j.centerOffset.toFixed(2) +
          ' | rows: ' + j.validRows +
          ' | lineQ: ' + j.confidence.toFixed(2) +
          ' | fps: ' + j.fps.toFixed(1);
      } catch (e) {
        visionEl.textContent = 'Vision read failed';
      }

      setTimeout(updateVision, 200);
    }

    frame.onload = () => {
      count++;
      statusEl.textContent = 'Frames loaded: ' + count;
      if (cameraEnabled) {
        setTimeout(nextFrame, 250);
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
    updateVision();
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

static void handleVision()
{
    uint32_t ageMs = vision.lastUpdateMs == 0 ? 0 : millis() - vision.lastUpdateMs;
    char json[384];

    snprintf(
        json,
        sizeof(json),
        "{\"type\":\"%s\",\"typeId\":%u,\"validRows\":%d,\"centerOffset\":%.3f,\"topOffset\":%.3f,\"bottomOffset\":%.3f,\"meanResidual\":%.3f,\"maxJump\":%.3f,\"confidence\":%.3f,\"frameId\":%lu,\"ageMs\":%lu,\"fps\":%.2f}",
        roadTypeName(vision.type),
        static_cast<unsigned>(vision.type),
        vision.validRows,
        vision.centerOffset,
        vision.topOffset,
        vision.bottomOffset,
        vision.meanResidual,
        vision.maxJump,
        vision.confidence,
        static_cast<unsigned long>(vision.frameId),
        static_cast<unsigned long>(ageMs),
        visionFps
    );

    server.send(200, "application/json", json);
}

static void sendBinaryBmpFromGray(const uint8_t *grayBuf, uint32_t width, uint32_t height)
{
    WiFiClient client = server.client();

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
        const uint8_t *src = grayBuf + static_cast<uint32_t>(y) * width;

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
    cacheGrayFrame(fb, latestGrayFrameId + 1);
    sendBinaryBmpFromGray(fb->buf, fb->width, fb->height);

    esp_camera_fb_return(fb);
}

static void startCameraServer()
{
    server.on("/", HTTP_GET, handleRoot);
    server.on("/capture", HTTP_GET, handleCapture);
    server.on("/binary", HTTP_GET, handleCapture);
    server.on("/imu", HTTP_GET, handleImu);
    server.on("/vision", HTTP_GET, handleVision);

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
    Serial.println("ESP32-S3 CAM + MPU6050 + UART telemetry");

    cameraReady = initCamera();

    if (!ENABLE_CAMERA) {
        Serial.println("Camera intentionally disabled.");
    } else if (!cameraReady) {
        Serial.println("Camera failed. Check ribbon cable and pin mapping.");
    }

    mpuReady = initMPU();

    beginV1UartLink();

    if (!mpuReady) {
        Serial.println("MPU failed. Camera still runs.");
    }

    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASSWORD);

    Serial.printf("WiFi AP: %s\r\n", AP_SSID);
    Serial.printf("Password: %s\r\n", AP_PASSWORD);
    Serial.printf("Open: http://%s/\r\n", WiFi.softAPIP().toString().c_str());
    Serial.printf("Camera enabled: %s\r\n", ENABLE_CAMERA ? "yes" : "no");

    startCameraServer();
}

void loop()
{
    updateMPU();
    updateVision();
    sendTelemetryPacket();

    server.handleClient();

    static uint32_t lastStatusMs = 0;
    uint32_t now = millis();

    if (now - lastStatusMs >= 2000) {
        lastStatusMs = now;

        Serial.printf(
            "alive camera=%s mpu=%s gyroZ=%.2f line=%s offset=%.2f lineQ=%.2f ip=%s clients=%u captures=%lu vision=%lu uart=%lu\r\n",
            cameraReady ? "ok" : "fail",
            mpuReady ? "ok" : "fail",
            filteredGyroZDps,
            roadTypeName(vision.type),
            vision.centerOffset,
            vision.confidence,
            WiFi.softAPIP().toString().c_str(),
            WiFi.softAPgetStationNum(),
            static_cast<unsigned long>(captureCount),
            static_cast<unsigned long>(visionCount),
            static_cast<unsigned long>(telemetryCount)
        );
    }
}
