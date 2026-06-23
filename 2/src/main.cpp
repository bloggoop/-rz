#include <Arduino.h>
#include <Wire.h>
#include "esp_camera.h"

#ifndef MPU6050_SDA_PIN
#define MPU6050_SDA_PIN 13
#endif

#ifndef MPU6050_SCL_PIN
#define MPU6050_SCL_PIN 14
#endif

// ESP32-CAM AI Thinker pinout
#define CAMERA_MODEL_AI_THINKER

#if defined(CAMERA_MODEL_AI_THINKER)
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27

#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22
#endif

constexpr uint32_t I2C_CLOCK_HZ = 400000;
constexpr uint8_t MPU6050_ADDR_LOW = 0x68;
constexpr uint8_t MPU6050_ADDR_HIGH = 0x69;
constexpr uint8_t MPU6050_REG_PWR_MGMT_1 = 0x6B;
constexpr uint8_t MPU6050_REG_SMPLRT_DIV = 0x19;
constexpr uint8_t MPU6050_REG_CONFIG = 0x1A;
constexpr uint8_t MPU6050_REG_GYRO_CONFIG = 0x1B;
constexpr uint8_t MPU6050_REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t MPU6050_REG_WHO_AM_I = 0x75;
constexpr uint8_t MPU6050_REG_ACCEL_XOUT_H = 0x3B;

constexpr float ACCEL_SCALE = 16384.0f; // +/-2g
constexpr float GYRO_SCALE = 131.0f;    // +/-250 dps
constexpr float RAD_TO_DEG_F = 57.2957795f;
constexpr float PID_OUTPUT_LIMIT = 100.0f;
constexpr float CAMERA_PREDICT_MS = 120.0f;

static uint8_t mpuAddress = 0;

struct MpuRawData {
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t temp;
    int16_t gx;
    int16_t gy;
    int16_t gz;
};

struct ImuState {
    float roll = 0.0f;
    float pitch = 0.0f;
    float gyroBiasX = 0.0f;
    float gyroBiasY = 0.0f;
    float gyroBiasZ = 0.0f;
    uint32_t lastMicros = 0;
};

struct PidController {
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    float integral = 0.0f;
    float prevError = 0.0f;
    float outMin = -PID_OUTPUT_LIMIT;
    float outMax = PID_OUTPUT_LIMIT;

    PidController() = default;
    PidController(float p, float i, float d) : kp(p), ki(i), kd(d) {}

    float update(float error, float dt)
    {
        if (dt <= 0.0f) {
            return 0.0f;
        }

        integral += error * dt;

        const float derivative = (error - prevError) / dt;
        float output = kp * error + ki * integral + kd * derivative;
        if (output > outMax) {
            output = outMax;
        } else if (output < outMin) {
            output = outMin;
        }

        if ((output >= outMax && error > 0.0f) || (output <= outMin && error < 0.0f)) {
            integral -= error * dt;
        }

        prevError = error;
        return output;
    }
};

struct CameraPrediction {
    bool valid = false;
    bool hasHistory = false;
    float currentX = 0.0f;
    float currentY = 0.0f;
    float predictedX = 0.0f;
    float predictedY = 0.0f;
    float velocityX = 0.0f;
    float velocityY = 0.0f;
    uint32_t lastUpdateMs = 0;
    int width = 0;
    int height = 0;
};

static ImuState imu;
static PidController pidRoll(1.8f, 0.02f, 0.04f);
static PidController pidPitch(1.8f, 0.02f, 0.04f);
static PidController pidCamX(0.8f, 0.00f, 0.15f);
static PidController pidCamY(0.8f, 0.00f, 0.15f);
static CameraPrediction cameraPrediction;

static bool i2cAddressResponds(uint8_t address)
{
    Wire.beginTransmission(address);
    return Wire.endTransmission() == 0;
}

static bool writeRegister(uint8_t address, uint8_t reg, uint8_t value)
{
    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

static bool readRegisters(uint8_t address, uint8_t startReg, uint8_t *buffer, size_t length)
{
    Wire.beginTransmission(address);
    Wire.write(startReg);
    if (Wire.endTransmission(false) != 0) {
        return false;
    }

    const size_t received = Wire.requestFrom(address, length);
    if (received != length) {
        return false;
    }

    for (size_t i = 0; i < length; i++) {
        buffer[i] = Wire.read();
    }
    return true;
}

static bool readRegister(uint8_t address, uint8_t reg, uint8_t &value)
{
    return readRegisters(address, reg, &value, 1);
}

static int16_t readInt16BE(const uint8_t *data)
{
    return static_cast<int16_t>((static_cast<uint16_t>(data[0]) << 8) | data[1]);
}

static bool readMpuRaw(MpuRawData &data)
{
    uint8_t buffer[14] = {};
    if (!readRegisters(mpuAddress, MPU6050_REG_ACCEL_XOUT_H, buffer, sizeof(buffer))) {
        return false;
    }

    data.ax = readInt16BE(&buffer[0]);
    data.ay = readInt16BE(&buffer[2]);
    data.az = readInt16BE(&buffer[4]);
    data.temp = readInt16BE(&buffer[6]);
    data.gx = readInt16BE(&buffer[8]);
    data.gy = readInt16BE(&buffer[10]);
    data.gz = readInt16BE(&buffer[12]);
    return true;
}

static uint8_t findMpu6050()
{
    Serial.println("Scanning I2C bus...");
    bool foundAny = false;

    for (uint8_t address = 1; address < 127; address++) {
        if (i2cAddressResponds(address)) {
            foundAny = true;
            Serial.printf("I2C device found at 0x%02X", address);
            if (address == MPU6050_ADDR_LOW || address == MPU6050_ADDR_HIGH) {
                Serial.print(" (possible MPU6050)");
            }
            Serial.println();
        }
    }

    if (!foundAny) {
        Serial.println("No I2C devices found. Check VCC, GND, SDA, SCL, and pull-ups.");
    }

    if (i2cAddressResponds(MPU6050_ADDR_LOW)) {
        return MPU6050_ADDR_LOW;
    }
    if (i2cAddressResponds(MPU6050_ADDR_HIGH)) {
        return MPU6050_ADDR_HIGH;
    }
    return 0;
}

static bool initMpu6050()
{
    mpuAddress = findMpu6050();
    if (mpuAddress == 0) {
        Serial.println("MPU6050 not found at 0x68 or 0x69.");
        return false;
    }

    uint8_t whoAmI = 0;
    if (!readRegister(mpuAddress, MPU6050_REG_WHO_AM_I, whoAmI)) {
        Serial.println("MPU6050 found, but WHO_AM_I read failed.");
        return false;
    }

    Serial.printf("MPU6050 address: 0x%02X, WHO_AM_I: 0x%02X\r\n", mpuAddress, whoAmI);
    if (whoAmI != 0x68) {
        Serial.println("Warning: WHO_AM_I is not 0x68. Wiring may be OK, but the chip may not be MPU6050.");
    }

    if (!writeRegister(mpuAddress, MPU6050_REG_PWR_MGMT_1, 0x00)) {
        Serial.println("Failed to wake MPU6050.");
        return false;
    }

    if (!writeRegister(mpuAddress, MPU6050_REG_SMPLRT_DIV, 9)) {
        Serial.println("Failed to set MPU6050 sample rate.");
        return false;
    }
    if (!writeRegister(mpuAddress, MPU6050_REG_CONFIG, 0x03)) {
        Serial.println("Failed to set MPU6050 DLPF.");
        return false;
    }
    if (!writeRegister(mpuAddress, MPU6050_REG_GYRO_CONFIG, 0x00)) {
        Serial.println("Failed to set MPU6050 gyro range.");
        return false;
    }
    if (!writeRegister(mpuAddress, MPU6050_REG_ACCEL_CONFIG, 0x00)) {
        Serial.println("Failed to set MPU6050 accel range.");
        return false;
    }

    delay(100);
    Serial.println("MPU6050 wake OK.");
    return true;
}

static void calibrateMpu(ImuState &state)
{
    constexpr int samples = 300;
    long gxSum = 0;
    long gySum = 0;
    long gzSum = 0;

    Serial.println("Calibrating MPU6050 bias. Keep the device still...");
    for (int i = 0; i < samples; i++) {
        MpuRawData data = {};
        if (readMpuRaw(data)) {
            gxSum += data.gx;
            gySum += data.gy;
            gzSum += data.gz;
        }
        delay(5);
    }

    state.gyroBiasX = static_cast<float>(gxSum) / samples;
    state.gyroBiasY = static_cast<float>(gySum) / samples;
    state.gyroBiasZ = static_cast<float>(gzSum) / samples;
    state.lastMicros = micros();

    MpuRawData data = {};
    if (readMpuRaw(data)) {
        const float ax = static_cast<float>(data.ax) / ACCEL_SCALE;
        const float ay = static_cast<float>(data.ay) / ACCEL_SCALE;
        const float az = static_cast<float>(data.az) / ACCEL_SCALE;
        state.roll = atan2f(ay, az) * RAD_TO_DEG_F;
        state.pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG_F;
    }

    Serial.printf("Gyro bias: X=%.2f Y=%.2f Z=%.2f\r\n",
                  state.gyroBiasX,
                  state.gyroBiasY,
                  state.gyroBiasZ);
}

static bool updateImu(ImuState &state, float &rollOut, float &pitchOut, MpuRawData &rawOut)
{
    if (!readMpuRaw(rawOut)) {
        return false;
    }

    const uint32_t now = micros();
    float dt = static_cast<float>(now - state.lastMicros) * 1e-6f;
    if (dt <= 0.0f || dt > 0.5f) {
        dt = 0.01f;
    }
    state.lastMicros = now;

    const float ax = static_cast<float>(rawOut.ax) / ACCEL_SCALE;
    const float ay = static_cast<float>(rawOut.ay) / ACCEL_SCALE;
    const float az = static_cast<float>(rawOut.az) / ACCEL_SCALE;
    const float gx = (static_cast<float>(rawOut.gx) - state.gyroBiasX) / GYRO_SCALE;
    const float gy = (static_cast<float>(rawOut.gy) - state.gyroBiasY) / GYRO_SCALE;

    const float rollAcc = atan2f(ay, az) * RAD_TO_DEG_F;
    const float pitchAcc = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG_F;

    // Complementary filter: gyro for short-term dynamics, accel for long-term drift correction.
    state.roll = 0.98f * (state.roll + gx * dt) + 0.02f * rollAcc;
    state.pitch = 0.98f * (state.pitch + gy * dt) + 0.02f * pitchAcc;

    rollOut = state.roll;
    pitchOut = state.pitch;
    return true;
}

static bool setupCamera()
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
    config.frame_size = FRAMESIZE_QQVGA;
    config.jpeg_quality = 12;
    config.fb_count = psramFound() ? 2 : 1;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode = CAMERA_GRAB_LATEST;

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed: 0x%X\r\n", err);
        return false;
    }

    sensor_t *sensor = esp_camera_sensor_get();
    if (sensor != nullptr) {
        sensor->set_brightness(sensor, 0);
        sensor->set_contrast(sensor, 1);
        sensor->set_saturation(sensor, -1);
    }

    Serial.println("Camera init OK.");
    return true;
}

static bool updateCameraPrediction(CameraPrediction &prediction)
{
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb == nullptr) {
        return false;
    }

    prediction.width = fb->width;
    prediction.height = fb->height;

    const uint8_t *buf = fb->buf;
    const int width = fb->width;
    const int height = fb->height;
    const int step = 2;
    const uint8_t threshold = 140;

    uint64_t sumX = 0;
    uint64_t sumY = 0;
    uint64_t sumW = 0;

    for (int y = 0; y < height; y += step) {
        const int row = y * width;
        for (int x = 0; x < width; x += step) {
            const uint8_t v = buf[row + x];
            if (v >= threshold) {
                const uint32_t w = static_cast<uint32_t>(v - threshold + 1);
                sumX += static_cast<uint64_t>(x) * w;
                sumY += static_cast<uint64_t>(y) * w;
                sumW += w;
            }
        }
    }

    esp_camera_fb_return(fb);

    if (sumW == 0) {
        prediction.valid = false;
        return false;
    }

    const float cx = static_cast<float>(sumX) / static_cast<float>(sumW);
    const float cy = static_cast<float>(sumY) / static_cast<float>(sumW);
    const uint32_t now = millis();

    if (!prediction.hasHistory) {
        prediction.currentX = cx;
        prediction.currentY = cy;
        prediction.predictedX = cx;
        prediction.predictedY = cy;
        prediction.velocityX = 0.0f;
        prediction.velocityY = 0.0f;
        prediction.lastUpdateMs = now;
        prediction.hasHistory = true;
        prediction.valid = true;
        return true;
    }

    float dt = static_cast<float>(now - prediction.lastUpdateMs) * 1e-3f;
    if (dt <= 0.0f) {
        dt = 0.05f;
    }
    prediction.lastUpdateMs = now;

    const float instVx = (cx - prediction.currentX) / dt;
    const float instVy = (cy - prediction.currentY) / dt;
    prediction.velocityX = 0.7f * prediction.velocityX + 0.3f * instVx;
    prediction.velocityY = 0.7f * prediction.velocityY + 0.3f * instVy;
    prediction.currentX = cx;
    prediction.currentY = cy;

    const float predictSec = CAMERA_PREDICT_MS * 0.001f;
    prediction.predictedX = cx + prediction.velocityX * predictSec;
    prediction.predictedY = cy + prediction.velocityY * predictSec;
    if (prediction.predictedX < 0.0f) {
        prediction.predictedX = 0.0f;
    } else if (prediction.predictedX > static_cast<float>(width - 1)) {
        prediction.predictedX = static_cast<float>(width - 1);
    }
    if (prediction.predictedY < 0.0f) {
        prediction.predictedY = 0.0f;
    } else if (prediction.predictedY > static_cast<float>(height - 1)) {
        prediction.predictedY = static_cast<float>(height - 1);
    }

    prediction.valid = true;
    return true;
}

static void printStatus(const float roll,
                        const float pitch,
                        const float rollPid,
                        const float pitchPid,
                        const MpuRawData &raw,
                        const CameraPrediction &prediction,
                        const float camPidX,
                        const float camPidY)
{
    const float tempC = (raw.temp / 340.0f) + 36.53f;

    Serial.printf("IMU roll=%7.2f pitch=%7.2f | PID r=%7.2f p=%7.2f | temp=%5.2fC | ",
                  roll,
                  pitch,
                  rollPid,
                  pitchPid,
                  tempC);

    if (prediction.valid) {
        const float errX = prediction.predictedX - (prediction.width * 0.5f);
        const float errY = prediction.predictedY - (prediction.height * 0.5f);
        Serial.printf("CAM pos=(%6.1f,%6.1f) pred=(%6.1f,%6.1f) err=(%6.1f,%6.1f) pid=(%6.2f,%6.2f)\r\n",
                      prediction.currentX,
                      prediction.currentY,
                      prediction.predictedX,
                      prediction.predictedY,
                      errX,
                      errY,
                      camPidX,
                      camPidY);
    } else {
        Serial.printf("CAM no target\r\n");
    }
}

void setup()
{
    Serial.begin(115200);
    delay(2000);

    Serial.println();
    Serial.println("ESP32-CAM MPU6050 PID + camera prediction");
    Serial.printf("MPU SDA=%d SCL=%d I2C=%luHz\r\n",
                  MPU6050_SDA_PIN,
                  MPU6050_SCL_PIN,
                  static_cast<unsigned long>(I2C_CLOCK_HZ));

    Wire.begin(MPU6050_SDA_PIN, MPU6050_SCL_PIN);
    Wire.setClock(I2C_CLOCK_HZ);

    const bool cameraOk = setupCamera();
    const bool mpuOk = initMpu6050();

    if (mpuOk) {
        calibrateMpu(imu);
    }

    if (!cameraOk) {
        Serial.println("Camera is disabled until restart.");
    }
    if (!mpuOk) {
        Serial.println("MPU6050 is disabled until restart.");
    }
}

void loop()
{
    float roll = 0.0f;
    float pitch = 0.0f;
    MpuRawData raw = {};

    const bool imuOk = (mpuAddress != 0) && updateImu(imu, roll, pitch, raw);
    if (!imuOk) {
        Serial.println("MPU6050 read failed. Re-scanning...");
        mpuAddress = 0;
        delay(500);
        initMpu6050();
        if (mpuAddress != 0) {
            calibrateMpu(imu);
        }
        return;
    }

    const float rollError = 0.0f - roll;
    const float pitchError = 0.0f - pitch;
    const float rollPid = pidRoll.update(rollError, 0.01f);
    const float pitchPid = pidPitch.update(pitchError, 0.01f);

    const bool cameraOk = updateCameraPrediction(cameraPrediction);
    float camPidX = 0.0f;
    float camPidY = 0.0f;
    if (cameraOk && cameraPrediction.valid) {
        const float errorX = (cameraPrediction.predictedX - (cameraPrediction.width * 0.5f));
        const float errorY = (cameraPrediction.predictedY - (cameraPrediction.height * 0.5f));
        camPidX = pidCamX.update(errorX, 0.05f);
        camPidY = pidCamY.update(errorY, 0.05f);
    }

    printStatus(roll, pitch, rollPid, pitchPid, raw, cameraPrediction, camPidX, camPidY);
    delay(50);
}
