#include <Arduino.h>
#include <Wire.h>

#ifndef MPU6050_SDA_PIN
#define MPU6050_SDA_PIN 4
#endif

#ifndef MPU6050_SCL_PIN
#define MPU6050_SCL_PIN 5
#endif

constexpr uint32_t I2C_CLOCK_HZ = 400000;
constexpr uint8_t MPU6050_ADDR_LOW = 0x68;
constexpr uint8_t MPU6050_ADDR_HIGH = 0x69;
constexpr uint8_t MPU6050_REG_PWR_MGMT_1 = 0x6B;
constexpr uint8_t MPU6050_REG_WHO_AM_I = 0x75;
constexpr uint8_t MPU6050_REG_ACCEL_XOUT_H = 0x3B;

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

    delay(100);
    Serial.println("MPU6050 wake OK. Reading raw accel/gyro data...");
    return true;
}

void setup()
{
    Serial.begin(115200);
    delay(2000);

    Serial.println();
    Serial.println("ESP32-S3 MPU6050 I2C test");
    Serial.printf("SDA=%d SCL=%d I2C=%luHz\r\n",
                  MPU6050_SDA_PIN,
                  MPU6050_SCL_PIN,
                  static_cast<unsigned long>(I2C_CLOCK_HZ));

    Wire.begin(MPU6050_SDA_PIN, MPU6050_SCL_PIN);
    Wire.setClock(I2C_CLOCK_HZ);

    initMpu6050();
}

void loop()
{
    if (mpuAddress == 0) {
        delay(2000);
        initMpu6050();
        return;
    }

    MpuRawData data = {};
    if (!readMpuRaw(data)) {
        Serial.println("MPU6050 read failed. Re-scanning...");
        mpuAddress = 0;
        delay(500);
        return;
    }

    const float tempC = (data.temp / 340.0f) + 36.53f;
    Serial.printf("accel raw: X=%6d Y=%6d Z=%6d | gyro raw: X=%6d Y=%6d Z=%6d | temp=%.2fC\r\n",
                  data.ax,
                  data.ay,
                  data.az,
                  data.gx,
                  data.gy,
                  data.gz,
                  tempC);

    delay(500);
}
