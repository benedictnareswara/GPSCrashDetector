#include "mpu6050.hpp"

#include <Wire.h>
#include <math.h>

#include "app_config.hpp"

namespace {
uint8_t gMpuAddress = 0x68;
bool gMpuReady = false;
int gLastTxError = 0;
int gLastRxCount = 0;

const uint8_t REG_WHO_AM_I = 0x75;
const uint8_t REG_PWR_MGMT_1 = 0x6B;
const uint8_t REG_ACCEL_XOUT_H = 0x3B;

bool writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(gMpuAddress);
  Wire.write(reg);
  Wire.write(value);
  gLastTxError = Wire.endTransmission();
  return gLastTxError == 0;
}

bool readBytes(uint8_t startReg, uint8_t *buffer, size_t length) {
  Wire.beginTransmission(gMpuAddress);
  Wire.write(startReg);
  gLastTxError = Wire.endTransmission(false);
  if (gLastTxError != 0) {
    gLastRxCount = 0;
    return false;
  }

  size_t received = Wire.requestFrom(gMpuAddress, static_cast<uint8_t>(length));
  gLastRxCount = static_cast<int>(received);
  if (received != length) {
    return false;
  }

  for (size_t i = 0; i < length; i++) {
    buffer[i] = static_cast<uint8_t>(Wire.read());
  }
  return true;
}

bool probeAddress(uint8_t address) {
  Wire.beginTransmission(address);
  int tx = Wire.endTransmission();
  return tx == 0;
}

bool detectMpuAddress(uint8_t &address) {
  const uint8_t candidates[] = {0x68, 0x69};
  for (uint8_t i = 0; i < sizeof(candidates); i++) {
    if (probeAddress(candidates[i])) {
      address = candidates[i];
      return true;
    }
  }
  return false;
}

bool readRegister(uint8_t reg, uint8_t &value) {
  uint8_t b = 0;
  if (!readBytes(reg, &b, 1)) {
    return false;
  }
  value = b;
  return true;
}
}  // namespace

bool initMpu6050() {
  if (!detectMpuAddress(gMpuAddress)) {
    Serial.println("No I2C device found at 0x68 or 0x69.");
    Serial.println("Check wiring: SDA=20, SCL=21, GND common, and module power.");
    gMpuReady = false;
    return false;
  }

  Serial.print("MPU candidate found at 0x");
  Serial.println(gMpuAddress, HEX);

  uint8_t whoAmI = 0;
  if (!readRegister(REG_WHO_AM_I, whoAmI)) {
    Serial.print("WHO_AM_I read failed. txErr=");
    Serial.print(gLastTxError);
    Serial.print(" rxCount=");
    Serial.println(gLastRxCount);
    gMpuReady = false;
    return false;
  }

  Serial.print("WHO_AM_I = 0x");
  Serial.println(whoAmI, HEX);

  if (whoAmI != 0x68) {
    Serial.println("Unexpected WHO_AM_I value.");
  }

  if (!writeRegister(REG_PWR_MGMT_1, 0x00)) {
    Serial.print("Failed to wake MPU6050. txErr=");
    Serial.println(gLastTxError);
    gMpuReady = false;
    return false;
  }

  Serial.println("MPU6050 awake. Streaming raw accel/gyro/temp...");
  gMpuReady = true;
  return true;
}

bool isMpuReady() {
  return gMpuReady;
}

bool readMpuRaw(MpuRawData &data) {
  uint8_t raw[14];
  if (!readBytes(REG_ACCEL_XOUT_H, raw, sizeof(raw))) {
    return false;
  }

  data.ax = static_cast<int16_t>((raw[0] << 8) | raw[1]);
  data.ay = static_cast<int16_t>((raw[2] << 8) | raw[3]);
  data.az = static_cast<int16_t>((raw[4] << 8) | raw[5]);
  data.temp = static_cast<int16_t>((raw[6] << 8) | raw[7]);
  data.gx = static_cast<int16_t>((raw[8] << 8) | raw[9]);
  data.gy = static_cast<int16_t>((raw[10] << 8) | raw[11]);
  data.gz = static_cast<int16_t>((raw[12] << 8) | raw[13]);
  return true;
}

float computeTiltDegrees(const MpuRawData &data) {
  const float ax = static_cast<float>(data.ax);
  const float ay = static_cast<float>(data.ay);
  const float az = static_cast<float>(data.az);
  const float magnitude = sqrtf(ax * ax + ay * ay + az * az);

  if (magnitude < 1.0f) {
    return 0.0f;
  }

  float cosine = az / magnitude;
  if (cosine > 1.0f) {
    cosine = 1.0f;
  } else if (cosine < -1.0f) {
    cosine = -1.0f;
  }

  return acosf(cosine) * (180.0f / PI);
}

float computeAccelMagnitudeG(const MpuRawData &data) {
  const float ax = static_cast<float>(data.ax);
  const float ay = static_cast<float>(data.ay);
  const float az = static_cast<float>(data.az);
  const float rawMag = sqrtf(ax * ax + ay * ay + az * az);
  return rawMag / MPU_ACCEL_LSB_PER_G;
}

float rawTempToC(int16_t rawTemp) {
  return static_cast<float>(rawTemp) / 340.0f + 36.53f;
}

int getMpuLastTxError() {
  return gLastTxError;
}

int getMpuLastRxCount() {
  return gLastRxCount;
}
