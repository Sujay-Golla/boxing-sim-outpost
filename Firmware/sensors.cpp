/**
 * sensors.cpp — MPU-6050 driver, complementary filter, Core 0 mailbox publisher.
 *
 * MAJOR CHANGE (v2):
 *   - Left arm IMUs on I2C0 (GP4/GP5): left_upper @ 0x68, left_forearm @ 0x69
 *   - Right arm IMUs on I2C1 (GP2/GP3): right_upper @ 0x68, right_forearm @ 0x69
 *   - Snapshot fields renamed to left_upper, left_forearm, right_upper, right_forearm
 *   - Removed all Player 1 / Player 2 loops and references
 *
 * Complementary filter:
 *   θ_filtered = α · (θ_filtered + ω · dt) + (1 − α) · θ_accel
 */

#include "sensors.h"
#include <Wire.h>
#include <math.h>

static const uint8_t MPU_REG_PWR_MGMT_1   = 0x6B;
static const uint8_t MPU_REG_SMPLRT_DIV   = 0x19;
static const uint8_t MPU_REG_CONFIG       = 0x1A;
static const uint8_t MPU_REG_GYRO_CONFIG  = 0x1B;
static const uint8_t MPU_REG_ACCEL_CONFIG = 0x1C;
static const uint8_t MPU_REG_ACCEL_XOUT_H = 0x3B;
static const uint8_t MPU_REG_WHO_AM_I     = 0x75;
static const uint8_t MPU_WHO_AM_I_VALUE   = 0x68;

SensorMailbox g_sensorMailbox = {};
SensorManager g_sensors;

// ---------------------------------------------------------------------------
// SensorManager::begin
// ---------------------------------------------------------------------------
bool SensorManager::begin() {
  // [CHANGED] Left arm — I2C0
  Wire.setSDA(LEFT_I2C_SDA);
  Wire.setSCL(LEFT_I2C_SCL);
  Wire.begin();
  Wire.setClock(400000);

  // [CHANGED] Right arm — I2C1
  Wire1.setSDA(RIGHT_I2C_SDA);
  Wire1.setSCL(RIGHT_I2C_SCL);
  Wire1.begin();
  Wire1.setClock(400000);

  _left_upper   = { &Wire,  MPU6050_ADDR_LOW,  {}, 0.0f, 0.0f, 0.0f, micros(), false };
  _left_forearm = { &Wire,  MPU6050_ADDR_HIGH, {}, 0.0f, 0.0f, 0.0f, micros(), false };
  _right_upper   = { &Wire1, MPU6050_ADDR_LOW,  {}, 0.0f, 0.0f, 0.0f, micros(), false };
  _right_forearm = { &Wire1, MPU6050_ADDR_HIGH, {}, 0.0f, 0.0f, 0.0f, micros(), false };

  bool ok = true;
  ok &= initMpu(_left_upper);
  ok &= initMpu(_left_forearm);
  ok &= initMpu(_right_upper);
  ok &= initMpu(_right_forearm);

  _lastTickMs = millis();
  _sequence   = 0;
  _ready      = false;
  g_sensorMailbox.readIndex = 0;

  return ok;
}

// ---------------------------------------------------------------------------
// SensorManager::initMpu
// ---------------------------------------------------------------------------
bool SensorManager::initMpu(ImuDevice& dev) {
  TwoWire& bus = *dev.bus;
  const uint8_t addr = dev.address;

  bus.beginTransmission(addr);
  bus.write(MPU_REG_PWR_MGMT_1);
  bus.write(0x00);
  if (bus.endTransmission() != 0) {
    dev.initialized = false;
    dev.state.healthy = false;
    return false;
  }

  bus.beginTransmission(addr);
  bus.write(MPU_REG_SMPLRT_DIV);
  bus.write(0x09);
  bus.endTransmission();

  bus.beginTransmission(addr);
  bus.write(MPU_REG_CONFIG);
  bus.write(0x03);
  bus.endTransmission();

  bus.beginTransmission(addr);
  bus.write(MPU_REG_GYRO_CONFIG);
  bus.write(MPU_GYRO_FS << 3);
  bus.endTransmission();

  bus.beginTransmission(addr);
  bus.write(MPU_REG_ACCEL_CONFIG);
  bus.write(MPU_ACCEL_FS << 3);
  bus.endTransmission();

  {
    const uint32_t t0 = millis();
    while ((millis() - t0) < 50) { /* one-time oscillator settle */ }
  }

  bus.beginTransmission(addr);
  bus.write(MPU_REG_WHO_AM_I);
  bus.endTransmission(false);
  bus.requestFrom(addr, (uint8_t)1);
  if (!bus.available() || bus.read() != MPU_WHO_AM_I_VALUE) {
    dev.initialized = false;
    dev.state.healthy = false;
    return false;
  }

  dev.initialized = true;
  dev.state.healthy = true;
  dev.lastUpdateUs = micros();
  return true;
}

// ---------------------------------------------------------------------------
// SensorManager::readRaw
// ---------------------------------------------------------------------------
bool SensorManager::readRaw(ImuDevice& dev,
                            int16_t& ax, int16_t& ay, int16_t& az,
                            int16_t& gx, int16_t& gy, int16_t& gz) {
  if (!dev.initialized) {
    dev.state.healthy = false;
    return false;
  }

  TwoWire& bus = *dev.bus;
  const uint8_t addr = dev.address;

  bus.beginTransmission(addr);
  bus.write(MPU_REG_ACCEL_XOUT_H);
  if (bus.endTransmission(false) != 0) {
    dev.state.healthy = false;
    return false;
  }

  if (bus.requestFrom(addr, (uint8_t)14) != 14) {
    dev.state.healthy = false;
    return false;
  }

  ax = (int16_t)((bus.read() << 8) | bus.read());
  ay = (int16_t)((bus.read() << 8) | bus.read());
  az = (int16_t)((bus.read() << 8) | bus.read());
  bus.read(); bus.read();
  gx = (int16_t)((bus.read() << 8) | bus.read());
  gy = (int16_t)((bus.read() << 8) | bus.read());
  gz = (int16_t)((bus.read() << 8) | bus.read());

  dev.state.healthy = true;
  return true;
}

float SensorManager::accelToPitchDeg(float axG, float ayG, float azG) {
  return atan2f(-axG, sqrtf(ayG * ayG + azG * azG)) * 57.2957795f;
}

float SensorManager::accelToRollDeg(float axG, float ayG, float azG) {
  return atan2f(ayG, azG) * 57.2957795f;
}

// ---------------------------------------------------------------------------
// Complementary filter
// θ_filtered = α·(θ_filtered + ω·dt) + (1−α)·θ_accel
// ---------------------------------------------------------------------------
void SensorManager::applyComplementaryFilter(ImuDevice& dev,
                                             int16_t ax, int16_t ay, int16_t az,
                                             int16_t gx, int16_t gy, int16_t gz,
                                             uint32_t nowUs) {
  float dt = (nowUs - dev.lastUpdateUs) * 1.0e-6f;
  if (dt <= 0.0f || dt > 0.25f) {
    dt = 0.01f;
  }
  dev.lastUpdateUs = nowUs;

  const float axG = ax / ACCEL_LSB_PER_G_2;
  const float ayG = ay / ACCEL_LSB_PER_G_2;
  const float azG = az / ACCEL_LSB_PER_G_2;

  const float gxDps = gx / GYRO_LSB_PER_DPS_250;
  const float gyDps = gy / GYRO_LSB_PER_DPS_250;
  const float gzDps = gz / GYRO_LSB_PER_DPS_250;

  const float pitchAccel = accelToPitchDeg(axG, ayG, azG);
  const float rollAccel  = accelToRollDeg(axG, ayG, azG);

  const float alpha = COMP_FILTER_ALPHA;
  const float oneMinusAlpha = 1.0f - alpha;

  dev.filtPitch = alpha * (dev.filtPitch + gxDps * dt) + oneMinusAlpha * pitchAccel;
  dev.filtRoll  = alpha * (dev.filtRoll  + gyDps * dt) + oneMinusAlpha * rollAccel;
  dev.filtYaw  += gzDps * dt;

  dev.state.angles.pitch = dev.filtPitch;
  dev.state.angles.roll  = dev.filtRoll;
  dev.state.angles.yaw   = dev.filtYaw;
  dev.state.accelMagG    = sqrtf(axG * axG + ayG * ayG + azG * azG);
}

// ---------------------------------------------------------------------------
// Publish snapshot — [CHANGED] explicit left/right field names
// ---------------------------------------------------------------------------
void SensorManager::publishSnapshot(uint32_t nowUs) {
  const uint8_t writeIndex = 1u - g_sensorMailbox.readIndex;

  SensorSnapshot& snap = g_sensorMailbox.buffers[writeIndex];
  snap.timestampUs   = nowUs;
  snap.sequence      = ++_sequence;
  snap.left_upper    = _left_upper.state;
  snap.left_forearm  = _left_forearm.state;
  snap.right_upper   = _right_upper.state;
  snap.right_forearm = _right_forearm.state;

  g_sensorMailbox.readIndex = writeIndex;
  _ready = true;
}

// ---------------------------------------------------------------------------
// SensorManager::update — reads all four named IMU segments
// ---------------------------------------------------------------------------
void SensorManager::update(uint32_t nowMs) {
  if ((nowMs - _lastTickMs) < SENSOR_INTERVAL_MS) {
    return;
  }
  _lastTickMs = nowMs;

  const uint32_t nowUs = micros();
  int16_t ax, ay, az, gx, gy, gz;

  if (readRaw(_left_upper, ax, ay, az, gx, gy, gz)) {
    applyComplementaryFilter(_left_upper, ax, ay, az, gx, gy, gz, nowUs);
  }
  if (readRaw(_left_forearm, ax, ay, az, gx, gy, gz)) {
    applyComplementaryFilter(_left_forearm, ax, ay, az, gx, gy, gz, nowUs);
  }
  if (readRaw(_right_upper, ax, ay, az, gx, gy, gz)) {
    applyComplementaryFilter(_right_upper, ax, ay, az, gx, gy, gz, nowUs);
  }
  if (readRaw(_right_forearm, ax, ay, az, gx, gy, gz)) {
    applyComplementaryFilter(_right_forearm, ax, ay, az, gx, gy, gz, nowUs);
  }

  publishSnapshot(nowUs);
}
