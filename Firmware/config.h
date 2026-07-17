/**
 * config.h — Global constants, pin map, data structures, and tuning parameters
 * for the 1-player dual-arm RP2040 avatar mirror system.
 *
 * MAJOR CHANGE (v2): Scoped down from 2-player boxing to 1-player Left/Right
 * arm avatar. Removed all match/scoring state. 6 servos (3 per arm).
 *
 * TFT_eSPI library setup:
 *   Copy TFT_eSPI_User_Setup.h into libraries/TFT_eSPI/User_Setup.h.
 */

#ifndef BOXING_CONFIG_H
#define BOXING_CONFIG_H

#include <Arduino.h>

// ---------------------------------------------------------------------------
// Arm / servo topology  [CHANGED: was NUM_PLAYERS=2, 6 servos each]
// ---------------------------------------------------------------------------
#define NUM_ARMS              2
#define SERVOS_PER_ARM        3
#define TOTAL_SERVOS          (NUM_ARMS * SERVOS_PER_ARM)   // 6 total

enum ArmId : uint8_t {
  ARM_LEFT  = 0,
  ARM_RIGHT = 1
};

/** Three DOF per arm: shoulder yaw, shoulder pitch, elbow pitch. */
enum ServoRole : uint8_t {
  SERVO_SHOULDER_YAW = 0,
  SERVO_SHOULDER_PITCH,
  SERVO_ELBOW_PITCH,
  SERVO_ROLE_COUNT
};

// ---------------------------------------------------------------------------
// I2C pin assignments  [CHANGED: Left=I2C0, Right=I2C1 — was Player1/Player2]
// ---------------------------------------------------------------------------
#define LEFT_I2C_SDA          4
#define LEFT_I2C_SCL          5
#define RIGHT_I2C_SDA         0
#define RIGHT_I2C_SCL         1

#define MPU6050_ADDR_LOW      0x68   // AD0 LOW  — upper arm
#define MPU6050_ADDR_HIGH     0x69   // AD0 HIGH — forearm

// ---------------------------------------------------------------------------
// Servo GPIO  [CHANGED: 6 servos on GP6–GP11, 3 per arm]
// ---------------------------------------------------------------------------
static const uint8_t SERVO_PINS[TOTAL_SERVOS] = {
  // Left arm
  13, 14, 15,   // Shoulder Yaw, Shoulder Pitch, Elbow Pitch
  // Right arm
  10, 11, 12  // Shoulder Yaw, Shoulder Pitch, Elbow Pitch
};

static const uint8_t SERVO_NEUTRAL[SERVO_ROLE_COUNT] = {
  90, 90, 90
};

struct ServoLimit {
  uint8_t minDeg;
  uint8_t maxDeg;
};

static const ServoLimit SERVO_LIMITS[SERVO_ROLE_COUNT] = {
  { 30, 150 },   // shoulder yaw
  { 45, 135 },   // shoulder pitch
  { 40, 140 }    // elbow pitch
};

static const float KINEMATIC_GAIN[SERVO_ROLE_COUNT] = {
  0.85f, 0.90f, 0.95f
};

// ---------------------------------------------------------------------------
// TFT SPI0 pins (unchanged)
// ---------------------------------------------------------------------------
#define TFT_PIN_SCK           18
#define TFT_PIN_MOSI          19
#define TFT_PIN_MISO          20
#define TFT_PIN_CS            21
#define TFT_PIN_DC            22
#define TFT_PIN_RST           26

// ---------------------------------------------------------------------------
// Timing (non-blocking cooperative scheduler)
// ---------------------------------------------------------------------------
#define SENSOR_INTERVAL_MS    10    // 100 Hz on Core 0
#define CONTROL_INTERVAL_MS   20    //  50 Hz on Core 1
#define DISPLAY_INTERVAL_MS   100   //  10 Hz on Core 1

// ---------------------------------------------------------------------------
// MPU-6050 / complementary filter tuning (unchanged)
// ---------------------------------------------------------------------------
#define MPU_GYRO_FS           0
#define MPU_ACCEL_FS          0
#define COMP_FILTER_ALPHA     0.96f
#define GYRO_LSB_PER_DPS_250  131.0f
#define ACCEL_LSB_PER_G_2     16384.0f

// ---------------------------------------------------------------------------
// Kinematics EMA smoothing
// ---------------------------------------------------------------------------
#define SERVO_EMA_BETA        0.25f

// ---------------------------------------------------------------------------
// Data structures  [CHANGED: explicit Left/Right IMU naming, no match state]
// ---------------------------------------------------------------------------

struct ImuAngles {
  float pitch;
  float roll;
  float yaw;
};

struct ImuState {
  ImuAngles angles;
  float     accelMagG;
  bool      healthy;
};

/** Snapshot published by Core 0 — explicit per-segment naming. */
struct SensorSnapshot {
  ImuState left_upper;
  ImuState left_forearm;
  ImuState right_upper;
  ImuState right_forearm;
  uint32_t timestampUs;
  uint32_t sequence;
};

/** Smoothed servo commands for one arm (degrees). */
struct ArmServoCommands {
  float shoulder_yaw;
  float shoulder_pitch;
  float elbow_pitch;
};

/** Runtime performance counters for diagnostic display (Core 1). */
struct SystemStats {
  float sensorFps;    // Core 0 publish rate (estimated on Core 1)
  float controlFps;   // kinematics loop rate
  float displayFps;   // TFT refresh rate
};

// ---------------------------------------------------------------------------
// Lock-free double buffer for cross-core sensor handoff (unchanged pattern)
// ---------------------------------------------------------------------------
struct SensorMailbox {
  SensorSnapshot buffers[2];
  volatile uint8_t readIndex;
};

extern SensorMailbox g_sensorMailbox;

#endif // BOXING_CONFIG_H
