/**
 * kinematics.cpp — Maps 4 IMU orientations to 6 MG996R servos (3 per arm).
 *
 * MAJOR CHANGE (v2):
 *   - Removed Player 2, hit detection, and match scoring entirely
 *   - 6-servo layout: Left GP6–8, Right GP9–11
 *   - Kinematic mapping per arm:
 *       Shoulder Yaw   ← upper_arm.yaw
 *       Shoulder Pitch ← upper_arm.pitch
 *       Elbow Pitch    ← forearm.pitch − upper_arm.pitch  (relative flexion)
 *
 * EMA smoothing:
 *   θ_servo = β·θ_target + (1−β)·θ_servo_prev
 */

#include "kinematics.h"
#include <math.h>

KinematicsEngine g_kinematics;

// ---------------------------------------------------------------------------
// KinematicsEngine::begin
// ---------------------------------------------------------------------------
bool KinematicsEngine::begin() {
  for (uint8_t i = 0; i < TOTAL_SERVOS; ++i) {
    const ServoRole role = static_cast<ServoRole>(i % SERVO_ROLE_COUNT);
    _servoAttached[i] = _servos[i].attach(SERVO_PINS[i]);
    if (_servoAttached[i]) {
      _servos[i].write(SERVO_NEUTRAL[role]);
    }
  }

  for (uint8_t a = 0; a < NUM_ARMS; ++a) {
    _commands[a].shoulder_yaw   = SERVO_NEUTRAL[SERVO_SHOULDER_YAW];
    _commands[a].shoulder_pitch = SERVO_NEUTRAL[SERVO_SHOULDER_PITCH];
    _commands[a].elbow_pitch    = SERVO_NEUTRAL[SERVO_ELBOW_PITCH];
    for (uint8_t r = 0; r < SERVO_ROLE_COUNT; ++r) {
      _emaState[a][r] = SERVO_NEUTRAL[r];
    }
  }

  return true;
}

uint8_t KinematicsEngine::servoIndex(ArmId arm, ServoRole role) const {
  return static_cast<uint8_t>(arm) * SERVO_ROLE_COUNT + role;
}

// ---------------------------------------------------------------------------
// mapImuToServo — [CHANGED] 3-DOF per arm from upper + forearm pair
// ---------------------------------------------------------------------------
float KinematicsEngine::mapImuToServo(ArmId arm, ServoRole role,
                                      const ImuState& upper,
                                      const ImuState& forearm) const {
  (void)arm;

  const float relElbowPitch = forearm.angles.pitch - upper.angles.pitch;

  float imuDeg = 0.0f;
  switch (role) {
    case SERVO_SHOULDER_YAW:
      imuDeg = upper.angles.yaw;
      break;
    case SERVO_SHOULDER_PITCH:
      imuDeg = upper.angles.pitch;
      break;
    case SERVO_ELBOW_PITCH:
      imuDeg = relElbowPitch;
      break;
    default:
      imuDeg = 0.0f;
      break;
  }

  const float neutral = SERVO_NEUTRAL[role];
  const float gain    = KINEMATIC_GAIN[role];
  return neutral + imuDeg * gain;
}

float KinematicsEngine::clampServo(ServoRole role, float deg) const {
  const ServoLimit& lim = SERVO_LIMITS[role];
  if (deg < lim.minDeg) return lim.minDeg;
  if (deg > lim.maxDeg) return lim.maxDeg;
  return deg;
}

// ---------------------------------------------------------------------------
// writeArmServos — push one arm's three smoothed angles to hardware
// ---------------------------------------------------------------------------
void KinematicsEngine::writeArmServos(ArmId arm) {
  const float values[SERVO_ROLE_COUNT] = {
    _commands[arm].shoulder_yaw,
    _commands[arm].shoulder_pitch,
    _commands[arm].elbow_pitch
  };

  for (uint8_t r = 0; r < SERVO_ROLE_COUNT; ++r) {
    const uint8_t idx = servoIndex(arm, static_cast<ServoRole>(r));
    if (!_servoAttached[idx]) continue;
    _servos[idx].write(static_cast<uint8_t>(lroundf(values[r])));
  }
}

// ---------------------------------------------------------------------------
// KinematicsEngine::update — mirror left and right arms independently
// ---------------------------------------------------------------------------
void KinematicsEngine::update(const SensorSnapshot& sensors, uint32_t nowMs) {
  (void)nowMs;

  struct ArmPair {
    ArmId     arm;
    ImuState  upper;
    ImuState  forearm;
  };

  const ArmPair pairs[] = {
    { ARM_LEFT,  sensors.left_upper,  sensors.left_forearm  },
    { ARM_RIGHT, sensors.right_upper, sensors.right_forearm },
  };

  for (const ArmPair& ap : pairs) {
    if (!ap.upper.healthy && !ap.forearm.healthy) {
      continue;
    }

    float targets[SERVO_ROLE_COUNT];
    for (uint8_t r = 0; r < SERVO_ROLE_COUNT; ++r) {
      const ServoRole role = static_cast<ServoRole>(r);
      targets[r] = clampServo(role, mapImuToServo(ap.arm, role, ap.upper, ap.forearm));
      _emaState[ap.arm][r] = SERVO_EMA_BETA * targets[r] +
                              (1.0f - SERVO_EMA_BETA) * _emaState[ap.arm][r];
    }

    _commands[ap.arm].shoulder_yaw   = _emaState[ap.arm][SERVO_SHOULDER_YAW];
    _commands[ap.arm].shoulder_pitch = _emaState[ap.arm][SERVO_SHOULDER_PITCH];
    _commands[ap.arm].elbow_pitch    = _emaState[ap.arm][SERVO_ELBOW_PITCH];

    writeArmServos(ap.arm);
  }
}
