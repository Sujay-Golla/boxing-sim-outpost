/**
 * kinematics.h — IMU-to-servo mapping and EMA smoothing (Core 1).
 *
 * MAJOR CHANGE (v2): 6 servos (3 per arm). Removed hit detection and match state.
 */

#ifndef BOXING_KINEMATICS_H
#define BOXING_KINEMATICS_H

#include "config.h"
#include <Servo.h>

class KinematicsEngine {
public:
  bool begin();
  void update(const SensorSnapshot& sensors, uint32_t nowMs);

  const ArmServoCommands& commands(ArmId arm) const { return _commands[arm]; }

private:
  Servo             _servos[TOTAL_SERVOS];
  bool              _servoAttached[TOTAL_SERVOS];
  ArmServoCommands  _commands[NUM_ARMS];
  float             _emaState[NUM_ARMS][SERVO_ROLE_COUNT];

  uint8_t servoIndex(ArmId arm, ServoRole role) const;
  float mapImuToServo(ArmId arm, ServoRole role,
                      const ImuState& upper, const ImuState& forearm) const;
  float clampServo(ServoRole role, float deg) const;
  void  writeArmServos(ArmId arm);
};

extern KinematicsEngine g_kinematics;

#endif // BOXING_KINEMATICS_H
