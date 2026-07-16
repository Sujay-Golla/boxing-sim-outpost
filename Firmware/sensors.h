/**
 * sensors.h — MPU-6050 acquisition and complementary-filter fusion (Core 0).
 *
 * MAJOR CHANGE (v2): Devices named left_upper, left_forearm, right_upper,
 * right_forearm instead of per-player arrays.
 */

#ifndef BOXING_SENSORS_H
#define BOXING_SENSORS_H

#include "config.h"

class SensorManager {
public:
  bool begin();
  void update(uint32_t nowMs);
  bool isReady() const { return _ready; }

private:
  struct ImuDevice {
    TwoWire* bus;
    uint8_t  address;
    ImuState state;
    float filtPitch;
    float filtRoll;
    float filtYaw;
    uint32_t lastUpdateUs;
    bool     initialized;
  };

  // [CHANGED] Explicit per-segment devices (was _imuUpper/_imuFore[NUM_PLAYERS])
  ImuDevice _left_upper;
  ImuDevice _left_forearm;
  ImuDevice _right_upper;
  ImuDevice _right_forearm;

  uint32_t _lastTickMs;
  uint32_t _sequence;
  bool     _ready;

  bool initMpu(ImuDevice& dev);
  bool readRaw(ImuDevice& dev, int16_t& ax, int16_t& ay, int16_t& az,
               int16_t& gx, int16_t& gy, int16_t& gz);
  void applyComplementaryFilter(ImuDevice& dev,
                                int16_t ax, int16_t ay, int16_t az,
                                int16_t gx, int16_t gy, int16_t gz,
                                uint32_t nowUs);
  void publishSnapshot(uint32_t nowUs);

  static float accelToPitchDeg(float axG, float ayG, float azG);
  static float accelToRollDeg(float axG, float ayG, float azG);
};

extern SensorManager g_sensors;

#endif // BOXING_SENSORS_H
