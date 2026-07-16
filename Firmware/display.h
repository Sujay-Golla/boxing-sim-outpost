/**
 * display.h — Diagnostic / Mirror TFT UI (Core 1).
 *
 * MAJOR CHANGE (v2): Replaced 2-player fight scoreboard with real-time
 * left/right arm angle readout and system FPS counters.
 */

#ifndef BOXING_DISPLAY_H
#define BOXING_DISPLAY_H

#include "config.h"

class DisplayManager {
public:
  bool begin();
  void update(const SensorSnapshot& sensors,
              const ArmServoCommands leftCmd,
              const ArmServoCommands rightCmd,
              SystemStats& stats,
              uint32_t nowMs);

private:
  uint32_t _lastDrawMs;

  // FPS measurement windows
  uint32_t _fpsWindowStartMs;
  uint32_t _lastSensorSeq;
  uint16_t _sensorFrameCount;
  uint16_t _displayFrameCount;

  // Cached angle values for dirty-region updates
  // IMU indices: 0=left_upper 1=left_forearm 2=right_upper 3=right_forearm
  float _lastImuAngles[4][3];
  float _lastServoDeg[6];   // 3 left + 3 right

  void drawStaticChrome();
  void drawFpsBar(const SystemStats& stats);
  void drawArmPanel(int x,
                    const ImuState& upper, const ImuState& forearm,
                    const ArmServoCommands& cmd,
                    int upperImuIdx, int foreImuIdx, int servoBaseIdx);
  void drawImuHealth(const SensorSnapshot& sensors);
  void updateFpsCounters(const SensorSnapshot& sensors, SystemStats& stats, uint32_t nowMs);
  bool angleChanged(float a, float b) const;
};

extern DisplayManager g_display;

#endif // BOXING_DISPLAY_H
