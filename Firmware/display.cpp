/**
 * display.cpp — 1-player Diagnostic / Mirror mode UI via TFT_eSPI.
 *
 * MAJOR CHANGE (v2):
 *   - Removed match state machine, scores, rounds, countdown
 *   - Shows real-time IMU angles for left_upper, left_forearm,
 *     right_upper, right_forearm
 *   - Shows mapped servo angles for both arms
 *   - Displays sensor / control / display FPS
 *
 * Low-flicker: static chrome once; numeric fields repaint only on change.
 */

#include "display.h"
#include <TFT_eSPI.h>
#include <math.h>

static TFT_eSPI tft = TFT_eSPI();

DisplayManager g_display;

static const int SCREEN_W = 320;
static const int SCREEN_H = 240;
static const int FPS_WINDOW_MS = 1000;

// ---------------------------------------------------------------------------
// DisplayManager::begin
// ---------------------------------------------------------------------------
bool DisplayManager::begin() {
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setTextDatum(TL_DATUM);

  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 3; ++j) _lastImuAngles[i][j] = -999.0f;
  }
  for (int i = 0; i < 6; ++i) _lastServoDeg[i]  = -999.0f;

  _lastDrawMs       = millis();
  _fpsWindowStartMs = millis();
  _lastSensorSeq    = 0;
  _sensorFrameCount = 0;
  _displayFrameCount = 0;

  drawStaticChrome();
  return true;
}

bool DisplayManager::angleChanged(float a, float b) const {
  return fabsf(a - b) >= 0.5f;
}

// ---------------------------------------------------------------------------
// FPS counters — estimated on Core 1 from sensor sequence + local frame counts
// ---------------------------------------------------------------------------
void DisplayManager::updateFpsCounters(const SensorSnapshot& sensors,
                                       SystemStats& stats,
                                       uint32_t nowMs) {
  if (sensors.sequence != _lastSensorSeq) {
    _sensorFrameCount++;
    _lastSensorSeq = sensors.sequence;
  }
  _displayFrameCount++;

  const uint32_t elapsed = nowMs - _fpsWindowStartMs;
  if (elapsed >= FPS_WINDOW_MS) {
    const float sec = elapsed * 0.001f;
    stats.sensorFps  = _sensorFrameCount / sec;
    stats.displayFps = _displayFrameCount / sec;
    // controlFps is set by the main loop before calling update()
    _sensorFrameCount  = 0;
    _displayFrameCount = 0;
    _fpsWindowStartMs  = nowMs;
  }
}

// ---------------------------------------------------------------------------
// Static chrome — drawn once at boot
// ---------------------------------------------------------------------------
void DisplayManager::drawStaticChrome() {
  tft.fillScreen(TFT_BLACK);

  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextDatum(TC_DATUM);
  tft.drawString("AVATAR MIRROR", SCREEN_W / 2, 4);

  tft.setTextSize(1);
  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.drawString("Diagnostic Mode", SCREEN_W / 2, 24);

  tft.drawFastHLine(4, 34, SCREEN_W - 8, TFT_DARKGREY);

  // Column headers
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.drawString("LEFT ARM", 8, 42);
  tft.setTextColor(TFT_ORANGE, TFT_BLACK);
  tft.drawString("RIGHT ARM", 168, 42);

  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.drawString("Seg       P   R   Y", 8, 56);
  tft.drawString("Seg       P   R   Y", 168, 56);
  tft.drawString("Srv  Yaw Pit Elb", 8, 108);
  tft.drawString("Srv  Yaw Pit Elb", 168, 108);

  tft.drawFastHLine(4, 148, SCREEN_W - 8, TFT_DARKGREY);

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("FPS  Sensor Control Display", 8, 154);
}

// ---------------------------------------------------------------------------
// FPS bar — repaints every display tick
// ---------------------------------------------------------------------------
void DisplayManager::drawFpsBar(const SystemStats& stats) {
  char buf[48];
  tft.fillRect(8, 166, SCREEN_W - 16, 14, TFT_BLACK);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setTextSize(1);
  snprintf(buf, sizeof(buf), "     %3.0f     %3.0f      %3.0f",
           stats.sensorFps, stats.controlFps, stats.displayFps);
  tft.drawString(buf, 8, 166);
}

// ---------------------------------------------------------------------------
// drawArmPanel — IMU angles + servo output for one arm
// ---------------------------------------------------------------------------
void DisplayManager::drawArmPanel(int x,
                                  const ImuState& upper,
                                  const ImuState& forearm,
                                  const ArmServoCommands& cmd,
                                  int upperImuIdx,
                                  int foreImuIdx,
                                  int servoBaseIdx) {
  char buf[40];
  tft.setTextSize(1);
  tft.setTextDatum(TL_DATUM);

  const ImuState* segs[2] = { &upper, &forearm };
  const int imuIdx[2] = { upperImuIdx, foreImuIdx };
  const char* segLabel[] = { "Uppr", "Fore" };

  for (int row = 0; row < 2; ++row) {
    const ImuAngles& ang = segs[row]->angles;
    const float vals[3] = { ang.pitch, ang.roll, ang.yaw };

    bool changed = false;
    for (int ax = 0; ax < 3; ++ax) {
      if (angleChanged(vals[ax], _lastImuAngles[imuIdx[row]][ax])) {
        changed = true;
      }
    }
    if (!changed) continue;

    tft.fillRect(x, 68 + row * 12, 148, 12, TFT_BLACK);
    tft.setTextColor(segs[row]->healthy ? TFT_WHITE : TFT_RED, TFT_BLACK);
    snprintf(buf, sizeof(buf), "%s %4.0f %4.0f %4.0f",
             segLabel[row], vals[0], vals[1], vals[2]);
    tft.drawString(buf, x, 68 + row * 12);

    for (int ax = 0; ax < 3; ++ax) {
      _lastImuAngles[imuIdx[row]][ax] = vals[ax];
    }
  }

  const float srvVals[3] = { cmd.shoulder_yaw, cmd.shoulder_pitch, cmd.elbow_pitch };
  bool srvChanged = false;
  for (int i = 0; i < 3; ++i) {
    if (angleChanged(srvVals[i], _lastServoDeg[servoBaseIdx + i])) {
      srvChanged = true;
    }
  }
  if (srvChanged) {
    tft.fillRect(x, 120, 148, 12, TFT_BLACK);
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    snprintf(buf, sizeof(buf), "     %3.0f %3.0f %3.0f",
             srvVals[0], srvVals[1], srvVals[2]);
    tft.drawString(buf, x, 120);
    for (int i = 0; i < 3; ++i) {
      _lastServoDeg[servoBaseIdx + i] = srvVals[i];
    }
  }
}

// ---------------------------------------------------------------------------
// IMU health indicators (bottom of screen)
// ---------------------------------------------------------------------------
void DisplayManager::drawImuHealth(const SensorSnapshot& sensors) {
  auto dot = [&](int x, bool ok) {
    tft.fillCircle(x, SCREEN_H - 10, 4, ok ? TFT_GREEN : TFT_RED);
  };

  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.setTextSize(1);
  tft.drawString("LU LF    RU RF", 8, SCREEN_H - 22);

  dot(24,  sensors.left_upper.healthy);
  dot(40,  sensors.left_forearm.healthy);
  dot(200, sensors.right_upper.healthy);
  dot(216, sensors.right_forearm.healthy);
}

// ---------------------------------------------------------------------------
// DisplayManager::update
// ---------------------------------------------------------------------------
void DisplayManager::update(const SensorSnapshot& sensors,
                            const ArmServoCommands leftCmd,
                            const ArmServoCommands rightCmd,
                            SystemStats& stats,
                            uint32_t nowMs) {
  updateFpsCounters(sensors, stats, nowMs);

  if ((nowMs - _lastDrawMs) < DISPLAY_INTERVAL_MS) {
    return;
  }
  _lastDrawMs = nowMs;

  drawArmPanel(8,
               sensors.left_upper, sensors.left_forearm, leftCmd,
               0, 1, 0);

  drawArmPanel(168,
               sensors.right_upper, sensors.right_forearm, rightCmd,
               2, 3, 3);

  drawFpsBar(stats);
  drawImuHealth(sensors);
}
