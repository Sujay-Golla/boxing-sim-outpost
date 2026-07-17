/**
 * boxing_simulator.ino — 1-player dual-arm avatar mirror (RP2040 Pico).
 *
 * MAJOR CHANGE (v2): Single operator, Left + Right arms. No match logic.
 *
 * Dual-core layout (Earle Philhower RP2040 core):
 *   Core 0 — setup() / loop()     : MPU-6050 acquisition @ 100 Hz
 *   Core 1 — setup1() / loop1()   : kinematics, servos, diagnostic TFT
 *
 * Serial @ 115200: 'S' = print sensor angles
 */

#include "config.h"
#include "sensors.h"
#include "kinematics.h"
#include "display.h"

static uint32_t g_lastControlMs = 0;
static SystemStats g_stats = {};

// Control-loop FPS measurement
static uint32_t g_ctrlFpsWindowStart = 0;
static uint16_t g_ctrlFrameCount = 0;

static SensorSnapshot readSensorMailbox() {
  const uint8_t idx = g_sensorMailbox.readIndex;
  return g_sensorMailbox.buffers[idx];
}

// =============================================================================
// CORE 0 — Sensor acquisition
// =============================================================================

void setup() {
  Serial.begin(115200);
  while (!Serial && (millis() < 3000)) { }

  Serial.println(F("Avatar Mirror — Core 0 starting sensors"));

  if (!g_sensors.begin()) {
    Serial.println(F("WARNING: One or more IMUs failed init."));
  } else {
    Serial.println(F("All 4 IMUs initialized (L/R upper+forearm)."));
  }
}

void loop() {
  g_sensors.update(millis());
  yield();
}

// =============================================================================
// CORE 1 — Kinematics, servos, diagnostic display
// =============================================================================

void setup1() {
  {
    const uint32_t t0 = millis();
    while ((millis() - t0) < 200) { yield(); }
  }

  Serial.println(F("Core 1 starting kinematics + display"));
  g_kinematics.begin();
  g_display.begin();
  g_lastControlMs    = millis();
  g_ctrlFpsWindowStart = millis();
  Serial.println(F("Ready — mirror mode active. Send 'S' for serial debug."));
}

void loop1() {
  const uint32_t nowMs = millis();

  if (Serial.available()) {
    const char c = Serial.read();
    if ((c == 'S' || c == 's') && g_sensors.isReady()) {
      const SensorSnapshot s = readSensorMailbox();
      Serial.print(F("L_up P=")); Serial.print(s.left_upper.angles.pitch, 1);
      Serial.print(F(" L_fr P=")); Serial.print(s.left_forearm.angles.pitch, 1);
      Serial.print(F(" R_up P=")); Serial.print(s.right_upper.angles.pitch, 1);
      Serial.print(F(" R_fr P=")); Serial.println(s.right_forearm.angles.pitch, 1);
    }
  }

  if (!g_sensors.isReady()) {
    yield();
    return;
  }

  const SensorSnapshot snap = readSensorMailbox();

  if ((nowMs - g_lastControlMs) >= CONTROL_INTERVAL_MS) {
    g_lastControlMs = nowMs;
    g_kinematics.update(snap, nowMs);

    g_ctrlFrameCount++;
    if ((nowMs - g_ctrlFpsWindowStart) >= 1000) {
      g_stats.controlFps = g_ctrlFrameCount /
                           ((nowMs - g_ctrlFpsWindowStart) * 0.001f);
      g_ctrlFrameCount     = 0;
      g_ctrlFpsWindowStart = nowMs;
    }
  }

  g_display.update(snap,
                   g_kinematics.commands(ARM_LEFT),
                   g_kinematics.commands(ARM_RIGHT),
                   g_stats,
                   nowMs);

  yield();
}
