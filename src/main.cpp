// ---------------------------------------------
// FILE: main.cpp
// PURPOSE: Entry point for the GPS Crash Detector running on Arduino Mega.
// HOW IT WORKS: Reads accelerometer data every 100ms and watches for a
//               G-force spike (>2.5G) followed by a sudden stop (<1.2G)
//               within 500ms. When this pattern is detected, it triggers
//               the safety state machine which manages buzzer alerts,
//               a 5-second cancel window, and event publishing to the ESP32.
// ---------------------------------------------
#include <Arduino.h>

#include "app_config.hpp"
#include "bridge_protocol.hpp"
#include "button_input.hpp"
#include "gps_parser.hpp"
#include "i2c_bitbang.hpp"
#include "mpu6050.hpp"
#include "safety_fsm.hpp"
#include "types.hpp"

namespace {

// Timing state for MPU polling
unsigned long gLastMpuReadMs = 0;
float gLastAccelMagnitudeG = 0.0f;

// G-force spike → stop crash detection state
bool gSpikeActive = false;
unsigned long gSpikeDetectedMs = 0;
float gSpikeAccelG = 0.0f;

void setBuzzer(bool enabled) {
  if (enabled) {
    tone(BUZZER_PIN, BUZZER_TONE_HZ);
  } else {
    noTone(BUZZER_PIN);
    digitalWrite(BUZZER_PIN, LOW);
  }
}

// Evaluates the two-phase crash pattern:
// Phase 1: Detect a G-force spike above GFORCE_SPIKE_THRESHOLD_G
// Phase 2: Within GFORCE_SPIKE_WINDOW_MS, detect a drop below GFORCE_STOP_THRESHOLD_G
// Returns true when the full spike→stop pattern completes (i.e., crash detected).
bool evaluateCrashPattern(float accelMagnitudeG) {
  if (!gSpikeActive) {
    if (accelMagnitudeG < GFORCE_SPIKE_THRESHOLD_G) {
      return false;
    }
    gSpikeActive = true;
    gSpikeDetectedMs = millis();
    gSpikeAccelG = accelMagnitudeG;
    Serial.print("SPIKE detected: ");
    Serial.print(accelMagnitudeG, 3);
    Serial.println("g");
    return false;
  }

  // Phase 2: spike is active — check for sudden stop within time window
  const unsigned long elapsed = millis() - gSpikeDetectedMs;

  if (elapsed > GFORCE_SPIKE_WINDOW_MS) {
    gSpikeActive = false;
    Serial.print("SPIKE expired: no stop within ");
    Serial.print(GFORCE_SPIKE_WINDOW_MS);
    Serial.println("ms, resetting");
    return false;
  }

  if (accelMagnitudeG > GFORCE_STOP_THRESHOLD_G) {
    return false;
  }

  // Spike followed by sudden stop within window → CRASH
  gSpikeActive = false;
  Serial.print("CRASH pattern: spike=");
  Serial.print(gSpikeAccelG, 3);
  Serial.print("g -> stop=");
  Serial.print(accelMagnitudeG, 3);
  Serial.print("g in ");
  Serial.print(elapsed);
  Serial.println("ms");
  return true;
}

void logMpuData(const MpuRawData &data, float accelMagnitudeG, bool crashTrigger) {
  Serial.print("ACC:");
  Serial.print(data.ax);
  Serial.print(',');
  Serial.print(data.ay);
  Serial.print(',');
  Serial.print(data.az);

  Serial.print(" G:");
  Serial.print(data.gx);
  Serial.print(',');
  Serial.print(data.gy);
  Serial.print(',');
  Serial.print(data.gz);

  Serial.print(" T:");
  Serial.print(rawTempToC(data.temp), 1);

  Serial.print(" |g|:");
  Serial.print(accelMagnitudeG, 3);

  if (gSpikeActive) {
    Serial.print(" SPK:");
    Serial.print(gSpikeAccelG, 3);
    Serial.print("/");
    Serial.print(millis() - gSpikeDetectedMs);
    Serial.print("ms");
  }

  if (crashTrigger) {
    Serial.print(" CRASH!");
  }

  Serial.print(" GPS:");
  if (hasFreshFix()) {
    const GpsFix &fix = getLatestFix();
    Serial.print(fix.latitude, 6);
    Serial.print(',');
    Serial.println(fix.longitude, 6);
  } else {
    Serial.println("--");
  }
}

void handleMpuNotReady() {
  static unsigned long lastMsgMs = 0;
  digitalWrite(LED_PIN, (millis() / 250) % 2);
  if (millis() - lastMsgMs < 1000) {
    return;
  }
  lastMsgMs = millis();
  Serial.println("MPU not ready. Fix wiring/address and press RESET.");
}

}  // namespace

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  setBuzzer(false);

  Serial.begin(115200);
  Serial1.begin(GPS_BAUD);
  Serial2.begin(BRIDGE_BAUD);
  i2cInit();

  initButtonInput();
  initSafetyStateMachine();

  delay(300);
  Serial.println("VestMicro: Arduino Mega 2560 R3 + MPU6050 bring-up");
  Serial.print("Crash: spike>=");
  Serial.print(GFORCE_SPIKE_THRESHOLD_G, 1);
  Serial.print("g stop<=");
  Serial.print(GFORCE_STOP_THRESHOLD_G, 1);
  Serial.print("g in ");
  Serial.print(GFORCE_SPIKE_WINDOW_MS);
  Serial.println("ms | Cancel: 5s");

  initMpu6050();
}

void loop() {
  processGpsStream();
  const bool buttonPressed = detectButtonPressedEdge();

  // Process button/state transitions every iteration, not just on MPU sample ticks
  updateSafetyStateMachine(buttonPressed,
                           false,
                           gLastAccelMagnitudeG,
                           setBuzzer,
                           publishBridgeEvent);

  if (!isMpuReady()) {
    handleMpuNotReady();
    return;
  }

  if (millis() - gLastMpuReadMs < MPU_SAMPLE_INTERVAL_MS) {
    return;
  }
  gLastMpuReadMs = millis();

  MpuRawData data;
  if (!readMpuRaw(data)) {
    Serial.println("MPU read failed (I2C).");
    return;
  }

  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  const float accelMagnitudeG = computeAccelMagnitudeG(data);
  gLastAccelMagnitudeG = accelMagnitudeG;

  const bool crashTrigger = evaluateCrashPattern(accelMagnitudeG);

  updateSafetyStateMachine(buttonPressed,
                           crashTrigger,
                           gLastAccelMagnitudeG,
                           setBuzzer,
                           publishBridgeEvent);

  logMpuData(data, accelMagnitudeG, crashTrigger);
}
