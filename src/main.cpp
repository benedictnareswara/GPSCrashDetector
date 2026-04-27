#include <Arduino.h>
#include <Wire.h>

#include "app_config.hpp"
#include "bridge_protocol.hpp"
#include "button_input.hpp"
#include "gps_parser.hpp"
#include "mpu6050.hpp"
#include "safety_fsm.hpp"
#include "types.hpp"

namespace {
unsigned long gLastMpuReadMs = 0;
float gLastTiltDeg = 0.0f;
float gLastAccelMagnitudeG = 0.0f;
}

void setBuzzer(bool enabled) {
  if (enabled) {
    tone(BUZZER_PIN, BUZZER_TONE_HZ);
  } else {
    noTone(BUZZER_PIN);
    digitalWrite(BUZZER_PIN, LOW);
  }
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  setBuzzer(false);

  Serial.begin(115200);
  Serial1.begin(GPS_BAUD);
  Serial2.begin(BRIDGE_BAUD);
  Wire.begin();
  Wire.setClock(100000);

  initButtonInput();
  initSafetyStateMachine();

  delay(300);
  Serial.println("VestMicro: Arduino Mega 2560 R3 + MPU6050 bring-up");
  Serial.println("NEO-6M on Serial1 (RX1=19, TX1=18) at 9600 baud");
  Serial.println("ESP32 bridge on Serial2 (RX2=17, TX2=16) at 115200 baud");
  Serial.println("Button pin=7 (INPUT_PULLUP), buzzer pin=6");
  Serial.println("Crash trigger: tilt>=35.0 deg OR accel>=1.35g");
  Serial.println("Crash cancel window: 5 seconds");

  initMpu6050();
}

void loop() {
  processGpsStream();
  const bool buttonPressed = detectButtonPressedEdge();

  // Always process button/state transitions immediately (not only on MPU sample intervals).
  updateSafetyStateMachine(buttonPressed,
                           false,
                           gLastTiltDeg,
                           gLastAccelMagnitudeG,
                           setBuzzer,
                           publishBridgeEvent);

  if (!isMpuReady()) {
    static unsigned long lastMsgMs = 0;
    digitalWrite(LED_PIN, (millis() / 250) % 2);
    if (millis() - lastMsgMs >= 1000) {
      lastMsgMs = millis();
      Serial.println("MPU not ready. Fix wiring/address and press RESET.");
    }
    return;
  }

  if (millis() - gLastMpuReadMs < MPU_SAMPLE_INTERVAL_MS) {
    return;
  }
  gLastMpuReadMs = millis();

  MpuRawData data;
  if (readMpuRaw(data)) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    float tiltDeg = computeTiltDegrees(data);
    float accelMagnitudeG = computeAccelMagnitudeG(data);
    gLastTiltDeg = tiltDeg;
    gLastAccelMagnitudeG = accelMagnitudeG;
    const bool crashTrigger = (tiltDeg >= TILT_TRIGGER_DEG) || (accelMagnitudeG >= ACCEL_TRIGGER_G);
    updateSafetyStateMachine(buttonPressed,
                             crashTrigger,
                             gLastTiltDeg,
                             gLastAccelMagnitudeG,
                             setBuzzer,
                             publishBridgeEvent);

    Serial.print("ACC raw: ");
    Serial.print(data.ax);
    Serial.print(',');
    Serial.print(data.ay);
    Serial.print(',');
    Serial.print(data.az);

    Serial.print(" | GYRO raw: ");
    Serial.print(data.gx);
    Serial.print(',');
    Serial.print(data.gy);
    Serial.print(',');
    Serial.print(data.gz);

    Serial.print(" | TEMP C: ");
    Serial.print(rawTempToC(data.temp), 2);

    Serial.print(" | TILT deg: ");
    Serial.print(tiltDeg, 1);

    Serial.print(" | ACCEL g: ");
    Serial.print(accelMagnitudeG, 3);

    Serial.print(" | GPS: ");
    const bool freshFix = hasFreshFix();
    const GpsFix &fix = getLatestFix();
    if (freshFix) {
      Serial.print(fix.latitude, 6);
      Serial.print(',');
      Serial.println(fix.longitude, 6);
    } else {
      Serial.println("no valid fix yet");
    }
  } else {
    Serial.print("Read failed (I2C). txErr=");
    Serial.print(getMpuLastTxError());
    Serial.print(" rxCount=");
    Serial.println(getMpuLastRxCount());
  }
}
