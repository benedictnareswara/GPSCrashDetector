#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

const uint8_t LED_PIN = LED_BUILTIN;
const uint8_t BUZZER_PIN = 6;
const uint8_t BUTTON_PIN = 7;
const uint32_t MPU_SAMPLE_INTERVAL_MS = 1000;
const uint32_t GPS_BAUD = 9600;
const uint32_t BRIDGE_BAUD = 115200;
const uint32_t GPS_FIX_STALE_MS = 10000;
const uint32_t CRASH_CANCEL_WINDOW_MS = 10000;
const uint32_t BUTTON_DEBOUNCE_MS = 40;
const uint32_t EVENT_COOLDOWN_MS = 3000;
const float MPU_ACCEL_LSB_PER_G = 16384.0f;
const float TILT_TRIGGER_DEG = 35.0f;
const float ACCEL_TRIGGER_G = 1.35f;

uint8_t gMpuAddress = 0x68;
bool gMpuReady = false;
int gLastTxError = 0;
int gLastRxCount = 0;
unsigned long gLastMpuReadMs = 0;
unsigned long gLastCrashExitMs = 0;
uint32_t gBridgeSequence = 0;
float gLastTiltDeg = 0.0f;
float gLastAccelMagnitudeG = 0.0f;

bool gButtonRawState = false;
bool gButtonStableState = false;
unsigned long gButtonLastEdgeMs = 0;

enum AlertState {
  ALERT_STANDBY,
  ALERT_CRASH_PENDING,
  ALERT_CRASH_CONFIRMED
};

AlertState gAlertState = ALERT_STANDBY;
bool gManualBuzzerLatched = false;
unsigned long gCrashWindowStartMs = 0;

char gGpsLine[100];
size_t gGpsLinePos = 0;

struct GpsFix {
  bool valid;
  float latitude;
  float longitude;
  unsigned long updatedMs;
};

GpsFix gLatestFix = {false, 0.0f, 0.0f, 0};

const uint8_t REG_WHO_AM_I = 0x75;
const uint8_t REG_PWR_MGMT_1 = 0x6B;
const uint8_t REG_ACCEL_XOUT_H = 0x3B;

struct MpuRawData {
  int16_t ax;
  int16_t ay;
  int16_t az;
  int16_t temp;
  int16_t gx;
  int16_t gy;
  int16_t gz;
};

bool parseNmeaCoordinate(const char *coord, char hemi, float &outDegrees) {
  if (coord == nullptr || coord[0] == '\0') {
    return false;
  }

  float raw = static_cast<float>(atof(coord));
  if (raw <= 0.0f) {
    return false;
  }

  int wholeDegrees = static_cast<int>(raw / 100.0f);
  float minutes = raw - static_cast<float>(wholeDegrees * 100);
  float decimal = static_cast<float>(wholeDegrees) + (minutes / 60.0f);

  if (hemi == 'S' || hemi == 'W') {
    decimal = -decimal;
  }

  outDegrees = decimal;
  return true;
}

bool parseRmcLine(char *line, GpsFix &fix) {
  if (strncmp(line, "$GPRMC", 6) != 0 && strncmp(line, "$GNRMC", 6) != 0) {
    return false;
  }

  char *tokens[16] = {nullptr};
  uint8_t count = 0;

  char *cursor = line;
  while (count < 16) {
    tokens[count++] = cursor;
    char *comma = strchr(cursor, ',');
    if (comma == nullptr) {
      break;
    }
    *comma = '\0';
    cursor = comma + 1;
  }

  if (count < 7) {
    return false;
  }

  const char *status = tokens[2];
  if (status == nullptr || status[0] != 'A') {
    return false;
  }

  const char *latText = tokens[3];
  const char *latHem = tokens[4];
  const char *lonText = tokens[5];
  const char *lonHem = tokens[6];
  if (latHem == nullptr || lonHem == nullptr || latHem[0] == '\0' || lonHem[0] == '\0') {
    return false;
  }

  float lat = 0.0f;
  float lon = 0.0f;
  if (!parseNmeaCoordinate(latText, latHem[0], lat)) {
    return false;
  }
  if (!parseNmeaCoordinate(lonText, lonHem[0], lon)) {
    return false;
  }

  fix.valid = true;
  fix.latitude = lat;
  fix.longitude = lon;
  fix.updatedMs = millis();
  return true;
}

void processGpsStream() {
  while (Serial1.available() > 0) {
    char c = static_cast<char>(Serial1.read());

    if (c == '\r') {
      continue;
    }

    if (c == '$') {
      gGpsLinePos = 0;
      gGpsLine[gGpsLinePos++] = c;
      continue;
    }

    if (c == '\n') {
      gGpsLine[gGpsLinePos] = '\0';
      if (gGpsLinePos > 6) {
        GpsFix parsed = {false, 0.0f, 0.0f, 0};
        if (parseRmcLine(gGpsLine, parsed)) {
          gLatestFix = parsed;
        }
      }
      gGpsLinePos = 0;
      continue;
    }

    if (gGpsLinePos < sizeof(gGpsLine) - 1) {
      gGpsLine[gGpsLinePos++] = c;
    } else {
      gGpsLinePos = 0;
    }
  }
}

float computeTiltDegrees(const MpuRawData &data) {
  const float ax = static_cast<float>(data.ax);
  const float ay = static_cast<float>(data.ay);
  const float az = static_cast<float>(data.az);
  const float magnitude = sqrtf(ax * ax + ay * ay + az * az);

  if (magnitude < 1.0f) {
    return 0.0f;
  }

  float cosine = az / magnitude;
  if (cosine > 1.0f) {
    cosine = 1.0f;
  } else if (cosine < -1.0f) {
    cosine = -1.0f;
  }

  return acosf(cosine) * (180.0f / PI);
}

bool writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(gMpuAddress);
  Wire.write(reg);
  Wire.write(value);
  gLastTxError = Wire.endTransmission();
  return gLastTxError == 0;
}

bool readBytes(uint8_t startReg, uint8_t *buffer, size_t length) {
  Wire.beginTransmission(gMpuAddress);
  Wire.write(startReg);
  gLastTxError = Wire.endTransmission(false);
  if (gLastTxError != 0) {
    gLastRxCount = 0;
    return false;
  }

  size_t received = Wire.requestFrom(gMpuAddress, static_cast<uint8_t>(length));
  gLastRxCount = static_cast<int>(received);
  if (received != length) {
    return false;
  }

  for (size_t i = 0; i < length; i++) {
    buffer[i] = static_cast<uint8_t>(Wire.read());
  }
  return true;
}

bool probeAddress(uint8_t address) {
  Wire.beginTransmission(address);
  int tx = Wire.endTransmission();
  return tx == 0;
}

bool detectMpuAddress(uint8_t &address) {
  const uint8_t candidates[] = {0x68, 0x69};
  for (uint8_t i = 0; i < sizeof(candidates); i++) {
    if (probeAddress(candidates[i])) {
      address = candidates[i];
      return true;
    }
  }
  return false;
}

bool readRegister(uint8_t reg, uint8_t &value) {
  uint8_t b = 0;
  if (!readBytes(reg, &b, 1)) {
    return false;
  }
  value = b;
  return true;
}

bool readMpuRaw(MpuRawData &data) {
  uint8_t raw[14];
  if (!readBytes(REG_ACCEL_XOUT_H, raw, sizeof(raw))) {
    return false;
  }

  data.ax = static_cast<int16_t>((raw[0] << 8) | raw[1]);
  data.ay = static_cast<int16_t>((raw[2] << 8) | raw[3]);
  data.az = static_cast<int16_t>((raw[4] << 8) | raw[5]);
  data.temp = static_cast<int16_t>((raw[6] << 8) | raw[7]);
  data.gx = static_cast<int16_t>((raw[8] << 8) | raw[9]);
  data.gy = static_cast<int16_t>((raw[10] << 8) | raw[11]);
  data.gz = static_cast<int16_t>((raw[12] << 8) | raw[13]);
  return true;
}

float rawTempToC(int16_t rawTemp) {
  return static_cast<float>(rawTemp) / 340.0f + 36.53f;
}

void setBuzzer(bool enabled) {
  digitalWrite(BUZZER_PIN, enabled ? HIGH : LOW);
}

bool hasFreshFix() {
  return gLatestFix.valid && (millis() - gLatestFix.updatedMs <= GPS_FIX_STALE_MS);
}

float computeAccelMagnitudeG(const MpuRawData &data) {
  const float ax = static_cast<float>(data.ax);
  const float ay = static_cast<float>(data.ay);
  const float az = static_cast<float>(data.az);
  const float rawMag = sqrtf(ax * ax + ay * ay + az * az);
  return rawMag / MPU_ACCEL_LSB_PER_G;
}

void publishBridgeEvent(const char *eventType, float tiltDeg, float accelMagnitudeG) {
  const bool freshFix = hasFreshFix();
  const unsigned long ageMs = freshFix ? (millis() - gLatestFix.updatedMs) : 0;

  // CSV event format for ESP32 parser:
  // EVT,<seq>,<event>,<valid>,<lat>,<lon>,<age_ms>,<tilt_deg>,<accel_g>
  Serial2.print("EVT,");
  Serial2.print(gBridgeSequence++);
  Serial2.print(',');
  Serial2.print(eventType);
  Serial2.print(',');
  Serial2.print(freshFix ? 1 : 0);
  Serial2.print(',');
  Serial2.print(gLatestFix.latitude, 6);
  Serial2.print(',');
  Serial2.print(gLatestFix.longitude, 6);
  Serial2.print(',');
  Serial2.print(ageMs);
  Serial2.print(',');
  Serial2.print(tiltDeg, 2);
  Serial2.print(',');
  Serial2.println(accelMagnitudeG, 3);
}

bool detectButtonPressedEdge() {
  const bool rawPressed = (digitalRead(BUTTON_PIN) == LOW);
  if (rawPressed != gButtonRawState) {
    gButtonRawState = rawPressed;
    gButtonLastEdgeMs = millis();
  }

  if ((millis() - gButtonLastEdgeMs) >= BUTTON_DEBOUNCE_MS && gButtonStableState != gButtonRawState) {
    gButtonStableState = gButtonRawState;
    if (gButtonStableState) {
      return true;
    }
  }
  return false;
}

void enterStandbyFromCrash(const char *eventType) {
  setBuzzer(false);
  gManualBuzzerLatched = false;
  gAlertState = ALERT_STANDBY;
  gLastCrashExitMs = millis();
  publishBridgeEvent(eventType, gLastTiltDeg, gLastAccelMagnitudeG);
}

void handleStandbyState(bool buttonPressed, bool crashTrigger) {
  if (crashTrigger && (millis() - gLastCrashExitMs >= EVENT_COOLDOWN_MS)) {
    gAlertState = ALERT_CRASH_PENDING;
    gCrashWindowStartMs = millis();
    gManualBuzzerLatched = false;
    setBuzzer(true);
    publishBridgeEvent("CRASH_START", gLastTiltDeg, gLastAccelMagnitudeG);
    Serial.println("STATE: CRASH_PENDING (10s cancel window)");
    return;
  }

  if (!buttonPressed) {
    return;
  }

  if (!gManualBuzzerLatched) {
    gManualBuzzerLatched = true;
    setBuzzer(true);
    publishBridgeEvent("MANUAL", gLastTiltDeg, gLastAccelMagnitudeG);
    Serial.println("STATE: STANDBY manual trigger -> buzzer ON");
    return;
  }

  gManualBuzzerLatched = false;
  setBuzzer(false);
  publishBridgeEvent("CLEAR", gLastTiltDeg, gLastAccelMagnitudeG);
  Serial.println("STATE: STANDBY manual clear -> buzzer OFF");
}

void handleCrashPendingState(bool buttonPressed) {
  if (buttonPressed) {
    enterStandbyFromCrash("CANCELED");
    Serial.println("STATE: CANCELED within 10s -> buzzer OFF");
    return;
  }

  if (millis() - gCrashWindowStartMs >= CRASH_CANCEL_WINDOW_MS) {
    gAlertState = ALERT_CRASH_CONFIRMED;
    setBuzzer(true);
    publishBridgeEvent("CRASH_CONFIRMED", gLastTiltDeg, gLastAccelMagnitudeG);
    Serial.println("STATE: CRASH_CONFIRMED -> buzzer latched ON");
  }
}

void handleCrashConfirmedState(bool buttonPressed) {
  if (buttonPressed) {
    enterStandbyFromCrash("CLEAR");
    Serial.println("STATE: CRASH_CONFIRMED acknowledged -> buzzer OFF");
  }
}

void updateSafetyStateMachine(bool buttonPressed, bool crashTrigger) {
  switch (gAlertState) {
    case ALERT_STANDBY:
      handleStandbyState(buttonPressed, crashTrigger);
      break;
    case ALERT_CRASH_PENDING:
      handleCrashPendingState(buttonPressed);
      break;
    case ALERT_CRASH_CONFIRMED:
      handleCrashConfirmedState(buttonPressed);
      break;
    default:
      gAlertState = ALERT_STANDBY;
      setBuzzer(false);
      break;
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

  // Wait briefly for Serial Monitor to connect on boards with USB serial.
  delay(300);
  Serial.println("VestMicro: Arduino Mega 2560 R3 + MPU6050 bring-up");
  Serial.println("NEO-6M on Serial1 (RX1=19, TX1=18) at 9600 baud");
  Serial.println("ESP32 bridge on Serial2 (RX2=17, TX2=16) at 115200 baud");
  Serial.println("Button pin=7 (INPUT_PULLUP), buzzer pin=6");
  Serial.println("Crash trigger: tilt>=35.0 deg OR accel>=1.35g");
  Serial.println("Crash cancel window: 10 seconds");

  if (!detectMpuAddress(gMpuAddress)) {
    Serial.println("No I2C device found at 0x68 or 0x69.");
    Serial.println("Check wiring: SDA=20, SCL=21, GND common, and module power.");
    gMpuReady = false;
    return;
  }

  Serial.print("MPU candidate found at 0x");
  Serial.println(gMpuAddress, HEX);

  uint8_t whoAmI = 0;
  if (!readRegister(REG_WHO_AM_I, whoAmI)) {
    Serial.print("WHO_AM_I read failed. txErr=");
    Serial.print(gLastTxError);
    Serial.print(" rxCount=");
    Serial.println(gLastRxCount);
    gMpuReady = false;
    return;
  }

  Serial.print("WHO_AM_I = 0x");
  Serial.println(whoAmI, HEX);

  if (whoAmI != 0x68) {
    Serial.println("Unexpected WHO_AM_I value.");
  }

  if (!writeRegister(REG_PWR_MGMT_1, 0x00)) {
    Serial.print("Failed to wake MPU6050. txErr=");
    Serial.println(gLastTxError);
    gMpuReady = false;
    return;
  }

  Serial.println("MPU6050 awake. Streaming raw accel/gyro/temp...");
  gMpuReady = true;
}

void loop() {
  processGpsStream();
  const bool buttonPressed = detectButtonPressedEdge();

  if (gAlertState == ALERT_CRASH_PENDING || gAlertState == ALERT_CRASH_CONFIRMED) {
    updateSafetyStateMachine(buttonPressed, false);
  }

  if (!gMpuReady) {
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
    updateSafetyStateMachine(buttonPressed, crashTrigger);

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
    if (freshFix) {
      Serial.print(gLatestFix.latitude, 6);
      Serial.print(',');
      Serial.println(gLatestFix.longitude, 6);
    } else {
      Serial.println("no valid fix yet");
    }
  } else {
    Serial.print("Read failed (I2C). txErr=");
    Serial.print(gLastTxError);
    Serial.print(" rxCount=");
    Serial.println(gLastRxCount);
  }
}
