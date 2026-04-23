#include <Arduino.h>

// Default UART2 pins for many ESP32 dev boards.
// RX2 receives from Mega TX2 through a level shifter.
static const int BRIDGE_RX_PIN = 16;
static const int BRIDGE_TX_PIN = 17;
static const uint32_t BRIDGE_BAUD = 115200;

char gLine[128];
size_t gLinePos = 0;

bool parseLegacyGpsLine(char *line, uint32_t &seq, bool &valid, float &lat, float &lon, unsigned long &ageMs) {
  // Expected CSV line:
  // GPS,<seq>,<valid>,<lat>,<lon>,<age_ms>
  char *token = strtok(line, ",");
  if (token == nullptr || strcmp(token, "GPS") != 0) {
    return false;
  }

  token = strtok(nullptr, ",");
  if (token == nullptr) {
    return false;
  }
  seq = static_cast<uint32_t>(strtoul(token, nullptr, 10));

  token = strtok(nullptr, ",");
  if (token == nullptr) {
    return false;
  }
  valid = (atoi(token) == 1);

  token = strtok(nullptr, ",");
  if (token == nullptr) {
    return false;
  }
  lat = static_cast<float>(atof(token));

  token = strtok(nullptr, ",");
  if (token == nullptr) {
    return false;
  }
  lon = static_cast<float>(atof(token));

  token = strtok(nullptr, ",");
  if (token == nullptr) {
    return false;
  }
  ageMs = static_cast<unsigned long>(strtoul(token, nullptr, 10));

  return true;
}

bool parseEventLine(char *line,
                    uint32_t &seq,
                    bool &valid,
                    float &lat,
                    float &lon,
                    unsigned long &ageMs,
                    char *eventType,
                    size_t eventTypeSize,
                    float &tiltDeg,
                    float &accelG) {
  // Expected CSV line:
  // EVT,<seq>,<event>,<valid>,<lat>,<lon>,<age_ms>,<tilt_deg>,<accel_g>
  char *token = strtok(line, ",");
  if (token == nullptr || strcmp(token, "EVT") != 0) {
    return false;
  }

  token = strtok(nullptr, ",");
  if (token == nullptr) {
    return false;
  }
  seq = static_cast<uint32_t>(strtoul(token, nullptr, 10));

  token = strtok(nullptr, ",");
  if (token == nullptr) {
    return false;
  }
  strncpy(eventType, token, eventTypeSize - 1);
  eventType[eventTypeSize - 1] = '\0';

  token = strtok(nullptr, ",");
  if (token == nullptr) {
    return false;
  }
  valid = (atoi(token) == 1);

  token = strtok(nullptr, ",");
  if (token == nullptr) {
    return false;
  }
  lat = static_cast<float>(atof(token));

  token = strtok(nullptr, ",");
  if (token == nullptr) {
    return false;
  }
  lon = static_cast<float>(atof(token));

  token = strtok(nullptr, ",");
  if (token == nullptr) {
    return false;
  }
  ageMs = static_cast<unsigned long>(strtoul(token, nullptr, 10));

  token = strtok(nullptr, ",");
  if (token == nullptr) {
    return false;
  }
  tiltDeg = static_cast<float>(atof(token));

  token = strtok(nullptr, ",");
  if (token == nullptr) {
    return false;
  }
  accelG = static_cast<float>(atof(token));

  return true;
}

void handleBridgeByte(char c) {
  if (c == '\r') {
    return;
  }

  if (c == '\n') {
    gLine[gLinePos] = '\0';
    if (gLinePos > 0) {
      uint32_t seq = 0;
      bool valid = false;
      float lat = 0.0f;
      float lon = 0.0f;
      unsigned long ageMs = 0;
      if (strncmp(gLine, "EVT,", 4) == 0) {
        char eventType[12] = {0};
        float tiltDeg = 0.0f;
        float accelG = 0.0f;
        if (parseEventLine(gLine, seq, valid, lat, lon, ageMs, eventType, sizeof(eventType), tiltDeg, accelG)) {
          Serial.print("EVT seq=");
          Serial.print(seq);
          Serial.print(" type=");
          Serial.print(eventType);
          Serial.print(" valid=");
          Serial.print(valid ? 1 : 0);
          Serial.print(" lat=");
          Serial.print(lat, 6);
          Serial.print(" lon=");
          Serial.print(lon, 6);
          Serial.print(" ageMs=");
          Serial.print(ageMs);
          Serial.print(" tilt=");
          Serial.print(tiltDeg, 2);
          Serial.print(" accelG=");
          Serial.println(accelG, 3);
        } else {
          Serial.print("Malformed EVT line: ");
          Serial.println(gLine);
        }
      } else if (strncmp(gLine, "GPS,", 4) == 0) {
        if (parseLegacyGpsLine(gLine, seq, valid, lat, lon, ageMs)) {
          Serial.print("GPS seq=");
          Serial.print(seq);
          Serial.print(" valid=");
          Serial.print(valid ? 1 : 0);
          Serial.print(" lat=");
          Serial.print(lat, 6);
          Serial.print(" lon=");
          Serial.print(lon, 6);
          Serial.print(" ageMs=");
          Serial.println(ageMs);
        } else {
          Serial.print("Malformed GPS line: ");
          Serial.println(gLine);
        }
      } else {
        Serial.print("Unknown bridge line: ");
        Serial.println(gLine);
      }
    }
    gLinePos = 0;
    return;
  }

  if (gLinePos < sizeof(gLine) - 1) {
    gLine[gLinePos++] = c;
  } else {
    // Overflow guard: reset buffer on oversized line.
    gLinePos = 0;
  }
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(BRIDGE_BAUD, SERIAL_8N1, BRIDGE_RX_PIN, BRIDGE_TX_PIN);

  delay(200);
  Serial.println("ESP32 bridge receiver ready");
  Serial.println("Expecting:");
  Serial.println("  EVT,<seq>,<event>,<valid>,<lat>,<lon>,<age_ms>,<tilt_deg>,<accel_g>");
  Serial.println("  GPS,<seq>,<valid>,<lat>,<lon>,<age_ms> (legacy)");
}

void loop() {
  while (Serial2.available() > 0) {
    char c = static_cast<char>(Serial2.read());
    handleBridgeByte(c);
  }
}
