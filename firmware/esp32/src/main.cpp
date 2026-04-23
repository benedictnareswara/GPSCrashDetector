#include <Arduino.h>

// UART link from Mega Serial2 -> ESP32 Serial2.
static const int BRIDGE_RX_PIN = 16;
static const int BRIDGE_TX_PIN = 17;
static const uint32_t BRIDGE_BAUD = 115200;

char gLine[160];
size_t gLinePos = 0;

bool parseLegacyGpsLine(char *line, uint32_t &seq, bool &valid, float &lat, float &lon, unsigned long &ageMs) {
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
          Serial.print("Malformed EVT: ");
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
          Serial.print("Malformed GPS: ");
          Serial.println(gLine);
        }
      } else {
        if (gLine[0] == '\0') {
          gLinePos = 0;
          return;
        }
        Serial.print("Unknown: ");
        Serial.println(gLine);
      }
    }
    gLinePos = 0;
    return;
  }

  if (gLinePos < sizeof(gLine) - 1) {
    gLine[gLinePos++] = c;
  } else {
    gLinePos = 0;
  }
}

void setup() {
  Serial.begin(115200);
#if defined(ARDUINO_ARCH_ESP32)
  Serial2.begin(BRIDGE_BAUD, SERIAL_8N1, BRIDGE_RX_PIN, BRIDGE_TX_PIN);
#else
  Serial2.begin(BRIDGE_BAUD);
#endif
  delay(200);
  Serial.println("ESP32 bridge app ready");
  Serial.println("Waiting for Mega event packets...");
}

void loop() {
  while (Serial2.available() > 0) {
    handleBridgeByte(static_cast<char>(Serial2.read()));
  }
}
