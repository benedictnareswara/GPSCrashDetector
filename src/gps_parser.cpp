#include "gps_parser.hpp"

#include <stdlib.h>
#include <string.h>

#include "app_config.hpp"

namespace {
char gGpsLine[100];
size_t gGpsLinePos = 0;
GpsFix gLatestFix = {false, 0.0f, 0.0f, 0};

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
}  // namespace

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

bool hasFreshFix() {
  return gLatestFix.valid && (millis() - gLatestFix.updatedMs <= GPS_FIX_STALE_MS);
}

const GpsFix &getLatestFix() {
  return gLatestFix;
}
