#include <Arduino.h>
#include <WiFi.h>
#include <MQTT.h>

// UART link from Mega Serial2 -> ESP32 Serial2.
static const int BRIDGE_RX_PIN = 16;
static const int BRIDGE_TX_PIN = 17;
static const uint32_t BRIDGE_BAUD = 115200;
static const size_t BRIDGE_LINE_BUFFER_SIZE = 180;
static const uint32_t WIFI_RETRY_MS = 5000;
static const uint32_t WIFI_SCAN_INTERVAL_MS = 20000;
static const uint32_t MQTT_RETRY_MS = 4000;
static const uint32_t HEARTBEAT_MS = 30000;
static const size_t EVENT_QUEUE_CAPACITY = 24;

#ifndef WIFI_SSID
#define WIFI_SSID "YOUR_WIFI_SSID"
#endif

#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"
#endif

#ifndef MQTT_BROKER
#define MQTT_BROKER "broker.hivemq.com"
#endif

#ifndef MQTT_PORT
#define MQTT_PORT 1883
#endif

#ifndef MQTT_USERNAME
#define MQTT_USERNAME ""
#endif

#ifndef MQTT_PASSWORD
#define MQTT_PASSWORD ""
#endif

#ifndef DEVICE_ID
#define DEVICE_ID "vestmicro-esp32-01"
#endif

WiFiClient gNetClient;
MQTTClient gMqttClient(1024);

struct BridgeEvent {
  uint32_t seq;
  bool valid;
  float lat;
  float lon;
  unsigned long ageMs;
  float tiltDeg;
  float accelG;
  char eventType[20];
  unsigned long receivedMs;
};

BridgeEvent gEventQueue[EVENT_QUEUE_CAPACITY];
size_t gQueueHead = 0;
size_t gQueueTail = 0;
size_t gQueueCount = 0;
uint32_t gDroppedEventCount = 0;

unsigned long gLastWiFiAttemptMs = 0;
unsigned long gLastWiFiScanMs = 0;
unsigned long gLastMqttAttemptMs = 0;
unsigned long gLastHeartbeatMs = 0;
wl_status_t gLastWiFiStatus = WL_IDLE_STATUS;
bool gWiFiWasConnected = false;

char gLine[BRIDGE_LINE_BUFFER_SIZE];
size_t gLinePos = 0;

bool isAcceptedEventType(const char *eventType) {
  return strcmp(eventType, "MANUAL") == 0 ||
         strcmp(eventType, "CRASH_START") == 0 ||
         strcmp(eventType, "CANCELED") == 0 ||
         strcmp(eventType, "CRASH_CONFIRMED") == 0 ||
         strcmp(eventType, "CLEAR") == 0;
}

void eventTopic(char *buffer, size_t size) {
  snprintf(buffer, size, "vestmicro/v1/devices/%s/events", DEVICE_ID);
}

void statusTopic(char *buffer, size_t size) {
  snprintf(buffer, size, "vestmicro/v1/devices/%s/status", DEVICE_ID);
}

void enqueueEvent(const BridgeEvent &event) {
  if (gQueueCount == EVENT_QUEUE_CAPACITY) {
    // Drop oldest when full so latest critical events can still enter.
    gQueueHead = (gQueueHead + 1) % EVENT_QUEUE_CAPACITY;
    gQueueCount--;
    gDroppedEventCount++;
  }

  gEventQueue[gQueueTail] = event;
  gQueueTail = (gQueueTail + 1) % EVENT_QUEUE_CAPACITY;
  gQueueCount++;
}

bool dequeueEvent(BridgeEvent &outEvent) {
  if (gQueueCount == 0) {
    return false;
  }
  outEvent = gEventQueue[gQueueHead];
  gQueueHead = (gQueueHead + 1) % EVENT_QUEUE_CAPACITY;
  gQueueCount--;
  return true;
}

bool peekEvent(BridgeEvent &outEvent) {
  if (gQueueCount == 0) {
    return false;
  }
  outEvent = gEventQueue[gQueueHead];
  return true;
}

void printEvent(const BridgeEvent &event, const char *prefix) {
  Serial.print(prefix);
  Serial.print(" seq=");
  Serial.print(event.seq);
  Serial.print(" type=");
  Serial.print(event.eventType);
  Serial.print(" valid=");
  Serial.print(event.valid ? 1 : 0);
  Serial.print(" lat=");
  Serial.print(event.lat, 6);
  Serial.print(" lon=");
  Serial.print(event.lon, 6);
  Serial.print(" ageMs=");
  Serial.print(event.ageMs);
  Serial.print(" tilt=");
  Serial.print(event.tiltDeg, 2);
  Serial.print(" accelG=");
  Serial.println(event.accelG, 3);
}

void buildEventJson(const BridgeEvent &event, char *json, size_t jsonSize) {
  snprintf(
      json,
      jsonSize,
      "{\"protocol\":\"v1\",\"device\":\"%s\",\"seq\":%lu,\"event\":\"%s\",\"valid\":%d,\"lat\":%.6f,\"lon\":%.6f,\"age_ms\":%lu,\"tilt_deg\":%.2f,\"accel_g\":%.3f,\"received_ms\":%lu,\"queue\":%u,\"dropped\":%lu}",
      DEVICE_ID,
      static_cast<unsigned long>(event.seq),
      event.eventType,
      event.valid ? 1 : 0,
      static_cast<double>(event.lat),
      static_cast<double>(event.lon),
      event.ageMs,
      static_cast<double>(event.tiltDeg),
      static_cast<double>(event.accelG),
      event.receivedMs,
      static_cast<unsigned int>(gQueueCount),
      static_cast<unsigned long>(gDroppedEventCount));
}

void publishHeartbeat() {
  if (!gMqttClient.connected()) {
    return;
  }
  if (millis() - gLastHeartbeatMs < HEARTBEAT_MS) {
    return;
  }
  gLastHeartbeatMs = millis();

  char topic[96];
  char payload[200];
  statusTopic(topic, sizeof(topic));
  snprintf(payload,
           sizeof(payload),
           "{\"device\":\"%s\",\"wifi\":%d,\"mqtt\":1,\"queue\":%u,\"dropped\":%lu,\"uptime_ms\":%lu}",
           DEVICE_ID,
           WiFi.status() == WL_CONNECTED ? 1 : 0,
           static_cast<unsigned int>(gQueueCount),
           static_cast<unsigned long>(gDroppedEventCount),
           millis());

  gMqttClient.publish(topic, payload, false, 1);
}

const char *wifiStatusText(wl_status_t status) {
  switch (status) {
    case WL_IDLE_STATUS:
      return "IDLE";
    case WL_NO_SSID_AVAIL:
      return "NO_SSID";
    case WL_SCAN_COMPLETED:
      return "SCAN_DONE";
    case WL_CONNECTED:
      return "CONNECTED";
    case WL_CONNECT_FAILED:
      return "CONNECT_FAILED";
    case WL_CONNECTION_LOST:
      return "CONNECTION_LOST";
    case WL_DISCONNECTED:
      return "DISCONNECTED";
    default:
      return "UNKNOWN";
  }
}

void reportWiFiTransitions() {
  const wl_status_t now = WiFi.status();
  if (now != gLastWiFiStatus) {
    Serial.print("WiFi status changed: ");
    Serial.println(wifiStatusText(now));
    gLastWiFiStatus = now;
  }

  if (now == WL_CONNECTED && !gWiFiWasConnected) {
    gWiFiWasConnected = true;
    Serial.print("WiFi connected, IP=");
    Serial.println(WiFi.localIP());
  }

  if (now != WL_CONNECTED) {
    gWiFiWasConnected = false;
  }
}

void scanNearbySsidsIfNeeded() {
  if (WiFi.status() != WL_NO_SSID_AVAIL) {
    return;
  }

  if (millis() - gLastWiFiScanMs < WIFI_SCAN_INTERVAL_MS) {
    return;
  }
  gLastWiFiScanMs = millis();

  Serial.println("WiFi scan: target SSID not visible, scanning nearby networks...");
  const int networkCount = WiFi.scanNetworks(false, true);
  if (networkCount <= 0) {
    Serial.println("WiFi scan: no networks found.");
    return;
  }

  bool targetFound = false;
  Serial.print("WiFi scan: found ");
  Serial.print(networkCount);
  Serial.println(" network(s)");

  for (int i = 0; i < networkCount; ++i) {
    const String ssid = WiFi.SSID(i);
    Serial.print("  - ");
    if (ssid.length() == 0) {
      Serial.print("<hidden>");
    } else {
      Serial.print(ssid);
    }
    Serial.print(" RSSI=");
    Serial.print(WiFi.RSSI(i));
    Serial.print("dBm");
    if (ssid == WIFI_SSID) {
      targetFound = true;
      Serial.print("  <TARGET>");
    }
    Serial.println();
  }

  if (!targetFound) {
    Serial.print("WiFi scan: target '");
    Serial.print(WIFI_SSID);
    Serial.println("' still not visible. Check 2.4GHz/visibility settings.");
  }
}

void ensureWiFiConnected() {
  const wl_status_t status = WiFi.status();
  if (status == WL_CONNECTED) {
    return;
  }

  if (millis() - gLastWiFiAttemptMs < WIFI_RETRY_MS) {
    return;
  }
  gLastWiFiAttemptMs = millis();

  Serial.print("WiFi: attempting connect to SSID '");
  Serial.print(WIFI_SSID);
  Serial.print("' (status=");
  Serial.print(wifiStatusText(status));
  Serial.println(")");

  WiFi.disconnect();
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void ensureMqttConnected() {
  if (WiFi.status() != WL_CONNECTED) {
    return;
  }

  if (gMqttClient.connected()) {
    return;
  }

  if (millis() - gLastMqttAttemptMs < MQTT_RETRY_MS) {
    return;
  }
  gLastMqttAttemptMs = millis();

  Serial.println("MQTT: attempting connect...");
  const bool connected = gMqttClient.connect(DEVICE_ID, MQTT_USERNAME, MQTT_PASSWORD);
  if (!connected) {
    Serial.println("MQTT: connect failed");
    return;
  }

  Serial.println("MQTT: connected");
  char topic[96];
  statusTopic(topic, sizeof(topic));
  gMqttClient.publish(topic, "{\"status\":\"online\"}", true, 1);
}

void publishQueuedEvent() {
  if (!gMqttClient.connected()) {
    return;
  }

  BridgeEvent event;
  if (!peekEvent(event)) {
    return;
  }

  char topic[96];
  char payload[360];
  eventTopic(topic, sizeof(topic));
  buildEventJson(event, payload, sizeof(payload));
  const bool ok = gMqttClient.publish(topic, payload, false, 1);
  if (!ok) {
    Serial.println("MQTT: publish failed, will retry");
    return;
  }

  BridgeEvent emitted;
  if (dequeueEvent(emitted)) {
    printEvent(emitted, "MQTT published");
  }
}

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

bool parseAnyEventLine(char *line, BridgeEvent &event) {
  uint32_t seq = 0;
  bool valid = false;
  float lat = 0.0f;
  float lon = 0.0f;
  unsigned long ageMs = 0;

  if (strncmp(line, "EVT,", 4) == 0) {
    float tiltDeg = 0.0f;
    float accelG = 0.0f;
    char eventType[20] = {0};
    if (!parseEventLine(line, seq, valid, lat, lon, ageMs, eventType, sizeof(eventType), tiltDeg, accelG)) {
      return false;
    }
    if (!isAcceptedEventType(eventType)) {
      return false;
    }

    event.seq = seq;
    event.valid = valid;
    event.lat = lat;
    event.lon = lon;
    event.ageMs = ageMs;
    event.tiltDeg = tiltDeg;
    event.accelG = accelG;
    strncpy(event.eventType, eventType, sizeof(event.eventType) - 1);
    event.eventType[sizeof(event.eventType) - 1] = '\0';
    event.receivedMs = millis();
    return true;
  }

  if (strncmp(line, "GPS,", 4) == 0) {
    if (!parseLegacyGpsLine(line, seq, valid, lat, lon, ageMs)) {
      return false;
    }
    event.seq = seq;
    event.valid = valid;
    event.lat = lat;
    event.lon = lon;
    event.ageMs = ageMs;
    event.tiltDeg = 0.0f;
    event.accelG = 0.0f;
    strncpy(event.eventType, "GPS_LEGACY", sizeof(event.eventType) - 1);
    event.eventType[sizeof(event.eventType) - 1] = '\0';
    event.receivedMs = millis();
    return true;
  }

  return false;
}

void handleBridgeLine(char *line) {
  if (line[0] == '\0') {
    return;
  }

  BridgeEvent event;
  if (parseAnyEventLine(line, event)) {
    printEvent(event, "RX");
    enqueueEvent(event);
    return;
  }

  Serial.print("Unknown or invalid bridge line: ");
  Serial.println(line);
}

void handleBridgeByte(char c) {
  if (c == '\r') {
    return;
  }

  if (c == '\n') {
    gLine[gLinePos] = '\0';
    handleBridgeLine(gLine);
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

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  gMqttClient.begin(MQTT_BROKER, MQTT_PORT, gNetClient);

  delay(200);
  Serial.println("ESP32 bridge MQTT gateway ready");
  Serial.println("Waiting for Mega bridge packets and network connection...");
}

void loop() {
  while (Serial2.available() > 0) {
    handleBridgeByte(static_cast<char>(Serial2.read()));
  }

  reportWiFiTransitions();
  scanNearbySsidsIfNeeded();
  ensureWiFiConnected();
  gMqttClient.loop();
  ensureMqttConnected();
  publishQueuedEvent();
  publishHeartbeat();
}
