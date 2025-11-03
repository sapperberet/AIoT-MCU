// ===== ESP32 LED + Button-triggered Face Detection via MQTT =====
// Requires: PubSubClient, ArduinoJson
// LED control topics (as before):
//   cmd:   home/esp32/led/cmd    payload: ON | OFF | TOGGLE
//   state: home/esp32/led/state  payload: ON | OFF (retained)
//   status:home/esp32/led/status payload: ONLINE | OFFLINE (LWT)
// Face trigger topics:
//   publish trigger: face/trigger/cmd        payload: {"device":"esp32-led-<mac>","action":"START"}
//   optional ack:    face/trigger/ack        payload: {"ok":true,"run_id":"..."} (user-defined)

#include <WiFi.h>
#include <WiFiUdp.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// ---------- USER CONFIG ----------
const char* WIFI_SSID = "<wifi name>";
const char* WIFI_PASS = "<password>";

// UDP beacon settings (your Docker beacon)
const uint16_t BEACON_PORT = 18830;
const char*    BEACON_NAME = "face-broker";

// LED pin (built-in LED on many ESP32 dev boards is GPIO 2; ESP32-C3 often GPIO 8)
#ifndef LED_PIN
#define LED_PIN 2
#endif
bool LED_ACTIVE_HIGH = true;

// Pushbutton pin (use internal pull-up; button to GND)
#ifndef BUTTON_PIN
#define BUTTON_PIN 13   // change if you prefer another GPIO
#endif
const uint32_t DEBOUNCE_MS = 40;   // debounce window
const uint32_t REARM_MS    = 400;  // min time between triggers

// MQTT topics
const char* TOPIC_CMD    = "home/esp32/led/cmd";
const char* TOPIC_STATE  = "home/esp32/led/state";
const char* TOPIC_STATUS = "home/esp32/led/status";

const char* TOPIC_FACE_TRIGGER_CMD = "face/trigger/cmd";
const char* TOPIC_FACE_TRIGGER_ACK = "face/trigger/ack";  // optional

// ---------- END USER CONFIG ----------

WiFiUDP udp;
WiFiClient net;
PubSubClient mqtt(net);
IPAddress brokerIp;
uint16_t brokerPort = 1883;
String clientId;
bool ledState = false;

// Button state
bool lastBtnLevel = HIGH;       // with pull-up, idle is HIGH
uint32_t lastChangeMs = 0;
uint32_t lastTriggerMs = 0;

// ------- Helpers -------
void setLed(bool on) {
  ledState = on;
  digitalWrite(LED_PIN, (LED_ACTIVE_HIGH ? (on ? HIGH : LOW) : (on ? LOW : HIGH)));
}

void publishState() {
  mqtt.publish(TOPIC_STATE, ledState ? "ON" : "OFF", true /*retain*/);
}

void handleLedCommand(const char* payload) {
  String cmd(payload); cmd.toUpperCase();
  if      (cmd == "ON")     setLed(true);
  else if (cmd == "OFF")    setLed(false);
  else if (cmd == "TOGGLE") setLed(!ledState);
  else return;
  publishState();
}

void onMqttMessage(char* topic, byte* payload, unsigned int length) {
  static char buf[256];
  unsigned int n = min(length, (unsigned int)sizeof(buf) - 1);
  memcpy(buf, payload, n);
  buf[n] = '\0';

  if (strcmp(topic, TOPIC_CMD) == 0) {
    handleLedCommand(buf);
    return;
  }

  if (strcmp(topic, TOPIC_FACE_TRIGGER_ACK) == 0) {
    // Optional: blink LED once on ack so you get tactile feedback
    // (You can also parse JSON for richer behavior.)
    StaticJsonDocument<256> doc;
    if (deserializeJson(doc, buf) == DeserializationError::Ok) {
      bool ok = doc["ok"] | false;
      if (ok) {
        // quick blink
        bool prev = ledState;
        setLed(true);  delay(80);
        setLed(false); delay(80);
        setLed(prev);
      }
    }
  }
}

// ------- UDP discovery of broker -------
IPAddress subnetBroadcast(IPAddress ip, IPAddress mask) {
  uint32_t ip_i   = (uint32_t)ip;
  uint32_t mask_i = (uint32_t)mask;
  uint32_t bcast  = ip_i | ~mask_i;
  return IPAddress(bcast);
}

bool parseAdvert(const char* json) {
  StaticJsonDocument<256> doc;
  if (deserializeJson(doc, json)) return false;
  const char* name = doc["name"] | "";
  const char* ip   = doc["ip"]   | "";
  int port         = doc["port"] | 1883;
  if (String(name) != BEACON_NAME) return false;
  IPAddress addr;
  if (!addr.fromString(ip)) return false;
  brokerIp = addr;
  brokerPort = (uint16_t)port;
  return true;
}

bool discoverPassive(uint32_t ms = 3000) {
  Serial.printf("[DISCOVER] passive listen %u ms\n", ms);
  udp.begin(BEACON_PORT);
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    int p = udp.parsePacket();
    if (p) {
      char buf[256]; int n = udp.read(buf, sizeof(buf) - 1); buf[n > 0 ? n : 0] = 0;
      if (parseAdvert(buf)) {
        Serial.printf("[DISCOVER] got advert %s:%u\n", brokerIp.toString().c_str(), brokerPort);
        udp.stop();
        return true;
      }
    }
    delay(30);
  }
  udp.stop();
  return false;
}

bool discoverActive(uint32_t ms = 3000) {
  Serial.printf("[DISCOVER] active query %u ms\n", ms);
  udp.begin(BEACON_PORT); // bind to receive replies
  // Build WHO_IS JSON
  StaticJsonDocument<128> q;
  q["type"] = "WHO_IS";
  q["name"] = BEACON_NAME;
  char qbuf[128]; size_t qlen = serializeJson(q, qbuf, sizeof(qbuf));

  IPAddress ip = WiFi.localIP();
  IPAddress mask = WiFi.subnetMask();
  IPAddress bcast = subnetBroadcast(ip, mask);

  uint32_t t0 = millis(), lastTx = 0;
  while (millis() - t0 < ms) {
    if (millis() - lastTx > 500) {
      udp.beginPacket(IPAddress(255,255,255,255), BEACON_PORT); udp.write((const uint8_t*)qbuf, qlen); udp.endPacket();
      udp.beginPacket(bcast, BEACON_PORT);                       udp.write((const uint8_t*)qbuf, qlen); udp.endPacket();
      lastTx = millis();
    }
    int p = udp.parsePacket();
    if (p) {
      char buf[256]; int n = udp.read(buf, sizeof(buf) - 1); buf[n > 0 ? n : 0] = 0;
      if (parseAdvert(buf)) {
        Serial.printf("[DISCOVER] got reply %s:%u\n", brokerIp.toString().c_str(), brokerPort);
        udp.stop();
        return true;
      }
    }
    delay(30);
  }
  udp.stop();
  return false;
}

bool discoverBroker(uint32_t timeout_ms = 12000) {
  uint32_t half = timeout_ms / 2;
  if (discoverPassive(half)) return true;
  return discoverActive(timeout_ms - half);
}

// ------- Connectivity -------
void ensureWifi() {
  if (WiFi.status() == WL_CONNECTED) return;
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("WiFi connecting");
  while (WiFi.status() != WL_CONNECTED) { delay(250); Serial.print("."); }
  Serial.printf("\nWiFi OK. IP: %s  Mask: %s  GW: %s\n",
                WiFi.localIP().toString().c_str(),
                WiFi.subnetMask().toString().c_str(),
                WiFi.gatewayIP().toString().c_str());
}

void publishFaceTrigger() {
  // Construct a small JSON payload with device id and action
  StaticJsonDocument<128> doc;
  doc["device"] = clientId;
  doc["action"] = "START";
  char buf[128];
  size_t n = serializeJson(doc, buf, sizeof(buf));
  bool ok = mqtt.publish(TOPIC_FACE_TRIGGER_CMD, (const uint8_t*)buf, n, false /*retain*/);
  Serial.printf("[FACE] trigger publish %s: %.*s\n", ok ? "OK" : "FAIL", (int)n, buf);
}

void ensureMqtt() {
  if (mqtt.connected()) return;

  if (!discoverBroker(12000)) {
    Serial.println("[MQTT] discovery failed; retry soon");
    delay(1500);
    return;
  }

  mqtt.setServer(brokerIp, brokerPort);
  mqtt.setCallback(onMqttMessage);

  // Last Will
  const char* willTopic = TOPIC_STATUS;
  const char* willMsg   = "OFFLINE";

  clientId = "esp32-led-" + String((uint32_t)ESP.getEfuseMac(), HEX);
  Serial.printf("[MQTT] connect to %s:%u as %s\n",
                brokerIp.toString().c_str(), brokerPort, clientId.c_str());

  bool ok = mqtt.connect(
    clientId.c_str(),
    nullptr, nullptr,               // username/password (none)
    willTopic, 0, true, willMsg
  );

  if (ok) {
    Serial.println("[MQTT] connected");
    mqtt.subscribe(TOPIC_CMD, 0);
    mqtt.subscribe(TOPIC_FACE_TRIGGER_ACK, 0);  // optional
    mqtt.publish(TOPIC_STATUS, "ONLINE", true);
    publishState();
  } else {
    Serial.printf("[MQTT] failed, rc=%d\n", mqtt.state());
  }
}

// ------- Button handling (debounced falling edge) -------
void serviceButton() {
  uint32_t now = millis();
  int level = digitalRead(BUTTON_PIN);

  if (level != lastBtnLevel) {
    lastChangeMs = now;
    lastBtnLevel = level;
    return;
  }

  // Stable state long enough?
  if ((now - lastChangeMs) < DEBOUNCE_MS) return;

  // Detect press (HIGH -> LOW, pull-up)
  static bool wasPressed = false;
  bool pressed = (level == LOW);
  if (pressed && !wasPressed) {
    // re-arm window to avoid repeated triggers if button is held
    if ((now - lastTriggerMs) > REARM_MS && mqtt.connected()) {
      publishFaceTrigger();
      lastTriggerMs = now;
    }
    wasPressed = true;
  } else if (!pressed && wasPressed) {
    wasPressed = false;
  }
}

// ------- Arduino setup/loop -------
void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(LED_PIN, OUTPUT);
  setLed(false);

  pinMode(BUTTON_PIN, INPUT_PULLUP);  // button to GND

  ensureWifi();
  ensureMqtt();
}

unsigned long lastReconnectAttempt = 0;

void loop() {
  if (WiFi.status() != WL_CONNECTED) ensureWifi();

  if (!mqtt.connected()) {
    unsigned long now = millis();
    if (now - lastReconnectAttempt > 3000) {
      lastReconnectAttempt = now;
      ensureMqtt();
    }
  } else {
    mqtt.loop();
  }

  serviceButton();
}
