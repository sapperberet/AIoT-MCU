
#include "../include/application.h"

/*********************************MCU globals**********************************/

/**********SERVOS DECLARATION**********/
Servo servoWindow1;
Servo servoWindow2;
Servo servoDoor;
Servo servoGarage;
Servo servoGate;
/**************************************/

/**********GLOBALS FOR DHT SENSOR**********/
DHT dht(DHT_PIN, DHT_TYPE);
/******************************************/

/**********GLOBALS FOR MQ135 SENSOR**********/
MQ135 mq135 = MQ135(MQ135_PIN);
/********************************************/

/**********GLOBALS FOR SOLAR SYSTEM**********/
bool solarActive = false;
/********************************************/

/**********GLOBALS FOR WIFI******************/
WiFiUDP udp;
WiFiClient net;
/********************************************/

/*********GLOBALS FOR BROKER*****************/
PubSubClient client(net); // (Instead of client(espClient))
IPAddress brokerIp;
uint16_t brokerPort = 1883;
String clientId;
unsigned long lastReconnectAttempt = 0;
unsigned long lastSensorPublish = 0; // (Timer alternative for delay)
/********************************************/

/*************************Network settings********************/
const char *WIFI_SSID = "Name of the NETWORK";
const char *WIFI_PASS = "Password of the NETWORK";
/*************************End of Network settings*************/

/************************UDP beacon settings******************/
const uint16_t BEACON_PORT = 18830;
const char *BEACON_NAME = "face-broker";
/************************End of UDP beacon settings **********/

/***********************MQTT topics***************************/
const char *TOPIC_CONTROL = "home/control"; // <--- (To receive commands)
const char *TOPIC_SENSORS = "home/sensors"; // <--- (To send sensor data)
const char *TOPIC_STATUS =
    "home/smart-system/status"; // (LWT Topic for this device)
/*************************************************************/

/*******************************End of MCU globals*****************************/

/************************Functions' definition*********************************/

void setRGB(int r, int g, int b) {
  analogWrite(RED_PIN, r);
  analogWrite(GREEN_PIN, g);
  analogWrite(BLUE_PIN, b);
}

IPAddress subnetBroadcast(IPAddress ip, IPAddress mask) {
  uint32_t ip_i = (uint32_t)ip;
  uint32_t mask_i = (uint32_t)mask;
  uint32_t bcast = ip_i | ~mask_i;
  return IPAddress(bcast);
}

bool parseAdvert(const char *json) {
  JsonDocument doc;
  if (deserializeJson(doc, json))
    return false;
  const char *name = doc["name"] | "";
  const char *ip = doc["ip"] | "";
  int port = doc["port"] | 1883;
  if (String(name) != BEACON_NAME)
    return false;
  IPAddress addr;
  if (!addr.fromString(ip))
    return false;
  brokerIp = addr;
  brokerPort = (uint16_t)port;
  return true;
}

bool discoverPassive(uint32_t ms) {
  Serial.printf("[DISCOVER] passive listen %u ms\n", ms);
  udp.begin(BEACON_PORT);
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    int p = udp.parsePacket();
    if (p) {
      char buf[256];
      int n = udp.read(buf, sizeof(buf) - 1);
      buf[n > 0 ? n : 0] = 0;
      if (parseAdvert(buf)) {
        Serial.printf("[DISCOVER] got advert %s:%u\n",
                      brokerIp.toString().c_str(), brokerPort);
        udp.stop();
        return true;
      }
    }
    delay(30);
  }
  udp.stop();
  return false;
}

bool discoverActive(uint32_t ms) {
  Serial.printf("[DISCOVER] active query %u ms\n", ms);
  udp.begin(BEACON_PORT);
  JsonDocument q;
  q["type"] = "WHO_IS";
  q["name"] = BEACON_NAME;
  char qbuf[128];
  size_t qlen = serializeJson(q, qbuf, sizeof(qbuf));
  IPAddress ip = WiFi.localIP();
  IPAddress mask = WiFi.subnetMask();
  IPAddress bcast = subnetBroadcast(ip, mask);
  uint32_t t0 = millis(), lastTx = 0;
  while (millis() - t0 < ms) {
    if (millis() - lastTx > 500) {
      udp.beginPacket(IPAddress(255, 255, 255, 255), BEACON_PORT);
      udp.write((const uint8_t *)qbuf, qlen);
      udp.endPacket();
      udp.beginPacket(bcast, BEACON_PORT);
      udp.write((const uint8_t *)qbuf, qlen);
      udp.endPacket();
      lastTx = millis();
    }
    int p = udp.parsePacket();
    if (p) {
      char buf[256];
      int n = udp.read(buf, sizeof(buf) - 1);
      buf[n > 0 ? n : 0] = 0;
      if (parseAdvert(buf)) {
        Serial.printf("[DISCOVER] got reply %s:%u\n",
                      brokerIp.toString().c_str(), brokerPort);
        udp.stop();
        return true;
      }
    }
    delay(30);
  }
  udp.stop();
  return false;
}

bool discoverBroker(uint32_t timeout_ms) {
  uint32_t half = timeout_ms / 2;
  if (discoverPassive(half))
    return true;
  return discoverActive(timeout_ms - half);
}

void ensureWifi() {
  if (WiFi.status() == WL_CONNECTED)
    return;
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("WiFi connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.printf(".");
  }
  Serial.printf("\nWiFi OK. IP: %s\n", WiFi.localIP().toString().c_str());
}

void callBack(char *topic, byte *message, unsigned int length) {
  Serial.printf("Message received on topic: ");
  Serial.println(topic);

  String messageTemp;
  for (int i = 0; i < length; i++) {
    messageTemp += (char)message[i];
  }

  Serial.printf("Message: ");
  Serial.println(messageTemp);

  // CONTROL IN APPLICATION (From your code)
  if (String(topic) == TOPIC_CONTROL) {
    if (messageTemp == "lights_on") {
      digitalWrite(LED_FLOOR1, HIGH);
      digitalWrite(LED_FLOOR2, HIGH);
      digitalWrite(LED_LANDSCAPE, HIGH);
    } else if (messageTemp == "lights_off") {
      digitalWrite(LED_FLOOR1, LOW);
      digitalWrite(LED_FLOOR2, LOW);
      digitalWrite(LED_LANDSCAPE, LOW);
    } else if (messageTemp == "fan_on") {
      analogWrite(FAN_ENA, 200);
      digitalWrite(FAN_IN1, HIGH);
      digitalWrite(FAN_IN2, LOW);
    } else if (messageTemp == "fan_off") {
      analogWrite(FAN_ENA, 0);
    }
    /* Adding other commands here */
  }
}

void ensureMqtt() {
  if (client.connected())
    return;

  // 1. Discover server (Instead of static IP)
  if (!discoverBroker(12000)) {
    Serial.println("[MQTT] discovery failed; retry soon");
    delay(1500);
    return;
  }

  // 2. Setup connection
  client.setServer(brokerIp, brokerPort);
  client.setCallback(callBack); // <--- Using your callBack

  const char *willTopic = TOPIC_STATUS;
  const char *willMsg = "OFFLINE";

  // <--- (Important) Unique Client ID, different from the first code
  clientId = "SmartHomeESP32-" + String((uint32_t)ESP.getEfuseMac(), HEX);
  Serial.printf("[MQTT] connect to %s:%u as %s\n", brokerIp.toString().c_str(),
                brokerPort, clientId.c_str());

  // 3. Connect with LWT
  bool ok = client.connect(clientId.c_str(), nullptr, nullptr, willTopic, 0,
                           true, willMsg);

  if (ok) {
    Serial.println("[MQTT] connected");
    client.subscribe(TOPIC_CONTROL); // <--- Subscribing to your topic
    client.publish(TOPIC_STATUS, "ONLINE", true);
  } else {
    Serial.printf("[MQTT] failed, rc=%d\n", client.state());
  }
}

/************************End of functions' definition**************************/
