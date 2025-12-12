/***********************Includes******************************/
#include <Adafruit_Sensor.h>
#include <ArduinoJson.h>
#include <DHT.h>
#include <MQ135.h>
#include <PubSubClient.h>
#include <ESP32Servo.h>
// #include <Servo.h>
#include <WiFi.h>
#include <WiFiUdp.h>

/***********************Pin Definitions***********************/

/**********SENSORS PINS**********/
#define MQ135_PIN 34
#define FLAME_PIN 14
#define DHT_PIN 4
#define LDR_PIN 35
#define RAIN_PIN 32

/**********FAN**********/
#define FAN_ENA 33
#define FAN_IN1 5
#define FAN_IN2 13

/**********BUZZER**********/
#define BUZZER_PIN 23

/**********LEDs**********/
#define LED_FLOOR1 25
#define LED_FLOOR2 26
#define LED_LANDSCAPE 27

/**********RGB**********/
#define RED_PIN 22
#define GREEN_PIN 2
#define BLUE_PIN 12

/**********SerVos**********/
#define SERVO_WINDOW1_PIN 15
#define SERVO_WINDOW2_PIN 16
#define SERVO_DOOR_PIN 18
#define SERVO_GARAGE_PIN 19
#define SERVO_GATE_PIN 21

/***********************Globals*******************************/

/**********SERVOS**********/
Servo servoWindow1;
Servo servoWindow2;
Servo servoDoor;
Servo servoGarage;
Servo servoGate;

/**********SENSORS**********/
#define DHT_TYPE DHT22
DHT dht(DHT_PIN, DHT_TYPE);
MQ135 mq135 = MQ135(MQ135_PIN);

bool solarActive = false;

/**********NETWORK**********/
WiFiUDP udp;
WiFiClient net;

/**********MQTT**********/
PubSubClient client(net);
IPAddress brokerIp;
uint16_t brokerPort = 1883;
String clientId;
unsigned long lastReconnectAttempt = 0;
unsigned long lastSensorPublish = 0;

/**********WiFi creds**********/
const char *WIFI_SSID = "Name of the NETWORK";
const char *WIFI_PASS = "Password of the NETWORK";

/**********UDP Discovery**********/
const uint16_t BEACON_PORT = 18830;
const char *BEACON_NAME = "face-broker";

/**********MQTT Topics**********/
const char *TOPIC_CONTROL = "home/control";
const char *TOPIC_SENSORS = "home/sensors";
const char *TOPIC_STATUS = "home/smart-system/status";

const char *TOPIC_FAN = "home/actuators/fan";
const char *TOPIC_LIGHT_FLOOR1 = "home/actuators/lights/floor1";
const char *TOPIC_LIGHT_FLOOR2 = "home/actuators/lights/floor2";
const char *TOPIC_LIGHT_LANDSCAPE = "home/actuators/lights/landscape";
const char *TOPIC_LIGHT_RGB = "home/actuators/lights/rgb";
const char *TOPIC_BUZZER = "home/actuators/buzzer";
const char *TOPIC_MOTOR_GARAGE = "home/actuators/motors/garage";
const char *TOPIC_MOTOR_FRONT_WIN = "home/actuators/motors/frontwindow";
const char *TOPIC_MOTOR_SIDE_WIN = "home/actuators/motors/sidewindow";
const char *TOPIC_MOTOR_DOOR = "home/actuators/motors/door";

/**********Sensors topics**********/
const char *TOPIC_GAS = "home/sensors/gas";
const char *TOPIC_LDR = "home/sensors/ldr";
const char *TOPIC_RAIN = "home/sensors/rain";
const char *TOPIC_VOLTAGE = "home/sensors/voltage";
const char *TOPIC_CURRENT = "home/sensors/current";
const char *TOPIC_HUMIDITY = "home/sensors/humidity";

/************************************************************/
/***********************Functions******************************/
IPAddress subnetBroadcast(IPAddress ip, IPAddress mask) {
  uint32_t ip_i = (uint32_t)ip;
  uint32_t mask_i = (uint32_t)mask;
  uint32_t bcast = ip_i | ~mask_i;
  return IPAddress(bcast);
}

void setRGB(int r, int g, int b) {
  analogWrite(RED_PIN, r);
  analogWrite(GREEN_PIN, g);
  analogWrite(BLUE_PIN, b);
}

bool parseAdvert(const char *json) {
  JsonDocument doc;
  if (deserializeJson(doc, json)) return false;
  const char *name = doc["name"] | "";
  const char *ip = doc["ip"] | "";
  int port = doc["port"] | 1883;

  if (String(name) != BEACON_NAME)
    return false;

  IPAddress addr;
  if (!addr.fromString(ip))
    return false;

  brokerIp = addr;
  brokerPort = port;
  return true;
}

bool discoverPassive(uint32_t ms) {
  udp.begin(BEACON_PORT);
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    int p = udp.parsePacket();
    if (p) {
      char buf[256];
      int n = udp.read(buf, sizeof(buf) - 1);
      buf[n > 0 ? n : 0] = 0;
      if (parseAdvert(buf)) {
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
  udp.begin(BEACON_PORT);
  JsonDocument q;
  q["type"] = "WHO_IS";
  q["name"] = BEACON_NAME;
  char qbuf[128];
  size_t qlen = serializeJson(q, qbuf, sizeof(qbuf));

  IPAddress bcast = subnetBroadcast(WiFi.localIP(), WiFi.subnetMask());
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
  if (discoverPassive(half)) return true;
  return discoverActive(timeout_ms - half);
}

void ensureWifi() {
  if (WiFi.status() == WL_CONNECTED) return;

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
  }
}

void callBack(char *topic, byte *message, unsigned int length) {
  String msg;
  for (int i = 0; i < length; i++)
    msg += (char)message[i];

  /************Actuating logic*****************/
  if (String(topic) == TOPIC_FAN) {
    if (msg == "in") {
      analogWrite(FAN_ENA, 200);
      digitalWrite(FAN_IN1, HIGH);
      digitalWrite(FAN_IN2, LOW);
    } else if (msg == "out") {
      analogWrite(FAN_ENA, 200);
      digitalWrite(FAN_IN1, LOW);
      digitalWrite(FAN_IN2, HIGH);
    } else if (msg == "on") {
      analogWrite(FAN_ENA, 200);
    } else if (msg == "off") {
      analogWrite(FAN_ENA, 0);
    }
  }

  if (String(topic) == TOPIC_LIGHT_FLOOR1) {
    digitalWrite(LED_FLOOR1, msg == "on");
  }

  if (String(topic) == TOPIC_LIGHT_FLOOR2) {
    digitalWrite(LED_FLOOR2, msg == "on");
  }

  if (String(topic) == TOPIC_LIGHT_LANDSCAPE) {
    digitalWrite(LED_LANDSCAPE, msg == "on");
  }

  if (String(topic) == TOPIC_LIGHT_RGB) {
    if (msg.startsWith("b ")) {
      int val = msg.substring(2).toInt();
      setRGB(val, val, val);
    } else if (msg.startsWith("c ")) {
      String c = msg.substring(2);
      if (c == "red") setRGB(255,0,0);
      if (c == "green") setRGB(0,255,0);
      if (c == "blue") setRGB(0,0,255);
    }
  }

  if (String(topic) == TOPIC_BUZZER) {
    digitalWrite(BUZZER_PIN, msg == "on");
  }

  if (String(topic) == TOPIC_MOTOR_GARAGE) {
    servoGarage.write(msg == "open" ? 90 : 0);
  }

  if (String(topic) == TOPIC_MOTOR_FRONT_WIN) {
    servoWindow1.write(msg == "open" ? 90 : 0);
  }

  if (String(topic) == TOPIC_MOTOR_SIDE_WIN) {
    servoWindow2.write(msg == "open" ? 90 : 0);
  }

  if (String(topic) == TOPIC_MOTOR_DOOR) {
    servoDoor.write(msg == "open" ? 90 : 0);
  }
}

void ensureMqtt() {
  if (client.connected()) return;

  if (!discoverBroker(12000)) {
    delay(1000);
    return;
  }

  client.setServer(brokerIp, brokerPort);
  client.setCallback(callBack);

  clientId = "SmartHomeESP32-" + String((uint32_t)ESP.getEfuseMac(), HEX);

  client.connect(clientId.c_str(), NULL, NULL, TOPIC_STATUS, 0, true, "OFFLINE");

  if (client.connected()) {
    client.subscribe(TOPIC_CONTROL);
    client.publish(TOPIC_STATUS, "ONLINE", true);

    client.subscribe(TOPIC_FAN);
    client.subscribe(TOPIC_LIGHT_FLOOR1);
    client.subscribe(TOPIC_LIGHT_FLOOR2);
    client.subscribe(TOPIC_LIGHT_LANDSCAPE);
    client.subscribe(TOPIC_LIGHT_RGB);
    client.subscribe(TOPIC_BUZZER);
    client.subscribe(TOPIC_MOTOR_GARAGE);
    client.subscribe(TOPIC_MOTOR_FRONT_WIN);
    client.subscribe(TOPIC_MOTOR_SIDE_WIN);
    client.subscribe(TOPIC_MOTOR_DOOR);
  }
}

/************************************************************/
/***********************Arduino Setup************************/

void setup() {
  Serial.begin(115200);
  dht.begin();

  pinMode(FAN_ENA, OUTPUT);
  pinMode(FAN_IN1, OUTPUT);
  pinMode(FAN_IN2, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  pinMode(MQ135_PIN, INPUT);
  pinMode(FLAME_PIN, INPUT);
  pinMode(LDR_PIN, INPUT);
  pinMode(RAIN_PIN, INPUT);

  pinMode(LED_FLOOR1, OUTPUT);
  pinMode(LED_FLOOR2, OUTPUT);
  pinMode(LED_LANDSCAPE, OUTPUT);

  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);

  servoWindow1.attach(SERVO_WINDOW1_PIN);
  servoWindow2.attach(SERVO_WINDOW2_PIN);
  servoDoor.attach(SERVO_DOOR_PIN);
  servoGarage.attach(SERVO_GARAGE_PIN);
  servoGate.attach(SERVO_GATE_PIN);

  ensureWifi();
  ensureMqtt();
}

/************************************************************/
/***********************Arduino Loop*************************/

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    ensureWifi();
  }

  unsigned long now = millis();

  if (!client.connected()) {
    if (now - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = now;
      ensureMqtt();
    }
  } else {
    client.loop();
  }

  if (now - lastSensorPublish > 1000) {
    lastSensorPublish = now;

    float humidity = dht.readHumidity();
    float temperature = dht.readTemperature();
    int smoke = analogRead(MQ135_PIN);
    int flame = digitalRead(FLAME_PIN);
    int lightLevel = analogRead(LDR_PIN);
    int rain = digitalRead(RAIN_PIN);

    if (smoke > 2000 || flame == LOW) {
      digitalWrite(BUZZER_PIN, HIGH);
      setRGB(255, 0, 0);
    } else {
      digitalWrite(BUZZER_PIN, LOW);

      if (rain == LOW) {
        servoWindow1.write(0);
        servoWindow2.write(0);
      } else {
        servoWindow1.write(90);
        servoWindow2.write(90);
      }

      if (temperature > 30) {
        analogWrite(FAN_ENA, 200);
        digitalWrite(FAN_IN1, HIGH);
        digitalWrite(FAN_IN2, LOW);
      } else {
        analogWrite(FAN_ENA, 0);
      }

      if (lightLevel < 1000) {
        digitalWrite(LED_FLOOR1, HIGH);
        digitalWrite(LED_FLOOR2, HIGH);
        digitalWrite(LED_LANDSCAPE, HIGH);
        setRGB(0, 0, 255);
      } else {
        digitalWrite(LED_FLOOR1, LOW);
        digitalWrite(LED_FLOOR2, LOW);
        digitalWrite(LED_LANDSCAPE, LOW);
        setRGB(255, 255, 0);
      }

      if (solarActive) {
        setRGB(0, 255, 0);
      }
    }

    if (client.connected()) {
      String payload = String("{\"temperature\":") + temperature +
                       ",\"humidity\":" + humidity + ",\"smoke\":" + smoke +
                       ",\"light\":" + lightLevel + ",\"rain\":" + rain +
                       ",\"flame\":" + flame + "}";

      client.publish(TOPIC_SENSORS, payload.c_str());
      client.publish(TOPIC_HUMIDITY, String(humidity).c_str());
      client.publish(TOPIC_GAS, String(smoke).c_str());
      client.publish(TOPIC_LDR, String(lightLevel).c_str());
      client.publish(TOPIC_RAIN, String(rain).c_str());
      client.publish(TOPIC_CURRENT, "0");
      client.publish(TOPIC_VOLTAGE, "0");
    }
  }
}

