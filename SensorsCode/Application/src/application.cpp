
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
const char *TOPIC_CONTROL = "home/control";
const char *TOPIC_SENSORS = "home/sensors";
const char *TOPIC_STATUS = "home/smart-system/status";

/**********Actuators topics***********/
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
/*************************************/

/************Sensors topics***********/
const char *TOPIC_GAS = "home/sensors/gas";
const char *TOPIC_LDR = "home/sensors/ldr";
const char *TOPIC_RAIN = "home/sensors/rain";
const char *TOPIC_VOLTAGE = "home/sensors/voltage";
const char *TOPIC_CURRENT = "home/sensors/current";
const char *TOPIC_HUMIDITY = "home/sensors/humidity";
/*************************************/

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

  /************Actuating logic*****************/
  /* FAN */
  if (String(topic) == TOPIC_FAN) {
    if (messageTemp == "in") {
      analogWrite(FAN_ENA, 200);
      digitalWrite(FAN_IN1, HIGH);
      digitalWrite(FAN_IN2, LOW);
    } else if (messageTemp == "out") {
      analogWrite(FAN_ENA, 200);
      digitalWrite(FAN_IN1, LOW);
      digitalWrite(FAN_IN2, HIGH);
    } else if (messageTemp == "on") {
      analogWrite(FAN_ENA, 200);
    } else if (messageTemp == "off") {
      analogWrite(FAN_ENA, 0);
    } else { /*Do Nothing*/
    }
  } else { /*Do Nothing*/
  }

  /* LIGHT FLOOR1 */
  if (String(topic) == TOPIC_LIGHT_FLOOR1) {
    if (messageTemp == "on") {
      digitalWrite(LED_FLOOR1, HIGH);
    } else if (messageTemp == "off") {
      digitalWrite(LED_FLOOR1, LOW);
    } else { /*Do Nothing*/
    }
  } else { /*Do Nothing*/
  }

  /* FLOOR2 */
  if (String(topic) == TOPIC_LIGHT_FLOOR2) {
    if (messageTemp == "on") {
      digitalWrite(LED_FLOOR2, HIGH);
    } else if (messageTemp == "off") {
      digitalWrite(LED_FLOOR2, LOW);
    } else { /*Do Nothing*/
    }
  } else { /*Do Nothing*/
  }

  /* LANDSCAPE */
  if (String(topic) == TOPIC_LIGHT_LANDSCAPE) {
    if (messageTemp == "on") {
      digitalWrite(LED_LANDSCAPE, HIGH);
    } else if (messageTemp == "off") {
      digitalWrite(LED_LANDSCAPE, LOW);
    } else { /*Do nothing*/
    }
  } else { /*Do Nothing*/
  }

  /* RGB */
  if (String(topic) == TOPIC_LIGHT_RGB) {
    if (messageTemp.startsWith("b ")) {
      int val = messageTemp.substring(2).toInt();
      setRGB(val, val, val);
    } else if (messageTemp.startsWith("c ")) {
      String c = messageTemp.substring(2);

      if (c == "red") {
        setRGB(255, 0, 0);
      }

      else if (c == "green") {
        setRGB(0, 255, 0);
      } else if (c == "blue") {
        setRGB(0, 0, 255);
      } else { /*Do Nothing*/
      }
    } else { /*Do Nothing*/
    }
  }

  /* BUZZER */
  if (String(topic) == TOPIC_BUZZER) {
    if (messageTemp == "on")
      digitalWrite(BUZZER_PIN, HIGH);
    if (messageTemp == "off")
      digitalWrite(BUZZER_PIN, LOW);
  } else { /*Do Nothing*/
  }

  /* MOTORS */
  if (String(topic) == TOPIC_MOTOR_GARAGE) {
    if (messageTemp == "open")
      servoGarage.write(90);
    if (messageTemp == "close")
      servoGarage.write(0);
  } else { /*Do Nothing*/
  }

  if (String(topic) == TOPIC_MOTOR_FRONT_WIN) {
    if (messageTemp == "open")
      servoWindow1.write(90);
    if (messageTemp == "close")
      servoWindow1.write(0);
  } else { /*Do nothing*/
  }

  if (String(topic) == TOPIC_MOTOR_SIDE_WIN) {
    if (messageTemp == "open")
      servoWindow2.write(90);
    if (messageTemp == "close")
      servoWindow2.write(0);
  } else { /*Do nothing*/
  }

  if (String(topic) == TOPIC_MOTOR_DOOR) {
    if (messageTemp == "open")
      servoDoor.write(90);
    if (messageTemp == "close")
      servoDoor.write(0);
  } else { /*Do nothing*/
  }

  /********Ending of actuating logic***********/
  /*if (String(topic) == TOPIC_CONTROL) {
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
  }*/
}

void ensureMqtt() {
  if (client.connected()) {
    return;
  } else { /*Do Nothing*/
  }

  if (!discoverBroker(12000)) {
    Serial.println("===> MQTT DISCOVERY FAILED <===");
    delay(1500);
    return;
  } else { /*Do Nothing*/
  }

  client.setServer(brokerIp, brokerPort); /*Setup connetion*/
  client.setCallback(callBack);           /*Using the callBack*/

  const char *willTopic = TOPIC_STATUS;
  const char *willMsg = "OFFLINE";

  clientId = "SmartHomeESP32-" +
             String((uint32_t)ESP.getEfuseMac(), HEX); /*Cliend ID*/
  Serial.printf("===> MQTT CONNECTED TO %s:%u AS %s <===\n",
                brokerIp.toString().c_str(), brokerPort, clientId.c_str());

  bool okConnected =
      client.connect(clientId.c_str(), nullptr, nullptr, willTopic, 0, true,
                     willMsg); /*Connecting to client*/

  if (okConnected) { /*Connecting to topics*/
    Serial.println("===> MQTT CONNECTED <===");
    client.subscribe(TOPIC_CONTROL); /*Subscribing to topic*/
    client.publish(TOPIC_STATUS, "ONLINE", true);
    client.subscribe(TOPIC_FAN);             /*Subsribing to fan*/
    client.subscribe(TOPIC_LIGHT_FLOOR1);    /*Subsribing to Light floor 1*/
    client.subscribe(TOPIC_LIGHT_FLOOR2);    /*Subsribing to Light floor 2*/
    client.subscribe(TOPIC_LIGHT_LANDSCAPE); /*Subsribing to Light ladscape*/
    client.subscribe(TOPIC_LIGHT_RGB);       /*Subsribing to light RGB*/
    client.subscribe(TOPIC_BUZZER);          /*Subsribing to buzzer*/
    client.subscribe(TOPIC_MOTOR_GARAGE);    /*Subsribing to Carage*/
    client.subscribe(TOPIC_MOTOR_FRONT_WIN); /*Subsribing to front window*/
    client.subscribe(TOPIC_MOTOR_SIDE_WIN);  /*Subsribing to side window*/
    client.subscribe(TOPIC_MOTOR_DOOR);      /*Subsribing to motor door*/
  } else {
    Serial.printf("===>MQTT FAILED, rc | %d <===\n", client.state());
  }
}

/************************End of functions' definition**************************/
