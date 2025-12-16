
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
WiFiClient net;
/********************************************/

/*********GLOBALS FOR BROKER*****************/
const char *BROKER_HOST = "broker.emqx.io";
const uint16_t BROKER_PORT = 1883;
PubSubClient client(net); // (Instead of client(espClient))
String clientId;
unsigned long lastReconnectAttempt = 0;
unsigned long lastSensorPublish = 0; // (Timer alternative for delay)
/********************************************/

/*************************Network settings********************/
const char *WIFI_SSID = "Wokwi-GUEST";
const char *WIFI_PASS = "";
/*************************End of Network settings*************/

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
const char *TOPIC_FLAME = "home/sensors/flame";
const char *TOPIC_TEMPERATURE = "home/sensors/temp";
const char *TOPIC_PUSH_BUTTON_FACE_DETECTION = "home/events/face-detection";
/*************************************/

/*************************************************************/

/*******************************End of MCU globals*****************************/

/************************Functions' definition*********************************/

void setRGB(int r, int g, int b) {
  analogWrite(RED_PIN, r);
  analogWrite(GREEN_PIN, g);
  analogWrite(BLUE_PIN, b);
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
  String topicStr = String(topic);
  /* FAN */
  if (topicStr == TOPIC_FAN) {
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
    }
  }

  /* LIGHT FLOOR1 */
  if (topicStr == TOPIC_LIGHT_FLOOR1) {
    if (messageTemp == "on") {
      digitalWrite(LED_FLOOR1, HIGH);
    } else if (messageTemp == "off") {
      digitalWrite(LED_FLOOR1, LOW);
    }
  }

  /* FLOOR2 */
  if (topicStr == TOPIC_LIGHT_FLOOR2) {
    if (messageTemp == "on") {
      digitalWrite(LED_FLOOR2, HIGH);
    } else if (messageTemp == "off") {
      digitalWrite(LED_FLOOR2, LOW);
    }
  }

  /* LANDSCAPE */
  if (topicStr == TOPIC_LIGHT_LANDSCAPE) {
    if (messageTemp == "on") {
      digitalWrite(LED_LANDSCAPE, HIGH);
    } else if (messageTemp == "off") {
      digitalWrite(LED_LANDSCAPE, LOW);
    }
  }

  /* RGB */
  if (topicStr == TOPIC_LIGHT_RGB) {
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
      }
    }
  }

  /* BUZZER */
  if (topicStr == TOPIC_BUZZER) {
    if (messageTemp == "on")
      digitalWrite(BUZZER_PIN, HIGH);
    if (messageTemp == "off")
      digitalWrite(BUZZER_PIN, LOW);
  }

  /* MOTORS */
  if (topicStr == TOPIC_MOTOR_GARAGE) {
    if (messageTemp == "open")
      servoGarage.write(90);
    if (messageTemp == "close")
      servoGarage.write(0);
  }

  if (topicStr == TOPIC_MOTOR_FRONT_WIN) {
    if (messageTemp == "open")
      servoWindow1.write(90);
    if (messageTemp == "close")
      servoWindow1.write(0);
  }

  if (topicStr == TOPIC_MOTOR_SIDE_WIN) {
    if (messageTemp == "open")
      servoWindow2.write(90);
    if (messageTemp == "close")
      servoWindow2.write(0);
  }

  if (topicStr == TOPIC_MOTOR_DOOR) {
    if (messageTemp == "open")
      servoDoor.write(90);
    if (messageTemp == "close")
      servoDoor.write(0);
  }
}

bool buttonPressed(int pinNumber){
  bool state = digitalRead(pinNumber);

  if (LOW == state){
    return (true);
  }
  else{
    return (false);
  }
}

void ensureMqtt() {
  if (client.connected()) {
    return;
  }

  client.setServer(BROKER_HOST, BROKER_PORT); /*Setup connetion*/
  client.setCallback(callBack);           /*Using the callBack*/

  const char *willTopic = TOPIC_STATUS;
  const char *willMsg = "OFFLINE";

  clientId = "SmartHomeESP32-" +
             String((uint32_t)ESP.getEfuseMac(), HEX); /*Cliend ID*/
  Serial.printf("===> MQTT CONNECTING TO %s:%u AS %s <===\n",
                BROKER_HOST, BROKER_PORT, clientId.c_str());

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
