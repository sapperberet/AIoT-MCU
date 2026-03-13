
#include "../include/application.h"

/*********************************MCU globals**********************************/

/**********SERVOS DECLARATION**********/
Servo servoFrontWinLeft;
Servo servoFrontWinRight;
Servo servoDoor;
Servo servoGarage;
Servo servoGateLeft;
Servo servoGateRight;
/**************************************/

/**********GATE SETTINGS & STATE**********/
int gateClosedAngle = 150;        /* Closed position angle */
int gateOpenAngle = 30;           /* Open position angle (120 degree opening) */
int gateUsStep = 1;              /* Microsecond step size for smooth movement (matches reference) */
int gateStepDelay = 1;            /* Delay between steps (ms) */
int gateCurrentAngle = 150;       /* Current angle of main gate */
int gateCurrentUs = 0;            /* Current pulse width (initialized in setup) */
volatile int gateTargetAngle = 150;   /* Target angle set by MQTT */
volatile bool gateNeedsMove = false;  /* Flag indicating gate needs to move */
/**************************************/

/**********FRONT WINDOW SETTINGS & STATE**********/
int frontWinClosedAngle = 150;    /* Closed position angle */
int frontWinOpenAngle = 30;       /* Open position angle (120 degree opening) */
int frontWinOffsetAngle = 10;     /* Offset for right door compensation */
int frontWinUsStep = 1;          /* Microsecond step size for smooth movement (matches reference) */
int frontWinStepDelay = 1;        /* Delay between steps (ms) */
int frontWinCurrentAngle = 150;   /* Current angle of front window */
int frontWinCurrentUs = 0;        /* Current pulse width (initialized in setup) */
volatile int frontWinTargetAngle = 150;  /* Target angle set by MQTT */
volatile bool frontWinNeedsMove = false; /* Flag indicating window needs to move */
/**************************************/

/**********DOOR SETTINGS & STATE**********/
int doorClosedAngle = 150;        /* Closed position angle */
int doorOpenAngle = 30;           /* Open position angle */
int doorUsStep = 20;              /* Microsecond step size for smooth movement (matches reference) */
int doorStepDelay = 1;            /* Delay between steps (ms) */
int doorCurrentAngle = 150;       /* Current angle of door */
int doorCurrentUs = 0;            /* Current pulse width (initialized in setup) */
volatile int doorTargetAngle = 150;      /* Target angle set by MQTT */
volatile bool doorNeedsMove = false;     /* Flag indicating door needs to move */
/**************************************/

/**********GARAGE SETTINGS & STATE**********/
int garageClosedAngle = 150;      /* Closed position angle */
int garageOpenAngle = 30;         /* Open position angle */
int garageUsStep = 20;            /* Microsecond step size for smooth movement (matches reference) */
int garageStepDelay = 1;          /* Delay between steps (ms) */
int garageCurrentAngle = 150;     /* Current angle of garage */
int garageCurrentUs = 0;          /* Current pulse width (initialized in setup) */
volatile int garageTargetAngle = 150;    /* Target angle set by MQTT */
volatile bool garageNeedsMove = false;   /* Flag indicating garage needs to move */
/**************************************/

/**********GLOBALS FOR DHT SENSOR**********/
DHTesp dht;  // ESP32-compatible DHT sensor - call dht.setup() in setup()
/*******************************************/

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
const uint16_t BEACON_PORT = 18830;
const char *BEACON_NAME = "face-broker";
PubSubClient client(net); // (Instead of client(espClient))
IPAddress brokerIp;
uint16_t brokerPort = 1883;
String clientId;
unsigned long lastReconnectAttempt = 0;
unsigned long lastSensorPublish = 0; // (Timer alternative for delay)
/********************************************/

/*************************Network settings********************/
#ifndef WIFI_SSID_VALUE
#define WIFI_SSID_VALUE ""
#endif

#ifndef WIFI_PASS_VALUE
#define WIFI_PASS_VALUE ""
#endif

const char *WIFI_SSID = WIFI_SSID_VALUE;
const char *WIFI_PASS = WIFI_PASS_VALUE;
/*************************End of Network settings*************/

/***********************MQTT topics***************************/
const char *TOPIC_CONTROL = "home/control";
const char *TOPIC_SENSORS = "home/sensors";
const char *TOPIC_STATUS = "home/smart-system/status";

/**********Actuators topics***********/
const char *TOPIC_FAN = "home/actuators/fan";
const char *TOPIC_LIGHT_FLOOR1 = "home/actuators/lights/floor1";
const char *TOPIC_LIGHT_FLOOR2 = "home/actuators/lights/floor2";
const char *TOPIC_LIGHT_RGB = "home/actuators/lights/rgb";
const char *TOPIC_BUZZER = "home/actuators/buzzer";
const char *TOPIC_MOTOR_GARAGE = "home/actuators/motors/garage";
const char *TOPIC_MOTOR_FRONT_WIN = "home/actuators/motors/frontwindow";
const char *TOPIC_MOTOR_DOOR = "home/actuators/motors/door";
const char *TOPIC_MOTOR_GATE = "home/actuators/motors/gate";
/************************************/

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

// RGB DISABLED
// void setRGB(int r, int g, int b) {
//   analogWrite(RED_PIN, r);
//   analogWrite(GREEN_PIN, g);
//   analogWrite(BLUE_PIN, b);
// }

// ------- UDP discovery of broker (copied from esp_face_detection) -------
IPAddress subnetBroadcast(IPAddress ip, IPAddress mask) {
  uint32_t ip_i = (uint32_t)ip;
  uint32_t mask_i = (uint32_t)mask;
  uint32_t bcast = ip_i | ~mask_i;
  return IPAddress(bcast);
}

bool parseAdvert(const char *json) {
  StaticJsonDocument<256> doc;
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
  udp.begin(BEACON_PORT); // bind to receive replies
  StaticJsonDocument<128> q;
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
  String topicStr = String(topic);
  /* FAN (Active LOW - LOW=ON, HIGH=OFF) */
  if (topicStr == TOPIC_FAN) {
    if (messageTemp == "in") {
      digitalWrite(FAN_OUT_PIN, HIGH);  // Turn off outward first
      digitalWrite(FAN_IN_PIN, LOW);    // Turn on inward
    } else if (messageTemp == "out") {
      digitalWrite(FAN_IN_PIN, HIGH);   // Turn off inward first
      digitalWrite(FAN_OUT_PIN, LOW);   // Turn on outward
    } else if (messageTemp == "off") {
      digitalWrite(FAN_IN_PIN, HIGH);   // Turn off both
      digitalWrite(FAN_OUT_PIN, HIGH);
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

  /* RGB - DISABLED */
  // if (topicStr == TOPIC_LIGHT_RGB) {
  //   ...
  // }

  /* BUZZER (Active LOW - LOW=ON, HIGH=OFF) */
  if (topicStr == TOPIC_BUZZER) {
    if (messageTemp == "on")
      digitalWrite(BUZZER_PIN, LOW);   // Active LOW: LOW = ON
    if (messageTemp == "off")
      digitalWrite(BUZZER_PIN, HIGH);  // Active LOW: HIGH = OFF
  }

  /* MOTORS */
  if (topicStr == TOPIC_MOTOR_GARAGE) {
    if (messageTemp == "open") {
      garageTargetAngle = garageOpenAngle;
      garageNeedsMove = true;
      Serial.printf("[GARAGE] Open requested, target=%d\n", garageTargetAngle);
    }
    if (messageTemp == "close") {
      garageTargetAngle = garageClosedAngle;
      garageNeedsMove = true;
      Serial.printf("[GARAGE] Close requested, target=%d\n", garageTargetAngle);
    }
  }

  if (topicStr == TOPIC_MOTOR_FRONT_WIN) {
    if (messageTemp == "open") {
      frontWinTargetAngle = frontWinOpenAngle;
      frontWinNeedsMove = true;
      Serial.printf("[FRONT_WIN] Open requested, target=%d\n", frontWinTargetAngle);
    } else if (messageTemp == "close") {
      frontWinTargetAngle = frontWinClosedAngle;
      frontWinNeedsMove = true;
      Serial.printf("[FRONT_WIN] Close requested, target=%d\n", frontWinTargetAngle);
    }
  }

  if (topicStr == TOPIC_MOTOR_DOOR) {
    if (messageTemp == "open") {
      doorTargetAngle = doorOpenAngle;
      doorNeedsMove = true;
      Serial.printf("[DOOR] Open requested, target=%d\n", doorTargetAngle);
    }
    else if (messageTemp == "close") {
      doorTargetAngle = doorClosedAngle;
      doorNeedsMove = true;
      Serial.printf("[DOOR] Close requested, target=%d\n", doorTargetAngle);
    }
  }

  /* MAIN GATE */
  if (topicStr == TOPIC_MOTOR_GATE) {
    if (messageTemp == "open") {
      gateTargetAngle = gateOpenAngle;
      gateNeedsMove = true;
      Serial.printf("[GATE] Open requested, target=%d, flag=%d\n", gateTargetAngle, gateNeedsMove);
    } else if (messageTemp == "close") {
      gateTargetAngle = gateClosedAngle;
      gateNeedsMove = true;
      Serial.printf("[GATE] Close requested, target=%d, flag=%d\n", gateTargetAngle, gateNeedsMove);
    }
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

  // MUST connect WiFi first before using UDP discovery!
  ensureWifi();

  if (!discoverBroker(12000)) {
    Serial.println("[MQTT] discovery failed; retry soon");
    delay(1500);
    return;
  }

  client.setServer(brokerIp, brokerPort); /*Setup connetion*/
  client.setCallback(callBack);           /*Using the callBack*/

  const char *willTopic = TOPIC_STATUS;
  const char *willMsg = "OFFLINE";

  clientId = "SmartHomeESP32-" +
             String((uint32_t)ESP.getEfuseMac(), HEX); /*Cliend ID*/
  Serial.printf("===> MQTT CONNECTING TO %s:%u AS %s <===\n",
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
    client.subscribe(TOPIC_LIGHT_RGB);       /*Subsribing to light RGB*/
    client.subscribe(TOPIC_BUZZER);          /*Subsribing to buzzer*/
    client.subscribe(TOPIC_MOTOR_GARAGE);    /*Subsribing to Carage*/
    client.subscribe(TOPIC_MOTOR_FRONT_WIN); /*Subsribing to front window*/
    client.subscribe(TOPIC_MOTOR_DOOR);      /*Subsribing to motor door*/
    client.subscribe(TOPIC_MOTOR_GATE);       /*Subscribing to main gate*/
  } else {
    Serial.printf("===>MQTT FAILED, rc | %d <===\n", client.state());
  }
}

/************************Gate and Window Control Functions********************/

void initGateServos() {
  servoGateLeft.setPeriodHertz(50);
  servoGateRight.setPeriodHertz(50);
  
  servoGateLeft.attach(SERVO_GATE_LEFT_PIN, SERVO_MIN_US, SERVO_MAX_US);
  servoGateRight.attach(SERVO_GATE_RIGHT_PIN, SERVO_MIN_US, SERVO_MAX_US);
  
  // Initialize current position from closed angle
  gateCurrentAngle = gateClosedAngle;
  gateCurrentUs = map(gateClosedAngle, 0, 180, SERVO_MIN_US, SERVO_MAX_US);
  
  // Calculate the starting position for the mirrored right door
  int invertedUs = map(gateCurrentUs, SERVO_MIN_US, SERVO_MAX_US, SERVO_MAX_US, SERVO_MIN_US);
  
  // Start at closed position
  servoGateLeft.writeMicroseconds(gateCurrentUs);
  servoGateRight.writeMicroseconds(invertedUs);
  
  Serial.println("--- Main Gate Servos Initialized ---");
}

void initFrontWindowServos() {
  servoFrontWinLeft.setPeriodHertz(50);
  servoFrontWinRight.setPeriodHertz(50);
  
  servoFrontWinLeft.attach(SERVO_FRONT_WIN_LEFT_PIN, SERVO_MIN_US, SERVO_MAX_US);
  servoFrontWinRight.attach(SERVO_FRONT_WIN_RIGHT_PIN, SERVO_MIN_US, SERVO_MAX_US);
  
  // Initialize current position from closed angle
  frontWinCurrentAngle = frontWinClosedAngle;
  frontWinCurrentUs = map(frontWinClosedAngle, 0, 180, SERVO_MIN_US, SERVO_MAX_US);
  
  // Calculate starting positions with offset
  int startRightUs = getFrontWinRightUs(frontWinCurrentUs);
  
  servoFrontWinLeft.writeMicroseconds(frontWinCurrentUs);
  servoFrontWinRight.writeMicroseconds(startRightUs);
  
  Serial.println("--- Front Window Servos Initialized ---");
}

void moveGateTo(int targetAngle) {
  targetAngle = constrain(targetAngle, 0, 180);
  int targetUs = map(targetAngle, 0, 180, SERVO_MIN_US, SERVO_MAX_US);

  Serial.print("Moving gate to angle: ");
  Serial.println(targetAngle);

  // Moving FORWARD (opening)
  if (gateCurrentUs < targetUs) {
    for (int us = gateCurrentUs; us <= targetUs; us += gateUsStep) {
      servoGateLeft.writeMicroseconds(us);
      // Calculate the exact opposite microsecond for the right door
      int rightUs = map(us, SERVO_MIN_US, SERVO_MAX_US, SERVO_MAX_US, SERVO_MIN_US);
      servoGateRight.writeMicroseconds(rightUs);
      delay(gateStepDelay);
      yield();  // Feed watchdog to prevent WDT timeout
    }
  }
  // Moving BACKWARD (closing)
  else if (gateCurrentUs > targetUs) {
    for (int us = gateCurrentUs; us >= targetUs; us -= gateUsStep) {
      servoGateLeft.writeMicroseconds(us);
      int rightUs = map(us, SERVO_MIN_US, SERVO_MAX_US, SERVO_MAX_US, SERVO_MIN_US);
      servoGateRight.writeMicroseconds(rightUs);
      delay(gateStepDelay);
      yield();  // Feed watchdog to prevent WDT timeout
    }
  }

  // Lock both exactly onto the final target
  int finalRightUs = map(targetUs, SERVO_MIN_US, SERVO_MAX_US, SERVO_MAX_US, SERVO_MIN_US);
  servoGateLeft.writeMicroseconds(targetUs);
  servoGateRight.writeMicroseconds(finalRightUs);

  gateCurrentAngle = targetAngle;
  gateCurrentUs = targetUs;
}

void openGate() {
  moveGateTo(gateOpenAngle);
}

void closeGate() {
  moveGateTo(gateClosedAngle);
}

int getFrontWinRightUs(int leftUs) {
  // 1. Calculate the mirror image (standard inverted behavior)
  int mirroredUs = map(leftUs, SERVO_MIN_US, SERVO_MAX_US, SERVO_MAX_US, SERVO_MIN_US);
  
  // 2. Calculate what the offset angle looks like in pulse width
  int offsetUs = map(frontWinOffsetAngle, 0, 180, 0, SERVO_MAX_US - SERVO_MIN_US);
  
  // 3. Add the offset (can change '+' to '-' if it moves the wrong way)
  return constrain(mirroredUs + offsetUs, SERVO_MIN_US, SERVO_MAX_US);
}

void moveFrontWindowTo(int targetAngle) {
  targetAngle = constrain(targetAngle, 0, 180);
  int targetUs = map(targetAngle, 0, 180, SERVO_MIN_US, SERVO_MAX_US);

  Serial.print("Moving front window to angle: ");
  Serial.println(targetAngle);

  // Moving FORWARD
  if (frontWinCurrentUs < targetUs) {
    for (int us = frontWinCurrentUs; us <= targetUs; us += frontWinUsStep) {
      servoFrontWinLeft.writeMicroseconds(us);
      servoFrontWinRight.writeMicroseconds(getFrontWinRightUs(us));
      delay(frontWinStepDelay);
    }
  }
  // Moving BACKWARD
  else if (frontWinCurrentUs > targetUs) {
    for (int us = frontWinCurrentUs; us >= targetUs; us -= frontWinUsStep) {
      servoFrontWinLeft.writeMicroseconds(us);
      servoFrontWinRight.writeMicroseconds(getFrontWinRightUs(us));
      delay(frontWinStepDelay);
    }
  }

  // Final Lock
  servoFrontWinLeft.writeMicroseconds(targetUs);
  servoFrontWinRight.writeMicroseconds(getFrontWinRightUs(targetUs));

  frontWinCurrentAngle = targetAngle;
  frontWinCurrentUs = targetUs;
}

void openFrontWindow() {
  moveFrontWindowTo(frontWinOpenAngle);
}

void closeFrontWindow() {
  moveFrontWindowTo(frontWinClosedAngle);
}

void processServoCommands() {
  // Process gate movement if requested
  if (gateNeedsMove) {
    Serial.printf("[SERVO] Processing gate move to %d\n", gateTargetAngle);
    gateNeedsMove = false;  // Clear flag first to allow new commands
    moveGateTo(gateTargetAngle);
    Serial.println("[SERVO] Gate move complete");
  }
  
  // Process front window movement if requested
  if (frontWinNeedsMove) {
    Serial.printf("[SERVO] Processing front window move to %d\n", frontWinTargetAngle);
    frontWinNeedsMove = false;  // Clear flag first to allow new commands
    moveFrontWindowTo(frontWinTargetAngle);
    Serial.println("[SERVO] Front window move complete");
  }
  
  // Process door movement if requested
  if (doorNeedsMove) {
    Serial.printf("[SERVO] Processing door move to %d\n", doorTargetAngle);
    doorNeedsMove = false;
    moveDoorTo(doorTargetAngle);
    Serial.println("[SERVO] Door move complete");
  }
  
  // Process garage movement if requested
  if (garageNeedsMove) {
    Serial.printf("[SERVO] Processing garage move to %d\n", garageTargetAngle);
    garageNeedsMove = false;
    moveGarageTo(garageTargetAngle);
    Serial.println("[SERVO] Garage move complete");
  }
}

// Door smooth movement function
void moveDoorTo(int targetAngle) {
  targetAngle = constrain(targetAngle, 0, 180);
  int targetUs = map(targetAngle, 0, 180, SERVO_MIN_US, SERVO_MAX_US);

  Serial.print("Moving door to angle: ");
  Serial.println(targetAngle);

  // Moving FORWARD
  if (doorCurrentUs < targetUs) {
    for (int us = doorCurrentUs; us <= targetUs; us += doorUsStep) {
      servoDoor.writeMicroseconds(us);
      delay(doorStepDelay);
      yield();
    }
  }
  // Moving BACKWARD
  else if (doorCurrentUs > targetUs) {
    for (int us = doorCurrentUs; us >= targetUs; us -= doorUsStep) {
      servoDoor.writeMicroseconds(us);
      delay(doorStepDelay);
      yield();
    }
  }

  // Final Lock
  servoDoor.writeMicroseconds(targetUs);
  doorCurrentAngle = targetAngle;
  doorCurrentUs = targetUs;
}

// Garage smooth movement function
void moveGarageTo(int targetAngle) {
  targetAngle = constrain(targetAngle, 0, 180);
  int targetUs = map(targetAngle, 0, 180, SERVO_MIN_US, SERVO_MAX_US);

  Serial.print("Moving garage to angle: ");
  Serial.println(targetAngle);

  // Moving FORWARD
  if (garageCurrentUs < targetUs) {
    for (int us = garageCurrentUs; us <= targetUs; us += garageUsStep) {
      servoGarage.writeMicroseconds(us);
      delay(garageStepDelay);
      yield();
    }
  }
  // Moving BACKWARD
  else if (garageCurrentUs > targetUs) {
    for (int us = garageCurrentUs; us >= targetUs; us -= garageUsStep) {
      servoGarage.writeMicroseconds(us);
      delay(garageStepDelay);
      yield();
    }
  }

  // Final Lock
  servoGarage.writeMicroseconds(targetUs);
  garageCurrentAngle = targetAngle;
  garageCurrentUs = targetUs;
}

/************************End of functions' definition**************************/
