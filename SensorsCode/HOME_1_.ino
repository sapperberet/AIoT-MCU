#include <WiFi.h>
#include <WiFiUdp.h>        // <--- (Required for compatibility)
#include <PubSubClient.h> 
#include <ArduinoJson.h>    // <--- (Required for compatibility)
#include <DHT.h>
#include <Adafruit_Sensor.h>
#include <MQ135.h>
#include <Servo.h>

//=========================================================
// ===== Devices (From your original code) =====
//=========================================================
//INPUT and OUTPUT PINS
// SENSORS
#define MQ135_PIN 34 
#define FLAME_PIN 14 
#define DHT_PIN 4 
#define LDR_PIN 35 
#define RAIN_PIN 32 
#define FAN_ENA 33
#define FAN_IN1 5
#define FAN_IN2 13
#define BUZZER_PIN 23

//RELAY and LEDS
#define LED_FLOOR1 25
#define LED_FLOOR2 26
#define LED_LANDSCAPE 27

// RGB STRIP PINS
#define RED_PIN 22
#define GREEN_PIN 2
#define BLUE_PIN 12

// ALL SERVOS
Servo servoWindow1;
Servo servoWindow2;
Servo servoDoor;
Servo servoGarage;
Servo servoGate;

//PINS FOR SERVOS MOTORS
#define SERVO_WINDOW1_PIN 15
#define SERVO_WINDOW2_PIN 16
#define SERVO_DOOR_PIN 18
#define SERVO_GARAGE_PIN 19
#define SERVO_GATE_PIN 21

//SETUP FOR SENSORS
#define DHT_TYPE DHT22
DHT dht(DHT_PIN, DHT_TYPE);
MQ135 mq135 = MQ135(MQ135_PIN);

// SIMULATED FLAG FOR SOLAR PANEL STATUS
bool solarActive = false;


//=========================================================
// ===== Network Settings (For compatibility with the advanced code) =====
//=========================================================
const char* WIFI_SSID = "Your_WiFi_Name";     // <--- Enter your network name
const char* WIFI_PASS = "Your_WiFi_Password"; // <--- Enter your network password

// UDP beacon settings (Must match the advanced code)
const uint16_t BEACON_PORT = 18830;     // <--- (Matches advanced code)
const char* BEACON_NAME = "face-broker"; // <--- (Matches advanced code)

// MQTT topics (Specific to this code)
const char* TOPIC_CONTROL = "home/control"; // <--- (To receive commands)
const char* TOPIC_SENSORS = "home/sensors"; // <--- (To send sensor data)
const char* TOPIC_STATUS  = "home/smart-system/status"; // (LWT Topic for this device)

WiFiUDP udp;
WiFiClient net; // (Instead of espClient)
PubSubClient client(net); // (Instead of client(espClient))
IPAddress brokerIp;
uint16_t brokerPort = 1883;
String clientId;
unsigned long lastReconnectAttempt = 0;
unsigned long lastSensorPublish = 0; // (Timer alternative for delay)


//=========================================================
// ===== Helper Functions (From original code) =====
//=========================================================
void setRGB(int r, int g, int b) {
  analogWrite(RED_PIN, r);
  analogWrite(GREEN_PIN, g);
  analogWrite(BLUE_PIN, b);
}

//=========================================================
// ===== Server Discovery Functions (Copied for compatibility) =====
//=========================================================
IPAddress subnetBroadcast(IPAddress ip, IPAddress mask) {
  uint32_t ip_i   = (uint32_t)ip;
  uint32_t mask_i = (uint32_t)mask;
  uint32_t bcast  = ip_i | ~mask_i;
  return IPAddress(bcast);
}

bool parseAdvert(const char* json) {
  JsonDocument doc;
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
  udp.begin(BEACON_PORT); 
  JsonDocument q;
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
      udp.beginPacket(bcast, BEACON_PORT);                   udp.write((const uint8_t*)qbuf, qlen); udp.endPacket();
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
// ------- End of Discovery Functions -------


//=========================================================
// ===== Connection Functions (Modified for compatibility) =====
//=========================================================
void ensureWifi() {
  if (WiFi.status() == WL_CONNECTED) return;
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("WiFi connecting");
  while (WiFi.status() != WL_CONNECTED) { delay(250); Serial.printf("."); }
  Serial.printf("\nWiFi OK. IP: %s\n", WiFi.localIP().toString().c_str());
}

// MQTT Callback Function (Your original callback function)
void callback(char* topic, byte* message, unsigned int length) {
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
    } 
    else if (messageTemp == "lights_off") {
      digitalWrite(LED_FLOOR1, LOW);
      digitalWrite(LED_FLOOR2, LOW);
      digitalWrite(LED_LANDSCAPE, LOW);
    }
    else if (messageTemp == "fan_on") {
      analogWrite(FAN_ENA, 200);
      digitalWrite(FAN_IN1, HIGH);
      digitalWrite(FAN_IN2, LOW);
    }
    else if (messageTemp == "fan_off") {
      analogWrite(FAN_ENA, 0);
    }
    // ... (You can add other commands here, e.g., servos)
  }
}

// Connection function (Modified for this code)
void ensureMqtt() {
  if (client.connected()) return;

  // 1. Discover server (Instead of static IP)
  if (!discoverBroker(12000)) {
    Serial.println("[MQTT] discovery failed; retry soon");
    delay(1500);
    return;
  }
  
  // 2. Setup connection
  client.setServer(brokerIp, brokerPort);
  client.setCallback(callback); // <--- Using your callback

  const char* willTopic = TOPIC_STATUS;
  const char* willMsg   = "OFFLINE";

  // <--- (Important) Unique Client ID, different from the first code
  clientId = "SmartHomeESP32-" + String((uint32_t)ESP.getEfuseMac(), HEX); 
  Serial.printf("[MQTT] connect to %s:%u as %s\n",
                brokerIp.toString().c_str(), brokerPort, clientId.c_str());

  // 3. Connect with LWT
  bool ok = client.connect(
    clientId.c_str(),
    nullptr, nullptr,
    willTopic, 0, true, willMsg
  );

  if (ok) {
    Serial.println("[MQTT] connected");
    client.subscribe(TOPIC_CONTROL); // <--- Subscribing to your topic
    client.publish(TOPIC_STATUS, "ONLINE", true);
  } else {
    Serial.printf("[MQTT] failed, rc=%d\n", client.state());
  }
}

/******************************************************************************/
/* The following is the setup function, which will run once in the program */
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

  Serial.println("Smart Home (Advanced MQTT + Discovery) Initialized!");

  // (Connecting using the new method - instead of the old code)
  ensureWifi();
  ensureMqtt();
}
/******************************************************************************/


/******************************************************************************/
/* The follosing is the loop function, which will loop over and over through the program */
void loop() {
  // (Connecting using the new method - Non-Blocking)
  if (WiFi.status() != WL_CONNECTED) ensureWifi();

  unsigned long now = millis(); // Get time once at the start

  if (!client.connected()) {
    if (now - lastReconnectAttempt > 5000) { // Retry every 5 seconds
      lastReconnectAttempt = now;
      ensureMqtt();
    }
  } else {
    client.loop(); // (Necessary for processing messages)
  }

  //
  // ===== (All your original logic starts here) =====
  //
  
  // (We will use a timer instead of delay(2000) to send data)
  if (now - lastSensorPublish > 2000) { // Send every 2 seconds
    lastSensorPublish = now;

    // READING ALL VALUES FOR ALL SENSORS
    float humidity = dht.readHumidity();
    float temperature = dht.readTemperature();
    int smoke = analogRead(MQ135_PIN);
    int flame = digitalRead(FLAME_PIN);
    int lightLevel = analogRead(LDR_PIN);
    int rain = digitalRead(RAIN_PIN);

    Serial.printf("------ SENSORS DATA ------\n");
    Serial.printf("Temp: "); Serial.println(temperature);
    Serial.printf("Humidity: "); Serial.println(humidity);
    Serial.printf("Smoke: "); Serial.println(smoke);
    Serial.printf("Flame: "); Serial.println(flame);
    Serial.printf("Light: "); Serial.println(lightLevel);
    Serial.printf("Rain: "); Serial.println(rain);
    Serial.println("-------------------------");

    // SMOKE & FLAME ALARM
    if (smoke > 2000 || flame == LOW) {
      digitalWrite(BUZZER_PIN, HIGH);
      setRGB(255, 0, 0); //  Red = Fire Mode
      Serial.println(" Warning! Smoke or Flame Detected!");
    } else {
      digitalWrite(BUZZER_PIN, LOW);

      // WINDOW CONTROL (RAIN)
      if (rain == LOW) {
        servoWindow1.write(0);
        servoWindow2.write(0);
        Serial.println("🌧 Rain Detected → Closing All Windows");
      } else {
        servoWindow1.write(90);
        servoWindow2.write(90);
      }

      // FAN CONTROL
      if (temperature > 30) {
        analogWrite(FAN_ENA, 200);
        digitalWrite(FAN_IN1, HIGH);
        digitalWrite(FAN_IN2, LOW);
        Serial.println(" High Temp: Fan ON");
      } else {
        analogWrite(FAN_ENA, 0);
      }

      // LIGHTING CONTROL + MODE
      if (lightLevel < 1000) {
        digitalWrite(LED_FLOOR1, HIGH);
        digitalWrite(LED_FLOOR2, HIGH);
        digitalWrite(LED_LANDSCAPE, HIGH);
        setRGB(0, 0, 255); //  Blue = Night Mode
        Serial.println(" Night Mode → All Lights ON");
      } else {
        digitalWrite(LED_FLOOR1, LOW);
        digitalWrite(LED_FLOOR2, LOW);
        digitalWrite(LED_LANDSCAPE, LOW);
        setRGB(255, 255, 0); //  Yellow = Day Mode
        Serial.println(" Daytime → All Lights OFF");
      }

      // SOLAR PANEL STATUS (SIMULATED)
      if (solarActive) {
        setRGB(0, 255, 0); //  Green = Solar Power Active
        Serial.println(" Solar Energy Active");
      }
    }

    // SEND THE DATA FOR THE SENSOR TO THE SERVER 
    if (client.connected()) {
      String payload = String("{\"temperature\":") + temperature +
                       ",\"humidity\":" + humidity +
                       ",\"smoke\":" + smoke +
                       ",\"light\":" + lightLevel +
                       ",\"rain\":" + rain +
                       ",\"flame\":" + flame + "}";
      client.publish(TOPIC_SENSORS, payload.c_str());
    }
  }
}
/******************************************************************************/
