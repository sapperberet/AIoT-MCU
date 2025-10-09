#include <WiFi.h>
#include <DHT.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MQ135.h>
#include <Servo.h>
//INPUT and OUTPUT PINS
// SENSORS
#define MQ135_PIN 34        // Smoke Sensor
#define DHT_PIN 4           // Temp & Humidity Sensor
#define LDR_PIN 35          // Light Sensor
#define RAIN_PIN 32         // Rain Sensor
#define FAN_ENA 33
#define FAN_IN1 12
#define FAN_IN2 13
#define BUZZER_PIN 23
//RELAY and LEDS
#define LED_FLOOR1 25       // LIGHT FLOOR NUMBER(1)
#define LED_FLOOR2 26       // LIGHT FLOOR NUMBER(2)
#define LED_LANDSCAPE 27    // LIGHTS FOR (Landscape)
// ALL SERVOS
Servo servoWindow1;
Servo servoWindow2;
Servo servoWindow3;
Servo servoDoor;
Servo servoGarage;
servo servogate_gage;
//PINS FOR SERVOS MOTORS
#define SERVO_WINDOW1_PIN 15
#define SERVO_WINDOW2_PIN 16
#define SERVO_WINDOW3_PIN 17
#define SERVO_DOOR_PIN 18
#define SERVO_GARAGE_PIN 19
#define servo_gate_gage_PIN 21
//SETUP FOR SENSORS
#define DHT_TYPE DHT22
DHT dht(DHT_PIN, DHT_TYPE);
Adafruit_MQ135 mq135 = Adafruit_MQ135(MQ135_PIN);
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(115200);
  dht.begin();
  // OUTPUTS
  pinMode(FAN_ENA, OUTPUT);
  pinMode(FAN_IN1, OUTPUT);
  pinMode(FAN_IN2, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  // INPUTS
  pinMode(MQ135_PIN, INPUT);
  pinMode(LDR_PIN, INPUT);
  pinMode(RAIN_PIN, INPUT);
  // LIGHTING OUTPUTS
  pinMode(LED_FLOOR1, OUTPUT);
  pinMode(LED_FLOOR2, OUTPUT);
  pinMode(LED_LANDSCAPE, OUTPUT);
  //SERVOS
  servoWindow1.attach(SERVO_WINDOW1_PIN);
  servoWindow2.attach(SERVO_WINDOW2_PIN);
  servoWindow3.attach(SERVO_WINDOW3_PIN);
  servoDoor.attach(SERVO_DOOR_PIN);
  servoGarage.attach(SERVO_GARAGE_PIN);
  servo_gate_gage.attach(servo_gate_gage_PIN);
  Serial.println("Smart Home (3 Windows + 3 LED Zones) Initialized!");
}
// MAIN LOOPS
void loop() {
  // READING ALL VALUES FOR ALL SENSORS
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();
  int smoke = analogRead(MQ135_PIN);
  int lightLevel = analogRead(LDR_PIN);
  int rain = digitalRead(RAIN_PIN);
  // SHOW THE OUTPUT OF SENSORS IN A SERIALS
  Serial.println("------ SENSOR DATA ------");
  Serial.print("Temp: "); Serial.println(temperature);
  Serial.print("Humidity: "); Serial.println(humidity);
  Serial.print("Smoke: "); Serial.println(smoke);
  Serial.print("Light: "); Serial.println(lightLevel);
  Serial.print("Rain: "); Serial.println(rain);
  Serial.println("-------------------------");
  //SMOKE ALARM
  if (smoke > 2000) {
    digitalWrite(BUZZER_PIN, HIGH);
    Serial.println("Smoke Detected!");
  } else {
    digitalWrite(BUZZER_PIN, LOW);
  }
  // CONTROLLING ON ALL WINDOW WHEN RAINING
  if (rain == LOW) {
    // IF IT RAIN ALL SERVOS IS OFF
    servoWindow1.write(0);
    servoWindow2.write(0);
    servoWindow3.write(0);
    Serial.println("Rain Detected → Closing All Windows");
  } else {
    // WHEN THE WEATHER IS GOOD ALL SERVOS IS ON
    servoWindow1.write(90);
    servoWindow2.write(90);
    servoWindow3.write(90);
  }
  // CONTROLLING IN ALL FANS
  if (temperature > 30) {
    analogWrite(FAN_ENA, 200);
    digitalWrite(FAN_IN1, HIGH);
    digitalWrite(FAN_IN2, LOW);
    Serial.println("High Temp: Fan ON");
  } else {
    analogWrite(FAN_ENA, 0);
  }
  //CONTROLLING IN ALL LIGHTS
  if (lightLevel < 1000) {
    // DARK MOOD
    digitalWrite(LED_FLOOR1, HIGH);
    digitalWrite(LED_FLOOR2, HIGH);
    digitalWrite(LED_LANDSCAPE, HIGH);
    Serial.println("Night Mode → All Lights ON");
  } else {
    // LIGHT MODE
    digitalWrite(LED_FLOOR1, LOW);
    digitalWrite(LED_FLOOR2, LOW);
    digitalWrite(LED_LANDSCAPE, LOW);
    Serial.println("Daytime → All Lights OFF");
  }
  delay(2000);
}
