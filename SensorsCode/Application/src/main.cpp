
#include "../include/application.h"

void setup() {
  Serial.begin(115200);
  delay(1000);
  remoteLog("=== Smart Home ESP32 Starting ===");
  
  // Initialize DHT sensor (DHTesp for ESP32 compatibility)
  dht.setup(DHT_PIN, DHTesp::DHT22);
  
  // Initialize output pins
  pinMode(FAN_IN_PIN, OUTPUT);
  pinMode(FAN_OUT_PIN, OUTPUT);
  digitalWrite(FAN_IN_PIN, HIGH);   // Start OFF (active LOW)
  digitalWrite(FAN_OUT_PIN, HIGH);  // Start OFF (active LOW)
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, HIGH);   // Start OFF (active LOW)
  pinMode(LED_FLOOR1, OUTPUT);
  pinMode(LED_FLOOR2, OUTPUT);
  
  // Initialize input pins
  pinMode(FLAME_PIN, INPUT);
  pinMode(RAIN_PIN, INPUT);
  
  // Initialize servos
  initGateServos();
  initFrontWindowServos();
  initGarageServo();
  
  // Initialize door servo with smooth movement support
  servoDoor.attach(SERVO_DOOR_PIN, SERVO_MIN_US, SERVO_MAX_US);
  doorCurrentUs = map(doorClosedAngle, 0, 180, SERVO_MIN_US, SERVO_MAX_US);
  servoDoor.writeMicroseconds(doorCurrentUs);  // Start closed at 150 degrees
  remoteLog("--- Door Servo Initialized ---");
  
  // Connect WiFi first, then start OTA/WebSerial, then MQTT
  ensureWifi();
  setupRemoteAccess();
  ensureMqtt();
  
  remoteLog("=== Setup Complete ===");
}

void loop() {
  static unsigned long lastSensorPublish = 0;
  static unsigned long lastReconnect = 0;
  unsigned long now = millis();
  
  // MQTT reconnect with throttling
  if (!client.connected() && (now - lastReconnect > 5000)) {
    lastReconnect = now;
    remoteLog("[MQTT] Not connected, calling ensureMqtt()...");
    ensureMqtt();
  }

  handleRemoteAccess();
  
  // Process MQTT messages
  if (client.connected()) {
    client.loop();
  }
  
  // Process any pending servo movements (non-blocking)
  processServoCommands();
  
  // Publish sensors every 5 seconds
  if (now - lastSensorPublish >= 5000) {
    lastSensorPublish = now;
    
    // Read DHT (ESP32-safe library)
    TempAndHumidity data = dht.getTempAndHumidity();
    float t = data.temperature;
    float h = data.humidity;
    if (isnan(t) || isnan(h)) {
      remoteLogf("[DHT] Read failed (status=%d)", dht.getStatus());
    }
    
    // Read other sensors
    int mq = analogRead(MQ135_PIN);
    int flame = digitalRead(FLAME_PIN);
    int light = analogRead(LDR_PIN);
    int rain = digitalRead(RAIN_PIN);
    
    // Publish each sensor to its own topic
    if (client.connected()) {
      char buf[16];
      
      // Temperature
      snprintf(buf, sizeof(buf), "%.1f", isnan(t) ? 0.0f : t);
      client.publish(TOPIC_TEMPERATURE, buf);
      
      // Humidity
      snprintf(buf, sizeof(buf), "%.1f", isnan(h) ? 0.0f : h);
      client.publish(TOPIC_HUMIDITY, buf);
      
      // Gas (MQ135)
      snprintf(buf, sizeof(buf), "%d", mq);
      client.publish(TOPIC_GAS, buf);
      
      // Flame
      snprintf(buf, sizeof(buf), "%d", flame);
      client.publish(TOPIC_FLAME, buf);
      
      // Light (LDR)
      snprintf(buf, sizeof(buf), "%d", light);
      client.publish(TOPIC_LDR, buf);
      
      // Rain
      snprintf(buf, sizeof(buf), "%d", rain);
      client.publish(TOPIC_RAIN, buf);
    }
    
    remoteLogf("Sensors: T=%.1f H=%.1f MQ=%d Flame=%d Light=%d Rain=%d",
           t, h, mq, flame, light, rain);
  }
  
  delay(10);
}
