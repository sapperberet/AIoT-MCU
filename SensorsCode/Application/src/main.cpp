
#include "../include/application.h"

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Initialize DHT sensor (DHTesp for ESP32 compatibility)
  delay(2000);
  dht.setup(DHT_PIN, DHT_SENSOR_TYPE);
  
  // Initialize output pins
  pinMode(FAN_IN_PIN, OUTPUT);
  pinMode(FAN_OUT_PIN, OUTPUT);
  digitalWrite(FAN_IN_PIN, HIGH);   // Start OFF (active LOW)
  digitalWrite(FAN_OUT_PIN, HIGH);  // Start OFF (active LOW)
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, HIGH);   // Start OFF (active LOW)
  pinMode(LED_FLOOR1, OUTPUT);
  pinMode(LED_FLOOR2, OUTPUT);
  digitalWrite(LED_FLOOR1, HIGH);   // Start OFF (active LOW)
  digitalWrite(LED_FLOOR2, HIGH);   // Start OFF (active LOW)
  
  // Initialize input pins
  pinMode(FLAME_PIN, INPUT);
  pinMode(RAIN_PIN, INPUT);
  pinMode(VOLTAGE_PIN, INPUT);
  pinMode(CURRENT_PIN, INPUT);

  // Start network and remote access before first application log
  ensureWifi();
  setupRemoteAccess();
  remoteLog("=== Smart Home ESP32 Starting ===");
  remoteLogf("WiFi OK. IP: %s", WiFi.localIP().toString().c_str());
  
  // Initialize servos
  initGateServos();
  initFrontWindowServos();
  initGarageServo();
  initDoorServo();
  initRGB();

  // Connect MQTT after network and WebSerial are ready
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
    int rainRaw = digitalRead(RAIN_PIN);
    int rain = (rainRaw == LOW) ? 1 : 0; // Dry=0, Wet=1 (active-LOW rain modules)
    int voltageRaw = analogRead(VOLTAGE_PIN);
    int currentRaw = analogRead(CURRENT_PIN);

    float sensedVoltage = (static_cast<float>(voltageRaw) / ADC_MAX_VALUE) *
                          ADC_REF_VOLTAGE * VOLTAGE_DIVIDER_RATIO;
    float currentVoltage = (static_cast<float>(currentRaw) / ADC_MAX_VALUE) *
                           ADC_REF_VOLTAGE;
    float sensedCurrent = (currentVoltage - CURRENT_ZERO_VOLTAGE) /
                          CURRENT_SENSITIVITY;
    if (fabsf(sensedCurrent) < CURRENT_NOISE_THRESHOLD) {
      sensedCurrent = 0.0f;
    }
    
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

      // Voltage
      snprintf(buf, sizeof(buf), "%.2f", sensedVoltage);
      client.publish(TOPIC_VOLTAGE, buf);

      // Current
      snprintf(buf, sizeof(buf), "%.2f", sensedCurrent);
      client.publish(TOPIC_CURRENT, buf);
    }
    
    remoteLogf("Sensors: T=%.1f H=%.1f MQ=%d Flame=%d Light=%d Rain=%d V=%.2f I=%.2f",
           t, h, mq, flame, light, rain, sensedVoltage, sensedCurrent);
  }
  
  delay(10);
}
