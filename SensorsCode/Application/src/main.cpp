
#include "../include/application.h"

void setup() {
  Serial.begin(115200); /* Beginning serial monitor to baud 115200 */
  dht.begin();          /* beginning the dht sensor */

  /****************Initializing the mode of each pin***************************/
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
  pinMode(PUSH_BUTTON, INPUT_PULLUP);

  servoWindow1.attach(SERVO_WINDOW1_PIN);
  servoWindow2.attach(SERVO_WINDOW2_PIN);
  servoDoor.attach(SERVO_DOOR_PIN);
  servoGarage.attach(SERVO_GARAGE_PIN);
  servoGate.attach(SERVO_GATE_PIN);
  /******************************************************************************/

  Serial.println("===> Initialized! INITIALIZING MQTT AND WIFI <===");

  ensureWifi(); /* Connecting Wifi */
  ensureMqtt(); /* Connecting MQTT */
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) { /* Checking if the wifi is connected */
    Serial.printf("===> WIFI NOT CONNECTED <===\n");
    Serial.printf("===> TRYING TO CONNECT TO (%s) <===\n", WIFI_SSID);
    ensureWifi();
  }

  unsigned long now = millis(); /* Getting th time at start */

  if (!(client.connected())) {
    if ((now - lastReconnectAttempt) >
        5000) {                   /* Retry to connect every 5 seconds */
      lastReconnectAttempt = now; /* Current time */
      ensureMqtt();
    } else { /* DO NOTHING */
    }
  } else {
    client.loop(); /* For processing messages */
  }

  if ((now - lastSensorPublish) > 1000) { /* Sending data every 1 seconds */
    lastSensorPublish = now;

    /* Reading sensors values */
    float humidity = dht.readHumidity();       /* Humidity */
    float temperature = dht.readTemperature(); /* Temperature */
    int smoke = analogRead(MQ135_PIN);         /* Smoke */
    int flame = digitalRead(FLAME_PIN);        /* Flame */
    int lightLevel = analogRead(LDR_PIN);      /* Light level */
    int rain = digitalRead(RAIN_PIN);          /* Rain */
    bool faceDetectionRun = buttonPressed(PUSH_BUTTON);  /* Checks if the push button is pressed */

    // /* Monitoring all sensors data */
    // Serial.printf("------ SENSORS DATA ------\n");
    // Serial.printf("Temp: %0.3f\n", temperature);
    // Serial.printf("Humidity: %0.3f\n", humidity);
    // Serial.printf("Smoke: %i\n", smoke);
    // Serial.printf("Flame: %i\n", flame);
    // Serial.printf("Light: %i\n", lightLevel);
    // Serial.printf("Rain: %i\n", rain);
    // Serial.printf("-------------------------\n");

    // /* Smoke and flame alarm */
    // if (smoke > 2000 || flame == LOW) { /* True */
    //   digitalWrite(BUZZER_PIN, HIGH);
    //   setRGB(255, 0, 0); /* Red: fire mode */
    //   Serial.println("===> WARNING!! SMOKE OR FLAME DETECTED <===");
    // } else { /* False */
    //   digitalWrite(BUZZER_PIN, LOW);

    //   /* Controlling windows */
    //   if (rain == LOW) { /* There is rain */
    //     servoWindow1.write(0);
    //     servoWindow2.write(0);
    //     Serial.println("===> RAIN DETECTED || CLOSING ALL WINDOWS <===");
    //   } else { /* There is no rain */
    //     servoWindow1.write(90);
    //     servoWindow2.write(90);
    //   }

    //   /* Controlling fan */
    //   if (temperature > 30) { /* Hot temperature */
    //     analogWrite(FAN_ENA, 200);
    //     digitalWrite(FAN_IN1, HIGH);
    //     digitalWrite(FAN_IN2, LOW);
    //     Serial.printf("===> TEMPERATURE: %0.3f || FAN ON <===\n", temperature);
    //   } else { /* Normal temperature */
    //     analogWrite(FAN_ENA, 0);
    //     Serial.printf("===> TEMPERATURE: %0.3f || FAN OFF <===\n", temperature);
    //   }

    //   /* Controlling lighting and the mode */
    //   if (lightLevel < 1000) { /* Night mode */
    //     digitalWrite(LED_FLOOR1, HIGH);
    //     digitalWrite(LED_FLOOR2, HIGH);
    //     digitalWrite(LED_LANDSCAPE, HIGH);
    //     setRGB(0, 0, 255); /* Blue: night mode */
    //     Serial.println("===> NIGHT MODE || ALL LIGHTS ON <===");
    //   } else { /* Daytime mode */
    //     digitalWrite(LED_FLOOR1, LOW);
    //     digitalWrite(LED_FLOOR2, LOW);
    //     digitalWrite(LED_LANDSCAPE, LOW);
    //     setRGB(255, 255, 0); /* Yellow: daytime mode */
    //     Serial.println("===> DAYTIME || ALL LIGHTS OFF <===");
    //   }

    //   /* Status of solar panel */
    //   if (true == solarActive) { /* Checking the power system */
    //     setRGB(0, 255, 0);       /* Green: solar mode is active */
    //     Serial.println("===> SOLAR ENERGY ACTIVE <===");
    //   } else { /* DO NOTHING*/
    //   }
    // }

    if (client.connected()) { /* Checking the connectivity of the server */
      /* Sending sensors' data to using json to the terminal */
      String payload = String("{\"temperature\":") + temperature +
                       ",\"humidity\":" + humidity + ",\"smoke\":" + smoke +
                       ",\"light\":" + lightLevel + ",\"rain\":" + rain +
                       ",\"flame\":" + flame + "}";
      client.publish(TOPIC_SENSORS, payload.c_str());

      /* Sending individual topics */
      client.publish(TOPIC_HUMIDITY, String(humidity).c_str());
      client.publish(TOPIC_GAS, String(smoke).c_str());
      client.publish(TOPIC_LDR, String(lightLevel).c_str());
      client.publish(TOPIC_RAIN, String(rain).c_str());
      client.publish(TOPIC_FLAME, String(flame).c_str());
      client.publish(TOPIC_TEMPERATURE, String(temperature).c_str());
      client.publish(TOPIC_CURRENT, "0"); /* Just for simulation */
      client.publish(TOPIC_VOLTAGE, "0"); /* Just for simulation */
      if (true == faceDetectionRun){
        client.publish(TOPIC_PUSH_BUTTON_FACE_DETECTION, String(faceDetectionRun).c_str());
      }
      else{/* DO NOTHING */}
    } else {                              /* DO NOTHING */
    }
  }
}
