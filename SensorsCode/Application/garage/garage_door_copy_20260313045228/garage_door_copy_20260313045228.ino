#include <ESP32Servo.h>

Servo leftDoor;
Servo rightDoor; 

// --- HARDWARE SETTINGS ---
#define LEFT_PIN   13  
#define RIGHT_PIN  19  

// --- TUNING SETTINGS ---
const int MIN_US = 500;   // Pulse width at 0 degrees
const int MAX_US = 2400;  // Pulse width at 180 degrees

// STARTUP: The door will snap to this angle when turned on
int currentAngle = 90; 
int currentUs; 

// --- SPEED SETTINGS ---
int usStep = 10;     
int stepDelay = 0.1;  

// --- HELPER: CALCULATE RIGHT DOOR POSITION ---
// Now it ONLY performs a perfect mirror (inverts the signal)
int getRightUs(int leftUs) {
  return map(leftUs, MIN_US, MAX_US, MAX_US, MIN_US);
}

void setup() {
  Serial.begin(115200);
  
  // Ensure the startup angle is within the absolute 0-180 limits
  currentAngle = constrain(currentAngle, 0, 180);
  
  leftDoor.setPeriodHertz(50); 
  rightDoor.setPeriodHertz(50); 
  
  // 1. ATTACH SERVOS FIRST 
  // This connects the hardware timer to the pins
  leftDoor.attach(LEFT_PIN, MIN_US, MAX_US); 
  rightDoor.attach(RIGHT_PIN, MIN_US, MAX_US); 
  
  // 2. IMMEDIATELY WRITE STARTUP POSITION
  // Because they are already attached, this forces a fast "snap" to the start angle
  currentUs = map(currentAngle, 0, 180, MIN_US, MAX_US);
  leftDoor.writeMicroseconds(currentUs);
  rightDoor.writeMicroseconds(getRightUs(currentUs));
  
  Serial.println("--- Garage Door System Ready ---");
  Serial.print("Doors snapped to mirrored angle: ");
  Serial.println(currentAngle);
}

void loop() {
  if (Serial.available() > 0) {
    
    // Read the relative move amount (e.g. typing "20" or "-20")
    int moveAmount = Serial.parseInt(); 
    
    // Clear the serial buffer
    while (Serial.available() > 0) {
      Serial.read();
    }

    if (moveAmount != 0) {
      
      int potentialAngle = currentAngle + moveAmount;
      
      // Absolute limits are now hardcoded to 0 and 180
      int targetAngle = constrain(potentialAngle, 0, 180);

      if (targetAngle == currentAngle) {
        Serial.println("Absolute limit reached! Cannot move beyond 0 or 180.");
      } else {
        Serial.print("Moving to: ");
        Serial.println(targetAngle);

        int targetUs = map(targetAngle, 0, 180, MIN_US, MAX_US);

        // --- SMOOTH GLIDE LOGIC ---
        if (currentUs < targetUs) { // OPENING
          for (int us = currentUs; us <= targetUs; us += usStep) {
            leftDoor.writeMicroseconds(us);
            rightDoor.writeMicroseconds(getRightUs(us)); 
            delay(stepDelay); 
          }
        } 
        else if (currentUs > targetUs) { // CLOSING
          for (int us = currentUs; us >= targetUs; us -= usStep) {
            leftDoor.writeMicroseconds(us);
            rightDoor.writeMicroseconds(getRightUs(us)); 
            delay(stepDelay); 
          }
        }

        // Lock final positions to prevent drift
        leftDoor.writeMicroseconds(targetUs);
        rightDoor.writeMicroseconds(getRightUs(targetUs));
        
        currentAngle = targetAngle; 
        currentUs = targetUs;
      }
    }
  }
}