#ifndef _APPLICATION_H_
#define _APPLICATOIN_H_

/***********************Files includes*****************************************/
#include "platform.h"
/******************************************************************************/

/***********************Application includes***********************************/
#include <ArduinoJson.h>
#include <DHTesp.h>  // ESP32-compatible DHT library (replaces DHT.h)
#include <ESP32Servo.h>
#include <MQ135.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <WiFiUdp.h>
/******************************************************************************/

/******************Application declrations based on MCU's type*****************/
#if (DOIT_ESP_32_DIVKIT_V1 == MCU_TYPE)

/***********************Pins diagram**************************/

/**********SENSORS PINS**********/
#define MQ135_PIN 34 /* Gas sensor */
#define FLAME_PIN 14 /* Flame sensor */
#define DHT_PIN 4    /* Temperature and humidity sensor */
#define LDR_PIN 35   /* Light sensor */
#define RAIN_PIN 32  /* Rain sensor */

/**********FAN (Active LOW)**********/
#define FAN_IN_PIN 5   /* Fan inward flow - active LOW */
#define FAN_OUT_PIN 17 /* Fan outward flow - active LOW */

/**********BUZZER**********/
#define BUZZER_PIN 23 /* Buzzer */

/**********FLOORS' LEDs**********/
#define LED_FLOOR1 25    /* First floor LED */
#define LED_FLOOR2 26    /* Second floor LED */

/**********RGB STREP - DISABLED**********/
// #define RED_PIN 13   /* R color */
// #define GREEN_PIN 16 /* G color */
// #define BLUE_PIN 27  /* B color */

/**********Servo MOTORS**********/
#define SERVO_FRONT_WIN_LEFT_PIN 2    /* Left servo for front window - GPIO 2 OK for output */
#define SERVO_FRONT_WIN_RIGHT_PIN 12  /* Right servo for front window - GPIO 12 OK for output */
#define SERVO_DOOR_PIN 18             /* Servo controlling house's door */
#define SERVO_GARAGE_PIN 19           /* Servo controlling garage's door */
#define SERVO_GATE_LEFT_PIN 21        /* Left servo for main gate */
#define SERVO_GATE_RIGHT_PIN 22       /* Right servo for main gate */

/**********Servo Pulse Width Settings**********/
#define SERVO_MIN_US 500   /* Minimum pulse width */
#define SERVO_MAX_US 2400  /* Maximum pulse width */

/**********Push button**********/
#define PUSH_BUTTON 36  /* Push button pin (input only) */

/***********************End of pins diagram*******************/

/**************************MCU globals declaration************************/

/**********SERVOS DECLARATION**********/
extern Servo servoFrontWinLeft;   /* Left servo for front window */
extern Servo servoFrontWinRight;  /* Right servo for front window */
extern Servo servoDoor;           /* Servo for the house's door */
extern Servo servoGarage;         /* Servo for the garage's door */
extern Servo servoGateLeft;       /* Left servo for main gate */
extern Servo servoGateRight;      /* Right servo for main gate */

/**********GATE SETTINGS & STATE**********/
extern int gateClosedAngle;       /* Closed position angle */
extern int gateOpenAngle;         /* Open position angle */
extern int gateUsStep;            /* Microsecond step size for smooth movement */
extern int gateStepDelay;         /* Delay between steps (ms) */
extern int gateCurrentAngle;      /* Current angle of main gate */
extern int gateCurrentUs;         /* Current pulse width of gate */
extern volatile int gateTargetAngle;  /* Target angle set by MQTT (non-blocking) */
extern volatile bool gateNeedsMove;   /* Flag indicating gate needs to move */

/**********FRONT WINDOW SETTINGS & STATE**********/
extern int frontWinClosedAngle;   /* Closed position angle */
extern int frontWinOpenAngle;     /* Open position angle */
extern int frontWinOffsetAngle;   /* Offset for right door compensation */
extern int frontWinUsStep;        /* Microsecond step size for smooth movement */
extern int frontWinStepDelay;     /* Delay between steps (ms) */
extern int frontWinCurrentAngle;  /* Current angle of front window */
extern int frontWinCurrentUs;     /* Current pulse width of front window */
extern volatile int frontWinTargetAngle; /* Target angle set by MQTT (non-blocking) */
extern volatile bool frontWinNeedsMove;  /* Flag indicating window needs to move */
/**************************************/

/**********DOOR SETTINGS & STATE**********/
extern int doorClosedAngle;       /* Closed position angle */
extern int doorOpenAngle;         /* Open position angle */
extern int doorUsStep;            /* Microsecond step size for smooth movement */
extern int doorStepDelay;         /* Delay between steps (ms) */
extern int doorCurrentAngle;      /* Current angle of door */
extern int doorCurrentUs;         /* Current pulse width of door */
extern volatile int doorTargetAngle;  /* Target angle set by MQTT */
extern volatile bool doorNeedsMove;   /* Flag indicating door needs to move */
/**************************************/

/**********GARAGE SETTINGS & STATE**********/
extern int garageClosedAngle;     /* Closed position angle */
extern int garageOpenAngle;       /* Open position angle */
extern int garageUsStep;          /* Microsecond step size for smooth movement */
extern int garageStepDelay;       /* Delay between steps (ms) */
extern int garageCurrentAngle;    /* Current angle of garage */
extern int garageCurrentUs;       /* Current pulse width of garage */
extern volatile int garageTargetAngle;  /* Target angle set by MQTT */
extern volatile bool garageNeedsMove;   /* Flag indicating garage needs to move */
/**************************************/

/**********GLOBALS FOR DHT SENSOR**********/
extern DHTesp dht;  /* ESP32-compatible DHT sensor */
/******************************************/

/**********GLOBALS FOR MQ135 SENSOR**********/
extern MQ135 mq135; /* Gas sensor's function */
/********************************************/

/**********GLOBALS FOR SOLAR SYSTEM**********/
extern bool solarActive; /* Boolean variable to check, wether the solar system
                            run or not */
/********************************************/

/**********GLOBALS FOR WIFI******************/
extern WiFiUDP udp;    /* Variable for UDP proadcasting */
extern WiFiClient net; /* Variable for wifi's client */
/********************************************/

/*********GLOBALS FOR BROKER*****************/
extern PubSubClient client;                /* MQTT protocol's client */
extern IPAddress brokerIp;                 /* MQTT protocol broker's ip */
extern uint16_t brokerPort;                /* MQTT protocol broker's port */
extern String clientId;                    /* Client's id */
extern unsigned long lastReconnectAttempt; /* Last attempt's value */
extern unsigned long lastSensorPublish;    /* Last sensor's value */
/********************************************/

/*************************End of MCU globals declaration******************/

/*************************Network settings declaration********************/
extern const char *WIFI_SSID; /* Wifi SSID */
extern const char *WIFI_PASS; /* Wifi password */
/*************************End of Network settings declaration*************/

/************************UDP beacon settings declaration******************/
extern const uint16_t BEACON_PORT; /* Reacieves packets from MQTT broker */
extern const char
    *BEACON_NAME; /* Prevents accidentally connecting to unknown devices */
/************************End of UDP beacon settings declaration**********/

/***********************MQTT topics declaration***************************/
extern const char *TOPIC_CONTROL; /* Recieve contorlling messages */
extern const char *TOPIC_SENSORS; /* Puplish sensors' data */
extern const char *TOPIC_STATUS;  /* Puplish the status */

/****************Actuators topics************/
extern const char *TOPIC_FAN;
extern const char *TOPIC_LIGHT_FLOOR1;
extern const char *TOPIC_LIGHT_FLOOR2;
extern const char *TOPIC_LIGHT_RGB;
extern const char *TOPIC_BUZZER;
extern const char *TOPIC_MOTOR_GARAGE;
extern const char *TOPIC_MOTOR_FRONT_WIN;
extern const char *TOPIC_MOTOR_DOOR;
/********************************************/

/****************Sensors topics**************/
extern const char *TOPIC_GAS;
extern const char *TOPIC_LDR;
extern const char *TOPIC_RAIN;
extern const char *TOPIC_VOLTAGE;
extern const char *TOPIC_CURRENT;
extern const char *TOPIC_HUMIDITY;
extern const char *TOPIC_FLAME;
extern const char *TOPIC_TEMPERATURE;
extern const char *TOPIC_PUSH_BUTTON_FACE_DETECTION;
/********************************************/
/**************************End of MQTT topics*****************************/

/*********************Functions declarations******************/

/**
 * @brief Set the RGB LED strip colors using PWM.
 *
 * This function writes analog values to the RGB pins to control
 * the LED color intensity.
 *
 * @param r  Intensity of the Red channel (0–255)
 * @param g  Intensity of the Green channel (0–255)
 * @param b  Intensity of the Blue channel (0–255)
 *
 * @return void
 */
void setRGB(int r, int g, int b);

/**
 * @brief Calculate the broadcast IP address based on IP and subnet mask.
 *
 * This function computes the broadcast address by OR-ing the IP
 * with the inverse of the subnet mask.
 *
 * @param ip    The local IP address of the ESP32.
 * @param mask  The subnet mask of the WiFi network.
 *
 * @return IPAddress   The calculated broadcast address.
 */
IPAddress subnetBroadcast(IPAddress ip, IPAddress mask);

/**
 * @brief Parse a UDP broker discovery JSON advertisement.
 *
 * This function reads a JSON string received via UDP
 * and extracts broker information (IP and port) if the
 * advertisement matches the expected BEACON_NAME.
 *
 * @param json  Null-terminated C-string containing the JSON payload.
 *
 * @return true   Successfully parsed and matches BEACON_NAME.
 * @return false  Invalid JSON or advertisement mismatch.
 */
bool parseAdvert(const char *json);

/**
 * @brief Passively listen for MQTT broker discovery packets.
 *
 * This function opens a UDP socket and waits for incoming UDP
 * advertisements within a specified timeout.
 *
 * @param ms  The listening duration in milliseconds.
 *
 * @return true   Broker discovered successfully.
 * @return false  No valid advertisement received.
 */
bool discoverPassive(uint32_t ms);

/**
 * @brief Actively broadcast a WHO_IS request to discover the MQTT broker.
 *
 * This function sends discovery packets to both the broadcast address
 * and the global broadcast (255.255.255.255), then waits for replies.
 *
 * @param ms  Total time in milliseconds to attempt active discovery.
 *
 * @return true   Broker responded with valid information.
 * @return false  No valid reply received during timeout.
 */
bool discoverActive(uint32_t ms);

/**
 * @brief Attempt both passive and active MQTT broker discovery.
 *
 * Splits the total timeout into two halves:
 *  1. Passive listening
 *  2. Active broadcasting
 *
 * @param timeout_ms  Total discovery timeout in milliseconds.
 *
 * @return true   A broker was discovered.
 * @return false  Broker discovery failed.
 */
bool discoverBroker(uint32_t timeout_ms);

/**
 * @brief Ensure WiFi is connected; if not, connect to the network.
 *
 * This function initiates a connection to the WiFi network using
 * the configured WIFI_SSID and WIFI_PASS. It blocks until connected.
 *
 * @return void
 */
void ensureWifi();

/**
 * @brief MQTT message callback handler.
 *
 * This function is automatically called by the MQTT client when a
 * message is received. It processes control commands (lights, fans, etc.)
 * and updates hardware components accordingly.
 *
 * @param topic     The MQTT topic the message was received on.
 * @param message   Pointer to message payload bytes.
 * @param length    Length of the message payload.
 *
 * @return void
 */
void callBack(char *topic, byte *message, unsigned int length);

/**
 * @brief Ensure MQTT client is connected; try to reconnect if needed.
 *
 * This function:
 *   1. Uses UDP discovery to find the broker.
 *   2. Configures MQTT connection settings.
 *   3. Establishes connection with LWT (Last Will & Testament).
 *   4. Subscribes to control topics.
 *
 * @return void
 */
void ensureMqtt();

/**
 * @brief Check if the Push button is pushed or not.
 * 
 * This function is reading if the push button is pushed or not.
 * 
 * @param pinNumber Number of the pin, which the push button is connected too.
 * 
 * @return true if the button is pushed
 * @return false if the button isn't pushed
 */
bool buttonPressed(int pinNumber);

/**
 * @brief Initialize the main gate servos with proper settings.
 * 
 * This function sets up both gate servos with PWM frequency and
 * attaches them to their respective pins.
 * 
 * @return void
 */
void initGateServos();

/**
 * @brief Initialize the front window servos with proper settings.
 * 
 * This function sets up both front window servos with PWM frequency and
 * attaches them to their respective pins.
 * 
 * @return void
 */
void initFrontWindowServos();

/**
 * @brief Open the main gate with smooth synchronized movement.
 * 
 * This function moves both gate doors from closed to open position
 * with smooth gliding motion, mirroring left and right doors.
 * 
 * @return void
 */
void openGate();

/**
 * @brief Close the main gate with smooth synchronized movement.
 * 
 * This function moves both gate doors from open to closed position
 * with smooth gliding motion, mirroring left and right doors.
 * 
 * @return void
 */
void closeGate();

/**
 * @brief Move the main gate to a specific angle.
 * 
 * This function provides smooth synchronized movement for both
 * gate doors to a target angle.
 * 
 * @param targetAngle The target angle (0-180) for the gate.
 * 
 * @return void
 */
void moveGateTo(int targetAngle);

/**
 * @brief Calculate the right door's pulse width with offset for front window.
 * 
 * This function computes the mirrored position for the right window
 * door, adding an offset to compensate for mechanical differences.
 * 
 * @param leftUs The pulse width of the left door in microseconds.
 * 
 * @return int The calculated pulse width for the right door.
 */
int getFrontWinRightUs(int leftUs);

/**
 * @brief Open the front window with smooth synchronized movement.
 * 
 * This function moves both window doors from closed to open position
 * with smooth gliding motion and offset compensation.
 * 
 * @return void
 */
void openFrontWindow();

/**
 * @brief Close the front window with smooth synchronized movement.
 * 
 * This function moves both window doors from open to closed position
 * with smooth gliding motion and offset compensation.
 * 
 * @return void
 */
void closeFrontWindow();

/**
 * @brief Move the front window to a specific angle.
 * 
 * This function provides smooth synchronized movement for both
 * window doors to a target angle with offset compensation.
 * 
 * @param targetAngle The target angle (0-180) for the window.
 * 
 * @return void
 */
void moveFrontWindowTo(int targetAngle);

/**
 * @brief Move door smoothly to a specified angle.
 * 
 * @param targetAngle The target angle (0-180) for the door.
 * 
 * @return void
 */
void moveDoorTo(int targetAngle);

/**
 * @brief Move garage smoothly to a specified angle.
 * 
 * @param targetAngle The target angle (0-180) for the garage.
 * 
 * @return void
 */
void moveGarageTo(int targetAngle);

/**
 * @brief Process pending servo movement commands.
 * 
 * This function should be called from the main loop to handle
 * non-blocking servo movements that were requested via MQTT.
 * 
 * @return void
 */
void processServoCommands();

/************End of functions' declaration********************/

/**********MQTT topic for main gate**********/
extern const char *TOPIC_MOTOR_GATE;

#endif /* DOIT_ESP_32_DIVKIT_V1 == MCU_TYPE */

#endif /* _APPLICATION_H_ */
/******************************************************************************/
