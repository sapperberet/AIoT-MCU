#ifndef _APPLICATION_H_
#define _APPLICATOIN_H_

/***********************Files includes*****************************************/
#include "platform.h"
/******************************************************************************/

/***********************Application includes***********************************/
#include <Adafruit_Sensor.h>
#include <ArduinoJson.h>
#include <DHT.h>
#include <MQ135.h>
#include <PubSubClient.h>
#include <Servo.h>
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

/**********FAN**********/
#define FAN_ENA 33 /* First fan */
#define FAN_IN1 5  /* Second fan */
#define FAN_IN2 13 /* Third fan */

/**********BUZZER**********/
#define BUZZER_PIN 23 /* Buzzer */

/**********FLOORS' LEDs**********/
#define LED_FLOOR1 25    /* First floor LED */
#define LED_FLOOR2 26    /* Second floor LED */
#define LED_LANDSCAPE 27 /* Third floor LED */

/**********RGB STREP**********/
#define RED_PIN 22  /* R color */
#define GREEN_PIN 2 /* G color */
#define BLUE_PIN 12 /* B color */

/**********Servo MOTORS**********/
#define SERVO_WINDOW1_PIN 15 /* Servo controlling window 1 */
#define SERVO_WINDOW2_PIN 16 /* Servo controlling window 2 */
#define SERVO_DOOR_PIN 18    /* Servo controlling house's door */
#define SERVO_GARAGE_PIN 19  /* Servo controlling garage's door */
#define SERVO_GATE_PIN 21    /* Servo controlling villa's gate */

/***********************End of pins diagram*******************/

/**************************MCU globals declaration************************/

/**********SERVOS DECLARATION**********/
extern Servo servoWindow1; /* Servo of windows 1 */
extern Servo servoWindow2; /* Servo of windows 2 */
extern Servo servoDoor;    /* Servo for the house's door */
extern Servo servoGarage;  /* Servo for the garage's door */
extern Servo servoGate;    /* Servo for the villa's door */
/**************************************/

/**********GLOBALS FOR DHT SENSOR**********/
#define DHT_TYPE DHT22 /* Temperature and humidity sensor's variable */
extern DHT dht;        /* Temperature and humidity sensor's function */
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
extern const char *TOPIC_LIGHT_LANDSCAPE;
extern const char *TOPIC_LIGHT_RGB;
extern const char *TOPIC_BUZZER;
extern const char *TOPIC_MOTOR_GARAGE;
extern const char *TOPIC_MOTOR_FRONT_WIN;
extern const char *TOPIC_MOTOR_SIDE_WIN;
extern const char *TOPIC_MOTOR_DOOR;
/********************************************/

/****************Sensors topics**************/
extern const char *TOPIC_GAS;
extern const char *TOPIC_LDR;
extern const char *TOPIC_RAIN;
extern const char *TOPIC_VOLTAGE;
extern const char *TOPIC_CURRENT;
extern const char *TOPIC_HUMIDITY;
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

/************End of functions' declaration********************/

#endif
/******************************************************************************/

#endif
