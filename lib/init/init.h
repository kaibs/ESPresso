#pragma once

#include <Arduino.h>
// Libraries for WiFi and MQTT
#include <PubSubClient.h>
#include <WiFi.h>
// Libraries for Temperature Sensor
#include <OneWire.h>
#include <DallasTemperature.h>
// Libraries for Display
#include <Wire.h>
#include "SH1106Wire.h" 
// Library for PID controller
#include <PID_v1.h>

// Number of Settings
extern const byte numOfSettings;
extern const char* namesSettings[];
// number of states during drip
extern const byte numOfpowerStates; // without 'Home'
extern const char* namespowerStates[];
// Main Menu States
extern const byte numOfMainStates;
extern const char* namesMainStates[];
extern int settings[];
extern int powerStates[];
extern const int minSetting[]; // "T_set", "K_p", "K_i", "K_d", "Offset"
extern const int maxSetting[]; // "T_set", "K_p", "K_i", "K_d", "Offset"

// Index+1 of setting currently beeing edited, = 0 if no setting is manipulated
extern byte editSetting;
extern byte change;

extern byte menuCounter;
extern boolean clicked; //true wenn encoder knopf einmal gedrückt wurde
extern boolean mainMenu;
extern boolean settingsMenu;
extern boolean clockwise;
extern boolean anticlockwise;
extern boolean powerState;
extern volatile boolean standBy;

extern byte xVecSymbStatus[];
extern byte lengthSymbStatus;
extern byte lengthDripSymbStatus;
extern byte xVecDripSymbStatus[];

extern unsigned long compareTime;

extern double currentTemperature;

extern int WindowSize;
extern unsigned long windowStartTime;

// Variables for EEPROM
extern const uint8_t EEPROM_SIZE;

extern String receivedString;
extern unsigned long mqttPubTimer;

// WiFI and MQTT variables
// extern boolean online_mode;

//mqtt messages
extern const uint8_t MSG_BUFFER_SIZE;
extern char msg_temp[];
extern char msg_state[];

// Variable for the HW-Timers
extern double current_shot_timer;
extern boolean shot_timer_active;

// Variables for MQTT Server
extern WiFiClient ESPresso;
extern PubSubClient client;

// Variables for PID double setpoint;
extern double setpoint;
extern double input;
extern double output;
// init of PID 
extern PID gaggiaPID;

// Init of display
extern SH1106Wire display;

// Init temperature sensor 
extern OneWire oneWire;
extern DallasTemperature sensors;

// Init hw timers
extern hw_timer_t * standby_timer;
extern hw_timer_t * shot_timer;