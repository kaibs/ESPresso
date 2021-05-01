#include "init.h"

#include <Arduino.h>
#include <PID_v1.h>

// Libraries for Display
#include <Wire.h>
#include "SH1106Wire.h" 

// Libraries for Temperature Sensor
#include <OneWire.h>
#include <DallasTemperature.h>

// Libararies for WiFI and MQTT
#include <WiFi.h>
#include <PubSubClient.h>

#include <EEPROM.h>

// include (standard) settings
#include "settings.h"

// Number of Settings
const byte numOfSettings = 7; // !! 'Home' and 'Reset' do not count as setting 
const char* namesSettings[] = {"Home", "Temp", "K_P", "K_I", "K_D", "Temp Offset", "WiFi", "MQTT", "Reset"};
// number of states during drip
const byte numOfpowerStates = 3; // without 'Home'
const char* namespowerStates[] = {"Home", "Temp", "Temp Des", "Timer"};
// Main Menu States
const byte numOfMainStates = 2;
const char* namesMainStates[] = {"Coffee", "Settings"};
int settings[5];
int powerStates[numOfpowerStates];
const int minSetting[] = {70, 0, 0, 0, -10}; // "T_set", "K_p", "K_i", "K_d", "Offset"
const int maxSetting[] = {110, 100, 100, 100, 10}; // "T_set", "K_p", "K_i", "K_d", "Offset"

// Index+1 of setting currently beeing edited, = 0 if no setting is manipulated
byte editSetting = 0;
byte change;

byte menuCounter;
boolean clicked = false; //true wenn encoder knopf einmal gedrückt wurde
boolean mainMenu = true;
boolean settingsMenu = false;
boolean clockwise = false;
boolean anticlockwise = false;
boolean powerState = false;
volatile boolean standBy = false;

byte xVecSymbStatus[numOfSettings + 2];
byte lengthSymbStatus;
byte lengthDripSymbStatus;
byte xVecDripSymbStatus[numOfpowerStates + 2];

unsigned long compareTime;

double currentTemperature;

int WindowSize = 500;
unsigned long windowStartTime;

// Variables for EEPROM
const uint8_t EEPROM_SIZE = 5;

String receivedString;
unsigned long mqttPubTimer;

// WiFI and MQTT variables
// boolean online_mode = false;

//mqtt messages
const uint8_t MSG_BUFFER_SIZE = 50;
char msg_temp[MSG_BUFFER_SIZE];
char msg_state[MSG_BUFFER_SIZE];

// Variable for the HW-Timers
double current_shot_timer;
boolean shot_timer_active = false;


// Variables for MQTT Server
WiFiClient ESPresso;
PubSubClient client(ESPresso);

// init of PID 
// Variables for PID double setpoint;
double setpoint = 100;
double input;
double output;
PID gaggiaPID(&input, &output, &setpoint, settings[1], (double)settings[2]/100, settings[3], DIRECT);

// Init of display
SH1106Wire display(0x3c, display_sda, display_scl);

// Init temperature sensor 
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors (&oneWire);

// Init Timers
hw_timer_t * standby_timer = NULL;
hw_timer_t * shot_timer = NULL;
