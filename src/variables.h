
// Number of Settings
const byte numOfSettings = 7;
const char* namesSettings[] = {"Home", "Temp", "K_P", "K_I", "K_D", "Temp Offset", "WiFi", "MQTT", "Reset"};
// number of states during drip
const byte numOfpowerStates = 3; //ohne Pause und Abbrechen
const char* namespowerStates[] = {"Home", "Temp", "Temp Des", "Timer"};
// Main Menu States
const byte numOfMainStates = 2;
const char* namesMainStates[] = {"Coffee", "Settings"};
//                       {T_soll, K_p, K_i, K_d, Temp Offset}
int standardSettings[] = {   90,  50,  10,   5, 0}; // TODO: EEPROM statt Definition
int settings[5];
int powerStates[numOfpowerStates];
const int minSetting[] = {70, 0, 0, 0, -10}; // "T_set", "K_p", "K_i", "K_d" 
const int maxSetting[] = {110, 100, 100, 100, 10}; // "T_set", "K_p", "K_i", "K_d"

// Setting which currently is edited, = 0 if no setting is manipulated
byte editSetting = 0;
byte change;

// Declare Pins for  Encoder
// Used for generating interrupts using CLK signal
const byte Pin_CLK = 27; //PIN 4 auf NANO, GPIO12 ESP32
// Used for reading DT signal
const byte Pin_DT = 13; //PIN 2 auf NANO, GPIO13 ESP32
// Used for the push button switch
const byte Pin_SW = 14; //PIN 3 auf NANO, GPIO14 ESP32
// Declare Pin used for solenoid valve control
//const byte pinSol = 5;

byte menuCounter;
boolean clicked = false; //true wenn encoder knopf einmal gedrückt wurde
boolean mainMenu = true;
boolean settingsMenu = false;
boolean clockwise = false;
boolean anticlockwise = false;
boolean powerState = false;

byte xVecSymbStatus[numOfSettings + 2];
byte lengthSymbStatus;
byte lengthDripSymbStatus;
byte xVecDripSymbStatus[numOfpowerStates + 2];

unsigned long compareTime;

// Variables for PID double setpoint;
double setpoint = 100;
double input;
double output;

double currentTemperature;

int WindowSize = 500;
unsigned long windowStartTime;

// Variables for EEPROM
#define EEPROM_SIZE 5

String receivedString;
unsigned long mqttPubTimer;

// WiFI and MQTT variables
boolean online_mode;

//mqtt messages
#define MSG_BUFFER_SIZE	(50)
char msg_temp[MSG_BUFFER_SIZE];

