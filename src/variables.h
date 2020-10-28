
// Number of Settings
const byte numOfSettings = 4;
const char* namesSettings[] = {"Home", "Temp", "K_P", "K_I", "K_D", "Reset"};
// number of states during drip
const byte numOfpowerStates = 3; //ohne Pause und Abbrechen
const char* namespowerStates[] = {"Temp", "Temp Des", "Timer", "Standby"};
//                       {T_soll, K_p, K_i, K_d}
int standardSettings[] = {   90,  50,  10,   5}; // TODO: EEPROM statt Definition
int settings[4];
int powerStates[numOfpowerStates];
const int minSetting[] = {70, 0, 0, 0}; // "T_set", "K_p", "K_i", "K_d" 
const int maxSetting[] = {110, 100, 100, 100}; // "T_set", "K_p", "K_i", "K_d"

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

int WindowSize = 500;
unsigned long windowStartTime;

// Variables for EEPROM
#define EEPROM_SIZE 4

