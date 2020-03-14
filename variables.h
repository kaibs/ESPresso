  
// Number of Settings
const byte numOfSettings = 4;
const char* namesSettings[] = {"Return", "T_set", "K_p", "K_i", "K_d", "Reset" , "Power On"};
// number of states during drip
const byte numOfpowerStates = 3; //ohne Pause und Abbrechen
const char* namespowerStates[] = {"T_current", "T_set", "Parameter", "Standby"};
//byte addr = 0;
//byte hoursBuf=0;
//byte minutesBuf=0;
//byte secondsBuf=0;
//int correctionPause=0;
//byte correctionHourPause=0;
//                       {T_soll, K_p, K_i, K_d}
int standardSettings[] = {   100,  50,  10,   5}; // TODO: EEPROM statt Definition
int settings[4];
int powerStates[numOfpowerStates];
const int minSetting[] = {70, 0, 0, 0}; // "T_set", "K_p", "K_i", "K_d" 
const int maxSetting[] = {110, 100, 100, 100}; // "T_set", "K_p", "K_i", "K_d"
//int secsDripStart;
// Variables for displaying total time in hrs:min:secs
//byte runHours = 0;
//int secsRemaining;
//byte runMinutes = 0;
//byte runSeconds = 0;
//byte correctionHour;
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
boolean pauseDrip = false;
boolean clockwise = false;
boolean anticlockwise = false;
boolean powerState = false;
boolean preInfusion = false;

byte xVecSymbStatus[numOfSettings + 3];
byte lengthSymbStatus;
byte lengthDripSymbStatus;
byte xVecDripSymbStatus[numOfpowerStates + 2];

unsigned long compareTime;

// Variables for PID double setpoint;
double setpoint = 100;
double input;
double output;
// PID PARAMETER -------------------------------------------------------------------------------
double Kp = 50;
double Ki = 0.1;
double Kd = 5;

int WindowSize = 500;
unsigned long windowStartTime;


// Variables for EEPROM
#define EEPROM_SIZE 4


