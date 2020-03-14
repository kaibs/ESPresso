
// Number of Settings
const byte numOfSettings = 4;
const char* namesSettings[] = {"Return", "Tropfen/Minute", "Zeit MV offen[ms]", "Wasser Drip [ml]","Leeren", "Start Drip"};
// number of states during drip
const byte numOfDripStates = 5; //ohne Pause und Abbrechen
const char* namesDripStates[] = {"Pausieren", "Gesamtzeit", "Restzeit(approx.)", "Anzahl Tropfen", "Einstellungen","Temperatur", "Drip abbrechen"};
byte addr = 0;
byte hoursBuf=0;
byte minutesBuf=0;
byte secondsBuf=0;
int correctionPause=0;
byte correctionHourPause=0;
int settings[numOfSettings];
int dripStates[numOfDripStates];
const byte minSetting[] = {1, 10, 50};
const int maxSetting[] = {100, 1000, 500};
int secsDripStart;
// Variables for displaying total time in hrs:min:secs
byte runHours = 0;
int secsRemaining;
byte runMinutes = 0;
byte runSeconds = 0;
char buf[21];
byte correctionHour;
// Setting which currently is edited, = 0 if no setting is manipulated
byte editSetting = 0;
byte change;

// Declare Pins for  Encoder
// Used for generating interrupts using CLK signal
const byte Pin_CLK = 12; //PIN 4 auf NANO, GPIO2 ESP32
// Used for reading DT signal
const byte Pin_DT = 13; //PIN 2 auf NANO, GPIO12 ESP32
// Used for the push button switch
const byte Pin_SW = 14; //PIN 3 auf NANO, GPIO14 ESP32
// Declare Pin used for solenoid valve control
const byte pinSol = 5;

byte menuCounter;
boolean clicked = false; //true wenn encoder knopf einmal gedrückt wurde
boolean mainMenu = true;
boolean settingsMenu = false;
boolean pauseDrip = false;
boolean clockwise = false;
boolean anticlockwise = false;
boolean dripState = false;
boolean closeNext = false; // TRUE: just dripped, now close valve as next step
boolean preInfusion = false;

byte xVecSymbStatus[numOfSettings + 2];
byte lengthSymbStatus;
byte lengthDripSymbStatus;
byte xVecDripSymbStatus[numOfDripStates + 2];

unsigned long compareTime;
