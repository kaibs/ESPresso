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

#include "credentials.h"

// Used e.g. for the timers
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"

#define TIMER_DIVIDER         16  //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds
#define TIMER_INTERVAL0_SEC   (3.4179) // sample test interval for the first timer
#define TIMER_INTERVAL1_SEC   (5.78)   // sample test interval for the second timer
#define TEST_WITHOUT_RELOAD   0        // testing will be done without auto reload
#define TEST_WITH_RELOAD      1        // testing will be done with auto reload

// Variables for MQTT Server
WiFiClient ESPresso;
PubSubClient client(ESPresso);

// initialize display 1,3" OLED 
SH1106Wire  display(0x3c, 21,22);

// Data wire for temperature sensor is plugged into port 33 on the Arduino
#define ONE_WIRE_BUS 33
// include custom xbm images for display
#include "images.h"
// include variables
#include "variables.h"
// SSR at port D25
#define ssr 25

// init of PID 
PID gaggiaPIT(&input, &output, &setpoint, settings[1], (double)settings[2]/100, settings[3], DIRECT);

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors (&oneWire);

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
// FUNKTIONEN   FUNKTIONEN   FUNKTIONEN   FUNKTIONEN   FUNKTIONEN
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////

void state_transition(char* new_state){
  /*
  Possible States:
  - mainMenu
  - settingsMenu
  - powerState
  */
  digitalWrite(ssr, LOW);
  mainMenu=false;
  settingsMenu=false;
  powerState=false;
  if (strcmp(new_state,"mainMenu")==0){
    menuCounter = 0;
    mainMenu=true;  
  }
  if (strcmp(new_state,"settingsMenu")==0){
    menuCounter = 1;
    settingsMenu=true;
    editSetting = 0;
  }
  if (strcmp(new_state,"powerState")==0){
    menuCounter = 1;
    powerState=true;
    gaggiaPIT.SetTunings(settings[1],double(settings[2])/100, settings[3]);
    //PID gaggiaPIT(&input, &output, &setpoint, settings[1], (double)settings[2]/100, settings[3], DIRECT);
    setpoint = settings[0]; // Desired Temperature
  }
}

void publish_temp(){
    char helpval[8];
    dtostrf(currentTemperature, 6, 2, helpval);
    snprintf (msg_temp, MSG_BUFFER_SIZE, helpval);
    client.publish(MQTT_TOPIC_TEMP, msg_temp);
    //Serial.println("Published");
}

/////////////////////////////// callback for mqtt /////////////////////////////////
void mqtt_callback(char* topic, byte* payload, unsigned int length){
  if (strcmp(topic,MQTT_TOPIC_SWITCH)==0){
    receivedString = "";
    for (int i=0;i<length;i++) {
      receivedString += (char)payload[i];
    }
    if (receivedString == "ON"){
        state_transition("powerState");
    }
    if (receivedString == "OFF"){
      state_transition("mainMenu");
    }
  }
}

// Connect to WIFI
void setup_wifi(){
  display.clear();
  display.setFont(ArialMT_Plain_10);
  WiFi.setHostname("ESPresso");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  byte progress = -1; //
  while (WiFi.status() != WL_CONNECTED) {
      progress = progress + 1;
      
      display.setTextAlignment(TEXT_ALIGN_CENTER);
      display.drawString(64, 5, "Connecting to WiFi");
      display.drawProgressBar(34, 20, 60, 10, progress*5);
      
      display.display();

      if (progress > 19){
        // Connection failed, continue in Offline Mode
        display.drawString(64,45, "Connection failed");
        display.display();
        delay(1000);
        break;
      }else{
        delay(500);
      }
  }
  if (WiFi.status() == WL_CONNECTED){
      display.drawString(64,45, "Connected!");
      display.display();
      delay(1000);
  }
}

// Connect to MQTT Server
void setup_mqtt() {
  display.clear();
  display.setFont(ArialMT_Plain_10);
  byte progress = -1; //

  client.setClient(ESPresso);
  client.setServer(MQTT_SERVER_IP, 1883);
  client.setCallback(mqtt_callback);

  while (!client.connected()) {
    progress = progress + 1;
    
    display.setTextAlignment(TEXT_ALIGN_CENTER);
    display.drawString(64, 5, "Connecting to MQTT");
    display.drawProgressBar(34, 20, 60, 10, progress*5);
    
    display.display();
    // Try to connect to MQTT Server and retry if not succeeded
    if (client.connect("ESPresso", MQTT_USER, MQTT_PASSWORD)) {
      // Subscribe to specified topics for Gaggia
      client.subscribe(MQTT_TOPIC_SWITCH);
      display.drawString(64,45, "Connected!");
      display.display();
      delay(1000);
      
    } 
    if (progress > 19){
      // Connection failed, continue in Offline Mode
      display.drawString(64,45, "Connection failed");
      display.display();
      delay(1000);
      break;
    }else{
      delay(500);
    }
  }
  if (client.connected()){
    online_mode = true;
  }

}

/////////////////////////////// Handle mqtt stuff //////////////////////////////////
void mqtt_stuff(){
  // Check, whether still connected to MQTT-Server, if not: reconnect
  if(!client.connected()){
    if(!WiFi.isConnected()){
      setup_wifi();
    }else{
      setup_mqtt();
    }
  }
  // Publish current temperature every 20 seconds if in mainMenu oder settingsMenu, every second when in powerState
  unsigned long currentTimeDiff = millis()-mqttPubTimer;
  unsigned long currentSubDiff = millis()-mqttPubTimer;
  if (settingsMenu == true || mainMenu == true){
    if (currentTimeDiff > 20000){
      publish_temp();
      mqttPubTimer = millis();
    }
  }else if (powerState == true){
    if (currentTimeDiff > 1000){
      publish_temp();
      mqttPubTimer = millis();
    }
  }else if (settingsMenu == true || mainMenu == true){
    if (currentSubDiff > 1000){
    }
  }
  // Loop the mqtt client
  client.loop();
  //delay(100);

}


void resetPID(){
  windowStartTime = millis();
  // Settings for PID
  gaggiaPIT.SetOutputLimits(0, WindowSize);
  gaggiaPIT.SetMode(AUTOMATIC);
}

/////////////////////////////// displayTemperature////////////////////////////////
void displayTemperature(){
  sensors.requestTemperatures();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);

  currentTemperature = sensors.getTempCByIndex(0)+settings[4];
  int currentTempInt = (int)currentTemperature;
  String current_Temp_string;
  if (currentTempInt < 0){
    current_Temp_string = "ERR";
  } else if (currentTempInt < 100){
    current_Temp_string = "  " + String(currentTempInt);
  } else{
    current_Temp_string = String(currentTempInt);
  }
    display.setFont(ArialMT_Plain_10);

  display.drawString(2, 0, String(current_Temp_string));
  display.drawCircle(25,2,1);
  display.drawString(28,0, "C");
}

///////////////////////////////displaySettings////////////////////////////////////
void displaySettings(int index) {
  display.clear();
  displayTemperature();
  display.flipScreenVertically();
  // Settingsbutton oben mitte
  display.drawXbm(59, 2, 10, 10, settingsbutton10);
  // Left Side: home button, middle: circle indicators
  if (index == 0){
      display.drawXbm(xVecSymbStatus[0], 54, 10, 10, home_10_filled);
  } else{
      display.drawXbm(xVecSymbStatus[0], 54, 10, 10, home_10);
  }
  for (int i = 1; i < numOfSettings + 1; i++) { 
    byte x = xVecSymbStatus[i];
    if (i == index && i < numOfSettings + 1) {
      display.drawXbm(x, 54, 10, 10, roundfilled_10);
    } else if (i > 0 && i < numOfSettings + 1) {
      display.drawXbm(x, 54, 10, 10, round_10);
    }
  }
  if (index == numOfSettings+1){
      display.drawXbm(xVecSymbStatus[numOfSettings+1], 54, 10, 10, trash_10_filled);
  } else{
      display.drawXbm(xVecSymbStatus[numOfSettings+1], 54, 10, 10, trash_10);
  }
  
  // Display name of setting in upper right corner
  display.setTextAlignment(TEXT_ALIGN_RIGHT);
  display.setFont(ArialMT_Plain_10);
  display.drawString(128, 0, namesSettings[index]);

  
  // Display Value of current setting
  if (index <= numOfSettings && menuCounter > 0) {
    if(index == 3){ // K_i
      display.setTextAlignment(TEXT_ALIGN_CENTER);
      display.setFont(ArialMT_Plain_16);
      if(settings[2] == 100){
         display.drawString(64, 25, "1.0");
      }else{ 
        // String(double(settings[2]/100), 2)
        // display.drawString(50, 35, "0."+ String(settings[2]));
        display.drawString(64, 25, String((double)settings[2]/100, 2));
      }
    }else if(index == 1 || index == 5){ // Desired Temperature || Temp Offset
      // Display °C after setting
      display.setTextAlignment(TEXT_ALIGN_CENTER);
      display.setFont(ArialMT_Plain_16);  
      display.drawCircle(73, 25, 2);
      display.drawString(79, 25, "C");
      display.drawString(58, 25, String(settings[index - 1]));
    }else if(index == 6){ // Display WiFi Settings
      display.setTextAlignment(TEXT_ALIGN_LEFT);
      display.setFont(ArialMT_Plain_10);
      if (WiFi.status() != WL_CONNECTED){
        display.setTextAlignment(TEXT_ALIGN_CENTER);
        display.drawString(64,20, "No WiFi Connection.");
        display.drawString(64,35, "Press to retry.");
      }else {
        display.setTextAlignment(TEXT_ALIGN_CENTER);
        display.drawString(64,20, "IP: "+ WiFi.localIP().toString());
      }

    }else if(index == 7){// Display MQTT Settings
      display.setTextAlignment(TEXT_ALIGN_LEFT);
      display.setFont(ArialMT_Plain_10);
      if (client.state() != 0){
        display.setTextAlignment(TEXT_ALIGN_CENTER);
        display.drawString(64,18, "No MQTT Connection.");
        display.drawString(64,29, "State: " + String(client.state()));
        display.drawString(64,40, "Press to retry.");
      }else {
        display.setTextAlignment(TEXT_ALIGN_CENTER);
        display.drawString(64,20, "Connected!");
      }

    }else{ // Just display regular setting
      display.setTextAlignment(TEXT_ALIGN_CENTER);
      display.setFont(ArialMT_Plain_16);
      display.drawString(64, 25, String(settings[index - 1]));
    }  
  }
  if(index == numOfSettings+1){ // Reset
      display.drawXbm(54, 23, 20, 20, trash_20);
  }
  if(index == 0){ // Home Menu
      display.drawXbm(54, 23, 20, 20, home_20);
  }

  display.display();
}
///////////////////////////////displayMainMenu///////////////////////////////////
void displayMainMenu(byte index) {
  // index == 1 -> mainMenu // Standby
  // index == 2 -> pauseMenu !! nicht mehr existent !!
  display.clear();
  display.drawXbm(59, 0, 10, 10, home_10);
  displayTemperature();
  
  //display.drawLine(0,11,128,11,WHITE);
  switch (menuCounter) {
    case 1:
      display.setTextAlignment(TEXT_ALIGN_LEFT);
      display.drawXbm(12, 20, 40, 40,espresso_cup40);
      display.drawXbm(81, 25, 30,30, settingsbutton30);
      display.setTextAlignment(TEXT_ALIGN_RIGHT);
      display.drawString(128, 0, namesMainStates[0]);
      break;
    case 2:
      display.setTextAlignment(TEXT_ALIGN_LEFT);
      display.drawXbm(17, 25, 30, 30, espresso_cup30);
      display.drawXbm(76, 20, 40, 40, settingsbutton40);
      display.setTextAlignment(TEXT_ALIGN_RIGHT);
      display.drawString(128, 0, namesMainStates[1]);
      break;
  }
  display.display();
}
///////////////////////////////displaypowerState///////////////////////////////////
void displaypowerStates(byte index) {
  display.clear();
  display.drawXbm(59, 2, 10, 10, espresso_10);
  displayTemperature(); // display Temperature in upper left corner

  // Left Side: - middle: circle, right side: stop
  for (int i = 1; i < numOfpowerStates; i++) { 
    byte x = xVecDripSymbStatus[i];
    if (i == index && i < numOfpowerStates+1) {
      display.drawXbm(x, 54, 10,10,roundfilled_10);
    } else if (i >= 0 && i < numOfpowerStates+1) {
      display.drawXbm(x, 54, 10, 10, round_10);
    }
  }
  // Draw Home Icon to return to the main menu
  if (index == 0){
      display.drawXbm(xVecDripSymbStatus[0], 54, 10, 10, home_10_filled);
  } else{
      display.drawXbm(xVecDripSymbStatus[0], 54, 10, 10, home_10);
  }
  // Draw hourglass for timer function
  if (index == 3){
      display.drawXbm(xVecDripSymbStatus[3], 54, 10, 10, hourglass_10_filled);
  } else{
      display.drawXbm(xVecDripSymbStatus[3], 54, 10, 10, hourglass_10);
  }
  // Display Name of menu in upper right corner
  display.setTextAlignment(TEXT_ALIGN_RIGHT);
  display.setFont(ArialMT_Plain_10);
  display.drawString(128, 0, namespowerStates[index]);

  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.setFont(ArialMT_Plain_16);
  // Display Value of current setting
  if (index <= numOfSettings && menuCounter > 0) {
    if(index == 1){ // Current Temperature
      display.drawCircle(78, 25, 2);
      display.drawString(84, 25, "C");
      if (currentTemperature<0){
        display.drawString(58, 25, "ERR");
      }else{
        display.drawString(58, 25, String(currentTemperature,1));
      }
    }else if(index == 2){ // Desired Temperature
      display.drawCircle(73, 25, 2);
      display.drawString(79, 25, "C");
      display.drawString(58, 25, String(settings[0]));
    }else{
      display.drawString(64, 25, String(settings[index - 1]));
    }  
  }
  if(index == 0){ // Reset
      display.drawXbm(54, 23, 20, 20, home_20);
  }

  display.display();
}

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
// INTERRUPT     INTERRUPT     INTERRUPT     INTERRUPT     INTERRUPT
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////

void IRAM_ATTR isr_encoder () {
  portENTER_CRITICAL_ISR(&mux);
  static unsigned long lastInterruptTime = 0;
  // unsigned long interruptTime = millis();
  // If interrupts come faster than 50ms, assume it's a bounce and ignore
  if (millis() - lastInterruptTime > 100) { //100
    if (digitalRead(Pin_CLK) == HIGH)
    {
      clockwise = true;
    } else {
      anticlockwise = true;
    }
    // Keep track of when we were here last (no more than every 5ms)
    lastInterruptTime = millis();
  }
  portEXIT_CRITICAL_ISR(&mux);
}
void IRAM_ATTR isr_button() {
  portENTER_CRITICAL_ISR(&mux);
  static unsigned long lastInterruptTimeButton = 0;
  //unsigned long interruptTimeButton = millis();
  // If interrupts come faster than 5ms, assume it's a bounce and ignore
  if (millis() - lastInterruptTimeButton > 15) {
    clicked = true;
    // Keep track of when we were here last (no more than every 5ms)
    lastInterruptTimeButton = millis();
  }
  portEXIT_CRITICAL_ISR(&mux);
}

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
// SETUP    SETUP    SETUP    SETUP    SETUP    SETUP    SETUP
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(115200);
  (Pin_CLK, INPUT);

  EEPROM.begin(EEPROM_SIZE);

  pinMode(Pin_DT, INPUT_PULLUP);
  pinMode(Pin_SW, INPUT_PULLUP);
  pinMode(ssr, OUTPUT);

  sensors.begin(); //begin measurement of temperature sensor
  sensors.setResolution(9);
 
  //digitalWrite(Pin_SW, HIGH); //turn pullup resistor on
  attachInterrupt(digitalPinToInterrupt(Pin_DT), isr_encoder, FALLING);
  attachInterrupt(digitalPinToInterrupt(Pin_SW), isr_button, RISING);
  
  //Display initialisieren auf 0x3c Adresse
  display.init();
  display.flipScreenVertically();
  // Calculate position of symbols in status bar in settings menu
  // display is 128x64, we need #(4 + backbutton + reset + playbutton) positions
  byte maxAbstand = (128 - (numOfSettings + 2) * 10) / (numOfSettings + 1);
  lengthSymbStatus = (10 + maxAbstand) * (numOfSettings + 2) - maxAbstand;
  xVecSymbStatus[0] = 64 - round(lengthSymbStatus / 2);
  for (int i = 1; i < numOfSettings+2; i++) {
    xVecSymbStatus[i] = xVecSymbStatus[i - 1] + 10 + maxAbstand;
  }
  // Same for powerState symbols
  // numOfpowerStates + 1 (all states + stop button)
  maxAbstand = (128 - (numOfpowerStates + 1) * 10) / (numOfpowerStates + 1);
  lengthDripSymbStatus = (10 + maxAbstand) * (numOfpowerStates + 1) - maxAbstand ;
  xVecDripSymbStatus[0] = 64 - round(lengthDripSymbStatus / 2);
  for (int i = 1; i < numOfpowerStates + 1; i++) {
    xVecDripSymbStatus[i] = xVecDripSymbStatus[i - 1] + 10 + maxAbstand;
  }
  for (int i = 0; i < EEPROM_SIZE; i++) {
    //EEPROM.get(i + 1, settings[i]);
    // Incase value of setting is already to high/low set to max/min value
    if (settings[i] > maxSetting[i]) {
      settings[i] = maxSetting[i];
    }
    if (settings[i] < minSetting[i]) {
      settings[i] = minSetting[i];
    }
  }

  // Setup mqtt Timer variable with current Time
  mqttPubTimer = millis();
  
  // Get the last values for the settings from EEPROM
  settings[0] = EEPROM.read(0);
  settings[1] = EEPROM.read(1);
  settings[2] = EEPROM.read(2);
  settings[3] = EEPROM.read(3);
  settings[4] = EEPROM.read(4);

  resetPID();

  // WiFI and MQTT are each given 5 tries. If there is no connection, disable online_mode
  for (int i = 0; i < 5; i++){
    // Setup and connect to WiFi
    setup_wifi();
    if(WiFi.isConnected()){
      break;
    }
  }
  if(WiFi.isConnected()){
    for (int i = 0; i < 5; i++){
      // Setup MQTT and subscribe to stated topics
      setup_mqtt();
      if(client.connected()){
        break;
      }
    }
  }
  if (WiFi.isConnected() and client.connected()){
    online_mode = true;
  }else{
    online_mode = false;
  }

  // Initialize 2 of the 4 available Timers
  
  // I: Espressoshot-Timer


  // II: Auto-PowerOff Timer
  
}

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
// MAIN LOOP     MAIN LOOP     MAIN LOOP     MAIN LOOP     MAIN LOOP
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////

void loop() {
  /////////////////////////// MAIN MENU/STANDBY /////////////////////////////
  menuCounter = 1;
  while (mainMenu) {
    if (anticlockwise) {
      if (menuCounter < 2) {
        menuCounter++;
      } else if (menuCounter > 1) {
        menuCounter--;
      }
      anticlockwise = false;
    }
    if (clockwise) {
      if (menuCounter > 1) {
        menuCounter--;
      } else if (menuCounter < 2) {
        menuCounter++;
      }
      clockwise = false;
    }
    displayMainMenu(1); // 1 is main menu, 2 is Settings menu
    // If encoder Button is pressed, switch to respective menu
    if (clicked) {
      switch (menuCounter) {
        case 1: // Power On
          state_transition("powerState");        
          break;
        case 2: // Settings
          state_transition("settingsMenu");
          break;
      }
      mainMenu = false;
      clicked = false;      
    }
    if(online_mode){
      // Call MQTT handler
      mqtt_stuff();
    }
    
  }
  /////////////////////////// SETTINGS MENU /////////////////////////////
  while (settingsMenu) {
    if (clockwise) {
      if (menuCounter < numOfSettings + 1) {
        menuCounter++;
      } else if (menuCounter > numOfSettings) {
        menuCounter = 0;
      }
      clockwise = false;
    }
    if (anticlockwise) {
      if (menuCounter > 0) {
        menuCounter--;
      } else if (menuCounter < numOfSettings) {
        menuCounter = numOfSettings + 1;
      }
      anticlockwise = false;
    }
    displaySettings(menuCounter);

    // If there is a setting to edit, do so
    while (editSetting != 0) {
      switch (editSetting) {
        case 1: // T_set
          change = 1;
          break;
        case 2: // K_p
          change = 1;
          break;
        case 3: // K_i
          change = 1;
          break;
        case 4: // K_d
          change = 1;
          break;
        case 5: // Temperature Offset
          change = 1;
          break;
      }
      if(editSetting < numOfSettings + 2){
        if (anticlockwise) {
          if (settings[editSetting - 1] - change > minSetting[editSetting - 1]) {
            settings[editSetting - 1] -= change;
          } else {
            settings[editSetting - 1] = minSetting[editSetting - 1];
          }
          anticlockwise = false;
        }
        if (clockwise) {
          if (settings[editSetting - 1] + change < maxSetting[editSetting - 1]) {
            settings[editSetting - 1] += change;
          } else {
            settings[editSetting - 1] = maxSetting[editSetting - 1];
          }
          clockwise = false;
        }
      }
      displaySettings(editSetting);
      // exit edit of setting in case button is pressed
      if (clicked) {        
        EEPROM.write(editSetting-1, settings[editSetting - 1]);
        EEPROM.commit();
        // TODO
        menuCounter = editSetting;
        editSetting = 0;
        clicked = false;
      }
    }
    if (clicked) {
      switch (menuCounter) {
        case 0: // User choose return
          state_transition("mainMenu");
          break;
        // Reset EEPROM Parameters to Standard Setttings
        case (numOfSettings+1):
          settings[0] = standardSettings[0];
          settings[1] = standardSettings[1];
          settings[2] = standardSettings[2];
          settings[3] = standardSettings[3];
          settings[4] = standardSettings[4];
          EEPROM.write(0, settings[0]);
          EEPROM.write(1, settings[1]);
          EEPROM.write(2, settings[2]);
          EEPROM.write(3, settings[3]);        
          EEPROM.write(4, settings[4]);  
          EEPROM.commit();
          break;
        // Reconnect to WiFi
        case 6:
          setup_wifi();
          break;
        // Reconnect to MQTT
        case 7:
          setup_mqtt();
          break;
      }
      // Only try to edit setting, when there is a editable one (all except home, wifi, mqtt, reset)
      if (menuCounter <= numOfSettings - 2 && menuCounter > 0) {
        if (editSetting != 0) {
          editSetting = 0;
        } else {
          editSetting = menuCounter; // editSetting between 1 and numOfSettings-1
        }
      }
      clicked = false;
    }
    if(online_mode){
      // Call MQTT handler
      mqtt_stuff();
    }
  }
  //////////////////////////////////// powerState ///////////////////////////////////////////////
  resetPID();
  while (powerState) {
    if (clockwise) {
      if (menuCounter < numOfpowerStates) {
        menuCounter++;
      }else if (menuCounter == numOfpowerStates) {
        menuCounter = 0;
      }
      clockwise = false;
    }
    if (anticlockwise) {
      if (menuCounter > 0) {
        menuCounter--;
      } else if (menuCounter < numOfpowerStates) {
        menuCounter = numOfpowerStates;
      }
      anticlockwise = false;
    }
    displaypowerStates(menuCounter);
    sensors.requestTemperatures();
    input = sensors.getTempCByIndex(0)+settings[4]; // measured temperature + offset
    gaggiaPIT.Compute();
    unsigned long now = millis();
    if (now - windowStartTime > WindowSize)
    { //time to shift the Relay Window
      windowStartTime += WindowSize;
    }
    if (output > now - windowStartTime){
      digitalWrite(ssr, HIGH);
      // Serial.println("HIGH"); // Debugging 
    }
    else{    
      digitalWrite(ssr, LOW);
      // Serial.println("LOW"); // Debugging 
    }
    // Serial.print(input); // Debugging 
    // Serial.print(","); // Debugging 
    // Serial.println(output); // Debugging 
    if (clicked) {
      switch (menuCounter) {
        case (0): // main menu button
          state_transition("mainMenu");
          break;
        case(2): // Timer
          // TODO: implement timer functionality
          break;
      }
      clicked = false;
    }
    if(online_mode){
      // Call MQTT handler
      mqtt_stuff();
    }
  }
}
