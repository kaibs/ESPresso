// Libraries for Display
#include <Wire.h>
#include "SH1106Wire.h" 

// Libraries for Temperature Sensor
#include <OneWire.h>
#include <DallasTemperature.h>

#include <EEPROM.h>

// initialize display 1,3" OLED 
SH1106Wire  display(0x3c, 21,22);

// Data wire for temperature sensor is plugged into port 33 on the Arduino
#define ONE_WIRE_BUS 33
// include custom xbm images for display
#include "images.h"
// include variables
#include "variables.h"

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;


// FUNKTIONEN   FUNKTIONEN   FUNKTIONEN   FUNKTIONEN   FUNKTIONEN

///////////////////////////////displaySettings////////////////////////////////////
void displaySettings(int index) {
  display.clear();
  display.drawXbm(56, 0, 14, 14, settingsbutton_14);
  // Left Side: back button, middle: circle, right side: play
  display.drawXbm(xVecSymbStatus[0], 54, 10, 10, back_10);
  for (int i = 1; i < numOfSettings + 2; i++) { /////////////////////////////////
    //display.drawXBitmap(xVecSymbStatus[i], 54, 10, 10, round_10);
    byte x = xVecSymbStatus[i];
    if (i == index && i < numOfSettings + 1) {
      display.drawXbm(x, 54, 10, 10, roundfilled_10);
    } else if (i > 0 && i < numOfSettings + 1) {
      display.drawXbm(x, 54, 10, 10, round_10);
    }
  }
  display.drawXbm(xVecSymbStatus[numOfSettings + 1], 54, 10, 10,playbutton_10);
  display.drawString(0, 20, namesSettings[index]);
  // Display Value of current setting
  if (index <= numOfSettings && menuCounter > 0) {
    display.drawString(50, 35, String(settings[index - 1]));
  }
  display.display();
}
///////////////////////////////displayMainMenu///////////////////////////////////
void displayMainMenu(byte index) {
  // index == 1 -> mainMenu
  // index == 2 -> pauseMenu
  display.clear();
  switch (index) {
    case 1:
      //display.drawXBitmap(10,0,homebutton_10, 10,10, WHITE);
      display.drawString(5, 0, "Main Menu");
      break;
    case 2:
      //display.drawXBitmap(10,0,pausebutton_10, 10,10, WHITE);
      display.drawString(5, 0, "Drip Paused");
  }
  //display.drawLine(0,11,128,11,WHITE);
  switch (menuCounter) {
    case 1:
      display.drawXbm(12, 20, 40, 40,playbutton40);
      display.drawXbm(81, 25, 30,30, settingsbutton30);
      break;
    case 2:
      display.drawXbm(17, 25, 30, 30, playbutton30);
      display.drawXbm(76, 20, 40, 40, settingsbutton40);
      break;
  }
  //Serial.println(menuCounter);
  display.display();
}
///////////////////////////////displayDripState///////////////////////////////////
void displayDripStates(byte index) {
  display.clear();
  //display.drawXBitmap(56,0,playbutton_14, 14,14, WHITE);
  display.drawString(5, 0, "Dripping");
  // Left Side: pause button, middle: circle, right side: stop
  display.drawXbm(xVecDripSymbStatus[0], 54, 10, 10, pausebutton_10);
  for (int i = 1; i < numOfDripStates + 2; i++) { /////////////////////////////////
    byte x = xVecDripSymbStatus[i];
    if (i == index && i < numOfDripStates + 1) {
      display.drawXbm(x, 54, 10,10,roundfilled_10);
    } else if (i > 0 && i < numOfDripStates + 1) {
      display.drawXbm(x, 54, 10, 10, round_10);
    }
  }
  display.drawXbm(xVecDripSymbStatus[numOfDripStates + 1], 54, 10,10, stopbutton_10);
  display.drawString(0, 20, namesDripStates[index]);
  // Display Value of current setting
  if (index == 1 || index == 2) { //index == 1 -> total time, index == 2 -> verbleibende Zeit
    if (index == 1) {
      runHours = (millis()) / 3600000 - correctionHourPause; // vernachlässigt Startzeit (ab 1h Warten Fehler)
      secsRemaining = ((millis() / 1000 - correctionPause) - secsDripStart) % 3600;
      runMinutes = secsRemaining / 60;
      runSeconds = secsRemaining % 60;
      EEPROM.put(4, runHours); // save total seconds to EEPROM
      EEPROM.put(5, runMinutes);
      EEPROM.put(6, runSeconds);
    } else {
      runHours = (settings[2] * 10 - dripStates[2]) * ((settings[0] + settings[1])) / 3600000; // vernachlässigt Startzeit (ab 1h Warten Fehler)
      secsRemaining = ((settings[2] * 10 - dripStates[2]) * ((settings[0] + settings[1])/1000)) % 3600;
      runMinutes = secsRemaining / 60;
      runSeconds = secsRemaining % 60;
    }
    sprintf(buf, "%02d:%02d:%02d", runHours, runMinutes, runSeconds);
    display.drawString(10, 35, buf);
  } else if (index == 4) { // Einstellungen
    // set Settings state

    //sprintf(buf, "toffen:%04dtzu:%04d", settings[0], settings[1]);
  }else if(index == 5) { // Temperature
    sensors.requestTemperatures(); // Send the command to get temperatures
    display.drawString(50,35,String(sensors.getTempCByIndex(0)));
    Serial.println(sensors.getTempCByIndex(0));
     
  }else if (index <= numOfDripStates && menuCounter > 0) {
    display.drawString(50, 35, String(dripStates[index - 1]));
  }
  
  display.display();
}
//////////////////////////////startDripState//////////////////////////////
void startDripState() {
  EEPROM.get(4, correctionHour);
  EEPROM.get(5, minutesBuf);
  EEPROM.get(6, secondsBuf);
  compareTime = millis();

  if(!preInfusion){
    //digitalWrite(pinSol, HIGH);  // open solenoid valve
    while(millis()-compareTime < 1000){  
    }
    //digitalWrite(pinSol, LOW);  // close solenoid valve
  }
  correctionPause = (millis() / 1000)- secondsBuf - minutesBuf * 60;
  correctionHourPause = (millis() / 3600000)- correctionHour;
//  if (secondsBuf == 0) {
//    secsDripStart = ((millis()) / 1000) % 3600;
//  }
  dripState = true;
  preInfusion = true;
}

// INTERRUPT     INTERRUPT     INTERRUPT     INTERRUPT     INTERRUPT

void IRAM_ATTR isr_encoder () {
  portENTER_CRITICAL_ISR(&mux);
  Serial.println("Interrupt Encoder");
  static unsigned long lastInterruptTime = 0;
  // unsigned long interruptTime = millis();
  // If interrupts come faster than 5ms, assume it's a bounce and ignore
  if (millis() - lastInterruptTime > 50) {
    if (digitalRead(Pin_CLK) == HIGH)
    {
      clockwise = true;
      Serial.println("CLW");
    } else {
      anticlockwise = true;
      Serial.println("CCLW");
    }
    // Restrict value from 0 to +100
    //    virtualPosition = min(100, max(0, virtualPosition));
    // Keep track of when we were here last (no more than every 5ms)
    lastInterruptTime = millis();
  }
  portEXIT_CRITICAL_ISR(&mux);
}
void IRAM_ATTR isr_button() {
  Serial.println("Interrupt Clicked");
  portENTER_CRITICAL_ISR(&mux);
  static unsigned long lastInterruptTimeButton = 0;
  //unsigned long interruptTimeButton = millis();
  // If interrupts come faster than 5ms, assume it's a bounce and ignore
  if (millis() - lastInterruptTimeButton > 15) {
    clicked = true;
    // Keep track of when we were here last (no more than every 5ms)
    lastInterruptTimeButton = millis();
    Serial.println("Click");
  }
  portEXIT_CRITICAL_ISR(&mux);
}
// SETUP    SETUP    SETUP    SETUP    SETUP    SETUP    SETUP
void setup() {
  Serial.begin(115200);
  pinMode(Pin_CLK, INPUT);
  pinMode(Pin_DT, INPUT_PULLUP);
  pinMode(Pin_SW, INPUT_PULLUP);
  //pinMode(pinSol, OUTPUT);
  sensors.begin(); //begin measurement of temperature sensor
  sensors.setResolution(11);
 
  //digitalWrite(Pin_SW, HIGH); //turn pullup resistor on
  attachInterrupt(digitalPinToInterrupt(Pin_DT), isr_encoder, FALLING);
  attachInterrupt(digitalPinToInterrupt(Pin_SW), isr_button, RISING);
  
  //Display initialisieren auf 0x3c Adresse
  display.init();
  display.flipScreenVertically();
  // Calculate position of symbols in status bar in settings menu
  // display is 128x64, we need #(3 + backbutton + playbutton) positions
  byte maxAbstand = (128 - (numOfSettings + 2) * 10) / (numOfSettings + 1);
  lengthSymbStatus = (10 + maxAbstand) * (numOfSettings + 2) - maxAbstand;
  xVecSymbStatus[0] = 64 - round(lengthSymbStatus / 2);
  for (int i = 1; i < numOfSettings+2; i++) {
    xVecSymbStatus[i] = xVecSymbStatus[i - 1] + 10 + maxAbstand;
    
  }
  // Same for DripState symbols
  maxAbstand = (128 - (numOfDripStates + 2) * 10) / (numOfDripStates + 1);
  lengthDripSymbStatus = (10 + maxAbstand) * (numOfDripStates + 2) - maxAbstand ;
  xVecDripSymbStatus[0] = 64 - round(lengthDripSymbStatus / 2);
  for (int i = 1; i < numOfDripStates + 2; i++) {
    xVecDripSymbStatus[i] = xVecDripSymbStatus[i - 1] + 10 + maxAbstand;
  }
  for (int i = 0; i < numOfSettings-1; i++) {
    EEPROM.get(i + 1, settings[i]);
    // Incase value of setting is already to high/low set to max/min value
    if (settings[i] > maxSetting[i]) {
      settings[i] = maxSetting[i];
    }
    if (settings[i] < minSetting[i]) {
      settings[i] = minSetting[i];
    }
  }
  // Set Total Time to 0
  EEPROM.put(4, 0); //hours
  EEPROM.put(5, 0); //minutes
  EEPROM.put(6, 0); //seconds
}
// MAIN LOOP     MAIN LOOP     MAIN LOOP     MAIN LOOP     MAIN LOOP
void loop() {
  /////////////////////////// MAIN MENU /////////////////////////////
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
    displayMainMenu(1); // 1 is main menu, 2 is pause menu
    // If encoder Button is pressed, switch to respective menu
    if (clicked) {
      switch (menuCounter) {
        case 1:
          startDripState();
          break;
        case 2:
          settingsMenu = true;
          //menuCounter = 1;
          break;
      }
      mainMenu = false;
      clicked = false;
      break;
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
      if(editSetting == 1){
        change = 1;
      }else{
        change = 10;
      }
      if(editSetting == 4){
        //digitalWrite(pinSol, HIGH);  // open solenoid valve
      }
      if(editSetting < numOfSettings){ // exclude Setting 4 (leeren)
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
      
        EEPROM.put(editSetting, settings[editSetting - 1]);
      }
      displaySettings(editSetting);
      // exit edit of setting in case button is pressed
      if (clicked) {
        menuCounter = editSetting;
        if(editSetting==4){
          //digitalWrite(pinSol, LOW);  // close solenoid valve
        }
        editSetting = 0;
        clicked = false;
    }
  }
    if (clicked) {
      switch (menuCounter) {
        case 0: // User chose return
          //menuCounter = 1;
          mainMenu = true;
          settingsMenu = false;
          break;
        case (numOfSettings+1):
          startDripState();
          settingsMenu = false;
          break;
      }
      if (menuCounter <= numOfSettings && menuCounter > 0) {
        if (editSetting != 0) {
          editSetting = 0;
        } else {
          editSetting = menuCounter;
        }
      }
      clicked = false;
    }
  
  //Serial.println(editSetting);

 }
  //////////////////////////////////// dripState ///////////////////////////////////////////////
  while (dripState) {
    if (clockwise) {
      if (menuCounter < numOfDripStates + 1) {
        menuCounter++;
      }else if (menuCounter > numOfDripStates) {
        menuCounter = 0;
      }
      clockwise = false;
    }
    if (anticlockwise) {
      if (menuCounter > 0) {
        menuCounter--;
      } else if (menuCounter < numOfDripStates) {
        menuCounter = numOfDripStates + 1;
      }
      anticlockwise = false;
    }
    displayDripStates(menuCounter);
    // Open and close valve with given opening and closing times
    
    if (!closeNext) {
      if (millis() - compareTime > settings[0]) {
        //digitalWrite(pinSol, LOW);  // close solenoid valve after drop
        closeNext = true;
        compareTime = millis();
        dripStates[2] += 1;
      }
    }
    if (closeNext) {
      if (millis() - compareTime > ((60000/settings[0])-settings[1])) {
        //digitalWrite(pinSol, HIGH);  // open solenoid valve
        closeNext = false;
        compareTime = millis();
      }
    }
    if (clicked) {
      switch (menuCounter) {
        case (0): //Pause button
          pauseDrip = true;
          dripState = false;
          menuCounter = 1;
          break;
        case (numOfDripStates+1): //Stop Drip button
          // Reset storage for total time
          EEPROM.put(4, 0);
          EEPROM.put(5, 0);
          EEPROM.put(6, 0);
          dripStates[2] = 0; // Tropfen auf 0 zurücksetzen
          mainMenu = true;
          //menuCounter = 1;
          dripState = false;
          preInfusion = false;
          break;
      }
      clicked = false;
    }
  }
  while (pauseDrip) {
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
    displayMainMenu(2); // paused Menu
    if (clicked) {
      startDripState();
      pauseDrip = false;
    }
  }
}
