// Include settings
// #include "settings.h"
#include "ui.h"

#include "init.h"

#include <SH1106Wire.h>

// Include images
#include "images.h"

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
void displayMainMenu() {
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

  // Display desired contents
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
    }else if(index == 3){ // Shot Timer
      // current_shot_timer = timerAlarmReadSeconds(shot_timer);
      // current_shot_timer = timerGetCountUp(shot_timer);
      current_shot_timer = timerReadSeconds(shot_timer);
      // timer_get_counter_time_sec(TIMER_GROUP_0, TIMER_0, &current_shot_timer);
      String shot_timer_String = String(current_shot_timer, 0);
      display.drawString(63,25, shot_timer_String);
      display.drawString(63+(shot_timer_String.length())*8, 25, "s");
    }else{
      display.drawString(63, 25, String(settings[index - 1]));
    }  
  }
  if(index == 0){ // Display large home icon
      display.drawXbm(54, 23, 20, 20, home_20);
  }

  display.display();
}