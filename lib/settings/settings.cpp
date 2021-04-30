// #pragma once

#include "settings.h"

// #include "init.h"
#include <Arduino.h>
// Libraries for Display
#include <Wire.h>
#include "SH1106Wire.h" 

//                           {T_soll, K_p, K_i, K_d, Temp Offset}
const int standardSettings[] = {   90,   50,  10,   5,           0}; 

// Time [s] after which ESPresso goes to standby from:
// (1) Main Menu
const int AUTO_STANDBY_MAIN = 3;
// (2) Power State
const int AUTO_STANDBY_POWER = 10;

// Publish MQTT Data ever (...) seconds
const uint8_t MQTT_TIMER_MAIN = 20;
const uint8_t MQTT_TIMER_POWER = 1;

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// PORTS ///////////////////////////////////////

// Data wire for temperature sensor is plugged into port 33 on the ESP32
const uint8_t ONE_WIRE_BUS = 33;

// initialize display 1,3" OLED 
const uint8_t display_sda = 21;
const uint8_t display_scl = 22;

// Heater SSR at port D25
const uint8_t ssr = 25;

// Declare Pins for  Encoder
// Used for generating interrupts using CLK signal
const byte encoder_clk = 27; // GPIO27 ESP32
// Used for reading DT signal
const byte encoder_dt = 13; // GPIO13 ESP32
// Used for the push button switch
const byte encoder_sw = 14; // GPIO14 ESP32


