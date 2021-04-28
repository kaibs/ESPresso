#pragma once

#include <Arduino.h>
#include <SH1106.h>

// Declaration of the settings variables

//                       {T_soll, K_p, K_i, K_d, Temp Offset}
extern const int standardSettings[];

// Time [s] after which ESPresso goes to standby from:
// (1) Main Menu
extern const int AUTO_STANDBY_MAIN;
// (2) Power State
extern const int AUTO_STANDBY_POWER;

// Publish MQTT Data ever (...) seconds
extern const uint8_t MQTT_TIMER_MAIN;
extern const uint8_t MQTT_TIMER_POWER;

/////////////////////////////////// PORTS ///////////////////////////////////////

// Data wire for temperature sensor is plugged into port 33 on the ESP32
extern const uint8_t ONE_WIRE_BUS;

// 1,3" OLED display
extern const uint8_t display_sda;
extern const uint8_t display_scl;

// Heater SSR
extern const uint8_t ssr;

// Declare Pins for  Encoder
// Used for generating interrupts using CLK signal
extern const byte encoder_clk; // GPIO27 ESP32
// Used for reading DT signal
extern const byte encoder_dt; // GPIO13 ESP32
// Used for the push button switch
extern const byte encoder_sw; // GPIO14 ESP32

