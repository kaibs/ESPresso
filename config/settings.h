


//                       {T_soll, K_p, K_i, K_d, Temp Offset}
int standardSettings[] = {   90,   50,  10,   5,           0}; 

// Time [s] after which ESPresso goes to standby from:
// (1) Main Menu
#define AUTO_STANDBY_MAIN 600
// (2) Power State
#define AUTO_STANDBY_POWER 1800

// Publish MQTT Data ever (...) seconds (not implemented yet)
#define MQTT_TIMER_MAIN 20
#define MQTT_TIMER_POWER 1


/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// PORTS ///////////////////////////////////////

// Data wire for temperature sensor is plugged into port 33 on the ESP32
#define ONE_WIRE_BUS 33
// initialize display 1,3" OLED 
SH1106Wire  display(0x3c, 21,22);
// Heater SSR at port D25
#define ssr 25