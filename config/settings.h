


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
