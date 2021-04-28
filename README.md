***Template for Credentials*** \
Save your credentials in a file 
```
include/credentials.h
```
using the following template:

```
const char* MQTT_SERVER_IP = "YOUR_SERVER_IP";
const char* MQTT_USER = "YOUR_USER_NAME";
const char* MQTT_PASSWORD = "YOUR_MQTT_PASSWORD";

const char* WIFI_SSID = "YOUR_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";

// Subscription Topics
const char* MQTT_TOPIC_SWITCH = "/home/bedroom/gaggia/switch";
// Published Topics
const char* MQTT_TOPIC_TEMP = "/home/bedroom/gaggia/temperature";
// State of GAGGIA
const char* MQTT_TOPIC_STATE = "/home/bedroom/gaggia/state";
```