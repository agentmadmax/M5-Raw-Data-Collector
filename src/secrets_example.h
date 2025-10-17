// At the top of main.cpp
#include "secrets.h"   // will hold EAP_SSID, EAP_USERNAME, EAP_PASSWORD

// Remove the hardcoded creds in setup_wifi() and use the defines:
const char* EAP_SSID     = "WIFI_EAP_SSID";
const char* EAP_USERNAME = "WIFI_EAP_USERNAME";
const char* EAP_PASSWORD = "WIFI_EAP_PASSWORD";
