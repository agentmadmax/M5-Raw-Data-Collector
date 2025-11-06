
#pragma once

#include <Arduino.h>
#include <M5Unified.h>
#include <WiFi.h>
#include "esp_wifi.h"
#include "esp_wpa2.h"
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>
#include <time.h>
#include <ArduinoJson.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>

// ---- Project-wide constants ----
static const int   HTTP_PORT = 8080;
static const int   IMU_RAW_SAMPLES = 64;
static const int   DEFAULT_SAMPLING_HZ = 100;
static const char* NTP_SERVER = "pool.ntp.org";

// MQTT defaults (can be changed at runtime via web UI)
extern char mqtt_server[40];

// Active state bitmask
constexpr uint8_t ACCEL_BIT = 0x02;
constexpr uint8_t GYRO_BIT  = 0x04;
constexpr uint8_t EMG_BIT   = 0x08;

enum {
  STATE_IDLE       = 0,
  STATE_GYRO_ONLY  = GYRO_BIT,
  STATE_ACCEL_ONLY = ACCEL_BIT,
  STATE_EMG_ONLY   = EMG_BIT,
  STATE_ALL_HUMAN  = (ACCEL_BIT | GYRO_BIT | EMG_BIT),
};

// Globals shared across modules
extern AsyncWebServer server;
extern WiFiClient     espClient;
extern PubSubClient   mqttClient;
extern String         client_id;

extern uint8_t active_state;
extern int     sampling_frequency;

// Secrets (optional). Create include/secrets.h to override defaults.
#ifdef __has_include
#  if __has_include("secrets.h")
#    include "secrets.h"
#  endif
#endif

#ifndef EAP_SSID
#endif
#ifndef EAP_USERNAME
#endif
#ifndef EAP_PASSWORD
#endif

#ifndef WIFI_TIMEOUT_MS
  #define WIFI_TIMEOUT_MS 15000
#endif

// EMG pin
#ifndef EMG_SENSOR_PIN
  #define EMG_SENSOR_PIN 32
#endif