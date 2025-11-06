#include "config.h"
#include "imu.h"
#include "calibration.h"
#include "raw_mode.h"
#include "web_server.h"
#include "wifi_setup.h"

#include <ArduinoOTA.h>
#include <sys/time.h>
#include <Wire.h>
#include "esp_wifi.h"   // for esp_wifi_set_ps()

extern PubSubClient mqttClient;
extern uint8_t active_state;

void setup() {
  auto cfg = M5.config();
  cfg.internal_imu = false;
  cfg.external_imu = false;
  M5.begin(cfg);

  Serial.begin(115200);
  M5.Display.setRotation(1);
  M5.Display.println("Initializing...");

  // Faster I2C for MPU6886
  Wire.setClock(400000);

  // --- IMU + Calibration ---
  setupMPU6886();
  calibrateSensors();
  M5.Display.println("IMU Calibrated");

  // --- Wi-Fi ---
  setup_wifi();
  // Disable Wi-Fi power save to avoid periodic stalls
  esp_wifi_set_ps(WIFI_PS_NONE);
  M5.Display.println("Wi-Fi Connected (PS=NONE)");

  // --- Web Server ---
  setup_web_server();
  M5.Display.println("Web Server Active");

  // --- NTP Time Sync ---
  configTime(0, 0, NTP_SERVER);
  struct tm timeinfo;
  Serial.print("Syncing time");
  while (!getLocalTime(&timeinfo) || timeinfo.tm_year < 120) {
    Serial.print(".");
    delay(300);
  }
  Serial.println("\nTime synchronized.");

  char buf[64];
  strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &timeinfo);
  Serial.printf("UTC: %s\n", buf);

  // --- OTA ---
  ArduinoOTA.setHostname(client_id.c_str());
  ArduinoOTA.begin();
  Serial.println("OTA Ready");

  // --- Start gap-free sampling/publish tasks ---
  start_raw_mode_tasks();

  // Initial UI
  drawDisplay();
  M5.Display.println("Streaming…");
}

void loop() {
  M5.update();
  ArduinoOTA.handle();

  // MQTT reconnect loop
  if (!mqttClient.connected()) {
    static unsigned long lastTry = 0;
    if (millis() - lastTry > 5000) {
      lastTry = millis();
      reconnect_mqtt();
    }
  } else {
    mqttClient.loop();
  }

  // Cycle modes with Button A
  if (M5.BtnA.wasPressed()) {
    switch (active_state) {
      case STATE_IDLE:       active_state = STATE_GYRO_ONLY;  break;
      case STATE_GYRO_ONLY:  active_state = STATE_ACCEL_ONLY; break;
      case STATE_ACCEL_ONLY: active_state = STATE_EMG_ONLY;   break;
      case STATE_EMG_ONLY:   active_state = STATE_ALL_HUMAN;  break;
      case STATE_ALL_HUMAN:  active_state = STATE_IDLE;       break;
      default:               active_state = STATE_IDLE;       break;
    }
    drawDisplay();
    Serial.printf("Mode → %d\n", active_state);
  }

  // Long press to reset Wi-Fi
  if (M5.BtnA.pressedFor(3000)) {
    reset_wifi_settings();
  }

  // Optional live telemetry (kept; harmless)
  float ax, ay, az, gx, gy, gz;
  getMPU6886Data(&ax, &ay, &az, &gx, &gy, &gz);
  ax -= accel_offset_x; ay -= accel_offset_y; az -= accel_offset_z;
  gx -= gyro_offset_x;  gy -= gyro_offset_y;  gz -= gyro_offset_z;
  Serial.printf("ACC: %.2f %.2f %.2f | GYR: %.2f %.2f %.2f\n", ax, ay, az, gx, gy, gz);

  delay(200);
}
