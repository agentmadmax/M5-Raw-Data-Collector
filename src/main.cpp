
#include "config.h"
#include "imu.h"
#include "calibration.h"
#include "raw_mode.h"
#include "web_server.h"
#include "wifi_setup.h"

void setup() {
  auto cfg = M5.config();
  cfg.internal_imu = false; // ensure we own the IMU
  cfg.external_imu = false;
  M5.begin(cfg);
  Serial.begin(115200);
  M5.Display.setRotation(1);

  setupMPU6886();       // configure IMU first
  calibrateSensors();   // then calibrate

  setup_wifi();
  setup_web_server();

  configTime(0, 0, NTP_SERVER);
  struct tm timeinfo;
  while (!getLocalTime(&timeinfo) || timeinfo.tm_year < 120) {
    Serial.print(".");
    delay(300);
  }
  Serial.println("\nTime synchronized.");

  ArduinoOTA.setHostname(client_id.c_str());
  ArduinoOTA.begin();
  drawDisplay();
}

void loop() {
  M5.update();
  ArduinoOTA.handle();

  if (!mqttClient.connected()) {
    static unsigned long lastTry = 0;
    if (millis() - lastTry > 5000) {
      lastTry = millis();
      reconnect_mqtt();
    }
  } else {
    mqttClient.loop();
  }

  if (M5.BtnA.wasPressed()) {
    switch (active_state) {
      case STATE_IDLE:       active_state = STATE_GYRO_ONLY; break;
      case STATE_GYRO_ONLY:  active_state = STATE_ACCEL_ONLY; break;
      case STATE_ACCEL_ONLY: active_state = STATE_EMG_ONLY; break;
      case STATE_EMG_ONLY:   active_state = STATE_ALL_HUMAN; break;
      case STATE_ALL_HUMAN:  active_state = STATE_IDLE; break;
      default:               active_state = STATE_IDLE;
    }
    drawDisplay();
  }
  if (M5.BtnA.pressedFor(3000)) reset_wifi_settings();

  // Live serial telemetry (post-cal)
  float ax, ay, az, gx, gy, gz;
  getMPU6886Data(&ax, &ay, &az, &gx, &gy, &gz);
  ax -= accel_offset_x; ay -= accel_offset_y; az -= accel_offset_z;
  gx -= gyro_offset_x;  gy -= gyro_offset_y;  gz -= gyro_offset_z;
  Serial.printf("ACC: %.2f  %.2f  %.2f  |  GYR: %.2f  %.2f  %.2f\n", ax, ay, az, gx, gy, gz);

  if (mqttClient.connected()) handleHumanDataCollection();

  delay(200);
}
