
#include "imu.h"
#include "calibration.h"

float gyro_offset_x = 0.0f, gyro_offset_y = 0.0f, gyro_offset_z = 0.0f;
float accel_offset_x = 0.0f, accel_offset_y = 0.0f, accel_offset_z = 0.0f;

void calibrateSensors() {
  const int num_samples = 500;
  float ax_sum = 0, ay_sum = 0, az_sum = 0;
  float gx_sum = 0, gy_sum = 0, gz_sum = 0;

  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setTextSize(2);
  M5.Display.setCursor(10, 10);
  M5.Display.println("Calibrating...");
  M5.Display.setTextSize(1);
  M5.Display.println("\nKeep device flat");
  M5.Display.println("and very still.");
  Serial.println("Starting IMU calibration. Do not move the device.");

  delay(1000);
  for (int i = 0; i < num_samples; i++) {
    float ax, ay, az, gx, gy, gz;
    getMPU6886Data(&ax, &ay, &az, &gx, &gy, &gz);
    ax_sum += ax; ay_sum += ay; az_sum += az;
    gx_sum += gx; gy_sum += gy; gz_sum += gz;
    delay(5);
  }

  accel_offset_x = ax_sum / num_samples;
  accel_offset_y = ay_sum / num_samples;
  accel_offset_z = (az_sum / num_samples) - 1.0f; // expect +1g on Z when face-up

  gyro_offset_x = gx_sum / num_samples;
  gyro_offset_y = gy_sum / num_samples;
  gyro_offset_z = gz_sum / num_samples;

  Serial.println("Calibration complete.");
  Serial.printf("Accel Offsets: X=%.4f, Y=%.4f, Z=%.4f\n", accel_offset_x, accel_offset_y, accel_offset_z);
  Serial.printf("Gyro Offsets:  X=%.4f, Y=%.4f, Z=%.4f\n", gyro_offset_x, gyro_offset_y, gyro_offset_z);

  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setTextSize(2);
  M5.Display.println("Calibration");
  M5.Display.println("Complete!");
  delay(1000);
}
