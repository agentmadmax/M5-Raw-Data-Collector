
#include "imu.h"
#include "calibration.h"
#include "raw_mode.h"
#include "config.h"

RawPayload payload_buffer;

void handleHumanDataCollection() {
  if (active_state == STATE_IDLE) return;

  struct timeval tv;
  gettimeofday(&tv, NULL);
  payload_buffer.timestamp_us = (uint64_t)tv.tv_sec * 1000000ULL + tv.tv_usec;
  payload_buffer.sampling_frequency = sampling_frequency;

  unsigned long microPerSample = 1000000UL / sampling_frequency;
  unsigned long nextT = micros();

  for (int i = 0; i < IMU_RAW_SAMPLES; i++) {
    while (micros() < nextT) { delayMicroseconds(1); }
    nextT += microPerSample;

    float ax_raw=0, ay_raw=0, az_raw=0;
    float gx_raw=0, gy_raw=0, gz_raw=0;
    getMPU6886Data(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);

    if (active_state & ACCEL_BIT) {
      payload_buffer.imu_data[i].accX = ax_raw - accel_offset_x;
      payload_buffer.imu_data[i].accY = ay_raw - accel_offset_y;
      payload_buffer.imu_data[i].accZ = az_raw - accel_offset_z;
    } else {
      payload_buffer.imu_data[i].accX = 0;
      payload_buffer.imu_data[i].accY = 0;
      payload_buffer.imu_data[i].accZ = 0;
    }

    if (active_state & GYRO_BIT) {
      payload_buffer.imu_data[i].gyroX = gx_raw - gyro_offset_x;
      payload_buffer.imu_data[i].gyroY = gy_raw - gyro_offset_y;
      payload_buffer.imu_data[i].gyroZ = gz_raw - gyro_offset_z;
    } else {
      payload_buffer.imu_data[i].gyroX = 0;
      payload_buffer.imu_data[i].gyroY = 0;
      payload_buffer.imu_data[i].gyroZ = 0;
    }

    uint16_t emg = 0;
    if (active_state & EMG_BIT) {
      emg = analogRead(EMG_SENSOR_PIN);
    }
    payload_buffer.emg_data[i] = emg;
  }

  char topic[100];
  snprintf(topic, sizeof(topic), "human/%s/raw_binary", client_id.c_str());
  mqttClient.publish(topic, (const uint8_t*)&payload_buffer, sizeof(payload_buffer));
}
