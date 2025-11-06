
#pragma once
#include "config.h"

struct ImuSample {
  float accX, accY, accZ, gyroX, gyroY, gyroZ;
};

struct RawPayload {
  uint64_t timestamp_us;
  uint16_t sampling_frequency;
  ImuSample imu_data[IMU_RAW_SAMPLES];
  uint16_t emg_data[IMU_RAW_SAMPLES];
} __attribute__((packed));

extern RawPayload payload_buffer;

void handleHumanDataCollection();
void start_raw_mode_tasks();
