
#pragma once
#include "config.h"

extern float gyro_offset_x, gyro_offset_y, gyro_offset_z;
extern float accel_offset_x, accel_offset_y, accel_offset_z;

void calibrateSensors(); // interactive (keep device flat & still)
