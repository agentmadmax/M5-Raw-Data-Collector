
#pragma once
#include "config.h"

// Range config
extern float accel_range_g;      // 2,4,8,16
extern float gyro_range_dps;     // 250,500,1000,2000

extern float accel_scale_factor; // g/LSB
extern float gyro_scale_factor;  // dps/LSB

void setupMPU6886(); // I2C init + ranges
void getMPU6886Data(float* ax, float* ay, float* az, float* gx, float* gy, float* gz);
uint8_t readReg(uint8_t device_address, uint8_t register_address);

// low level helpers
void writeRegister(uint8_t address, uint8_t register_address, uint8_t value);
void readRegisters(uint8_t address, uint8_t register_address, uint8_t length, uint8_t* data);
