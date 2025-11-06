
#include "imu.h"

float accel_range_g = 2.0f;//2,4,8,16
float gyro_range_dps = 250.0f;//250,500,1000,,2000

//float accel_scale_factor = 1.0f / 8192.0f; // default for ±4g
//float accel_scale_factor = 1.0f / 4096.0f; // default for ±8g
//float accel_scale_factor = 1.0f / 2048.0ff; // default for ±16g
float accel_scale_factor = 1.0f / 16384.0f; // default for ±2g
//float gyro_scale_factor  = 1000.0f / 32768.0f;// default for ±1000 dps
float gyro_scale_factor = 250.0f / 32768.0f; // default for ±250
//float gyro_scale_factor = 500.0f / 32768.0f; // default for ±500
//float gyro_scale_factor = 2000.0f / 32768.0f; // default for ±2000

void writeRegister(uint8_t address, uint8_t register_address, uint8_t value) {
  Wire1.beginTransmission(address);
  Wire1.write(register_address);
  Wire1.write(value);
  Wire1.endTransmission();
}

void readRegisters(uint8_t address, uint8_t register_address, uint8_t length, uint8_t* data) {
  Wire1.beginTransmission(address);
  Wire1.write(register_address);
  Wire1.endTransmission(false);
  Wire1.requestFrom(address, (uint8_t)length);
  for (int i = 0; i < length; i++) data[i] = Wire1.read();
}

uint8_t readReg(uint8_t device_address, uint8_t register_address) {
  Wire1.beginTransmission(device_address);
  Wire1.write(register_address);
  Wire1.endTransmission(false);
  Wire1.requestFrom(device_address, (uint8_t)1);
  return Wire1.read();
}

void setupMPU6886() {
  Wire1.begin(21, 22);
  delay(100);

  // wake
  writeRegister(0x68, 0x6B, 0x00);
  delay(100);

  // accel range
  uint8_t accel_config_value;
  if (accel_range_g == 16.0f) accel_config_value = 0x18;
  else if (accel_range_g == 8.0f) accel_config_value = 0x10;
  else if (accel_range_g == 4.0f) accel_config_value = 0x08;
  else accel_config_value = 0x00; // ±2g

  writeRegister(0x68, 0x1C, accel_config_value);
  // scale factor strictly from register (defensive)
  switch ((accel_config_value >> 3) & 0x03) {
    case 0: accel_scale_factor = 1.0f / 16384.0f; break; // ±2g
    case 1: accel_scale_factor = 1.0f / 8192.0f;  break; // ±4g
    case 2: accel_scale_factor = 1.0f / 4096.0f;  break; // ±8g
    case 3: accel_scale_factor = 1.0f / 2048.0f;  break; // ±16g
  }

  // gyro range
  uint8_t gyro_config_value;
  if (gyro_range_dps == 2000.0f) gyro_config_value = 0x18;
  else if (gyro_range_dps == 1000.0f) gyro_config_value = 0x10;
  else if (gyro_range_dps == 500.0f)  gyro_config_value = 0x08;
  else gyro_config_value = 0x00; // 250 dps

  writeRegister(0x68, 0x1B, gyro_config_value);
  switch ((gyro_config_value >> 3) & 0x03) {
    case 0: gyro_scale_factor = 250.0f  / 32768.0f; break;
    case 1: gyro_scale_factor = 500.0f  / 32768.0f; break;
    case 2: gyro_scale_factor = 1000.0f / 32768.0f; break;
    case 3: gyro_scale_factor = 2000.0f / 32768.0f; break;
  }

  uint8_t a = readReg(0x68, 0x1C);
  uint8_t g = readReg(0x68, 0x1B);
  Serial.printf("MPU6886 configured. ACCEL_CONFIG=0x%02X  GYRO_CONFIG=0x%02X\n", a, g);
}

void getMPU6886Data(float* ax, float* ay, float* az, float* gx, float* gy, float* gz) {
  uint8_t data[14];
  readRegisters(0x68, 0x3B, 14, data);

  int16_t accel_x_raw = (data[0] << 8) | data[1];
  int16_t accel_y_raw = (data[2] << 8) | data[3];
  int16_t accel_z_raw = (data[4] << 8) | data[5];

  int16_t gyro_x_raw = (data[8] << 8) | data[9];
  int16_t gyro_y_raw = (data[10] << 8) | data[11];
  int16_t gyro_z_raw = (data[12] << 8) | data[13];

  *ax = (float)accel_x_raw * accel_scale_factor;
  *ay = (float)accel_y_raw * accel_scale_factor;
  *az = (float)accel_z_raw * accel_scale_factor;

  *gx = (float)gyro_x_raw * gyro_scale_factor;
  *gy = (float)gyro_y_raw * gyro_scale_factor;
  *gz = (float)gyro_z_raw * gyro_scale_factor;
}