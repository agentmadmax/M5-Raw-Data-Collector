# M5 Raw Data Collector (MPU6886 + EMG + MQTT)

ESP32/M5StickC Plus 2 firmware that:
- Configures the MPU6886 IMU at ±2g and ±1000 dps
- Calibrates accel/gyro on boot
- Streams raw (and calibrated) IMU + EMG data via MQTT
- Serves a small web UI for status + settings
- Supports WPA2-Enterprise (EAP) via `include/secrets.h`

## Hardware
- M5StickC Plus 2 (I2C: SDA=21, SCL=22)
- EMG sensor on GPIO 32 (analog)

## Build
- PlatformIO (Arduino framework)
- `platformio.ini` included

## Secrets
1. Copy `src/secrets_example.h` to `include/secrets.h`
2. Fill in your EAP credentials
3. `include/secrets.h` is ignored by git

## License
MIT
