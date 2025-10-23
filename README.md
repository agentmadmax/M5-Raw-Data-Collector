### Setup Instructions

1. Copy `include/secrets_example.h` → rename it to `include/secrets.h`
2. Edit it with your Wi-Fi SSID, username, and password.
3. Build and upload to your M5StickC Plus 2 via PlatformIO.


# M5 Raw Data Collector (MPU6886 + EMG + MQTT)


ESP32/M5StickC Plus 2 firmware that:
- Configures the MPU6886 IMU at ±(2,4,8,26)g and ±(250,500,1000,2000) dps
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
