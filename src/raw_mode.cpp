#include "imu.h"
#include "calibration.h"
#include "raw_mode.h"
#include "config.h"

#include <Arduino.h>
#include <sys/time.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

//  externs from config.h 
extern PubSubClient mqttClient;
extern String client_id;
extern uint8_t active_state;
extern int sampling_frequency;

// ===== Buffers & Queue =====
static RawPayload bufA, bufB;
static RawPayload* volatile fillBuf = &bufA;
static QueueHandle_t readyQ = nullptr;


// Helper: Quantize timestamp in nanoseconds to sampling interval

static inline uint64_t getQuantizedTimestampNs(uint16_t samplingHz) {
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  const uint64_t ts_ns = (uint64_t)tv.tv_sec * 1000000000ULL + (uint64_t)tv.tv_usec * 1000ULL;
  const uint64_t interval_ns = 1000000000ULL / samplingHz;
  return (ts_ns / interval_ns) * interval_ns;
}


// Sampling Task  (runs on Core 0, high priority)

static void samplingTask(void* /*param*/) {
  for (;;) {
    uint16_t fs = (sampling_frequency > 0) ? sampling_frequency : DEFAULT_SAMPLING_HZ;
    const uint64_t t0_ns = getQuantizedTimestampNs(fs);

    RawPayload* b = (RawPayload*)fillBuf;
    b->timestamp_us = t0_ns / 1000ULL;
    b->sampling_frequency = fs;

    const uint32_t microPerSample = 1000000UL / fs;
    uint32_t nextT = micros();

    for (int i = 0; i < IMU_RAW_SAMPLES; ++i) {
      while ((int32_t)(micros() - nextT) < 0) { /* tight wait */ }
      nextT += microPerSample;

      float ax, ay, az, gx, gy, gz;
      getMPU6886Data(&ax, &ay, &az, &gx, &gy, &gz);

      if (active_state & ACCEL_BIT) {
        b->imu_data[i].accX = ax - accel_offset_x;
        b->imu_data[i].accY = ay - accel_offset_y;
        b->imu_data[i].accZ = az - accel_offset_z;
      } else {
        b->imu_data[i].accX = b->imu_data[i].accY = b->imu_data[i].accZ = 0;
      }

      if (active_state & GYRO_BIT) {
        b->imu_data[i].gyroX = gx - gyro_offset_x;
        b->imu_data[i].gyroY = gy - gyro_offset_y;
        b->imu_data[i].gyroZ = gz - gyro_offset_z;
      } else {
        b->imu_data[i].gyroX = b->imu_data[i].gyroY = b->imu_data[i].gyroZ = 0;
      }

      b->emg_data[i] = (active_state & EMG_BIT) ? analogRead(EMG_SENSOR_PIN) : 0;

      if ((i % 8) == 0) taskYIELD();  
    }

    RawPayload* completed = b;
    fillBuf = (completed == &bufA) ? &bufB : &bufA;
    xQueueSend(readyQ, &completed, portMAX_DELAY);

    vTaskDelay(1); 
  }
}


// Publish Task  (runs on Core 0, medium priority)

static void publishTask(void* /*param*/) {
  char topic[96];
  snprintf(topic, sizeof(topic), "human/%s/raw_binary", client_id.c_str());

  for (;;) {
    RawPayload* toSend = nullptr;
    if (xQueueReceive(readyQ, &toSend, portMAX_DELAY) == pdTRUE) {
      bool ok = mqttClient.publish(topic,
          reinterpret_cast<const uint8_t*>(toSend),
          sizeof(RawPayload),
          false);

    if (!ok) {
    Serial.println(" MQTT publish failed — buffer not sent");
  }

    }
  }
}


// Compatibility shim (legacy call)

void handleHumanDataCollection() {

}


// Start both tasks on Core 0 (keep Core 1 free for UI/OTA/buttons)

void start_raw_mode_tasks() {
  if (!readyQ) readyQ = xQueueCreate(4, sizeof(RawPayload*));

  // Sampler: high priority on Core 0
  xTaskCreatePinnedToCore(
      samplingTask, "sampler", 4096, nullptr,
      configMAX_PRIORITIES - 2, nullptr, 0);

  // Publisher: normal priority on Core 1
  xTaskCreatePinnedToCore(
      publishTask, "publisher", 4096, nullptr,
      tskIDLE_PRIORITY + 2, nullptr, 1);

  Serial.println("Sampling + publishing started on Core 0 (UI on Core 1)");
}
