#include <M5Unified.h>
#include <WiFi.h>
#include "esp_wpa2.h"
#include "esp_wifi.h"
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>
#include <time.h>
#include <ArduinoJson.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>

// --- Server & Global Config ---
AsyncWebServer server(8080);
char mqtt_server[40] = "20.172.67.240";
char eap_username[64] = "";
char eap_password[64] = "";
// --- NEW: Global variables for sensor calibration offsets ---
float gyro_offset_x = 0.0, gyro_offset_y = 0.0, gyro_offset_z = 0.0;
float accel_offset_x = 0.0, accel_offset_y = 0.0, accel_offset_z = 0.0;

// --- MPU6886 LOW-LEVEL CONFIGURATION ---
float accel_range_g = 2.0;      // Set to 2, 4, 8, or 16
float gyro_range_dps = 1000.0;  // Set to 250, 500, 1000, or 2000

float accel_scale_factor;
float gyro_scale_factor;

// Function to write a single byte to an MPU6886 register
void writeRegister(uint8_t address, uint8_t register_address, uint8_t value) {
  Wire1.beginTransmission(address);
  Wire1.write(register_address);
  Wire1.write(value);
  Wire1.endTransmission();
}

// Function to read multiple bytes from MPU6886 registers
void readRegisters(uint8_t address, uint8_t register_address, uint8_t length, uint8_t* data) {
  Wire1.beginTransmission(address);
  Wire1.write(register_address);
  Wire1.endTransmission(false);
  Wire1.requestFrom(address, (uint8_t)length);
  for (int i = 0; i < length; i++) {
    data[i] = Wire1.read();
  }
}

// Function to initialize and configure the MPU6886
void setupMPU6886() {
  Wire1.begin(21, 22);
  delay(100);

  writeRegister(0x68, 0x6B, 0x00);
  delay(100);

  uint8_t accel_config_value;
  if (accel_range_g == 16.0) accel_config_value = 0x18;
  else if (accel_range_g == 8.0) accel_config_value = 0x10;
  else if (accel_range_g == 4.0) accel_config_value = 0x08;
  else accel_config_value = 0x00;
  writeRegister(0x68, 0x1C, accel_config_value);
 //accel_scale_factor = accel_range_g / 32768.0;
   accel_scale_factor = 1.0 / 16384.0;  // fixed scale for ±2g 
 //accel_scale_factor = 1.0 / 8192.0;   // fixed scale for ±4g
 //accel_scale_factor = 1.0 / 4096.0;   // fixed scale for ±8g
 //accel_scale_factor = 1.0 / 2048.0;   // fixed scale for ±16g



  uint8_t gyro_config_value;
  if (gyro_range_dps == 2000.0) gyro_config_value = 0x18;
  else if (gyro_range_dps == 1000.0) gyro_config_value = 0x10;
  else if (gyro_range_dps == 500.0) gyro_config_value = 0x08;
  else gyro_config_value = 0x00;
  writeRegister(0x68, 0x1B, gyro_config_value);
  gyro_scale_factor = gyro_range_dps / 32768.0;

  Serial.println("MPU6886 manually configured.");
}

// Function to read and process sensor data
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

// --- HARDWARE, STATES, & DATA STRUCTURES ---
#define EMG_SENSOR_PIN 32
const int IMU_RAW_SAMPLES = 64;

const uint8_t ACCEL_BIT = 2;
const uint8_t GYRO_BIT  = 4;
const uint8_t EMG_BIT   = 8;
const int STATE_IDLE       = 0;
const int STATE_GYRO_ONLY  = GYRO_BIT;
const int STATE_ACCEL_ONLY = ACCEL_BIT;
const int STATE_EMG_ONLY   = EMG_BIT;
const int STATE_ALL_HUMAN  = GYRO_BIT | ACCEL_BIT | EMG_BIT;

struct ImuSample {
  float accX, accY, accZ, gyroX, gyroY, gyroZ;
};

struct RawPayload {
  uint64_t timestamp_us;
  uint16_t sampling_frequency;
  ImuSample imu_data[IMU_RAW_SAMPLES];
  uint16_t emg_data[IMU_RAW_SAMPLES];
} __attribute__((packed));

RawPayload payload_buffer;
uint8_t active_state = STATE_IDLE;
int sampling_frequency = 100;
const char* ntpServer = "pool.ntp.org";
WiFiClient espClient;
PubSubClient client(espClient);
String client_id = "";
unsigned long lastRawPublishTime = 0;
unsigned long lastReconnectAttempt = 0;
unsigned int publish_count = 0;

// --- Web Page HTML ---
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>
  <title>M5 Motor Monitor</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.5.1/css/all.min.css">
  <link href="https://fonts.googleapis.com/css2?family=Poppins:wght@400;600&display=swap" rel="stylesheet">
  <style>
    body { font-family: 'Poppins', sans-serif; background-color: #1e272e; color: #d2dae2; margin: 0; padding: 10px; }
    .container { max-width: 800px; margin: 0 auto; }
    .header { text-align: center; color: #48dbfb; margin-bottom: 20px; }
    .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(250px, 1fr)); gap: 20px; }
    .card { background-color: #34495e; padding: 20px; border-radius: 12px; }
    .card h3 { margin-top: 0; border-bottom: 2px solid #48dbfb; padding-bottom: 10px; }
    .info-item { display: flex; align-items: center; margin-bottom: 10px; }
    .info-item i { width: 30px; text-align: center; color: #48dbfb; margin-right: 10px; }
    .value { color: #f1c40f; }
    input, select { width: 100%; box-sizing: border-box; padding: 10px; margin-top: 5px; border-radius: 6px; background-color: #2c3e50; color: #fff; }
    button { width: 100%; padding: 12px; margin-top: 10px; border-radius: 6px; border: none; background-color: #48dbfb; color: #1e272e; font-weight: bold; cursor: pointer; }
  </style>
</head><body>
  <div class="container">
    <div class="header"><h1><i class="fa-solid fa-person-running"></i> Activity Logger</h1></div>
    <div class="grid">
      <div class="card">
        <h3><i class="fa-solid fa-info-circle"></i> Status</h3>
        <div class="info-item"><i class="fa-solid fa-wifi"></i><span>WiFi:&nbsp;</span> <span id="wifi_ssid" class="value">...</span></div>
        <div class="info-item"><i class="fa-solid fa-network-wired"></i><span>IP:&nbsp;</span> <span id="ip_addr" class="value">...</span></div>
        <div class="info-item"><i id="battery-icon" class="fa-solid fa-bolt"></i><span>Voltage:&nbsp;</span> <span id="battery" class="value">...</span> V</div>
      </div>
      <div class="card">
        <h3><i class="fa-solid fa-sliders"></i> Mode Control</h3>
        <label for="mode">Select Mode:</label>
        <select id="mode" onchange="setMode()">
          <option value="0">Idle</option>
          <option value="4">Gyro Only</option>
          <option value="2">Accelerometer Only</option>
          <option value="8">EMG Only</option>
          <option value="14">All Sensors</option>
        </select>
      </div>
      <div class="card">
        <h3><i class="fa-solid fa-chart-line"></i> Analysis Settings</h3>
        <label for="sampling_rate">Sampling Rate (Hz):</label>
        <input type="number" id="sampling_rate" value="1000">
        <button onclick="setSamplingRate()">Set Rate</button>
      </div>
      <div class="card">
        <h3><i class="fa-solid fa-server"></i> Network Settings</h3>
        <label for="mqtt_server">MQTT Server IP:</label>
        <input type="text" id="mqtt_server">
        <button onclick="setMqttServer()">Set Server</button>
      </div>
    </div>
  </div>
  <script>
    function getStats() {
      fetch('/stats').then(response => response.json()).then(data => {
        document.getElementById('wifi_ssid').innerText = data.wifi_ssid;
        document.getElementById('ip_addr').innerText = data.ip_addr;
        document.getElementById('battery').innerText = data.battery.toFixed(2);
        document.getElementById('mode').value = data.active_state;
        if (document.activeElement !== document.getElementById('sampling_rate')) {
          document.getElementById('sampling_rate').value = data.sampling_rate;
        }
        if (document.activeElement !== document.getElementById('mqtt_server')) {
          document.getElementById('mqtt_server').value = data.mqtt_server;
        }
      }).catch(error => console.error('Error fetching stats:', error));
    }
    function setMode() { fetch(`/set?mode=${document.getElementById('mode').value}`); }
    function setSamplingRate() { fetch(`/set?rate=${document.getElementById('sampling_rate').value}`); }
    function setMqttServer() { fetch(`/set?mqtt_server=${document.getElementById('mqtt_server').value}`); }
    setInterval(getStats, 8000);
    window.onload = getStats;
  </script>
</body></html>
)rawliteral";

// --- Function Definitions ---
void drawDisplay();
void handleHumanDataCollection();
String getFormattedTimestamp();
bool reconnect();
void reset_wifi_settings();
void calibrateSensors();

// --- CORRECTED: Function to calibrate the IMU on startup ---
void calibrateSensors() {
     delay(7000);
    const int num_samples = 500;
    float ax_sum = 0, ay_sum = 0, az_sum = 0;
    float gx_sum = 0, gy_sum = 0, gz_sum = 0;

    M5.Display.fillScreen(TFT_BLACK);
    M5.Display.setTextSize(2);
    M5.Display.setCursor(10, 10);
    M5.Display.println("Calibrating...");
    M5.Display.setTextSize(1);
    M5.Display.println("\nPlease keep device");
    M5.Display.println("flat and still.");
    Serial.println("Starting IMU calibration. Do not move the device.");

    // Take a large number of readings using the custom function
    for (int i = 0; i < num_samples; i++) {
        float ax, ay, az, gx, gy, gz;
        
        // --- THIS IS THE KEY CHANGE ---
        // Use the same function as the main data collection loop
        getMPU6886Data(&ax, &ay, &az, &gx, &gy, &gz);
        // ---

        ax_sum += ax; ay_sum += ay; az_sum += az;
        gx_sum += gx; gy_sum += gy; gz_sum += gz;
        delay(5); // Small delay between samples
    }

    // Calculate the average offset for each axis
    accel_offset_x = ax_sum / num_samples;
    accel_offset_y = ay_sum / num_samples;
    // For the Z-axis, we expect 1g from gravity when flat.
    // The offset is the difference between the average reading and 1.0.
    accel_offset_z = (az_sum / num_samples) - 1.0; 

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
    delay(2000);
}

// --- Battery Drawing Function ---
void drawBatteryIcon(uint8_t percentage, bool is_charging) {
  int icon_x = M5.Display.width() - 32;
  int icon_y = 6;
  int icon_w = 25;
  int icon_h = 12;

  M5.Display.drawRect(icon_x, icon_y, icon_w, icon_h, TFT_WHITE);
  M5.Display.fillRect(icon_x + icon_w, icon_y + 3, 2, icon_h - 6, TFT_WHITE);

  uint16_t battery_color = (percentage > 50) ? TFT_GREEN :
                           (percentage > 20) ? TFT_YELLOW : TFT_RED;

  int fill_w = map(percentage, 0, 100, 0, icon_w - 2);
  M5.Display.fillRect(icon_x + 1, icon_y + 1, fill_w, icon_h - 2, battery_color);
  M5.Display.setFont(&fonts::Font0);
  M5.Display.setTextDatum(MC_DATUM);
  M5.Display.setTextSize(1);
  M5.Display.setTextColor(TFT_RED);
  M5.Display.drawString(String(percentage), icon_x + icon_w / 2, icon_y + icon_h / 2 + 1);
  M5.Display.setFont(&fonts::Font0);

  if (is_charging) {
    M5.Display.setTextColor(TFT_YELLOW);
    M5.Display.drawString("⚡", icon_x - 8, icon_y + icon_h / 2);
  }
}

// --- Display Function ---
void drawDisplay() {
  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);

  uint8_t bat_percent = M5.Power.getBatteryLevel();
  bool is_charging = M5.Power.isCharging();
  drawBatteryIcon(bat_percent, is_charging);
  M5.Display.setTextDatum(TL_DATUM);
  M5.Display.setTextSize(1.8);

  String activeSensors = "";
  switch (active_state) {
    case STATE_IDLE: break;
    case STATE_GYRO_ONLY: activeSensors = "GYRO"; break;
    case STATE_ACCEL_ONLY: activeSensors = "ACCEL"; break;
    case STATE_EMG_ONLY: activeSensors = "EMG"; break;
    case STATE_ALL_HUMAN: activeSensors = "ACCEL GYRO EMG"; break;
    default: activeSensors = "Unknown"; break;
  }

  if (active_state != STATE_IDLE && active_state != 0) {
    M5.Display.setCursor(5, 10);
    M5.Display.print("Mode: Raw Data");
    M5.Display.setTextSize(1.5);
    M5.Display.setCursor(5, 30);
    M5.Display.printf("Sensors: %s", activeSensors.c_str());
  }

  M5.Display.drawLine(0, 45, M5.Display.width(), 45, TFT_DARKGREY);
  M5.Display.setTextSize(1.8);
  M5.Display.setCursor(5, 55); M5.Display.printf("IP:   %s", WiFi.localIP().toString().c_str());
  M5.Display.setCursor(5, 70); M5.Display.printf("MAC:  %s", WiFi.macAddress().c_str());
  M5.Display.setCursor(5, 85); M5.Display.printf("MQTT: %s", mqtt_server);
  M5.Display.setCursor(5, 100); M5.Display.printf("Rate: %d Hz", sampling_frequency);
  M5.Display.setCursor(5, 115);
  M5.Display.print("Connection: ");
  M5.Display.setTextColor(client.connected() ? TFT_GREEN : TFT_RED, TFT_BLACK);
  M5.Display.print(client.connected() ? "OK" : "FAIL");
}

// --- Timestamp Formatter ---
String getFormattedTimestamp() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    return "1970-01-01T00:00:00Z";
  }
  char timeStringBuff[30];
  strftime(timeStringBuff, sizeof(timeStringBuff), "%Y-%m-%dT%H:%M:%SZ", &timeinfo);
  return String(timeStringBuff);
}

// --- MQTT Reconnect ---
bool reconnect() {
  Serial.print("Attempting MQTT connection...");

  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setCursor(5, 10);
  M5.Display.setTextSize(1.8);
  M5.Display.setTextColor(TFT_RED, TFT_BLACK);
  M5.Display.println("MQTT Disconnected!");
  M5.Display.drawLine(0, 30, M5.Display.width(), 30, TFT_DARKGREY);

  M5.Display.setTextSize(1.5);
  M5.Display.setCursor(5, 40);
  M5.Display.printf("IP:   %s\n", WiFi.localIP().toString().c_str());
  M5.Display.setCursor(5, 55);
  M5.Display.printf("WiFi: %s\n", WiFi.SSID().c_str());
  M5.Display.setCursor(5, 70);
  M5.Display.printf("MQTT: %s\n\n", mqtt_server);
  M5.Display.println("Attempting to reconnect...");

  if (client.connect(client_id.c_str())) {
    Serial.println("connected");
    drawDisplay();
  } else {
    Serial.print("failed, rc=");
    Serial.println(client.state());
  }
  return client.connected();
}

// --- WiFi Setup ---
void setup_wifi() {
  const char* EAP_SSID     = "PAWS-Secure";
  const char* EAP_USERNAME = "sc30598";
  const char* EAP_PASSWORD = "Madmax123@";
  const int WIFI_TIMEOUT_MS = 15000;

  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Display.setTextSize(1.8);
  M5.Display.setCursor(5, 10);
  M5.Display.printf("Connecting to:\n %s", EAP_SSID);

  WiFi.disconnect(true);
  WiFi.mode(WIFI_STA);

  esp_wifi_sta_wpa2_ent_set_identity((uint8_t *)EAP_USERNAME, strlen(EAP_USERNAME));
  esp_wifi_sta_wpa2_ent_set_username((uint8_t *)EAP_USERNAME, strlen(EAP_USERNAME));
  esp_wifi_sta_wpa2_ent_set_password((uint8_t *)EAP_PASSWORD, strlen(EAP_PASSWORD));
  esp_wifi_sta_wpa2_ent_set_disable_time_check(true);
  esp_wifi_sta_wpa2_ent_enable();

  WiFi.begin(EAP_SSID);

  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - startTime > WIFI_TIMEOUT_MS) {
      M5.Display.println("\n\nConnection failed.");
      delay(2000);
      break;
    }
    M5.Display.print(".");
    delay(500);
  }

  if (WiFi.status() == WL_CONNECTED) {
    M5.Display.println("\n\nConnected successfully!");
    delay(2000);
    return;
  }

  // --- WiFi Manager Fallback ---
  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setTextSize(1.8);
  M5.Display.setCursor(5, 10);
  M5.Display.println("WiFi Portal Started");
  M5.Display.setCursor(5, 40);
  M5.Display.println("Connect to:");
  M5.Display.setCursor(5, 70);
  M5.Display.printf("MAC: %s\n", WiFi.macAddress().c_str());

  WiFiManager wm;
  WiFiManagerParameter custom_mqtt_server("server", "MQTT Server IP", mqtt_server, 40);
  WiFiManagerParameter custom_eap_user("eap_user", "EAP Username (optional)", eap_username, 64);
  WiFiManagerParameter custom_eap_pass("eap_pass", "EAP Password (optional)", eap_password, 64);

  wm.addParameter(&custom_mqtt_server);
  wm.addParameter(&custom_eap_user);
  wm.addParameter(&custom_eap_pass);
  wm.setConnectTimeout(20);

  String mac = WiFi.macAddress();
  mac.replace(":", "");
  String portal_name = "M5-Portal-" + mac;

  if (!wm.autoConnect(portal_name.c_str())) {
    M5.Display.println("Portal Started.");
    M5.Display.println("Connect to ");
    M5.Display.println(portal_name);
    M5.Display.printf("IP: 192.168.4.1\n");
    if (!wm.startConfigPortal(portal_name.c_str())) {
      M5.Display.println("Setup Failed. Restarting.");
      delay(3000);
      ESP.restart();
    }
  }

  strcpy(mqtt_server, custom_mqtt_server.getValue());
  if (strcmp(mqtt_server, "") == 0) strcpy(mqtt_server, "sensorweb.us");

  strcpy(eap_username, custom_eap_user.getValue());
  strcpy(eap_password, custom_eap_pass.getValue());

  if (strlen(eap_username) > 0 && strlen(eap_password) > 0) {
    String ssid = wm.getWiFiSSID();
    WiFi.disconnect(true);
    WiFi.mode(WIFI_STA);
    esp_wifi_sta_wpa2_ent_set_identity((uint8_t *)eap_username, strlen(eap_username));
    esp_wifi_sta_wpa2_ent_set_username((uint8_t *)eap_username, strlen(eap_username));
    esp_wifi_sta_wpa2_ent_set_password((uint8_t *)eap_password, strlen(eap_password));
    esp_wifi_sta_wpa2_ent_enable();
    WiFi.begin(ssid.c_str());

    startTime = millis();
    M5.Display.fillScreen(TFT_BLACK);
    M5.Display.setCursor(5, 10);
    M5.Display.printf("Connecting to EAP:\n %s", ssid.c_str());
    while (WiFi.status() != WL_CONNECTED) {
      if (millis() - startTime > WIFI_TIMEOUT_MS) {
        M5.Display.println("\n\nEAP from portal FAILED.");
        delay(3000);
        break;
      }
      M5.Display.print(".");
      delay(500);
    }
  }
}

// --- Reset WiFi Settings ---
void reset_wifi_settings() {
  esp_wifi_restore();
  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setCursor(5, 10);
  M5.Display.println("WiFi credentials erased!");
  M5.Display.println("Rebooting...");
  delay(2000);
  ESP.restart();
}
uint8_t readReg(uint8_t device_address, uint8_t register_address) {
  Wire1.beginTransmission(device_address);
  Wire1.write(register_address);
  Wire1.endTransmission(false);
  Wire1.requestFrom(device_address, (uint8_t)1);
  return Wire1.read();
}

// --- Setup ---
void setup() {
  auto cfg = M5.config();
  cfg.internal_imu   = false;   // stops M5Unified from touching the chip
  cfg.external_imu   = false;
  M5.begin(cfg);
  Serial.begin(115200);
  M5.Display.setRotation(1);
  setupMPU6886();
  calibrateSensors();

  setup_wifi();
  uint8_t accel_cfg = readReg(0x68, 0x1C);
  uint8_t gyro_cfg  = readReg(0x68, 0x1B);
  Serial.printf("ACCEL_CONFIG=0x%02X  GYRO_CONFIG=0x%02X\n", accel_cfg, gyro_cfg);


  configTime(0, 0, ntpServer);
  struct tm timeinfo;
  while (!getLocalTime(&timeinfo) || timeinfo.tm_year < 120) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("\nTime synchronized.");

  client_id += "M5-HumanLogger-";
  client_id += WiFi.macAddress();
  client_id.replace(":", "");
  client.setServer(mqtt_server, 1883);

  // Web Endpoints
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", index_html);
  });

  server.on("/stats", HTTP_GET, [](AsyncWebServerRequest *request) {
    JsonDocument doc;
    doc["wifi_ssid"] = WiFi.SSID();
    doc["ip_addr"] = WiFi.localIP().toString();
    doc["battery"] = M5.Power.getBatteryVoltage() / 1000.0;
    doc["mqtt_connected"] = client.connected();
    doc["active_state"] = active_state;
    doc["sampling_rate"] = sampling_frequency;
    doc["mqtt_server"] = mqtt_server;
    String json_response;
    serializeJson(doc, json_response);
    request->send(200, "application/json", json_response);
  });

  server.on("/set", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("mode")) {
      active_state = request->getParam("mode")->value().toInt();
    }
    if (request->hasParam("rate")) {
      sampling_frequency = request->getParam("rate")->value().toInt();
    }
    if (request->hasParam("mqtt_server")) {
      String new_mqtt_server = request->getParam("mqtt_server")->value();
      strcpy(mqtt_server, new_mqtt_server.c_str());
      client.setServer(mqtt_server, 1883);
      client.disconnect();
      lastReconnectAttempt = 0;
    }
    drawDisplay();
    request->send(200, "text/plain", "OK");
  });

  server.begin();
  ArduinoOTA.setHostname(client_id.c_str());
  ArduinoOTA.begin();
  drawDisplay();
  Serial.println("Web server started.");
}

// --- Loop ---
void loop() {
  M5.update();
  ArduinoOTA.handle();

  if (!client.connected()) {
    long now = millis();
    if (now - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = now;
      if (reconnect()) lastReconnectAttempt = 0;
    }
  } else {
    client.loop();
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

  if (M5.BtnA.pressedFor(3000)) {
    reset_wifi_settings();
  }

  if (client.connected()) {
    handleHumanDataCollection();
  } 
  float ax, ay, az;
  float gx, gy, gz;
 getMPU6886Data(&ax, &ay, &az, &gx, &gy, &gz);
  ax -= accel_offset_x;
  ay -= accel_offset_y;
  az -= accel_offset_z;
  gx -= gyro_offset_x;
  gy -= gyro_offset_y;
  gz -= gyro_offset_z;

  // Print to Serial Monitor
  Serial.printf("ACC: %.2f  %.2f  %.2f  |  GYR: %.2f  %.2f  %.2f\n",
                ax, ay, az, gx, gy, gz);
              
 delay(200);
}

 // --- CORRECTED DATA COLLECTION FUNCTION ---
void handleHumanDataCollection() {
    // Exit if the device is in idle mode
    if (active_state == STATE_IDLE) {
        return;
    }

    // 1. Get timestamp and frequency for the entire payload
    struct timeval tv;
    gettimeofday(&tv, NULL);
    payload_buffer.timestamp_us = (uint64_t)tv.tv_sec * 1000000L + tv.tv_usec;
    payload_buffer.sampling_frequency = sampling_frequency;

    unsigned long microSecondsPerSample = 1000000 / sampling_frequency;
    unsigned long nextSampleTime = micros();

    // 2. Loop and collect sensor data for each sample
    for (int i = 0; i < IMU_RAW_SAMPLES; i++) {
        // Precise timing for consistent sampling rate
        while (micros() < nextSampleTime) {
            delayMicroseconds(1);
        }
        nextSampleTime += microSecondsPerSample;

        // Create temporary variables to hold the raw sensor readings
        float ax_raw = 0, ay_raw = 0, az_raw = 0;
        float gx_raw = 0, gy_raw = 0, gz_raw = 0;
        uint16_t emg = 0;

        // Read the raw data from the sensor using your custom function
        getMPU6886Data(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);

        // --- APPLY CALIBRATION AND STORE THE FINAL VALUE ---
        // Conditionally apply calibration based on the active state
        if (active_state & ACCEL_BIT) {
            payload_buffer.imu_data[i].accX = ax_raw - accel_offset_x;
            payload_buffer.imu_data[i].accY = ay_raw - accel_offset_y;
            payload_buffer.imu_data[i].accZ = az_raw - accel_offset_z; // Corrected Z-axis logic
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

        // Read EMG if active
        if (active_state & EMG_BIT) {
            emg = analogRead(EMG_SENSOR_PIN);
        }
        payload_buffer.emg_data[i] = emg;
    }

    // 3. Publish the complete, calibrated payload
    char topic[100];
    snprintf(topic, sizeof(topic), "human/%s/raw_binary", client_id.c_str());
    client.publish(topic, (const uint8_t*)&payload_buffer, sizeof(payload_buffer));
    publish_count++;
}

