
#include "config.h"
#include "web_server.h"

extern PubSubClient mqttClient;
extern uint8_t active_state;
extern int sampling_frequency;
extern char mqtt_server[40];

static const char index_html[] PROGMEM = R"HTML(
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
        <input type="number" id="sampling_rate" value="100">
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
)HTML";

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
    case 0: break;
    case STATE_GYRO_ONLY: activeSensors = "GYRO"; break;
    case STATE_ACCEL_ONLY: activeSensors = "ACCEL"; break;
    case STATE_EMG_ONLY: activeSensors = "EMG"; break;
    case STATE_ALL_HUMAN: activeSensors = "ACCEL GYRO EMG"; break;
    default: activeSensors = "Unknown"; break;
  }

  if (active_state != 0) {
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
  M5.Display.setTextColor(mqttClient.connected() ? TFT_GREEN : TFT_RED, TFT_BLACK);
  M5.Display.print(mqttClient.connected() ? "OK" : "FAIL");
}

void setup_web_server() {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", index_html);
  });

  server.on("/stats", HTTP_GET, [](AsyncWebServerRequest *request) {
    JsonDocument doc;
    doc["wifi_ssid"] = WiFi.SSID();
    doc["ip_addr"] = WiFi.localIP().toString();
    doc["battery"] = M5.Power.getBatteryVoltage() / 1000.0;
    doc["mqtt_connected"] = mqttClient.connected();
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
      strncpy(mqtt_server, new_mqtt_server.c_str(), sizeof(::mqtt_server)-1);
      mqtt_server[sizeof(::mqtt_server)-1] = 0;
      mqttClient.setServer(mqtt_server, 1883);
      mqttClient.disconnect();
    }
    drawDisplay();
    request->send(200, "text/plain", "OK");
  });

  server.begin();
  Serial.println("Web server started.");
}
