#include "config.h"
#include "web_server.h"
#include "secrets.h"

#include <WiFi.h>
#include "esp_wpa2.h"
#include "esp_wifi.h"
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <M5Unified.h>

// --- Global Variables ---
char mqtt_server[40] = "20.172.67.240";
const char* mqtt_username = "sahas";
const char* mqtt_password = "47521452";   // <-- your password

AsyncWebServer server(HTTP_PORT);
WiFiClient     espClient;
PubSubClient   mqttClient(espClient);
String         client_id;

uint8_t active_state = STATE_IDLE;
int     sampling_frequency = DEFAULT_SAMPLING_HZ;

// --- MQTT Reconnect ---
bool reconnect_mqtt() {
  Serial.print("Attempting MQTT connection...");
  // connect(clientID, username, password)
  if (mqttClient.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
    Serial.println("connected ✅");
    drawDisplay();
    return true;
  } else {
    Serial.printf("failed, rc=%d\n", mqttClient.state());
    return false;
  }
}

// --- Reset WiFi ---
void reset_wifi_settings() {
  esp_wifi_restore();
  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setCursor(5, 10);
  M5.Display.println("WiFi credentials erased!");
  M5.Display.println("Rebooting...");
  delay(1500);
  ESP.restart();
}

// --- WiFi Setup ---
void setup_wifi() {
  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Display.setTextSize(1.8);
  M5.Display.setCursor(5, 10);
  M5.Display.printf("Connecting to:\n %s", EAP_SSID);

  WiFi.disconnect(true);
  WiFi.mode(WIFI_STA);

  // WPA2 Enterprise (University Network)
  if (strlen(EAP_USERNAME) > 0 && strlen(EAP_PASSWORD) > 0) {
    esp_wifi_sta_wpa2_ent_set_identity((uint8_t *)EAP_USERNAME, strlen(EAP_USERNAME));
    esp_wifi_sta_wpa2_ent_set_username((uint8_t *)EAP_USERNAME, strlen(EAP_USERNAME));
    esp_wifi_sta_wpa2_ent_set_password((uint8_t *)EAP_PASSWORD, strlen(EAP_PASSWORD));
    esp_wifi_sta_wpa2_ent_set_disable_time_check(true);
    esp_wifi_sta_wpa2_ent_enable();
  }

  WiFi.begin(EAP_SSID);

  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - startTime > WIFI_TIMEOUT_MS) {
      M5.Display.println("\n\nConnection failed.");
      delay(1000);
      break;
    }
    M5.Display.print(".");
    delay(300);
  }

  // --- Fallback: WiFiManager Setup Portal ---
  if (WiFi.status() != WL_CONNECTED) {
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
    WiFiManagerParameter custom_eap_user("eap_user", "EAP Username (optional)", (char*)EAP_USERNAME, 64);
    WiFiManagerParameter custom_eap_pass("eap_pass", "EAP Password (optional)", (char*)EAP_PASSWORD, 64);

    wm.addParameter(&custom_mqtt_server);
    wm.addParameter(&custom_eap_user);
    wm.addParameter(&custom_eap_pass);
    wm.setConnectTimeout(20);

    String mac = WiFi.macAddress();
    mac.replace(":", "");
    String portal_name = "M5-Portal-" + mac;

    if (!wm.autoConnect(portal_name.c_str())) {
      if (!wm.startConfigPortal(portal_name.c_str())) {
        M5.Display.println("Setup Failed. Restarting.");
        delay(1500);
        ESP.restart();
      }
    }

    strncpy(mqtt_server, custom_mqtt_server.getValue(), sizeof(::mqtt_server)-1);
    mqtt_server[sizeof(::mqtt_server)-1] = 0;
  }

  // --- Connection Success ---
  M5.Display.println("\n\nConnected!");
  delay(600);

  // 🔋 Keep Wi-Fi fully awake (prevents keep-alive timeouts)
  WiFi.setSleep(false);                 // Arduino helper
  esp_wifi_set_ps(WIFI_PS_NONE);        // Ensure power-save is OFF

  // MQTT Client Setup
  client_id = "M5-HumanLogger-" + WiFi.macAddress();
  client_id.replace(":", "");
  mqttClient.setServer(mqtt_server, 1883);

  // 🔁 Longer keep-alive + a bit more socket slack
  mqttClient.setKeepAlive(30);          // was 15
  mqttClient.setSocketTimeout(10);      // default ~15; 10 is fine here
  mqttClient.setBufferSize(8192);       // already had this
}
