#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#ifdef ESP32
#include <AsyncTCP.h>
#else
#include <ESPAsyncTCP.h>
#endif
#include <AsyncMqttClient.h>

#include <EEPROM.h>
#include <ArduinoOTA.h>
#include <SoftwareSerial.h>

#include <CircularBuffer.h>

#include <Palazzetti.h>
#include "RemoteDebug.h"

#include "index_html.h"
#include "style_css.h"
#include "version.h"

#include "time.h"

#include <TFMPlus.h>
TFMPlus tfmP;  

int last_status = -1;
int last_power = -1;
int last_setpoint = 0;
long last_temperature = 0;
#ifdef ESP32
extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
}
TimerHandle_t WiFiTimer;
#else
#include <Ticker.h>
Ticker WiFiTimer;
Ticker MQTTTimer;
#endif

AsyncWebServer server(80);
AsyncMqttClient mqttClient;
SoftwareSerial lidarSerial;
CircularBuffer<byte, 9> lidarBuffer;
int distance = -1;
RemoteDebug Debug;
Palazzetti poele(&Serial2);

struct Configuration
{
  char WIFI_SSID[60] = "";
  char WIFI_PASS[60] = "";
  char MQTT_USER[60] = "";
  char MQTT_PASS[60] = "";
  char MQTT_SERVER[250] = "";
  char PROJECTNAME[30] = "project";
  byte CRC = 0;
} stConfig;

struct ProjectConfiguration
{
  int test = 0;
} stProjectConfig;

#define TIMEZONE "CET-1CEST,M3.5.0,M10.5.0/3" //Europe/Paris

unsigned long delaiWiFi = 0;
byte WiFiMode = -1;
const char *APWiFiPass = "1234567890";

unsigned long lastUpdate =-1;
#define UPDATEINTERVAL 10000

void onMqttMessage(char *topic, char *payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total);
void onMqttConnect(bool sessionPresent);
void thWifi();
String processor(const String &var);
void setTimezone(String timezone);
void initTime(String timezone);

void initTime(String timezone){
  struct tm timeinfo;

  Serial.println("Setting up time");
  configTime(0, 0, "pool.ntp.org");
  if(!getLocalTime(&timeinfo)){
    Serial.println("  Failed to obtain time");
    return;
  }

  Serial.println("  Got the time from NTP");
  setTimezone(timezone);
}

void setTimezone(String timezone){
  Serial.printf("  Setting Timezone to %s\n",timezone.c_str());
  setenv("TZ",timezone.c_str(),1); 
  tzset();
}

void onMqttMessage(char *topic, char *payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total)
{
  if (String("homeassistant/climate/poele/mode_cmd") == topic)
  {
    if (String("off") == payload)
    {
      poele.powerOff();
    }
    if (String("heat") == payload)
    {
      poele.powerOn();
    }
  }

  if (String("homeassistant/climate/poele/temp_cmd") == topic)
  {
    int setpoint = atoi(payload);
    if (setpoint!=0)
      poele.setSetPoint(setpoint);
  }
}

void onMqttConnect(bool sessionPresent)
{
  Serial.println("Connected to MQTT.");
  String payload = R"=====(
{
  "~": "homeassistant/sensor/tremispoele",
  "name": "Tremis Poele",
  "unique_id": "tremispoele",
  "stat_t": "~/battery",
  "dev_cla": "battery",
  "unit_of_meas": "%"
}
  )=====";
  mqttClient.publish("homeassistant/sensor/tremispoele/config", 0, true, payload.c_str());
  distance = -1;

  payload = R"=====(
{
  "~": "homeassistant/sensor/statuspoele",
  "name": "Status Poele",
  "unique_id": "statuspoele",
  "stat_t": "~/status",
}  )=====";

  mqttClient.publish("homeassistant/sensor/statuspoele/config", 0, true, payload.c_str());

payload = R"=====(
{
  "~": "homeassistant/sensor/powerpoele",
  "name": "Puissance Poele",
  "unique_id": "powerpoele",
  "stat_t": "~/power"
}
  )=====";
  mqttClient.publish("homeassistant/sensor/powerpoele/config", 0, true, payload.c_str());

payload = R"=====(
{
  "~": "homeassistant/climate/poele",
  "name": "Poele",
  "unique_id": "poele",
  "icon": "mdi:fire",
  "stat_t": "~/power",
  "temperature_command_topic": "~/temp_cmd",
  "temperature_state_topic": "~/temp_state",
  "current_temperature_topic": "~/cur_temp",
  "target_temp_step": 0.1,
  "min_temp": 7,
  "max_temp": 25,
  "modes": [
    "off",
    "heat",
    "auto"
    "heat"
  ],
  "mode_state_topic": "~/mode_state",
  "mode_command_topic": "~/mode_cmd"
}
  )=====";
  mqttClient.publish("homeassistant/climate/poele/config", 0, true, payload.c_str());

  mqttClient.onMessage(onMqttMessage);

}

void thWifi()
{
  if (stConfig.WIFI_SSID[0] != 0)
  {
    if (WiFi.status() != WL_CONNECTED)
    {
      if ((WiFiMode != 3) && (WiFiMode != 4))
      {
        Serial.println("Connexion au Wifi");
        WiFiMode = 3;
        WiFi.softAPdisconnect(false);
        WiFi.mode(WIFI_STA);
        WiFi.begin(stConfig.WIFI_SSID, stConfig.WIFI_PASS);
      }
    }
    else
    {
      if (WiFiMode != 2)
      {
        Serial.println("Wifi connecte");
        Serial.println(WiFi.localIP());
        WiFiMode = 2;
        initTime(TIMEZONE);
      }
      if (!mqttClient.connected())
      {
        mqttClient.connect();
      }
    }
  }
  else
  {
    if (WiFiMode != 1)
    {
      Serial.println("Creation AP");
      WiFiMode = 1;
      WiFi.mode(WIFI_AP);
      WiFi.softAP(stConfig.PROJECTNAME, APWiFiPass);
      Serial.print("[Server Connected] ");
      Serial.println(WiFi.softAPIP());
    }
  }

  if (millis() - delaiWiFi > 60000 * 1)
  {
    Serial.print("Wifi Timeout : ");
    Serial.println(WiFiMode);
    if (WiFiMode == 2)
    {
      delaiWiFi = millis();
    }
    if (WiFiMode == 4)
    {
      WiFiMode = 0;
    }
    if (WiFiMode == 3)
    {
      Serial.println("Creation AP");
      WiFiMode = 4;
      WiFi.mode(WIFI_AP);
      WiFi.softAP(stConfig.PROJECTNAME, APWiFiPass);
      Serial.print("[Server Connected] ");
      Serial.println(WiFi.softAPIP());
      delaiWiFi = millis();
    }
  }
}

String processor(const String &var)
{
  if (var == "VERSION")
    return VERSION;
  if (var == "NAME")
    return stConfig.PROJECTNAME;
  if (var == "SSID")
    return stConfig.WIFI_SSID;
  if (var == "WIFIPASS")
    return stConfig.WIFI_PASS;
  if (var == "MQTTSERVER")
    return stConfig.MQTT_SERVER;
  if (var == "MQTTUSER")
    return stConfig.MQTT_USER;
  if (var == "MQTTPASS")
    return stConfig.MQTT_PASS;

  return String();
}

void setup()
{
  Serial.begin(115200);
  delay(1000);
  WiFi.softAP("azerty");
  Serial.println("Startup...\r\nReading settings");
  EEPROM.begin(sizeof(stConfig) + sizeof(stProjectConfig));
  EEPROM.get(0, stConfig);
  EEPROM.get(sizeof(stConfig), stProjectConfig);
  if (stConfig.CRC != 0x31)
  {
    memset(&stConfig, 0, sizeof(stConfig));
    memset(&stProjectConfig, 0, sizeof(stProjectConfig));
    stConfig.CRC = 0x31;
    strcpy(stConfig.PROJECTNAME, "project");
    EEPROM.put(0, stConfig);
    EEPROM.put(sizeof(stConfig), stProjectConfig);
    EEPROM.commit();
  }
  EEPROM.end();

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            {
    AsyncWebServerResponse *response = request->beginResponse_P(200, "text/html", index_html,processor);
    request->send(response); });

  server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request)
            {
    AsyncWebServerResponse *response = request->beginResponse_P(200, "text/css", style_css);
    request->send(response); });

  server.on("/", HTTP_POST, [](AsyncWebServerRequest *request)
            {
    int params = request->params();
    for(int i=0;i<params;i++){
      AsyncWebParameter* p = request->getParam(i);
      if(p->isPost()){
        if (p->name() == "ssid") {
          strcpy(stConfig.WIFI_SSID,p->value().c_str());
        }
        if (p->name() == "pass") {
          strcpy(stConfig.WIFI_PASS,p->value().c_str());
        }
        if (p->name() == "mqttserver") {
          strcpy(stConfig.MQTT_SERVER,p->value().c_str());
        }
        if (p->name() == "mqttuser") {
          strcpy(stConfig.MQTT_USER,p->value().c_str());
        }
        if (p->name() == "mqttpass") {
          strcpy(stConfig.MQTT_PASS,p->value().c_str());
        }
      }
    }

    EEPROM.begin(sizeof(stConfig)+sizeof(stProjectConfig));
    EEPROM.put(0,stConfig);
    EEPROM.commit();  
    request->send(200, "text/plain", "Done");
    ESP.restart(); });

  Serial.println("Starting WebServer");
  server.begin();
  mqttClient.setServer(stConfig.MQTT_SERVER, 1883);
  mqttClient.setCredentials(stConfig.MQTT_USER, stConfig.MQTT_PASS);
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onMessage(onMqttMessage);

  Serial.println("Starting Wifi thread");
#ifdef ESP32
  WiFiTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(1000), pdTRUE, (void *)0, reinterpret_cast<TimerCallbackFunction_t>(thWifi));
  xTimerStart(WiFiTimer, 10);
#else
  WiFiTimer.attach(1, thWifi);
#endif
  Serial.println("Starting OTA");
  ArduinoOTA.begin();
  Serial.println("Setup Done");
  lidarSerial.begin(115200, SWSERIAL_8N1, 22, 23, false);
  
  tfmP.begin( &lidarSerial);
  tfmP.sendCommand( SET_FRAME_RATE, FRAME_1);
  Debug.begin(stConfig.PROJECTNAME);
  Debug.setResetCmdEnabled(true);
  Debug.showProfiler(true);
  Debug.showColors(true);
  
}

void loop()
{
  ArduinoOTA.handle();

  if (millis()-lastUpdate > UPDATEINTERVAL)
  {
    lastUpdate = millis();

    int status = poele.getState();
    int power = poele.getPower();
    int setpoint = poele.getSetPoint();
    long temperature = poele.getT1();

    if (status != last_status)
    {
      mqttClient.publish("homeassistant/sensor/statuspoele/status", 0, true, String(status).c_str());
      if ((status!=0)&&(status!=13))
      {
        mqttClient.publish("homeassistant/climate/poele/mode_state", 0, true, "heat");
      }
      else
      {
        mqttClient.publish("homeassistant/climate/poele/mode_state", 0, true, "off");
      }
      last_status = status;
    }

    if (power != last_power)
    {
      mqttClient.publish("homeassistant/sensor/powerpoele/power", 0, true, String(power).c_str());
      last_power = power;
    }

    if (setpoint != last_setpoint)
    {
      mqttClient.publish("homeassistant/climate/poele/temp_state", 0, true, String(setpoint).c_str());
      last_setpoint = setpoint;
    }

    if (temperature != last_temperature)
    {
      mqttClient.publish("homeassistant/climate/poele/cur_temp", 0, true, String(temperature).c_str());
      last_temperature = temperature;
    }

  }

  if (lidarSerial.available())
  {
    if (!lidarBuffer.push(lidarSerial.read()))
    {
      if ((lidarBuffer[0] == 0x59) && (lidarBuffer[1] == 0x59))
      {
        if ((lidarBuffer[2] | lidarBuffer[3] << 8) != distance)
        {
          distance = lidarBuffer[2] | lidarBuffer[3] << 8;
          Serial.println(lidarBuffer[2] | lidarBuffer[3] << 8);
          mqttClient.publish("homeassistant/sensor/tremispoele/battery", 0, true, String(map(distance, 0, 48, 100, 0)).c_str());
        }
      }
    }
  }
  Debug.handle();
  yield();
}