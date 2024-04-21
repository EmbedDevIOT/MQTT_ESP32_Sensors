/*
example: https://randomnerdtutorials.com/esp32-mqtt-publish-subscribe-arduino-ide/
# MQTT Service: https://dash.wqtt.ru/
# Log: gmail acc
# Author: EmbeddevIOT (Aleksey Baranov)
# Date: (create to 19.04.24)
# Discription: Control WiFi + MQTT + Yandex
############ Hardware ###########
# MCU: ESP32
# Hardware: Sensors
# Elegant OTA Update
########### M Q T T  ###########
# mqtt_name: u_4YVJEF
# mqtt_pass: v1HPYZgn
# mqtt_server: m5.wqtt.ru
# mqtt_port: 10073
*/
#include "Config.h"
extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
}

#include "WF.h"
#include "mqtt.h"

#define BMP180 // Set BMP180 sensors
// #define BME280 // Set BME280 sensors  0x77

// #define MQTT_HOST "m5.wqtt.ru"
#define MQTT_HOST IPAddress(89, 109, 8, 114) //http://89.109.8.114/

// Temperature MQTT Topics
#define MQTT_PUB_TEMP "embIO/bme280/temperature"
#define MQTT_PUB_PRES "embIO/bme280/pressure"

//=========================== GLOBAL VARIABLES =========================
char msg[50];

uint16_t tmrSec = 0;
uint16_t tmrMin = 0;
long lastMsg = 0;
int value = 0;
//======================================================================

//================================ OBJECTs =============================
TaskHandle_t TaskCore_0;
TaskHandle_t TaskCore_1;
TaskHandle_t SensorTaskCore_1;
TaskHandle_t LedTaskCore_1;

TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

AsyncMqttClient mqttClient;
Adafruit_BMP085 bmp;
//======================================================================

//============================== STRUCTURES =============================
GlobalConfig CFG;
MQ mqtt;
TOP Topics;
Sensors SNS_BME;
Sensors SNS_BMP;
Status ST;
//=======================================================================

//======================    FUNCTION PROTOTYPS     ======================
void HandlerCore0(void *pvParameters);
void HandlerCore1(void *pvParameters);
void SensorsHandler(void *pvParameters);
void LedsHandler(void *pvParameters);

// Sensors preset
#ifdef BME280
void GetBMEData(void);
#endif
#ifdef BMP180
void GetBMPData(void);
#endif

void connectToMqtt(void);
void onMqttConnect(bool sessionPresent);
void onMqttDisconnect(AsyncMqttClientDisconnectReason reason);
void onMqttPublish(uint16_t packetId);
void connectToWifi();
void WiFiEvent(WiFiEvent_t event);

// void reconnect();
// void callback(char *topic, byte *payload, unsigned int length);
// void publishMessage(const char *topic, String payload, boolean retained);

void ShowDBG(void);
//=======================================================================
//=======================       S E T U P       =========================
void setup()
{
  CFG.fw = "0.1.1";
  CFG.fwdate = "21.04.2024";

  Serial.begin(UARTSpeed);

  // BME INIT
  bmp.begin();
  Serial.println(F("BME...Done"));

  SystemInit();

  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void *)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void *)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, 11883);
  mqttClient.setCredentials(mqtt.username, mqtt.password);
  connectToWifi();
  // mqttClient.setServer(MQTT_HOST, MQTT_PORT);

  // WIFIinit(Client);

  // if (WiFi.status() == WL_CONNECTED)
  //   ST.WiFi_ON = true;

  // client.setServer(mqtt.server, mqtt.port);
  // client.setCallback(callback);

#ifdef BME280
  GetBMEData();
#endif
#ifdef BMP180
  GetBMPData();
#endif

  xTaskCreatePinnedToCore(
      HandlerCore0,
      "TaskCore_0",
      10000,
      NULL,
      1,
      &TaskCore_0,
      0);
  delay(100);

  xTaskCreatePinnedToCore(
      HandlerCore1,
      "TaskCore_1",
      10000,
      NULL,
      1,
      &TaskCore_1,
      1);
  delay(100);

  xTaskCreatePinnedToCore(
      SensorsHandler,
      "SensorTaskCore_1",
      2048,
      NULL,
      1,
      &SensorTaskCore_1,
      1);
  delay(500);

  xTaskCreatePinnedToCore(
      LedsHandler,
      "LedTaskCore_1",
      2048,
      NULL,
      1,
      &LedTaskCore_1,
      1);
  delay(100);
}
//=======================================================================

//=======================        L O O P        =========================
void loop() {}
//=======================================================================

//=======================================================================
// Pinned to Core 0. Network Stack Handler
void HandlerCore0(void *pvParameters)
{
  Serial.print("Task0 running on core ");
  Serial.println(xPortGetCoreID());
  for (;;)
  {
    // MQTT check if client is connected
    // if (!client.connected())
    //   reconnect();

    // client.loop();

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// Pinned to Core 1.
void HandlerCore1(void *pvParameters)
{
  Serial.print("Handler running on core ");
  Serial.println(xPortGetCoreID());
  for (;;)
  {
    GetBMPData();

    // Publish an MQTT message on topic esp32/BME280/temperature
    uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_TEMP, 0, true, String(SNS_BMP.Tf).c_str());
    Serial.printf("Publishing on topic %s at QoS 0, packetId: %i", "/temp", packetIdPub1);
    Serial.printf("Message: %.2f \n", SNS_BMP.Tf);

    // // Publish an MQTT message on topic esp32/BME2800/humidity
    // uint16_t packetIdPub2 = mqttClient.publish(MQTT_PUB_HUM, 1, true, String(hum).c_str());
    // Serial.printf("Publishing on topic %s at QoS 1, packetId %i: ", MQTT_PUB_HUM, packetIdPub2);
    // Serial.printf("Message: %.2f \n", hum);

    // Publish an MQTT message on topic esp32/BME2800/pressure
    uint16_t packetIdPub3 = mqttClient.publish(MQTT_PUB_PRES, 1, true, String(SNS_BMP.PmmHg).c_str());
    Serial.printf("Publishing on topic %s at QoS 1, packetId: %i", MQTT_PUB_PRES, packetIdPub3);
    Serial.printf("Message: %.3f \n", SNS_BMP.PmmHg);

    vTaskDelay(10000 / portTICK_PERIOD_MS);
  }
}

void SensorsHandler(void *pvParametrs)
{
  Serial.print("Sensors Handler running on core ");
  Serial.println(xPortGetCoreID());
  for (;;)
  {

    ShowDBG();
    vTaskDelay(3000 / portTICK_PERIOD_MS);
  }
}

void LedsHandler(void *pvParametrs)
{
  Serial.print("Leds state Handler running on core ");
  Serial.println(xPortGetCoreID());
  for (;;)
  {
    if (ST.WiFi_ON)
    {
      static bool led_state = true;
      led_state = !led_state;
      // digitalWrite(LEDIO2, led_state);
    }

    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}
//=======================================================================

//=======================   I2C BME Sensors     =========================
#ifdef BME280
// Get Data from BME280 Sensor
void GetBMEData()
{
  SNS_BME.Tf = bme.readTemperature();
  SNS_BME.H = (int)bme.readHumidity() + BME_CAL;
  SNS_BME.Pkpa = bme.readPressure();
  SNS_BME.PmmHg = (int)pressureToMmHg(SNS_BME.Pkpa);
}
#endif

#ifdef BMP180
// Get Data from BMP180 Sensor
void GetBMPData()
{
  SNS_BMP.Tf = bmp.readTemperature();
  SNS_BMP.Pa = bmp.readPressure();
  SNS_BMP.A = bmp.readAltitude();
  SNS_BMP.PmmHg = SNS_BMP.Pa / 133.3;
}
#endif
//=======================================================================

//=======================      D E B U G        =========================
void ShowDBG()
{
  char message[52];
  Serial.println(F("!!!!!!!!!!!!!!  DEBUG INFO  !!!!!!!!!!!!!!!!!!"));

#ifdef BME280
  sprintf(message, "T_BME:%0.2f *C | H_BME:%0d % | P_BHE:%d", SNS.BME_T, (int)SNS.BME_H, (int)SNS.BME_PmmHg);
  Serial.println(message);
#endif
#ifdef BMP180
  sprintf(message, "T:%0.2f *C | A:%0d % | P1:%i Pa | P2:%3.0f mmHg",
          SNS_BMP.Tf, (int)SNS_BMP.A, SNS_BMP.Pa, SNS_BMP.PmmHg);
  Serial.println(message);
#endif
  Serial.println(F("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"));
  Serial.println();
}
//=======================================================================

//=======================       M Q T T         =========================
/*** Call back Method for Receiving MQTT messages and Switching LED ****/
// void callback(char *topic, byte *payload, unsigned int length)
// {
//   String data_pay;
//   for (int i = 0; i < length; i++)
//   {
//     data_pay += String((char)payload[i]);
//   }
// }
void connectToWifi()
{
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(CFG.ssid, CFG.password);
}

void connectToMqtt()
{
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event)
{
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch (event)
  {
  case SYSTEM_EVENT_STA_GOT_IP:
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    connectToMqtt();
    break;
  case SYSTEM_EVENT_STA_DISCONNECTED:
    Serial.println("WiFi lost connection");
    xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
    xTimerStart(wifiReconnectTimer, 0);
    break;
  }
}

void onMqttConnect(bool sessionPresent)
{
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason)
{
  Serial.println("Disconnected from MQTT.");
  if (WiFi.isConnected())
  {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttPublish(uint16_t packetId)
{
  Serial.print("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}
//=======================================================================
