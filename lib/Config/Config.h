#ifndef _Config_H
#define _Config_H

#include <Arduino.h>

#include "SPI.h"
#include <Adafruit_BMP085.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "AsyncMqttClient.h"

#define UARTSpeed 115200

// Wifi Network Select (set)
#define WORK_NET

#define WiFi_
#define Client 0
#define AccessPoint 1

#define WiFiTimeON 15

#define LEDIO2 2

#define DEBUG
#define I2C_SCAN

#define DISABLE 0
#define ENABLE 1

// I2C Adress
#define BME_ADR 0x76
#define OLED_ADR 0x3C
#define RTC_ADR 0x68
//=======================================================================

//========================== ENUMERATION ================================
//=======================================================================

//=======================================================================

//=========================== GLOBAL CONFIG =============================
struct GlobalConfig
{
  uint16_t sn = 0;
  String fw = "";

  // System_Information
  String fwdate = "19.04.2024";
  String chipID = "";
  String MacAdr = "";

  String APSSID = "Beekeeper";
  String APPAS = "12345678";

#ifdef WORK_NET
  const char *ssid = "MkT";
  const char *password = "QFCxfXMA3";
#else
  const char *ssid = "EMBNET2G";         // WiFi Login
  const char *password = "Ae19co90$!eT"; // WiFi Pass
#endif

  byte IP1 = 192;
  byte IP2 = 168;
  byte IP3 = 1;
  byte IP4 = 37;
  byte GW1 = 192;
  byte GW2 = 168;
  byte GW3 = 1;
  byte GW4 = 1;
  byte MK1 = 255;
  byte MK2 = 255;
  byte MK3 = 255;
  byte MK4 = 0;

  byte WiFiMode = AccessPoint; // Режим работы WiFi
};
extern GlobalConfig CFG;
//=======================================================================

//=======================================================================
#define BME_CAL 4.2
struct Sensors
{
  float Tf = 0.0;   // Temperature 0.0 (*C)
  int Ti = 0;       // Temperature 0   (*C)
  int H = 0;        // Humidity        (%)
  float A = 0.0;    // Altitude        (meters)
  uint32_t Pa = 0;  // Pressure        (Pa)
  float Pkpa = 0.0; // Pressure        (kPa)
  float PmmHg = 0;  // Pressure        (mmHg)
};
extern Sensors SNS_BME;
extern Sensors SNS_BMP;
//=======================================================================

//===========================     M Q T T   =============================
struct MQ
{
  // MQTT broker credentials (set to NULL if not required)
  // const char *server = "m5.wqtt.ru";
  const char  *server = "http://89.109.8.114/";
  const char *username = "homeassistant";
  const char *password = "ohquai9ahwae8vigeing8gei2thiizaes1jeej3ethiejierah1ieph5cheajeeL";
  const int portTLS = 10073;
  const int port = 1883; //(http://89.109.8.114:8001/); //10072
};
extern MQ mqtt;

struct TOP
{
  const String device_topic = "/devCM";
  String prog = "/devCM/progs";
  String cnt = "cnt";
  String pwrState = "/PwrST";
  String temp = "/Temp";
};
extern TOP Topics;
//=======================================================================

//=======================================================================
struct Status
{
  bool debug = true;
  bool WiFi_ON = false;
};
extern Status ST;
//============================================================================

//============================================================================
void SystemInit(void); //  System Initialisation (variables and structure)
void I2C_Scanning(void);
void ShowInfoDevice(void); //  Show information or this Device
void GetChipID(void);
void FloatToCharArr(float payload, char *output_msg);
//============================================================================
#endif // _Config_H