#ifndef _Config_H
#define _Config_H

#include <Arduino.h>

#include <Wire.h>
#include <GyverBME280.h>


#define UARTSpeed 115200

#define WiFi_

#define WiFiTimeON 15

#define DEBUG
// #define I2C_SCAN

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
  String fwdate = "04.04.2024";
  String chipID = "";
  String MacAdr = "";

  String APSSID = "Beekeeper";
  String APPAS = "12345678";
};
extern GlobalConfig CFG;
//=======================================================================


//=======================================================================
#define BME_CAL 4.2
struct Sensors
{
  float BME_T = 0.0;     // Temperature BME280
  int   BME_H = 0;       // Humidity   BME280
  float BME_A = 0.0;     // Altitude   BME280 m
  float BME_Pkpa = 0;    // Pressure   BME280 hPa
  int   BME_PmmHg = 0;   // Pressure   BME280 mmHg
};
extern Sensors SNS;
//=======================================================================
//=======================================================================
struct Flag
{
  bool debug = true;
};
extern Flag ST;
//============================================================================

//============================================================================
void SystemInit(void);     //  System Initialisation (variables and structure)
void I2C_Scanning(void);
void ShowInfoDevice(void); //  Show information or this Device
void GetChipID(void);
//============================================================================
#endif // _Config_H