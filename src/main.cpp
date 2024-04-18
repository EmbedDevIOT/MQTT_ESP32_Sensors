/*
example: https://randomnerdtutorials.com/esp32-mqtt-publish-subscribe-arduino-ide/
*/
#include "Config.h"

#include "WF.h"
#include "mqtt.h"

#define BMP180 // Set BMP180 sensors

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

Adafruit_BMP085 bmp; // 0x76 -BME280 / 0x77 - BMP180
//======================================================================

//============================== STRUCTURES =============================
GlobalConfig CFG;
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
void ShowDBG(void);
//=======================================================================
//=======================       S E T U P       =========================
void setup()
{
  CFG.fw = "0.0.3";
  CFG.fwdate = "18.04.2024";

  Serial.begin(UARTSpeed);

  // BME INIT
  bmp.begin();
  SystemInit();

  Serial.println(F("BME...Done"));
  WIFIinit(Client);

  if (WiFi.status() == WL_CONNECTED)
    ST.WiFi_ON = true;

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
      2048,
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
    // ShowDBG();

    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void SensorsHandler(void *pvParametrs)
{
  Serial.print("Sensors Handler running on core ");
  Serial.println(xPortGetCoreID());
  for (;;)
  {
    GetBMPData();
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

/*******************************************************************************************************/
// Debug info
void ShowDBG()
{
  char message[52];
  Serial.println(F("!!!!!!!!!!!!!!  DEBUG INFO  !!!!!!!!!!!!!!!!!!"));

#ifdef BME280
  sprintf(message, "T_BME:%0.2f *C | H_BME:%0d % | P_BHE:%d", SNS.BME_T, (int)SNS.BME_H, (int)SNS.BME_PmmHg);
  Serial.println(message);
#endif
#ifdef BMP180
  sprintf(message, "T:%0.2f *C | A:%0d % | P1:%i Pa | P2:%3.0f mmHg", SNS_BMP.Tf, (int)SNS_BMP.A, SNS_BMP.Pa, SNS_BMP.PmmHg);
  Serial.println(message);
#endif
  Serial.println(F("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"));
  Serial.println();
}
/*******************************************************************************************************/
