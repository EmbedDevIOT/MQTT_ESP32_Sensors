#include "Config.h"
//=========================== GLOBAL VARIABLES =========================
uint16_t tmrSec = 0;
uint16_t tmrMin = 0;
//======================================================================

//================================ OBJECTs =============================
TaskHandle_t TaskCore_0;
TaskHandle_t TaskCore_1;
TaskHandle_t SensorTaskCore_1;
GyverBME280 bme; // 0x76
//======================================================================

//============================== STRUCTURES =============================
GlobalConfig CFG;
Sensors SNS;
Flag STATE;
//=======================================================================

//======================    FUNCTION PROTOTYPS     ======================
void HandlerCore0(void *pvParameters);
void HandlerCore1(void *pvParameters);
void SensorsHandler(void *pvParameters);
void GetBMEData(void);
void ShowDBG(void);
//=======================================================================
//=======================       S E T U P       =========================
void setup()
{
  CFG.fw = "0.1";
  CFG.fwdate = "18.04.2024";

  Serial.begin(UARTSpeed);

  SystemInit();

  // BME INIT
  bme.begin(0x76);
  Serial.println(F("BME...Done"));
  GetBMEData();

  xTaskCreatePinnedToCore(
      HandlerCore0,
      "TaskCore_0",
      10000,
      NULL,
      1,
      &TaskCore_0,
      0);
  delay(500);

  xTaskCreatePinnedToCore(
      HandlerCore1,
      "TaskCore_1",
      2048,
      NULL,
      1,
      &TaskCore_1,
      1);
  delay(500);

  xTaskCreatePinnedToCore(
      SensorsHandler,
      "SensorTaskCore_1",
      2048,
      NULL,
      1,
      &SensorTaskCore_1,
      1);
  delay(500);
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
    ShowDBG();

    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void SensorsHandler(void *pvParametrs)
{
  Serial.print("Sensors Handler running on core ");
  Serial.println(xPortGetCoreID());
  for (;;)
  {
    GetBMEData();
    vTaskDelay(3000 / portTICK_PERIOD_MS);
  }
}
//=======================================================================

//=======================   I2C BME Sensors     =========================
// Get Data from BME Sensor
void GetBMEData()
{
  SNS.BME_T = bme.readTemperature();
  SNS.BME_H = (int)bme.readHumidity() + BME_CAL;
  SNS.BME_Pkpa = bme.readPressure();
  SNS.BME_PmmHg = (int)pressureToMmHg(SNS.BME_Pkpa);
}
//=======================================================================

/*******************************************************************************************************/
// Debug info
void ShowDBG()
{
  char message[52];
  Serial.println(F("!!!!!!!!!!!!!!  DEBUG INFO  !!!!!!!!!!!!!!!!!!"));
  sprintf(message, "T_BME:%0.2f *C | H_BME:%0d % | P_BHE:%d", SNS.BME_T, (int)SNS.BME_H, (int)SNS.BME_PmmHg);
  Serial.println(message);
  Serial.println(F("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"));
  Serial.println();
}
/*******************************************************************************************************/
