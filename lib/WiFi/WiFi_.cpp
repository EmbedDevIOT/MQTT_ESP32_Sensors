#include "Config.h"
#include "WF.h"
#include "esp_wifi.h"

///////////////////////////////////
//    Wi-Fi   Initialisation     //
///////////////////////////////////
void WIFIinit(boolean mode)
{
   CFG.WiFiMode = mode;
   if (CFG.WiFiMode == 0)
   {
      Serial.print("SSID=");
      Serial.println(CFG.ssid);
      Serial.print("PASS=");
      Serial.println(CFG.password);

      IPAddress ip(CFG.IP1, CFG.IP2, CFG.IP3, CFG.IP4); // Static IP
      IPAddress gateway(CFG.GW1, CFG.GW2, CFG.GW3, CFG.GW4);
      IPAddress subnet(CFG.MK1, CFG.MK2, CFG.MK3, CFG.MK4);

      WiFi.disconnect();
      WiFi.mode(WIFI_STA);
      
      byte tries = 6;

      WiFi.begin(CFG.ssid, CFG.password);
      WiFi.config(ip, gateway, subnet);
      while (--tries && WiFi.status() != WL_CONNECTED)
      {
         Serial.print(".");
         delay(500);
      }
      if (WiFi.status() == WL_CONNECTED)
      {
         Serial.println("WiFi client");
         Serial.println("WiFi connected");
         Serial.println(WiFi.localIP());
         // ST.WiFi_ON = true;
      }
   }
   else 
   {
      char tmpssid[15];
      char tmppass[15];
      CFG.APSSID.toCharArray(tmpssid, 15);
      CFG.APPAS.toCharArray(tmppass, 15);

      IPAddress apIP(192, 168, 1, 1);
      WiFi.disconnect();
      WiFi.mode(WIFI_AP);
      // WiFi.mode(WIFI_STA);
      esp_wifi_set_ps(WIFI_PS_NONE);

      WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
      WiFi.softAP(tmpssid, tmppass);
      Serial.println("WiFi: AP");

      Serial.println(tmpssid);
      Serial.println(tmppass);
   }
}

//////////////////////////////////////////////
//       Get signal level         WiFi      //
//////////////////////////////////////////////
int GetSignalLevel()
{
   if (WiFi.status() == WL_CONNECTED)
      return WiFi.RSSI();
   return -100;
}

//////////////////////////////////////////////
//       Рекконект по истечению времени     //
//////////////////////////////////////////////
void CheckWiFiStatus()
{
   static int DisconnectTime = 0;

   if ((CFG.WiFiMode == 0) && (WiFi.status() != WL_CONNECTED))
   {
      DisconnectTime++;
      if (DisconnectTime == 300) // 5 min
      {
         DisconnectTime = 0;
         digitalWrite(LEDIO2, LOW);
         WIFIinit(Client);
      }
   }
}
