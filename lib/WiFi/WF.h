#ifndef WF_H
#define WF_H

#include "Config.h"
#include <WiFi.h>
#include <WiFiClient.h>

void WIFIinit(boolean mode);
int GetSignalLevel();
void CheckWiFiStatus();

#endif