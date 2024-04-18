#ifndef MQTT_H
#define MQTT_H

#include <Arduino.h>

class MQTT
{
public:
    MQTT();
    void Handler(void);
    void PublishMsg(char *msg);
private:
    void callback(char *topic, byte *message, unsigned int length);
    void reconnect();
};

#endif