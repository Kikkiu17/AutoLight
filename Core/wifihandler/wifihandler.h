/*
 * wifihandler.h
 *
 * Modified for: Home Assistant MQTT
 */

#ifndef WIFIHANDLER_WIFIHANDLER_H_
#define WIFIHANDLER_WIFIHANDLER_H_

#include "../ESP8266/esp8266.h"
#include "../settings.h"

void SWITCH_Init(Switch_t* sw, bool inverted, GPIO_TypeDef* port, uint16_t pin);
void SWITCH_Press(Switch_t* sw);
void SWITCH_UnPress(Switch_t* sw);

Response_t WIFIHANDLER_MQTT_Init(WIFI_t* wifi, const char* broker_ip, uint16_t port);

void WIFIHANDLER_MQTT_PublishDiscovery(WIFI_t* wifi);

void WIFIHANDLER_MQTT_PublishStates(WIFI_t* wifi);

void WIFIHANDLER_MQTT_Loop(WIFI_t* wifi);

void WIFIHANDLER_MQTT_SendNotification(WIFI_t* wifi, const char* message);

#endif /* WIFIHANDLER_WIFIHANDLER_H_ */