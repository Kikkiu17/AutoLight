/*
 * wifihandler.c
 *
 * Modified for: Home Assistant MQTT
 */

#include "wifihandler.h"
#include "../Flash/flash.h"
#include <stdio.h>
#include <string.h>

Switch_t switches[NUMBER_OF_SWITCHES];

void SWITCH_Init(Switch_t* sw, bool inverted, GPIO_TypeDef* port, uint16_t pin)
{
	sw->pressed = false;
	sw->inverted = inverted;
	sw->port = port;
	sw->pin = pin;
	sw->manual = false;
	HAL_GPIO_WritePin(port, pin, inverted);
}

void SWITCH_Press(Switch_t* sw)
{
	sw->pressed = true;
	HAL_GPIO_WritePin(sw->port, sw->pin, !(sw->inverted));
}

void SWITCH_UnPress(Switch_t* sw)
{
	sw->pressed = false;
	HAL_GPIO_WritePin(sw->port, sw->pin, sw->inverted);
}

Response_t WIFIHANDLER_MQTT_Init(WIFI_t* wifi, const char* broker_ip, uint16_t port)
{
	// the MQTT client ID is the hostname
	if (WIFI_MQTT_Config(wifi, wifi->hostname) != OK) return ERR;
	
	if (WIFI_MQTT_ConnectBroker(wifi, broker_ip, port) != OK) return ERR;
	
	char sub_topic[64];
	snprintf(sub_topic, 64, "snse/%s/relay/set", wifi->hostname);
	WIFI_MQTT_Subscribe(wifi, sub_topic, 1);
	
	return OK;
}

void WIFIHANDLER_MQTT_PublishDiscovery(WIFI_t* wifi)
{
	char topic[128];
	char payload[WIFI_BUF_MAX_SIZE];
	// DISCOVERY
	
	// RELAY
	snprintf(topic, 128, "homeassistant/light/%s_relay/config", wifi->hostname);
	snprintf(payload, WIFI_BUF_MAX_SIZE, MQTT_DISCOVERY_LIGHT, 
		"Relè Presa", wifi->hostname, wifi->hostname, wifi->hostname, wifi->hostname, ESP_NAME);
	WIFI_MQTT_Publish(wifi, topic, payload, 1, 1);
	
	// DISTANCE
	snprintf(topic, 128, "homeassistant/sensor/%s_dist/config", wifi->hostname);
	snprintf(payload, WIFI_BUF_MAX_SIZE, MQTT_DISCOVERY_SENSOR, 
		"Distanza sensore", wifi->hostname, "dist", "cm", "distance", wifi->hostname, "dist", wifi->hostname, ESP_NAME);
	WIFI_MQTT_Publish(wifi, topic, payload, 1, 1); 

	// LDO TEMP
	snprintf(topic, 128, "homeassistant/sensor/%s_templdo/config", wifi->hostname);
	snprintf(payload, WIFI_BUF_MAX_SIZE, MQTT_DISCOVERY_SENSOR, 
		"Temperatura LDO", wifi->hostname, "templdo", "°C", "temperature", wifi->hostname, "templdo", wifi->hostname, ESP_NAME);
	WIFI_MQTT_Publish(wifi, topic, payload, 1, 1);

	// RELAY TEMP
	snprintf(topic, 128, "homeassistant/sensor/%s_temprly/config", wifi->hostname);
	snprintf(payload, WIFI_BUF_MAX_SIZE, MQTT_DISCOVERY_SENSOR, 
		"Temperatura RELE\'", wifi->hostname, "temprly", "°C", "temperature", wifi->hostname, "temprly", wifi->hostname, ESP_NAME);
	WIFI_MQTT_Publish(wifi, topic, payload, 1, 1);
}

void WIFIHANDLER_MQTT_PublishStates(WIFI_t* wifi)
{
	char topic[128];
	char payload[16];
	
	// RELAY
	snprintf(topic, 128, "snse/%s/relay/state", wifi->hostname);
	WIFI_MQTT_Publish(wifi, topic, switches[RELAY_SWITCH].pressed ? "1" : "0", 1, 1);
	
	// DISTANCE
	snprintf(topic, 128, "snse/%s/dist/state", wifi->hostname);
	snprintf(payload, 16, "%lu", sens_distance);
	WIFI_MQTT_Publish(wifi, topic, payload, 1, 1);

	// LDO TEMP
	snprintf(topic, 128, "snse/%s/templdo/state", wifi->hostname);
	snprintf(payload, 16, "%d", ldo_temp);
	WIFI_MQTT_Publish(wifi, topic, payload, 1, 1);

	// RELAY TEMP
	snprintf(topic, 128, "snse/%s/temprly/state", wifi->hostname);
	snprintf(payload, 16, "%d", rly_temp);
	WIFI_MQTT_Publish(wifi, topic, payload, 1, 1);
}

void WIFIHANDLER_MQTT_Loop(WIFI_t* wifi)
{
    char topic_in[64];
    char payload_in[64];
    
    if (WIFI_MQTT_Receive(wifi, topic_in, payload_in, 1) == OK)
    {
        char expected_topic[64];
        snprintf(expected_topic, 64, "snse/%s/relay/set", wifi->hostname);
        
        if (strstr(topic_in, expected_topic) != NULL)
        {
            if (strncmp(payload_in, "1", 1) == 0)
                SWITCH_Press(&switches[RELAY_SWITCH]);
            else if (strncmp(payload_in, "0", 1) == 0)
                SWITCH_UnPress(&switches[RELAY_SWITCH]);
            
			ESP8266_ClearBuffer();
			
			char topic_out[128];
			snprintf(topic_out, 128, "snse/%s/relay/state", wifi->hostname);
			WIFI_MQTT_Publish(wifi, topic_out, switches[RELAY_SWITCH].pressed ? "1" : "0", 1, 1);
        }
        ESP8266_ClearBuffer();
    }
}

void WIFIHANDLER_MQTT_SendNotification(WIFI_t* wifi, const char* message)
{
    char topic[128];
    snprintf(topic, 128, "snse/%s/notify", wifi->hostname);
    WIFI_MQTT_Publish(wifi, topic, message, 1, 0);
}