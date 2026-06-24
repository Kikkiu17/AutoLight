/*
 * esp8266.h
 *
 * Created on: Apr 6, 2025
 * Author: Kikkiu
 * Modified for: MQTT Client
 */

#ifndef ESP8266_ESP8266_H_
#define ESP8266_ESP8266_H_

#include "stm32g0xx_hal.h"
#include "usart.h"
#include "../settings.h"

extern bool WIFI_response_sent;

typedef enum
{
	ERR 		= 0,
	TIMEOUT 	= 1,
	OK 			= 2,
	NULVAL		= 3,
	WAITING		= 4,
	FAIL		= 5,
} Response_t;

typedef struct
{
	char 		IP[15 + 1];
	char 		SSID[32 + 1];
	char		pw[64 + 1];
	char		buf[WIFI_BUF_MAX_SIZE + 1];
	char		hostname[HOSTNAME_MAX_SIZE + 1];
	char		name[NAME_MAX_SIZE + 1];
	char		time[8 + 1];	// hh:mm:ss
	int32_t	    last_time_read;
} WIFI_t;

int32_t bufferToInt(char* buf, uint32_t size);

void WIFI_Init(WIFI_t* wifi);

Response_t ESP8266_Init(void);
void ESP8266_ClearBuffer(void);
char* ESP8266_GetBuffer(void);
void ESP8266_HardwareReset(void);
Response_t ESP8266_ATReset(void);
Response_t ESP8266_CheckAT(void);
Response_t ESP8266_Restore(void);
Response_t ESP8266_ResetWaitReady(void);

Response_t ESP8266_WaitForStringCNDTROffset(char* str, int32_t offset, uint32_t timeout);
Response_t ESP8266_WaitForString(char* str, uint32_t timeout);
Response_t ESP8266_WaitKeepString(char* str, uint32_t timeout);

HAL_StatusTypeDef ESP8266_SendATCommandNoResponse(char* cmd, size_t size, uint32_t timeout);
Response_t ESP8266_SendATCommandResponse(char* cmd, size_t size, uint32_t timeout);
Response_t ESP8266_SendATCommandKeepString(char* cmd, size_t size, uint32_t timeout);
Response_t ESP8266_SendATCommandKeepStringNoResponse(char* cmd, size_t size);

Response_t WIFI_Connect(WIFI_t* wifi);
Response_t WIFI_GetConnectionInfo(WIFI_t* wifi);
Response_t WIFI_SetCWMODE(uint8_t mode);
Response_t WIFI_SetHostname(WIFI_t* wifi, const char* hostname);
Response_t WIFI_GetHostname(WIFI_t* wifi);
Response_t WIFI_SetName(WIFI_t* wifi, char* name);
Response_t WIFI_GetIP(WIFI_t* wifi);
Response_t WIFI_SetIP(WIFI_t* wifi, char* ip);

Response_t WIFI_GetTime(WIFI_t* wifi);
int32_t WIFI_GetTimeHour(WIFI_t* wifi);
int32_t WIFI_GetTimeMinutes(WIFI_t* wifi);
int32_t WIFI_GetTimeSeconds(WIFI_t* wifi);
Response_t WIFI_EnableNTPServer(WIFI_t* wifi, int8_t time_offset);

Response_t WIFI_MQTT_Config(WIFI_t* wifi, const char* client_id);
Response_t WIFI_MQTT_ConnectBroker(WIFI_t* wifi, const char* broker_ip, uint16_t port);
Response_t WIFI_MQTT_Subscribe(WIFI_t* wifi, const char* topic, uint8_t qos);
Response_t WIFI_MQTT_Publish(WIFI_t* wifi, const char* topic, const char* payload, uint8_t qos, uint8_t retain);
Response_t WIFI_MQTT_Receive(WIFI_t* wifi, char* topic_out, char* payload_out, uint32_t timeout);

#endif /* ESP8266_ESP8266_H_ */