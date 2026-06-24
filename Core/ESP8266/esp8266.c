/*
 * esp8266.c
 *
 * Created on: Apr 6, 2025
 * Author: Kikkiu
 * Modified for: MQTT Client
 */

#include "esp8266.h"
#include "stm32g030xx.h"
#include "stm32g0xx_hal_dma.h"
#include "stm32g0xx_hal_uart.h"
#include "stm32g0xx_hal_uart_ex.h"
#include "stm32g0xx_ll_dma.h"
#include "usart.h"
#include <string.h>
#include <inttypes.h>
#include <stdio.h>
#include <time.h>

#define CWMODE_MAX_SIZE 14
#define CIPSTA_IP_OFFSET 12

#define CWSTATE_STATE_OFFSET 9
#define CWSTATE_SSID_OFFSET 12
#define CWSTATE_IP_OFFSET 9
#define CWSTATE_NOAP 0
#define CWSTATE_CONNECTED_WITHOUTIP 1
#define CWSTATE_CONNECTED_WITHIP 2
#define CWSTATE_CONNECTING 3
#define CWSTATE_DISCONNECTED 4

volatile char uart_buffer[UART_BUFFER_SIZE + 1];
bool WIFI_response_sent = false;
uint32_t reconnect_time = 0;

void WIFI_Init(WIFI_t* wifi)
{
	if (wifi == NULL)
		return;
	memset(wifi, 0, sizeof(WIFI_t));
}

int32_t bufferToInt(char* buf, uint32_t size)
{
	if (buf == NULL) return -1;
	uint32_t n = 0;
	for (uint32_t i = 0; i < size; i++)
	{
		if (buf[i] < '0' || buf[i] > '9') return -1;
		n *= 10;
		n += buf[i] - '0';
	}
	return n;
}

Response_t ESP8266_WaitForStringCNDTROffset(char* str, int32_t offset, uint32_t timeout)
{
	if (str == NULL) return NULVAL;
	for (uint32_t i = 0; i < timeout; i++)
	{
		HAL_Delay(1);

		if (strstr((char*)uart_buffer, "ERR") != NULL)
		{
			ESP8266_ClearBuffer();
			return ERR;
		}

		if (UART_BUFFER_SIZE - UART_DMA_CHANNEL_HANDLE->CNDTR > (offset < 0) ? -offset : offset)
			if (strstr((char*)uart_buffer + (UART_BUFFER_SIZE - UART_DMA_CHANNEL_HANDLE->CNDTR) + offset, str) == NULL)
				continue;

		ESP8266_ClearBuffer();
		return OK;
	}

	if (strstr((char*)uart_buffer, "ERROR")) return ERR;

	return TIMEOUT;
}

Response_t ESP8266_WaitForString(char* str, uint32_t timeout)
{
	if (str == NULL) return NULVAL;
	for (uint32_t i = 0; i < timeout; i++)
	{
		HAL_Delay(1);

		if (strstr((char*)uart_buffer, "FAIL"))
		{
			ESP8266_ClearBuffer();
			return FAIL;
		}

		if (strstr((char*)uart_buffer, "ERR"))
		{
			ESP8266_ClearBuffer();
			return ERR;
		}

		if (!strstr((char*)uart_buffer, str)) continue;

		ESP8266_ClearBuffer();
		return OK;
	}

	if (strstr((char*)uart_buffer, "ERROR")) return ERR;

	return TIMEOUT;
}

Response_t ESP8266_WaitKeepString(char* str, uint32_t timeout)
{
	if (str == NULL) return NULVAL;
	for (uint32_t i = 0; i < timeout; i++)
	{
		HAL_Delay(1);
		char* ptr = strstr((char*)uart_buffer, str);
		if (ptr == NULL) continue;

		return OK;
	}

	if (strstr((char*)uart_buffer, "ERROR")) return ERR;

	return TIMEOUT;
}

HAL_StatusTypeDef ESP8266_SendATCommandNoResponse(char* cmd, size_t size, uint32_t timeout)
{
	if (cmd == NULL) return HAL_ERROR;
	return HAL_UART_Transmit(&STM_UART, (uint8_t*)cmd, size, UART_TX_TIMEOUT);
}

Response_t ESP8266_SendATCommandResponse(char* cmd, size_t size, uint32_t timeout)
{
	if (cmd == NULL) return NULVAL;
	ESP8266_ClearBuffer();
	if (HAL_UART_Transmit(&STM_UART, (uint8_t*)cmd, size, UART_TX_TIMEOUT) != HAL_OK)
		return ERR;
	return ESP8266_WaitForString("OK", timeout);
}

Response_t ESP8266_SendATCommandKeepString(char* cmd, size_t size, uint32_t timeout)
{
	if (cmd == NULL) return NULVAL;
	ESP8266_ClearBuffer();
	if (HAL_UART_Transmit(&STM_UART, (uint8_t*)cmd, size, UART_TX_TIMEOUT) != HAL_OK)
		return ERR;
	Response_t resp = ESP8266_WaitKeepString("OK", timeout);
	if (resp != ERR && resp != TIMEOUT)
		return OK;
	return resp;
}

Response_t ESP8266_SendATCommandKeepStringNoResponse(char* cmd, size_t size)
{
	if (cmd == NULL) return NULVAL;
	ESP8266_ClearBuffer();
	if (HAL_UART_Transmit(&STM_UART, (uint8_t*)cmd, size, UART_TX_TIMEOUT) != HAL_OK)
		return ERR;
	return OK;
}

Response_t ESP8266_CheckAT(void)
{
	return ESP8266_SendATCommandResponse("AT\r\n", 4, AT_SHORT_TIMEOUT);
}

Response_t ESP8266_Init(void)
{
	ESP8266_ClearBuffer();
	HAL_UARTEx_ReceiveToIdle_DMA(&STM_UART, (uint8_t*)uart_buffer, UART_BUFFER_SIZE);
	return ESP8266_ResetWaitReady();
}

void ESP8266_ClearBuffer(void)
{
	LL_DMA_DisableChannel(UART_DMA_TYPEDEF, UART_DMA_LL_CHANNEL);
	memset((char*)uart_buffer, 0, UART_BUFFER_SIZE + 1 - UART_DMA_CHANNEL_HANDLE->CNDTR);
	UART_DMA_CHANNEL_HANDLE->CNDTR = UART_BUFFER_SIZE;
	__HAL_UART_CLEAR_OREFLAG(&STM_UART);
    __HAL_UART_CLEAR_NEFLAG(&STM_UART);
    __HAL_UART_CLEAR_FEFLAG(&STM_UART);
	LL_DMA_EnableChannel(UART_DMA_TYPEDEF, UART_DMA_LL_CHANNEL);
}

char* ESP8266_GetBuffer(void)
{
	return (char*)uart_buffer;
}

void ESP8266_Reset(void)
{
	HAL_GPIO_WritePin(ESP_RST_PORT, ESP_RST_PIN, 0);
	HAL_Delay(5);
	HAL_GPIO_WritePin(ESP_RST_PORT, ESP_RST_PIN, 1);
}

Response_t ESP8266_ResetWaitReady(void)
{
	uint8_t attempt_number = 0;
	Response_t start_ok = ERR;
	while (start_ok != OK)
	{
		if (START_ATTEMPTS != -1 && attempt_number > START_ATTEMPTS)
			return TIMEOUT;
		attempt_number++;
		ESP8266_SendATCommandKeepString("AT+RST\r\n", 8, AT_SHORT_TIMEOUT);
		// hardware reset
		HAL_GPIO_WritePin(ESP_RST_PORT, ESP_RST_PIN, 0);
		HAL_Delay(1);
		HAL_GPIO_WritePin(ESP_RST_PORT, ESP_RST_PIN, 1);

		HAL_GPIO_WritePin(STATUS_Port, STATUS_Pin, 1);
		start_ok = ESP8266_WaitForStringCNDTROffset("ready", -7, 5000);
		HAL_GPIO_WritePin(STATUS_Port, STATUS_Pin, 0);
		__HAL_UART_CLEAR_OREFLAG(&huart1);
		ESP8266_ClearBuffer();
	}

	ESP8266_SendATCommandResponse("AT+SLEEP=0\r\n", 12, AT_SHORT_TIMEOUT);

	return start_ok;
}

Response_t ESP8266_ATReset(void)
{
	Response_t resp = ESP8266_SendATCommandResponse("AT+RST\r\n", 8, AT_SHORT_TIMEOUT);
	ESP8266_ClearBuffer();
	return resp;
}

Response_t ESP8266_Restore(void)
{
	Response_t resp = ESP8266_SendATCommandResponse("AT+RESTORE\r\n", 12, AT_SHORT_TIMEOUT);
	ESP8266_ClearBuffer();
	return resp;
}

Response_t WIFI_GetIP(WIFI_t* wifi)
{
	ESP8266_ClearBuffer();
	Response_t atstatus = ESP8266_SendATCommandKeepString("AT+CIFSR\r\n", 10, AT_SHORT_TIMEOUT);
	if (atstatus != OK) return atstatus;

	char* ptr = strstr((char*)uart_buffer, "\"");
	if (ptr == NULL) return ERR;

	uint32_t IP_start_index = (ptr + 1) - uart_buffer;

	ptr = strstr((char*)uart_buffer, "\"\r\n");
	if (ptr == NULL) return ERR;

	uint32_t IP_end_index = (ptr - 1) - uart_buffer;
	if (IP_end_index < IP_start_index) return ERR;

	uint32_t IP_size = IP_end_index - IP_start_index + 1;
	if (IP_size > WIFI_BUF_MAX_SIZE) return ERR;

	memcpy(wifi->IP, (char*)uart_buffer + IP_start_index, IP_size);
	return OK;
}

Response_t WIFI_GetConnectionInfo(WIFI_t* wifi)
{
	if (wifi == NULL) return NULVAL;
	if (ESP8266_SendATCommandKeepString("AT+CWSTATE?\r\n", 13, AT_SHORT_TIMEOUT) != OK)
		return ERR;
	
	char* ptr = strstr((char*)uart_buffer, "+CWSTATE:");
	if (ptr == NULL) return ERR;

	if (*(ptr + CWSTATE_STATE_OFFSET) - '0' != CWSTATE_CONNECTED_WITHIP)
		return ERR;

	uint32_t SSID_start_index = (ptr + CWSTATE_SSID_OFFSET) - uart_buffer;

	ptr = strstr((char*)uart_buffer, "\"\r\n");
	if (ptr == NULL) return ERR;

	uint32_t SSID_end_index = (ptr - 1) - uart_buffer;
	if (SSID_end_index < SSID_start_index) return ERR;

	uint32_t SSID_size = SSID_end_index - SSID_start_index + 1;
	if (SSID_size > sizeof(wifi->SSID)) return ERR;

	memcpy(wifi->SSID, (char*)uart_buffer + SSID_start_index, SSID_size);

	if (WIFI_GetIP(wifi) != OK) return ERR;

	return WIFI_GetHostname(wifi);
}

Response_t WIFI_Connect(WIFI_t* wifi)
{
	if (wifi == NULL) return NULVAL;
	Response_t result = ERR;
	result = ESP8266_SendATCommandKeepString("AT+CWSTATE?\r\n", 13, 5000);
	if (result != OK) return result;

	char *ptr = strstr((char*)uart_buffer, "+CWSTATE:");
	if (!ptr) return ERR;

	int state = *(ptr + CWSTATE_IP_OFFSET) - '0';

	if (state == CWSTATE_CONNECTING)
	{
		if (ESP8266_WaitKeepString("WIFI CONNECTED", 9000) == OK)
		{
			if (ESP8266_WaitForString("WIFI GOT IP", 18000) != OK)
				return FAIL;
			else
				state = CWSTATE_CONNECTED_WITHIP;
		}
		else return FAIL;
	}
	else if (state == CWSTATE_CONNECTED_WITHOUTIP)
	{
		if (ESP8266_WaitForString("WIFI GOT IP", 18000) != OK)
			return FAIL;
		else
			state = CWSTATE_CONNECTED_WITHIP;
	}

	if (state == CWSTATE_NOAP || state == CWSTATE_DISCONNECTED)
	{
		ESP8266_SendATCommandResponse("AT+CWAUTOCONN=0\r\n", 17, AT_SHORT_TIMEOUT); 
		ESP8266_SendATCommandResponse("AT+CWQAP\r\n", 10, AT_SHORT_TIMEOUT);

		if (WIFI_SetCWMODE(1) != OK) return FAIL;
		
		snprintf(wifi->buf, WIFI_BUF_MAX_SIZE, "AT+CWHOSTNAME=\"%s\"\r\n", ESP_HOSTNAME);
		if (ESP8266_SendATCommandResponse(wifi->buf, strlen(wifi->buf), AT_SHORT_TIMEOUT) != OK) return FAIL;

		snprintf(wifi->buf, WIFI_BUF_MAX_SIZE, "AT+CWJAP=\"%s\",\"%s\"\r\n", wifi->SSID, wifi->pw);
		ESP8266_SendATCommandNoResponse(wifi->buf, strlen(wifi->buf), 15000);

		if (ESP8266_WaitKeepString("WIFI CONNECTED", 9000) == OK)
		{
			if (ESP8266_WaitForString("WIFI GOT IP", 18000) != OK)
				return FAIL;

			if (WIFI_GetConnectionInfo(wifi) != OK)
				return ERR;
			else
			{
				snprintf(wifi->buf, WIFI_BUF_MAX_SIZE, "AT+CIPSTA=\"%s\"\r\n", wifi->IP);
				return ESP8266_SendATCommandResponse(wifi->buf, strlen(wifi->buf), 5000);
			}
		}
		else return FAIL;
	}
	else if (state == CWSTATE_CONNECTED_WITHIP)
	{
		snprintf(wifi->buf, WIFI_BUF_MAX_SIZE, "AT+CWHOSTNAME=\"%s\"\r\n", ESP_HOSTNAME);
		if (ESP8266_SendATCommandResponse(wifi->buf, strlen(wifi->buf), AT_SHORT_TIMEOUT) != OK) return FAIL;
		return WIFI_GetConnectionInfo(wifi);
	}

	return ERR;
}

Response_t WIFI_SetCWMODE(uint8_t mode)
{
	if (mode > 3) return ERR;

	char cwmode[CWMODE_MAX_SIZE + 1];
	memset(cwmode, 0, CWMODE_MAX_SIZE + 1);
	snprintf(cwmode, CWMODE_MAX_SIZE, "AT+CWMODE=%c\r\n", (char)(mode + '0'));
	return ESP8266_SendATCommandResponse(cwmode, CWMODE_MAX_SIZE, AT_SHORT_TIMEOUT);
}

Response_t WIFI_SetHostname(WIFI_t* wifi, const char* hostname)
{
	if (wifi == NULL || hostname == NULL) return NULVAL;
	uint32_t hostname_size = strnlen(hostname, HOSTNAME_MAX_SIZE);
	char hostnamestr[18 + hostname_size + 1];
	memset(hostnamestr, 0, 18 + hostname_size + 1);
	snprintf(hostnamestr, 18 + hostname_size, "AT+CWHOSTNAME=\"%s\"\r\n", hostname);
	Response_t atstatus = ESP8266_SendATCommandResponse(hostnamestr, 18 + hostname_size, AT_SHORT_TIMEOUT);
	if (atstatus == OK)
	{
		if (hostname_size <= HOSTNAME_MAX_SIZE)
			memcpy(wifi->hostname, hostname, hostname_size);
		else
			memcpy(wifi->hostname, hostname, HOSTNAME_MAX_SIZE);
	}
	return atstatus;
}

Response_t WIFI_GetHostname(WIFI_t* wifi)
{
	if (wifi == NULL) return NULVAL;
	Response_t atstatus = ESP8266_SendATCommandKeepString("AT+CWHOSTNAME?\r\n", 16, AT_SHORT_TIMEOUT);
	if (atstatus != OK) return atstatus;

	char* ptr = strstr((char*)uart_buffer, "+CWHOSTNAME:");
	if (ptr == NULL) return ERR;
	
	char* hostname_start_p = ptr + 12;
	ptr = strstr(hostname_start_p, "\r\n");
	if (ptr == NULL) return ERR;
	char* hostname_end_p = ptr - 1;

	uint32_t hostname_size = hostname_end_p - hostname_start_p + 1;
	if (hostname_size > HOSTNAME_MAX_SIZE) return ERR;

	memset(wifi->hostname, 0, HOSTNAME_MAX_SIZE);
	memcpy(wifi->hostname, hostname_start_p, hostname_size);

	ESP8266_ClearBuffer();

	return OK;
}

Response_t WIFI_SetName(WIFI_t* wifi, char* name)
{
	if (wifi == NULL) return ERR;
	if (name == NULL) return NULVAL;
	if (name[0] < 32 || name[0] > 126) return ERR;

	uint32_t name_size = strnlen(name, NAME_MAX_SIZE);

	if (name_size != NAME_MAX_SIZE)
		memset(savedata.name + name_size, 0, NAME_MAX_SIZE - name_size);
	strncpy(savedata.name, name, name_size);

	if (name_size != NAME_MAX_SIZE)
		memset(wifi->name + name_size, 0, NAME_MAX_SIZE - name_size);
	strncpy(wifi->name, name, name_size);

	return OK;
}

Response_t WIFI_SetIP(WIFI_t* wifi, char* ip)
{
	if (wifi == NULL || ip == NULL || ip[0] < '0' || ip[0] > '9') return ERR;

	uint32_t ip_length = strlen(ip);
	if (ip_length > 15) return ERR;

	memset(wifi->buf, 0, WIFI_BUF_MAX_SIZE);
	snprintf(wifi->buf, WIFI_BUF_MAX_SIZE, "AT+CIPSTA=\"%s\"\r\n", ip);
	Response_t atstatus = ESP8266_SendATCommandResponse(wifi->buf, strlen(wifi->buf), AT_SHORT_TIMEOUT);
	return atstatus;
}

/* ==============================================
 * GESTIONE TEMPO (NTP)
 * ============================================== */

static uint8_t getMonth(const char* m)
{
    const char* months = "JanFebMarAprMayJunJulAugSepOctNovDec";
    for (uint8_t i = 0; i < 12; i++)
    {
        if (m[0] == months[i*3] && m[1] == months[i*3+1] && m[2] == months[i*3+2])
            return i + 1;
    }
    return 1;
}

static uint8_t getItalyOffset(uint16_t year, uint8_t month, uint8_t day, uint8_t utc_h)
{
    if (month < 3 || month > 10) return 1;
    if (month > 3 && month < 10) return 2;

    uint8_t lastSunday = 31 - ((5 * year / 4 + 4) - (month == 10 ? 0 : 3)) % 7;

    if (month == 3) // marzo
        return (day > lastSunday || (day == lastSunday && utc_h >= 1)) ? 2 : 1;
    
    if (month == 10) // ottobre
        return (day > lastSunday || (day == lastSunday && utc_h >= 1)) ? 1 : 2;

    return 1;
}

Response_t WIFI_GetTime(WIFI_t* wifi)
{
    if (wifi == NULL) return NULVAL;
    wifi->last_time_read = uwTick;
    
    if (ESP8266_SendATCommandKeepString("AT+CIPSNTPTIME?\r\n", 17, AT_SHORT_TIMEOUT) != OK)
        return ERR;

    if (ESP8266_WaitKeepString("OK\r\n", AT_MEDIUM_TIMEOUT) != OK)
        return ERR;

    char* tag_ptr = strstr((char*)uart_buffer, "+CIPSNTPTIME:");
    if (tag_ptr)
    {
        char* colon = strstr(tag_ptr + 13, ":");
        if (!colon) return ERR;

        uint8_t utc_h = (uint8_t)bufferToInt(colon - 2, 2);
        uint8_t utc_m = (uint8_t)bufferToInt(colon + 1, 2);
        uint8_t utc_s = (uint8_t)bufferToInt(colon + 4, 2);
        
        uint8_t day   = (uint8_t)bufferToInt(colon - 5, 2);
        uint8_t month = getMonth(colon - 9);
        uint16_t year = bufferToInt(colon + 7, 4);

        uint8_t local_h = utc_h + getItalyOffset(year, month, day, utc_h);
        if (local_h >= 24) local_h -= 24;

        snprintf(wifi->time, 9, "%02d:%02d:%02d", 
                 local_h % 24, 
                 utc_m % 60, 
                 utc_s % 60);

        return OK;
    }

    return ERR;
}

int32_t WIFI_GetTimeHour(WIFI_t* wifi)
{
	if (wifi == NULL) return 0;
	if ((int32_t)uwTick - wifi->last_time_read > 1000) WIFI_GetTime(wifi);
	return bufferToInt(wifi->time, 2);
}

int32_t WIFI_GetTimeMinutes(WIFI_t* wifi)
{
	if (wifi == NULL) return 0;
	if ((int32_t)uwTick - wifi->last_time_read > 1000) WIFI_GetTime(wifi);
	return bufferToInt(wifi->time + 3, 2);
}

int32_t WIFI_GetTimeSeconds(WIFI_t* wifi)
{
	if (wifi == NULL) return 0;
	if ((int32_t)uwTick - wifi->last_time_read > 1000) WIFI_GetTime(wifi);
	return bufferToInt(wifi->time + 6, 2);
}

Response_t WIFI_EnableNTPServer(WIFI_t* wifi, int8_t time_offset)
{
	if (wifi == NULL) return NULVAL;

	Response_t atstatus = ERR;
	if ((atstatus = ESP8266_SendATCommandKeepString("AT+CIPSNTPCFG?\r\n", 16, AT_SHORT_TIMEOUT)) != OK) return atstatus;

	char* ptr = NULL;
	if ((ptr = strstr((char*)uart_buffer, "+CIPSNTPCFG:")) != NULL)
	{
		uint8_t ntp_enabled = *(ptr + 12) - '0';
		if (ntp_enabled)
			return OK;

		wifi->last_time_read = -1000;
		snprintf(wifi->buf, WIFI_BUF_MAX_SIZE, "AT+CIPSNTPCFG=1,%d,\"pool.ntp.org\",\"time.nist.gov\"\r\n", time_offset);
		return ESP8266_SendATCommandResponse(wifi->buf, strlen(wifi->buf), AT_SHORT_TIMEOUT);
	}

	return ERR;
}

/* ==============================================
 * MQTT CLIENT FUNCTIONS
 * ============================================== */

Response_t WIFI_MQTT_Config(WIFI_t* wifi, const char* client_id)
{
	if (wifi == NULL || client_id == NULL) return NULVAL;
	
	memset(wifi->buf, 0, WIFI_BUF_MAX_SIZE);
	// AT+MQTTUSERCFG=<LinkID>,<scheme>,<"client_id">,<"username">,<"password">,<cert_key_ID>,<CA_ID>,<"path">
	snprintf(wifi->buf, WIFI_BUF_MAX_SIZE, "AT+MQTTUSERCFG=0,1,\"%s\",\"\",\"\",0,0,\"\"\r\n", client_id);
	
	return ESP8266_SendATCommandResponse(wifi->buf, strlen(wifi->buf), AT_SHORT_TIMEOUT);
}

Response_t WIFI_MQTT_ConnectBroker(WIFI_t* wifi, const char* broker_ip, uint16_t port)
{
	if (wifi == NULL || broker_ip == NULL) return NULVAL;
	
	memset(wifi->buf, 0, WIFI_BUF_MAX_SIZE);
	// AT+MQTTCONN=<LinkID>,<"host">,<port>,<reconnect>
	snprintf(wifi->buf, WIFI_BUF_MAX_SIZE, "AT+MQTTCONN=0,\"%s\",%d,1\r\n", broker_ip, port);
	
	return ESP8266_SendATCommandResponse(wifi->buf, strlen(wifi->buf), 10000); 
}

Response_t WIFI_MQTT_Subscribe(WIFI_t* wifi, const char* topic, uint8_t qos)
{
	if (wifi == NULL || topic == NULL) return NULVAL;
	
	memset(wifi->buf, 0, WIFI_BUF_MAX_SIZE);
	// AT+MQTTSUB=<LinkID>,<"topic">,<qos>
	snprintf(wifi->buf, WIFI_BUF_MAX_SIZE, "AT+MQTTSUB=0,\"%s\",%d\r\n", topic, qos);
	
	return ESP8266_SendATCommandResponse(wifi->buf, strlen(wifi->buf), AT_SHORT_TIMEOUT);
}

Response_t WIFI_MQTT_Publish(WIFI_t* wifi, const char* topic, const char* payload, uint8_t qos, uint8_t retain)
{
	if (wifi == NULL || topic == NULL || payload == NULL) return NULVAL;

	uint32_t payload_len = strlen(payload);
	memset(wifi->buf, 0, WIFI_BUF_MAX_SIZE);

	snprintf(wifi->buf, WIFI_BUF_MAX_SIZE, "AT+MQTTPUBRAW=0,\"%s\",%lu,%d,%d\r\n", topic, (unsigned long)payload_len, qos, retain);

	ESP8266_ClearBuffer();
	if (HAL_UART_Transmit(&STM_UART, (uint8_t*)wifi->buf, strlen(wifi->buf), UART_TX_TIMEOUT) != HAL_OK) 
		return ERR;

	if (ESP8266_WaitKeepString(">", AT_SHORT_TIMEOUT) == OK)
	{
		ESP8266_ClearBuffer();
		
		if (HAL_UART_Transmit(&STM_UART, (uint8_t*)payload, payload_len, UART_TX_TIMEOUT) != HAL_OK) 
			return ERR;

		return ESP8266_WaitForString("OK", AT_MEDIUM_TIMEOUT);
	}

	return ERR;
}

Response_t WIFI_MQTT_Receive(WIFI_t* wifi, char* topic_out, char* payload_out, uint32_t timeout)
{
	if (wifi == NULL || topic_out == NULL || payload_out == NULL) return NULVAL;

	char* ptr = NULL;
	uint32_t start_time = uwTick;
	
	while (1)
	{
		if (uwTick - start_time > timeout) return TIMEOUT;
		if ((ptr = strstr((char*)uart_buffer, "+MQTTSUBRECV:")) != NULL) break;
	}

	char* topic_start = strstr(ptr, "\"");
	if (!topic_start) return ERR;
	topic_start++; 
	
	char* topic_end = strstr(topic_start, "\"");
	if (!topic_end) return ERR;
	
	uint32_t topic_len = topic_end - topic_start;
	strncpy(topic_out, topic_start, topic_len);
	topic_out[topic_len] = '\0';
	
	char* len_start = topic_end + 2; 
	char* len_end = strstr(len_start, ",");
	if (!len_end) return ERR;
	
	uint32_t payload_len = bufferToInt(len_start, len_end - len_start);
	
	char* payload_start = len_end + 1;
	
	start_time = uwTick;
	while (strlen(payload_start) < payload_len)
	{
		 if (uwTick - start_time > timeout) return TIMEOUT;
	}
	
	strncpy(payload_out, payload_start, payload_len);
	payload_out[payload_len] = '\0';
	
	ESP8266_ClearBuffer();
	return OK;
}

Response_t WIFI_MQTT_IsConnected(WIFI_t* wifi)
{
	if (wifi == NULL) return NULVAL;
	
	ESP8266_ClearBuffer();
	Response_t result = ESP8266_SendATCommandKeepString("AT+MQTTCONN?\r\n", 15, 5000);
	if (result != OK) return result;

	if (strstr((char*)uart_buffer, "+MQTTCONN:0,1") != NULL)
		return OK;
	
	return FAIL;
}