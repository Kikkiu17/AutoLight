/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "stm32g0xx_hal_rcc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../ESP8266/esp8266.h"
#include "../wifihandler/wifihandler.h"
#include "../wifihandler/userhandlers.h"
#include "../utils.h"
#include "../Flash/flash.h"
#include "../credentials.h"
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
Features_t features;
Switch_t trig;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RELAY_OFF_DELAY 20000	// milliseconds
#define RELAY_ON_DELAY 750		// milliseconds
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define BAT_ADC_CALIBRATION_VALUE 34 / 11
#define DIST_AVERAGE_VALUES 3
uint16_t mean_dist;
uint32_t relay_on_timestamp = 0;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
WIFI_t wifi;
Connection_t conn;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void BATTERY_GetVoltage(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */
  if (ESP8266_Init() == TIMEOUT)
  {
	  while (1)
		  __asm__("nop");
  }

  memcpy(wifi.SSID, ssid, strlen(ssid));
  memcpy(wifi.pw, password, strlen(password));
  WIFI_Connect(&wifi);
  WIFI_SetName(&wifi, (char*)ESP_NAME);
  WIFI_EnableNTPServer(&wifi, 2);

  WIFI_StartServer(&wifi, SERVER_PORT);

  SWITCH_Init(&(switches[RELAY_SWITCH]), false, RELAY_GPIO_Port, RELAY_Pin);
  SWITCH_Init(&trig, false, PULSE_GPIO_Port, PULSE_Pin);

#ifdef ENABLE_SAVE_TO_FLASH
  FLASH_ReadSaveData();
  WIFI_SetName(&wifi, savedata.name);
#endif

  HAL_TIM_Base_Start(&htim17);
  HAL_ADCEx_Calibration_Start(&hadc1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  Response_t wifistatus;
  uint32_t reconnection_timestamp = 0;
  while (1)
  {
	  if (uwTick - reconnection_timestamp > RECONNECTION_DELAY_MILLIS)
	  {
		  // check every RECONNECTION_DELAY_MINS if this device is connected to wifi. if it is, get
		  // latest connection info, otherwise connect
		  WIFI_Connect(&wifi);
		  reconnection_timestamp = uwTick;
	  }

	  // get battery voltage
	  BATTERY_GetVoltage();

	  // get distance with ultrasonic sensor
	  for (uint8_t i = 0; i < DIST_AVERAGE_VALUES; i++)
	  {
		  mean_dist += 343 * SENS_SendTrig(&trig) / 20000;
	  }
	  mean_dist /= DIST_AVERAGE_VALUES;
	  features.sensor_dist = mean_dist;

	  if (features.sensor_dist < TRIGGER_DISTANCE)	// centimeters
	  {
		  HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, 1);
		  if (relay_on_timestamp == 0 || switches[RELAY_SWITCH].pressed)
			  relay_on_timestamp = uwTick;
		  if (uwTick - relay_on_timestamp > RELAY_ON_DELAY)
		  {
			  relay_on_timestamp = uwTick;
			  SWITCH_Press(&(switches[RELAY_SWITCH]));
		  }
	  }
	  else
	  {
		  HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, 0);
		  if (uwTick - relay_on_timestamp > RELAY_OFF_DELAY && !switches[RELAY_SWITCH].manual)
		  {
			  relay_on_timestamp = 0;
			  SWITCH_UnPress(&(switches[RELAY_SWITCH]));
		  }
		  else if (!switches[RELAY_SWITCH].pressed)
			  relay_on_timestamp = 0;
	  }

	  wifistatus = WAITING;
	  // HANDLE WIFI CONNECTION
	  wifistatus = WIFI_ReceiveRequest(&wifi, &conn, 10);
	  if (wifistatus == OK)
	  {
		  HAL_GPIO_TogglePin(STATUS_Port, STATUS_Pin);
		  char* key_ptr = NULL;

		  if ((key_ptr = WIFI_RequestHasKey(&conn, "wifi")))
			  WIFIHANDLER_HandleWiFiRequest(&conn, key_ptr);
		  else if ((key_ptr = WIFI_RequestHasKey(&conn, "switch")))
			  WIFIHANDLER_HandleSwitchRequest(&conn, key_ptr);

		  else if (conn.request_type == GET)
		  {
			  if ((key_ptr = WIFI_RequestHasKey(&conn, "features")))
				  WIFIHANDLER_HandleFeaturePacket(&conn, (char*)FEATURES_TEMPLATE);
			  else if ((key_ptr = WIFI_RequestHasKey(&conn, "notification")))
				  WIFIHANDLER_HandleNotificationRequest(&conn, key_ptr);
			  // other GET requests code here...

			  else WIFI_SendResponse(&conn, "404 Not Found", "Unknown command", 15);
		  }

		  else if (conn.request_type == POST)
		  {
        if ((key_ptr = WIFI_RequestHasKey(&conn, "trigger_distance")))
          HANDLER_SetTriggerDistance(&conn, key_ptr);

			  // other POST requests code here...
		  }

		  // other requests code here...

		  HAL_GPIO_TogglePin(STATUS_Port, STATUS_Pin);
	  }
	  // OPTIONAL
	  else if (wifistatus != TIMEOUT)
	  {
		  sprintf(wifi.buf, "Status: %d", wifistatus);
		  WIFI_ResetComm(&wifi, &conn);
		  WIFI_SendResponse(&conn, "500 Internal server error", wifi.buf, strlen(wifi.buf));
	  }

	  // OPTIONAL
	  WIFI_ResetConnectionIfError(&wifi, &conn, wifistatus);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void BATTERY_GetVoltage(void)
{
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 250);
	features.bat.voltage_mv = HAL_ADC_GetValue(&hadc1) * BAT_ADC_CALIBRATION_VALUE;
	features.bat.voltage_integer =features. bat.voltage_mv / 1000;
	features.bat.voltage_decimal = (features.bat.voltage_mv - features.bat.voltage_integer * 1000) / 10;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
