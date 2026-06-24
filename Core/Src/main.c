/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include "../thermistor/adc_table.h"

#include "../ESP8266/esp8266.h"
#include "../Flash/flash.h"
#include "../wifihandler/wifihandler.h"
#include "../wifihandler/userhandlers.h"
#include "../credentials.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define AVG_VALUES 50
#define N_SENSORS 4
#define ISENSE_INDEX 0
#define VSENSE_INDEX 1
#define LDO_NTC_INDEX 2
#define RLY_NTC_INDEX 3

#define VOLTAGE_CAL_VALUE 0
#define CURRENT_CAL_VALUE -1753

#define CURRENT_DEADZONE 0.18   // amperes
#define MAX_RELAY_POWER 1000    // watts

#define ACTIVATION_DISTANCE 60  // cm
#define ACTIVATION_TIME 1500    // milliseconds
#define TURN_OFF_TIME 15000     // milliseconds
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t adc_buf[4 * AVG_VALUES];
uint16_t ldo_temp, rly_temp;
uint32_t voltage, current_int, current_dec, power;
uint16_t echo;
uint32_t sens_distance, last_distance;
WIFI_t wifi;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t start_check = 0;
uint32_t distance_check_time = 0, turn_off_check_time = 0;
uint8_t startup_finished = 0;

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
  if (!startup_finished) return;
  HAL_TIM_Base_Start(&htim17);
}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
  if (!startup_finished) return;
  HAL_TIM_Base_Stop(&htim17);
  echo = TIM17->CNT;
  __HAL_TIM_SET_COUNTER(&htim17, 0);
  // wait ~250ms before sending new trig pulse
  __HAL_TIM_ENABLE(&htim14);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (!startup_finished) return;
  if (htim->Instance == TIM14)
  {
    // timer is already stopped (one pulse mode)
    // send 10us trig pulse
    __HAL_TIM_ENABLE(&htim1);
  }
}

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
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM17_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc_buf, 4 * AVG_VALUES);

  // configure and start trig timer
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_Base_Start_IT(&htim14);

  // make sure relay is off. user should set it as RESET by default anyway on CubeMX...
  HAL_GPIO_WritePin(RLY_GPIO_Port, RLY_Pin, GPIO_PIN_RESET);

  if (ESP8266_Init() == TIMEOUT)
  {
	  while (1)
		  __NOP();
  }

#ifdef ENABLE_SAVE_TO_FLASH
  FLASH_ReadSaveData();
  if (WIFI_SetName(&wifi, savedata.name) == ERR)
    WIFI_SetName(&wifi, (char*)ESP_NAME); // happens when there is nothing saved to FLASH, so set default name
  WIFI_SetIP(&wifi, savedata.ip);         // if there is nothing saved to FLASH, this function does nothing
#else
  WIFI_SetName(&wifi, (char*)ESP_NAME);
#endif

  memcpy(wifi.SSID, ssid, strlen(ssid));
  memcpy(wifi.pw, password, strlen(password));
  HAL_GPIO_WritePin(STATUS_Port, STATUS_Pin, 1);
  uint32_t connect_status = WIFI_Connect(&wifi);
  if (connect_status == OK)
  {
    WIFI_EnableNTPServer(&wifi, 2);
    /*
    The first time the ESP connects to WiFi, the gateway assigns an IP to it, which now gets saved to FLASH.
    The next time the ESP connects, the gateway could assign a different IP; to prevent this, the function
    WIFI_SetIP(&wifi, savedata.ip); loads the IP previously saved on FLASH so that the ESP tries to connect
    and get this IP
    */
    strncpy(savedata.ip, wifi.IP, 15);
    FLASH_WriteSaveData();
  }
  HAL_GPIO_WritePin(STATUS_Port, STATUS_Pin, 0);

  SWITCH_Init(&(switches[RELAY_SWITCH]), false, RLY_GPIO_Port, RLY_Pin);

  startup_finished = 1;

  __HAL_TIM_ENABLE(&htim1);

  if (connect_status == OK)
  {
    if (WIFIHANDLER_MQTT_Init(&wifi, MQTT_BROKER_IP, MQTT_BROKER_PORT) == OK) 
    {
        WIFIHANDLER_MQTT_PublishDiscovery(&wifi);
        WIFIHANDLER_MQTT_PublishStates(&wifi);
    }
  }

  uint32_t last_mqtt_publish = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t isense_avg = 0, vsense_avg = 0, ldo_avg = 0, rly_avg = 0;
    sens_distance = 0;
    if (AVG_VALUES > 1)
    {
      for (uint32_t i = 0; i < N_SENSORS * AVG_VALUES; i += N_SENSORS)
      {
        isense_avg += adc_buf[i + ISENSE_INDEX];
        vsense_avg += adc_buf[i + VSENSE_INDEX];
        ldo_avg += adc_buf[i + LDO_NTC_INDEX];
        rly_avg += adc_buf[i + RLY_NTC_INDEX];
      }
  
      isense_avg /= AVG_VALUES;
      vsense_avg /= AVG_VALUES;
      ldo_avg /= AVG_VALUES;
      rly_avg /= AVG_VALUES;
    }
    else
    {
      isense_avg = adc_buf[ISENSE_INDEX];
      vsense_avg = adc_buf[VSENSE_INDEX];
      ldo_avg = adc_buf[LDO_NTC_INDEX];
      rly_avg = adc_buf[RLY_NTC_INDEX];
    }

    sens_distance = (echo * 343 / 20000 + last_distance) / 2;
    last_distance = sens_distance;

    //vcur = (float)isense / 4096.0 * 3.3;
    //current = vcur * 6.95;
    // vcur is the opamp vout
    // 6.95 is a calibration value. tested at 2.17A
    uint32_t current_mul_100 = isense_avg * (5599 + CURRENT_CAL_VALUE) / 1000;
    if (current_mul_100 < CURRENT_DEADZONE * 100)
      current_mul_100 = 0;
    current_int = current_mul_100 / 100;
    current_dec = current_mul_100 - current_int * 100;

    // calibration: output voltage at 297V mains is 0.499V and the voltage
    // increase is 0.04V (output) per each 1V (mains) increase
    //vvolt = (float)vsense / 4096.0 * 3.3;
    //voltage = ((vvolt - 0.499) / 0.040 + 297) * 0.707;
    // the below values have been all multiplied by 100 to avoid floating point math
    voltage = ((vsense_avg) * (7950 + VOLTAGE_CAL_VALUE) / 4096 - 1247 + 29700) * 707 / 100000;

    power = voltage * current_mul_100 / 100;

    ldo_temp = getTemperature(ldo_avg);
    rly_temp = getTemperature(rly_avg);

    if (sens_distance < ACTIVATION_DISTANCE)
    {
      // if distance is low enough, start check timer
      if (!start_check)
      {
        if (!switches[RELAY_SWITCH].pressed)
        {
          distance_check_time = uwTick;
          start_check = 1;
        }
      }
    }
    else
    {
      // if distance is too much, reset the distance check time
      distance_check_time = uwTick;

      // if the relay is on, start the turn off check time (once)
      if (switches[RELAY_SWITCH].pressed && turn_off_check_time == 0)
        turn_off_check_time = uwTick;
    }

    // if enough time has passed while the distance is low enough, turn on the relay
    if (start_check && uwTick - distance_check_time > ACTIVATION_TIME)
    {
      SWITCH_Press(&switches[RELAY_SWITCH]);
      start_check = 0;
      distance_check_time = uwTick;
      turn_off_check_time = 0;
    }

    // if enough time has passed while the distance is high enough, turn off the relay
    // also make sure to only turn off if it's not in manual mode (turned on by user)
    if (turn_off_check_time != 0 && uwTick - turn_off_check_time > TURN_OFF_TIME && !switches[RELAY_SWITCH].manual)
    {
      turn_off_check_time = 0;
      SWITCH_UnPress(&switches[RELAY_SWITCH]);
    }

    if (rly_temp > OVERTEMP_THRESHOLD || ldo_temp > OVERTEMP_THRESHOLD)
    {
      if (switches[RELAY_SWITCH].pressed) {
          SWITCH_UnPress(&switches[RELAY_SWITCH]);
          WIFIHANDLER_MQTT_PublishStates(&wifi);
          WIFIHANDLER_MQTT_SendNotification(&wifi, (char*)OVERTEMP_TEXT);
      }
    }

    if (power > MAX_RELAY_POWER)
    {
      if (switches[RELAY_SWITCH].pressed) {
          SWITCH_UnPress(&switches[RELAY_SWITCH]);
          WIFIHANDLER_MQTT_PublishStates(&wifi);
          WIFIHANDLER_MQTT_SendNotification(&wifi, (char*)OVERLOAD_TEXT);
      }
    }

    HAL_GPIO_WritePin(STATUS_Port, STATUS_Pin, GPIO_PIN_SET);
    WIFIHANDLER_MQTT_Loop(&wifi);
    HAL_GPIO_WritePin(STATUS_Port, STATUS_Pin, GPIO_PIN_RESET);

    WIFIHANDLER_ReconnectIfDisconnected(&wifi);

    if (HAL_GetTick() - last_mqtt_publish > MQTT_PUBLISH_INTERVAL) 
    {
        last_mqtt_publish = HAL_GetTick();
        WIFIHANDLER_MQTT_PublishStates(&wifi);
    }
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
