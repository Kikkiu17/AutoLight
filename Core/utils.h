/*
 * utils.h
 *
 *  Created on: Sep 20, 2025
 *      Author: kikkiu
 */

#ifndef UTILS_H_
#define UTILS_H_

#include "ESP8266/esp8266.h"

uint16_t SENS_SendTrig(Switch_t* sw)
{
	uint16_t diff = 0;
	uint16_t stop_time = 0;

	SWITCH_Press(sw);
	// 10 us delay
	TIM17->CNT = 0;
	uint16_t start_time = TIM17->CNT;
	while (TIM17->CNT - start_time < 10) { __NOP(); }
	SWITCH_UnPress(sw);

	while (!HAL_GPIO_ReadPin(ECHO_GPIO_Port, ECHO_Pin))
	{
		if (TIM17->CNT - start_time > 50000)
			return 50000;
		__NOP();
	}

	TIM17->CNT = 0;
	start_time = TIM17->CNT;
	//__HAL_TIM_URS_ENABLE(&htim17);
	while (HAL_GPIO_ReadPin(ECHO_GPIO_Port, ECHO_Pin)) { __NOP(); }
	stop_time = TIM17->CNT;
	diff = stop_time - start_time;

	return diff;
}

#endif /* UTILS_H_ */
