#ifndef __ADC_TABLE_H__
#define __ADC_TABLE_H__

#include <inttypes.h>

// celsius
#define LOW_TEMP 10
#define HIGH_TEMP 150

uint16_t getTemperature(uint16_t ntc_adc_val);

#endif