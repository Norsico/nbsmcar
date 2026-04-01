#ifndef _DEV_ADC_H_
#define _DEV_ADC_H_

#include "zf_driver_adc.h"

#define PWOER_ADC_PIN (ADC1_CH0_P10)
#define ADC_RESOLUTION (ADC_8BIT) // 不给太高，只要能读取即可
#define ADC_RANGE (256.0f)

void power_adc_init(void);
uint8 power_adc_judge(void);
float power_adc_get_voltage(void);

#endif