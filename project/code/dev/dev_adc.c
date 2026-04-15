#include "dev_adc.h"

static uint8 power_adc_ready = 0;

static float power_adc_calc_voltage(uint16 value)
{
	return value * 3.3f / ADC_RANGE * 11;
}

// ADC初始化
/* 初始化电池电压 ADC。 */
void power_adc_init(void){
	adc_init(PWOER_ADC_PIN,ADC_RESOLUTION);
	power_adc_ready = 1;
}
// 读取滤波后的ADC值
static uint16 power_adc_update(void){
	uint16 value = adc_mean_filter_convert(PWOER_ADC_PIN,3); // 3次采样平均
	return value;
}
/* 判断电池电压是否过低。 */
uint8 power_adc_judge(void){
	uint8 sign = 0;
	uint16 value = power_adc_update(); // 获取
	float voltage = 0.0f;
	voltage = power_adc_calc_voltage(value);
	if(voltage < 11.1){
		// 电压过低
		sign = 1;
	}
	return sign;
}

float power_adc_get_voltage(void)
{
	if(!power_adc_ready){
		return 0.0f;
	}
	return power_adc_calc_voltage(power_adc_update());
}

/* 查询 ADC 是否已初始化。 */
uint8 power_adc_is_ready(void)
{
	return power_adc_ready;
}
