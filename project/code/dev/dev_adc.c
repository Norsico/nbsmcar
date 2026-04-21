#include "dev_adc.h"

static float voltage=0.0;

/********* 初始化后默认ADC能够使用，不需要标志位 ********/

// ADC初始化
/* 初始化电池电压 ADC。 */
void power_adc_init(void){
	adc_init(PWOER_ADC_PIN,ADC_RESOLUTION);
}
// 读取滤波后的ADC值
uint16 power_adc_update(void){
	uint16 value = adc_mean_filter_convert(PWOER_ADC_PIN,3); // 3次采样平均

	voltage = (float)value * 3.3f / ADC_RANGE * 11.0f;
	return value;
}
/* 判断电池电压是否过低。 */
uint8 power_adc_judge(void){
	uint8 sign = 0;
	power_adc_update(); // 获取并刷新当前电压缓存

	if(voltage < 11.1){
		// 电压过低
		sign = 1;
	}
	return sign;
}

float power_adc_get_voltage(void)
{
	return voltage;
}
