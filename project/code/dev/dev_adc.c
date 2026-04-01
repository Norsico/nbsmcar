#include "dev_adc.h"

static float power_adc_voltage = 0.0f;

// ADC初始化
void power_adc_init(void){
	adc_init(PWOER_ADC_PIN,ADC_RESOLUTION);
}
// 读取滤波后的ADC值
static uint16 power_adc_update(void){
	uint16 value = adc_mean_filter_convert(PWOER_ADC_PIN,3); // 3次采样平均
	return value;
}
/**
 * @brief 判断电池电量,过低报警
 * @note 电阻分压比例 200k ：20k = 10 ：1
	实际电压为测试值 x 11
*/
uint8 power_adc_judge(void){
	uint8 sign = 0;
	uint16 value = power_adc_update(); // 获取
	power_adc_voltage = value * 3.3f / ADC_RANGE * 11;
	if(power_adc_voltage < 11.1f){
		// 电压过低
		sign = 1;
	}
	return sign;
}

// 获取电池电压(伏特)
float power_adc_get_voltage(void){
	return power_adc_voltage;
}