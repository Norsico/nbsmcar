#include "power.h"

static uint16 power_voltage_deci = 0;
static uint8 power_percent = 0;

/* 电量百分比 */
static uint8 power_calc_percent(uint16 voltage_deci)
{
    if(voltage_deci <= POWER_EMPTY_DECI)
    {
        return 0;
    }

    if(voltage_deci >= POWER_FULL_DECI)
    {
        return 100;
    }

    return (uint8)(((uint16)(voltage_deci - POWER_EMPTY_DECI) * 100U) /
                   (POWER_FULL_DECI - POWER_EMPTY_DECI));
}

/* 电压采样 */
static void power_sample_once(void)
{
    uint16 adc_value;
    uint32 voltage_calc;

    adc_value = adc_mean_filter_convert(POWER_ADC_PIN, POWER_ADC_SAMPLE_COUNT);
    voltage_calc = (uint32)adc_value * 363U + 2047U;
    power_voltage_deci = (uint16)(voltage_calc / 4095U);
    power_percent = power_calc_percent(power_voltage_deci);
}

/* 电量初始化 */
void power_init(void)
{
    adc_init(POWER_ADC_PIN, POWER_ADC_RESOLUTION);
    power_sample_once();
}

/* 电压值 */
uint16 power_get_voltage_deci(void)
{
    return power_voltage_deci;
}

/* 电量值 */
uint8 power_get_percent(void)
{
    return power_percent;
}
