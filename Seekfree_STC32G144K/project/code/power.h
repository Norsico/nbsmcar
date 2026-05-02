#ifndef _POWER_H_
#define _POWER_H_

#include "zf_common_headfile.h"

#define POWER_ADC_PIN                (ADC1_CH0_P10)          /* 电压采样 */
#define POWER_ADC_RESOLUTION         (ADC_12BIT)             /* ADC精度 */
#define POWER_ADC_SAMPLE_COUNT       (3)                     /* 采样次数 */

#define POWER_EMPTY_DECI             (114)                   /* 空电电压 */
#define POWER_FULL_DECI              (126)                   /* 满电电压 */

void power_init(void);
uint16 power_get_voltage_deci(void);
uint8 power_get_percent(void);

#endif
