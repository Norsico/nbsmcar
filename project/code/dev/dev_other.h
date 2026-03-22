#ifndef _DEV_GPIO_H_
#define _DEV_GPIO_H_

#include "zf_driver_gpio.h"
#include "zf_driver_pwm.h"

#define BUZZER_PWM (PWMC_CH2N_P65)
#define BUZZER_FREQ (1000)
#define LASER_PIN (IO_P67)

void other_init(void);
// ·äÃùÆ÷
void buzzer_on(void);
void buzzer_off(void);
// ¼¤¹â±Ê
void laser_on(void);
void laser_off(void);

#endif