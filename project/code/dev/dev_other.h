#ifndef _DEV_GPIO_H_
#define _DEV_GPIO_H_

#include "zf_driver_gpio.h"
#include "zf_driver_pwm.h"

#define BUZZER_PIN (IO_P65)
#define LASER_PIN (IO_P67)
#define SWITCH_PIN1 (IO_PB0)
#define SWITCH_PIN2 (IO_PB1)

void other_init(void);
// 蜂鸣器
void buzzer_on(void);
void buzzer_off(void);
// 激光笔
void laser_on(void);
void laser_off(void);
// 拨码开关
void switch_update(void);
uint8 switch_debug_enabled(void);
uint8 switch_ui_enabled(void);
uint8 switch_wifi_enabled(void);

#endif
