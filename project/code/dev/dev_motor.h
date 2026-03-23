#ifndef _DEV_MOTOR_H_
#define _DEV_MOTOR_H_

// 无刷电机速度控制

#include "zf_driver_pwm.h"

#define BLDC_MOTOR_1 (PWME_CH1N_PA1)
#define BLDC_MOTOR_2 (PWME_CH2N_PA3)
#define BLDC_MOTOR_FREQ (50) // 控制频率50，最高支持300


void bldc_motor_init(void);
void bldc_motor_set_duty(uint8 left_speed,uint8 right_speed);
void bldc_motor_stop(void);

#endif 