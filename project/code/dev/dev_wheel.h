#ifndef _DEV_WHEEL_H_
#define _DEV_WHEEL_H_

/*********************
* 车轮有刷电机控制
*********************/
#include "zf_driver_gpio.h"
#include "zf_driver_pwm.h"
#include "pid_control.h"

#define RIGHT_MOTOR_DIR_PIN        (IO_P75)
#define RIGHT_MOTOR_PWM_PIN        (PWMB_CH1_P74)
#define LEFT_MOTOR_DIR_PIN         (IO_P77)
#define LEFT_MOTOR_PWM_PIN         (PWMB_CH3_P76)

typedef enum
{
    RIGHT_MOTOR = 0,
    LEFT_MOTOR,
    CAR_MOTOR_COUNT
} car_wheel_index_enum;

// 左右轮PID控制器
extern pid_control_t wheel_pid_left;
extern pid_control_t wheel_pid_right;

// 函数 
void car_wheel_set_speed(car_wheel_index_enum wheel, int8 speed_percent); // 单独设置速度
void car_wheel_set_dual(int8 right_wheel_speed_percent, int8 left_wheel_speed_percent); // 设置占空比
void car_wheel_stop_all(void);  // 全部停止
void car_wheel_init(void);
void car_wheel_pid_init(void);
void car_wheel_set_target(float left_speed, float right_speed);
void car_wheel_updata(void); // 初始化

#endif


