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
extern int16 car_wheel_target_speed;
extern int16 ref_left_target;
extern int16 ref_right_target;

// 函数
void car_wheel_set_speed(car_wheel_index_enum wheel, int8 speed_percent); // 单独设置速度
void car_wheel_set_dual(int8 right_wheel_speed_percent, int8 left_wheel_speed_percent); // 设置占空比
void car_wheel_stop_all(void);  // 全部停止
void car_wheel_init(void); // 初始化
void car_wheel_pid_init(void); // pid结构体初始化
void car_wheel_set_target(int16 speed); // 设置整车目标速度，由内部自动换算左右轮目标
void car_wheel_control_reset(void); // 清空闭环状态并停轮
void car_wheel_emergency_brake(void); // 紧急状态下按编码器反拖刹车
void car_wheel_hold(void); // 保持后轮待转，不清 Start 速度目标
void car_wheel_update(void); // 自动调用pid进行车轮更新函数

#endif


