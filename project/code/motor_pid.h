/*********************************************************************************************************************
* 电机PID控制模块
*
* 功能说明：实现编码器-直流电机闭环PID控制
*
* 修改记录
* 日期              备注
* 2026-03-13        创建PID控制模块
********************************************************************************************************************/

#ifndef _MOTOR_PID_H_
#define _MOTOR_PID_H_

#include "zf_common_headfile.h"

// PID参数结构体
typedef struct {
    float Kp;               // 比例系数
    float Ki;               // 积分系数
    float Kd;               // 微分系数（可选）
    int16 target_speed;     // 目标速度
    int16 current_speed;    // 当前速度
    int16 error;            // 当前误差
    int16 last_error;       // 上次误差
    int16 pwm_output;       // PWM输出值
} motor_pid_t;

// 左右轮PID控制器
extern motor_pid_t motor_pid_left;
extern motor_pid_t motor_pid_right;

// 函数声明
void motor_pid_init(void);                              // PID初始化
void motor_pid_set_speed(int16 left_speed, int16 right_speed);  // 设置目标速度（脉冲数/10ms）
void motor_pid_update(void);                            // PID更新（在中断中调用）
int16 motor_pid_get_pwm_left(void);                     // 获取左轮PWM
int16 motor_pid_get_pwm_right(void);                    // 获取右轮PWM

#endif