#ifndef _PID_CONTROL_H_
#define _PID_CONTROL_H_

#include "zf_common_typedef.h"

/************ PID参数结构体 ************/
typedef struct
{
    float kp;           // 比例系数
    float ki;           // 积分系数
    float kd;           // 微分系数
    float max_out;      // 输出限幅最大值
    float min_out;      // 输出限幅最小值
    float deadband;     // 死区
} pid_param_t;

/************ PID控制器结构体 ************/
typedef struct
{
    pid_param_t param;    // PID参数
    float target;               // 目标值
    float current;              // 当前值
    float error;                // 当前误差
    float prev_error;           // 上一次误差
    float integral;             // 积分累加
    float output;               // PID输出
    float dt;                    // 采样周期(秒)
} pid_control_t;

// pid初始化函数，可直接编辑结构体参数，自行初始化
void pid_param_init(pid_control_t *pid, float kp, float ki, float kd, float max_out, float min_out);
void pid_init(pid_control_t *pid);
void pid_incremental_pi(pid_control_t* pid,int16 value,int16 target);
float pid_output_limit(float value, float max_val, float min_val);

#endif