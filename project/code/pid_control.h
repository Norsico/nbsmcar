#ifndef _PID_CONTROL_H_
#define _PID_CONTROL_H_

#include "zf_common_typedef.h"

/**
 * 说明，为尽可能优化，调整pid计算的浮点数为int16类型
 * 为保留三位有效数字，扩大1000倍，同时考虑到int16的范围
 * 三个系数有效值 在-32到32之间
 */

/************ PID参数结构体 ************/
typedef struct
{
    int16 kp;           // 比例系数 x 1000 
    int16 ki;           // 积分系数 x 1000
    int16 kd;           // 微分系数 x 1000
    int16 max_out;      // 输出限幅最大值
    int16 min_out;      // 输出限幅最小值
} pid_param_t;

/************ PID控制器结构体 ************/
typedef struct
{
    pid_param_t param;    // PID参数
    int16 target;               // 目标值
    int16 current;              // 当前值
    int16 error;                // 当前误差
    int16 prev_error;           // 上一次误差
    int16 integral;             // 积分累加
    int16 output;               // PID输出
    int16 dt;                    // 采样周期(秒)
} pid_control_t;

// pid初始化函数，可直接编辑结构体参数，自行初始化
void pid_param_init(pid_control_t *pid, int16 kp, int16 ki, int16 kd, int16 max_out, int16 min_out);
void pid_init(pid_control_t *pid);
void pid_incremental_pi(pid_control_t* pid,int16 value,int16 target);
int16 pid_output_limit(int16 value, int16 max_val, int16 min_val);

#endif