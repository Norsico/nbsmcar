#include "pid_control.h"

/************ 限幅函数 ************/
float pid_output_limit(float value, float max_val, float min_val)
{
    if (value > max_val) return max_val;
    if (value < min_val) return min_val;
    return value;
}

/************ PID参数初始化 ************/
void pid_param_init(pid_control_t *pid, float kp, float ki, float kd, float max_out, float min_out)
{
    pid->param.kp = kp;
    pid->param.ki = ki;
    pid->param.kd = kd;
    pid->param.max_out = max_out;
    pid->param.min_out = min_out;
    pid->param.deadband = 0.0f;
}

/************ PID控制器初始化 ************/
void pid_init(pid_control_t *pid)
{
    pid->target = 0.0f;
    pid->current = 0.0f;
    pid->error = 0.0f;
    pid->prev_error = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
    pid->dt = 0.01f;  // 默认10ms
}
/************ 增量式PI控制器 ************/
void pid_incremental_pi(pid_control_t* pid,int16 value,int16 target)
{
	// 计算误差
	float error = target-value;
	float prev_error = pid->error;
	// 写入
	pid->error = error;
	pid->prev_error = prev_error;
	// 增量式PI算法：Δu = Kp*(e(k)-e(k-1)) + Ki*e(k)
	pid->output += pid->param.kp * (error - prev_error) + pid->param.ki * error;
	// 限幅
	pid->output = pid_output_limit(pid->output,pid->param.max_out,pid->param.min_out);
}