#include "pid_control.h"

/************ 限幅函数 ************/
int16 pid_output_limit(int16 value, int16 max_val, int16 min_val)
{
    if (value > max_val) return max_val;
    if (value < min_val) return min_val;
    return value;
}

/************ PID参数初始化 ************/
void pid_param_init(pid_control_t *pid, int16 kp, int16 ki, int16 kd, int16 max_out, int16 min_out)
{
    pid->param.kp = kp;
    pid->param.ki = ki;
    pid->param.kd = kd;
    pid->param.max_out = max_out;
    pid->param.min_out = min_out;
}

/************ PID控制器初始化 ************/
void pid_init(pid_control_t *pid)
{
    pid->target = 0;
    pid->current = 0;
    pid->error = 0;
    pid->prev_error = 0;
    pid->integral = 0;
    pid->output = 0;
    pid->dt = 10;  // 默认10ms
}
/************ 增量式PI控制器 ************/
void pid_incremental_pi(pid_control_t* pid,int16 value,int16 target)
{
	// 计算误差
	int16 error = target-value;
	int16 prev_error = pid->error;
	// 写入
	pid->error = error;
	pid->prev_error = prev_error;
	// 增量式PI算法：Δu = Kp*(e(k)-e(k-1)) + Ki*e(k)
    // 先除10确保两位小数有效，再除100得到正常值
	pid->output += pid->param.kp/10 * (error - prev_error)/100 + pid->param.ki/10 * error/100;
	// 限幅
	pid->output = pid_output_limit(pid->output,pid->param.max_out,pid->param.min_out);
}