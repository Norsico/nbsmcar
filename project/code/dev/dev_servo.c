/*
 * dev_servo.c - 舵机串级PID控制实现
 *
 * 包含：角度环（外环）+ 角速度环（内环）串级PID
 */

#include "dev_servo.h"

/************ 限幅函数 ************/
static float servo_limit(float value, float max_val, float min_val)
{
    if (value > max_val) return max_val;
    if (value < min_val) return min_val;
    return value;
}

/************ PID参数初始化 ************/
static void pid_param_init(servo_pid_t *pid, float kp, float ki, float kd, float max_out, float min_out)
{
    pid->param.kp = kp;
    pid->param.ki = ki;
    pid->param.kd = kd;
    pid->param.max_out = max_out;
    pid->param.min_out = min_out;
    pid->param.deadband = 0.0f;
}

/************ PID控制器初始化 ************/
static void pid_init(servo_pid_t *pid)
{
    pid->target = 0.0f;
    pid->current = 0.0f;
    pid->error = 0.0f;
    pid->prev_error = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
    pid->dt = 0.01f;  // 默认10ms
}

/************ 单环PID计算 ************/
static float pid_calc(servo_pid_t *pid, float target, float current)
{
    float p_out, i_out, d_out;

    pid->target = target;
    pid->current = current;
    pid->error = target - current;

    // 死区处理
    if (pid->param.deadband > 0.0f)
    {
        if (pid->error > -pid->param.deadband && pid->error < pid->param.deadband)
        {
            pid->error = 0.0f;
        }
    }

    // 比例项
    p_out = pid->param.kp * pid->error;

    // 积分项 (带抗饱和)
    pid->integral = pid->integral + pid->error * pid->dt;
    // 积分限幅
    pid->integral = servo_limit(pid->integral, pid->param.max_out / pid->param.ki, pid->param.min_out / pid->param.ki);
    i_out = pid->param.ki * pid->integral;

    // 微分项
    d_out = pid->param.kd * (pid->error - pid->prev_error) / pid->dt;

    // 保存上一次误差
    pid->prev_error = pid->error;

    // 计算输出
    pid->output = p_out + i_out + d_out;

    // 输出限幅
    pid->output = servo_limit(pid->output, pid->param.max_out, pid->param.min_out);

    return pid->output;
}

/************ 串级控制器接口 ************/

void servo_cascade_init(servo_cascade_t *ctrl)
{
    pid_init(&ctrl->angle_loop);
    pid_init(&ctrl->gyro_loop);

    ctrl->gyro_rate = 0.0f;
    ctrl->error_to_rate_ratio = 1.0f;
    ctrl->servo_max = 45.0f;
    ctrl->servo_min = -45.0f;
    ctrl->servo_center = 0.0f;
}

void servo_angle_pid_config(servo_cascade_t *ctrl, float kp, float ki, float kd, float max_out, float min_out)
{
    pid_param_init(&ctrl->angle_loop, kp, ki, kd, max_out, min_out);
}

void servo_gyro_pid_config(servo_cascade_t *ctrl, float kp, float ki, float kd, float max_out, float min_out)
{
    pid_param_init(&ctrl->gyro_loop, kp, ki, kd, max_out, min_out);
}

void servo_set_error_to_rate_ratio(servo_cascade_t *ctrl, float ratio)
{
    ctrl->error_to_rate_ratio = ratio;
}

void servo_set_output_limit(servo_cascade_t *ctrl, float max_angle, float min_angle, float center)
{
    ctrl->servo_max = max_angle;
    ctrl->servo_min = min_angle;
    ctrl->servo_center = center;
}

void servo_set_dt(servo_cascade_t *ctrl, float dt)
{
    ctrl->angle_loop.dt = dt;
    ctrl->gyro_loop.dt = dt;
}

void servo_update_gyro(servo_cascade_t *ctrl, float gyro_rate)
{
    ctrl->gyro_rate = gyro_rate;
}

float servo_cascade_calc(servo_cascade_t *ctrl, float error)
{
    float angle_output;
    float gyro_target;
    float gyro_output;

    // ========== 外环：角度环 ==========
    // 位置误差 -> 期望角速度
    // 预留误差到角速度的转换接口
    gyro_target = error * ctrl->error_to_rate_ratio;

    // 外环PID计算
    angle_output = pid_calc(&ctrl->angle_loop, gyro_target, ctrl->gyro_rate);

    // ========== 内环：角速度环 ==========
    // 期望角速度 - 实测角速度 -> 舵机角度
    gyro_output = pid_calc(&ctrl->gyro_loop, angle_output, ctrl->gyro_rate);

    // 最终输出限幅
    gyro_output = servo_limit(gyro_output, ctrl->servo_max, ctrl->servo_min);

    // 加上中值偏移
    return gyro_output + ctrl->servo_center;
}

void servo_cascade_reset(servo_cascade_t *ctrl)
{
    pid_init(&ctrl->angle_loop);
    pid_init(&ctrl->gyro_loop);
    ctrl->gyro_rate = 0.0f;
}

float servo_get_output(servo_cascade_t *ctrl)
{
    return ctrl->gyro_loop.output + ctrl->servo_center;
}

float servo_get_target_gyro(servo_cascade_t *ctrl)
{
    return ctrl->angle_loop.target;
}

float servo_get_gyro_error(servo_cascade_t *ctrl)
{
    return ctrl->gyro_loop.error;
}
