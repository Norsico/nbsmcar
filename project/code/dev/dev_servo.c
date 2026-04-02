/*
 * dev_servo.c - 舵机串级PID控制实现
 *
 * 包含：角度环（外环）+ 角速度环（内环）串级PID
 */

#include "dev_servo.h"

// 前轮舵机默认工作在 80-110 度之间
#define CAR_SERVO_DUTY(angle)     ((float)PWM_DUTY_MAX / (1000.0 / (float)CAR_SERVO_FREQ) * (0.5 + (float)(angle) / 90.0))
#if ((CAR_SERVO_FREQ < 50) || (CAR_SERVO_FREQ > 300))
    #error "CAR_SERVO_FREQ ERROR!"
#endif

static uint8 g_car_servo_min_angle = CAR_SERVO_MIN_ANGLE;
static uint8 g_car_servo_max_angle = CAR_SERVO_MAX_ANGLE;

/* 屏幕调参时，左限幅只允许落在中心值左侧，避免把最小角调到中心右边。 */
static uint8 car_servo_limit_tune_min(uint8 angle)
{
    if(angle < CAR_SERVO_TUNE_MIN_ANGLE)
    {
        return CAR_SERVO_TUNE_MIN_ANGLE;
    }

    if(angle >= CAR_SERVO_CENTER_ANGLE)
    {
        return (uint8)(CAR_SERVO_CENTER_ANGLE - 1);
    }

    return angle;
}

/* 屏幕调参时，右限幅只允许落在中心值右侧，避免把最大角调到中心左边。 */
static uint8 car_servo_limit_tune_max(uint8 angle)
{
    if(angle > CAR_SERVO_TUNE_MAX_ANGLE)
    {
        return CAR_SERVO_TUNE_MAX_ANGLE;
    }

    if(angle <= CAR_SERVO_CENTER_ANGLE)
    {
        return (uint8)(CAR_SERVO_CENTER_ANGLE + 1);
    }

    return angle;
}

// 舵机角度限幅函数
static uint8 car_servo_limit_angle(uint8 angle)
{
    if(angle < g_car_servo_min_angle)
    {
        return g_car_servo_min_angle;
    }

    if(angle > g_car_servo_max_angle)
    {
        return g_car_servo_max_angle;
    }

    return angle;
}
// 设置舵机角度
void car_servo_set_angle(uint8 angle)
{
    uint8 safe_angle = 0;

    safe_angle = car_servo_limit_angle(angle);
    pwm_set_duty(CAR_SERVO_PWM_PIN, CAR_SERVO_DUTY(safe_angle));
}
// 回中函数
void car_servo_set_center(void)
{
    car_servo_set_angle(CAR_SERVO_CENTER_ANGLE);
}

void car_servo_set_limit(uint8 min_angle, uint8 max_angle)
{
    uint8 safe_min = 0;
    uint8 safe_max = 0;

    /* 先把用户输入收口到允许调节的安全范围内。 */
    safe_min = car_servo_limit_tune_min(min_angle);
    safe_max = car_servo_limit_tune_max(max_angle);
    if(safe_min >= safe_max)
    {
        /* 如果左右限幅交叉了，就退回工程默认值，避免舵机直接卡死。 */
        safe_min = CAR_SERVO_MIN_ANGLE;
        safe_max = CAR_SERVO_MAX_ANGLE;
    }

    g_car_servo_min_angle = safe_min;
    g_car_servo_max_angle = safe_max;
}

uint8 car_servo_get_min_angle(void)
{
    return g_car_servo_min_angle;
}

uint8 car_servo_get_max_angle(void)
{
    /* 当前巡线输出限幅都走运行时缓存，外部直接读这个值即可。 */
    return g_car_servo_max_angle;
}

// 舵机PWM初始化
void car_servo_init(void)
{
    /* 上电先回到编译期默认限幅，后面如果 flash 里有调参值再覆盖。 */
    g_car_servo_min_angle = CAR_SERVO_MIN_ANGLE;
    g_car_servo_max_angle = CAR_SERVO_MAX_ANGLE;
    // pwm初始化，直接回中
    pwm_init(CAR_SERVO_PWM_PIN, CAR_SERVO_FREQ, CAR_SERVO_DUTY(CAR_SERVO_CENTER_ANGLE));
}


/************ 单环PID计算 ************/
static float pid_calc(pid_control_t *pid, float target, float current)
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
    pid->integral = pid_output_limit(pid->integral, pid->param.max_out / pid->param.ki, pid->param.min_out / pid->param.ki);
    i_out = pid->param.ki * pid->integral;

    // 微分项
    d_out = pid->param.kd * (pid->error - pid->prev_error) / pid->dt;

    // 保存上一次误差
    pid->prev_error = pid->error;

    // 计算输出
    pid->output = p_out + i_out + d_out;

    // 输出限幅
    pid->output = pid_output_limit(pid->output, pid->param.max_out, pid->param.min_out);

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
    gyro_output = pid_output_limit(gyro_output, ctrl->servo_max, ctrl->servo_min);

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
