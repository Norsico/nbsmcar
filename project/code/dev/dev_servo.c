/*
 * dev_servo.c - 舵机串级PID控制实现
 *
 * 包含：角度环（外环）+ 角速度环（内环）串级PID
 */

#include "dev_servo.h"

// 前轮舵机默认工作在 80-110 度之间 ，angle 单位为0.01° ，频率为50或300，可以先除100，不失精度，保持占空比相同
//(float)(PWM_DUTY_MAX / 1000 * CAR_SERVO_FREQ) * (0.5 + (float)(angle) / 90.0)
#define CAR_SERVO_DUTY(angle)     (PWM_DUTY_MAX / 1000 * CAR_SERVO_FREQ) / 100 * (50 + angle / 90)
#if ((CAR_SERVO_FREQ < 50) || (CAR_SERVO_FREQ > 300))
    #error "CAR_SERVO_FREQ ERROR!"
#endif 

static uint16 g_car_servo_min_angle = CAR_SERVO_MIN_ANGLE;
static uint16 g_car_servo_max_angle = CAR_SERVO_MAX_ANGLE;
static uint16 g_car_servo_current_angle = CAR_SERVO_CENTER_ANGLE;

/* 屏幕调参时，左限幅保持在中心值左侧。 */static uint16 car_servo_limit_tune_min(uint16 angle)
{
    if(angle < CAR_SERVO_TUNE_MIN_ANGLE)
    {
        return CAR_SERVO_TUNE_MIN_ANGLE;
    }

    if(angle >= CAR_SERVO_CENTER_ANGLE)
    {
        return (CAR_SERVO_CENTER_ANGLE - 100);
    }

    return angle;
}

/* 屏幕调参时，右限幅保持在中心值右侧。 */
static uint16 car_servo_limit_tune_max(uint16 angle)
{
    if(angle > CAR_SERVO_TUNE_MAX_ANGLE)
    {
        return CAR_SERVO_TUNE_MAX_ANGLE;
    }

    if(angle <= CAR_SERVO_CENTER_ANGLE)
    {
        return (CAR_SERVO_CENTER_ANGLE + 100);
    }

    return angle;
}

// 舵机角度限幅函数
static uint16 car_servo_limit_angle(uint16 angle)
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
void car_servo_set_angle(uint16 angle)
{
    uint16 safe_angle = 0;

    safe_angle = car_servo_limit_angle(angle);
    g_car_servo_current_angle = safe_angle;
    pwm_set_duty(CAR_SERVO_PWM_PIN, CAR_SERVO_DUTY((safe_angle + 800)));
}
// 舵机回中
void car_servo_set_center(void)
{
    car_servo_set_angle(CAR_SERVO_CENTER_ANGLE);
}

void car_servo_set_limit(uint16 min_angle, uint16 max_angle)
{
    uint16 safe_min = 0;
    uint16 safe_max = 0;

    /* 调参输入先做安全限幅。 */
    safe_min = car_servo_limit_tune_min(min_angle);
    safe_max = car_servo_limit_tune_max(max_angle);
    if(safe_min >= safe_max)
    {
        /* 左右限幅交叉时退回默认值。 */
        safe_min = CAR_SERVO_MIN_ANGLE;
        safe_max = CAR_SERVO_MAX_ANGLE;
    }

    g_car_servo_min_angle = safe_min;
    g_car_servo_max_angle = safe_max;
}

uint16 car_servo_get_min_angle(void)
{
    return g_car_servo_min_angle;
}

uint16 car_servo_get_max_angle(void)
{
    /* 返回当前运行时右限幅。 */
    return g_car_servo_max_angle;
}

uint16 car_servo_get_current_angle(void)
{
    return g_car_servo_current_angle;
}

// 舵机PWM初始化
void car_servo_init(void)
{
    /* 上电先恢复编译期默认限幅。 */
    g_car_servo_min_angle = CAR_SERVO_MIN_ANGLE;
    g_car_servo_max_angle = CAR_SERVO_MAX_ANGLE;
    g_car_servo_current_angle = CAR_SERVO_CENTER_ANGLE;
    // pwm初始化，直接回中
    pwm_init(CAR_SERVO_PWM_PIN, CAR_SERVO_FREQ, CAR_SERVO_DUTY(CAR_SERVO_CENTER_ANGLE));
}


