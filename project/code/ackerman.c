/*
 * ackerman.c - 阿克曼几何学运动学算法实现
 *
 * 提供纯运动学解算，不包含PID闭环控制
 * 参考 SmartCCar_CCD_open1 的差速计算实现
 */

#include "ackerman.h"
#include "math.h"

/************ 全局状态 ************/
static ackerman_kinematic_t ackerman_kinematic = {0};

/**
 * @brief 初始化阿克曼运动学状态
 */
void ackerman_init(void)
{
    ackerman_kinematic.steer_angle = 0.0f;
    ackerman_kinematic.speed = 0.0f;
    ackerman_kinematic.left_wheel_speed = 0.0f;
    ackerman_kinematic.right_wheel_speed = 0.0f;
}

/**
 * @brief 设置转向角
 */
void ackerman_set_steer_angle(float steer_angle)
{
    // 限制转向角范围
    if (steer_angle > CAR_MAX_STEER_ANGLE) {
        steer_angle = CAR_MAX_STEER_ANGLE;
    } else if (steer_angle < -CAR_MAX_STEER_ANGLE) {
        steer_angle = -CAR_MAX_STEER_ANGLE;
    }

    ackerman_kinematic.steer_angle = steer_angle;
}

/**
 * @brief 计算左右轮速度
 */
void ackerman_calc_wheel_speeds(float speed, float steer_angle)
{
    float v_dif;
    float tan_delta;

    ackerman_set_steer_angle(steer_angle);
    steer_angle = ackerman_kinematic.steer_angle;

    // 保存当前速度
    ackerman_kinematic.speed = speed;

    // 阿克曼差速因子: v_dif = (tread_width / wheelbase) * tan(δ)
    tan_delta = tan(steer_angle * 3.1415926f / 180.0f);
    v_dif = (CAR_TREAD_WIDTH / CAR_WHEELBASE) * tan_delta;

    // 左右轮速度计算
    // δ > 0 表示右转，此时左后轮应略快、右后轮应略慢
    // v_left  = speed * (1 + v_dif/2)
    // v_right = speed * (1 - v_dif/2)
    ackerman_kinematic.left_wheel_speed = speed * (1.0f + v_dif / 2.0f);
    ackerman_kinematic.right_wheel_speed = speed * (1.0f - v_dif / 2.0f);
}

/**
 * @brief 获取左轮速度
 */
float ackerman_get_left_speed(void)
{
    return ackerman_kinematic.left_wheel_speed;
}

/**
 * @brief 获取右轮速度
 */
float ackerman_get_right_speed(void)
{
    return ackerman_kinematic.right_wheel_speed;
}

/**
 * @brief 获取当前转向角
 */
float ackerman_get_steer_angle(void)
{
    return ackerman_kinematic.steer_angle;
}

/**
 * @brief 获取当前车速
 */
float ackerman_get_speed(void)
{
    return ackerman_kinematic.speed;
}

/**
 * @brief 获取阿克曼运动学状态
 */
const ackerman_kinematic_t* ackerman_get_state(void)
{
    return &ackerman_kinematic;
}
