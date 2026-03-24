/*
 * ackerman.h - 阿克曼几何学运动学算法
 *
 * 提供纯运动学解算，不包含PID闭环控制
 * 参考 SmartCCar_CCD_open1 的差速计算实现
 */

#ifndef __ACKERMAN_H__
#define __ACKERMAN_H__

/************ 头文件 ************/
#include "zf_common_typedef.h"

/************ 车辆参数（需根据实际车辆测量） ************/
#define CAR_WHEELBASE        200.0f   // 轴距 (mm)
#define CAR_TREAD_WIDTH      150.0f   // 轮距 (mm)
#define CAR_MAX_STEER_ANGLE  30.0f    // 最大转向角 (度)
#define CAR_WHEEL_RADIUS     30.0f    // 轮子半径 (mm)

/************ 阿克曼运动学状态结构体 ************/
typedef struct
{
    float steer_angle;        // 当前转向角 (度)
    float speed;              // 当前车速 (百分比 0-100)
    float left_wheel_speed;   // 左轮速度 (百分比)
    float right_wheel_speed;  // 右轮速度 (百分比)
} ackerman_kinematic_t;

/************ 接口函数 ************/

/**
 * @brief 初始化阿克曼运动学状态
 */
extern void ackerman_init(void);

/**
 * @brief 计算左右轮速度
 * @param speed 车速 (百分比 0-100)
 * @param steer_angle 转向角 (度, 左负右正)
 *
 * 阿克曼核心公式:
 *   v_dif = (tread_width / wheelbase) * tan(δ * π/180)
 *   v_left  = speed * (1 - v_dif/2)
 *   v_right = speed * (1 + v_dif/2)
 */
extern void ackerman_calc_wheel_speeds(float speed, float steer_angle);

/**
 * @brief 获取左轮速度
 * @return 左轮速度 (百分比)
 */
extern float ackerman_get_left_speed(void);

/**
 * @brief 获取右轮速度
 * @return 右轮速度 (百分比)
 */
extern float ackerman_get_right_speed(void);

/**
 * @brief 设置转向角
 * @param steer_angle 转向角 (度, 左负右正)
 */
extern void ackerman_set_steer_angle(float steer_angle);

/**
 * @brief 获取当前转向角
 * @return 转向角 (度)
 */
extern float ackerman_get_steer_angle(void);

/**
 * @brief 获取当前车速
 * @return 车速 (百分比)
 */
extern float ackerman_get_speed(void);

/**
 * @brief 获取阿克曼运动学状态
 * @return 指向当前状态的常量指针
 */
extern const ackerman_kinematic_t* ackerman_get_state(void);

#endif /* __ACKERMAN_H__ */
