/*
 * ackerman.h - 阿克曼几何学运动学算法
 *
 * 提供纯运动学解算，不包含PID闭环控制
 * 参考 SmartCCar_CCD_open1 的差速计算实现
 *
 * [2026-04-20] 浮点转整型版本
 *   角度单位: 0.01度 (int16, 1 = 0.01°)
 *   速度单位: 0.01m/s (int16, 1 = 0.01m/s)
 */

#ifndef __ACKERMAN_H__
#define __ACKERMAN_H__

/************ 头文件 ************/
#include "zf_common_typedef.h"

/************ 车辆参数（需根据实际车辆测量） ************/
/* 注意: 宏定义值 * 100 为实际物理量 */
/* 例如 CAR_WHEELBASE = 200 表示 200mm */
#define CAR_WHEELBASE        200     // 轴距 (mm)
#define CAR_TREAD_WIDTH      150     // 轮距 (mm)
#define CAR_MAX_STEER_ANGLE  3000    // 最大转向角 (0.01度, 即30度)
#define CAR_WHEEL_RADIUS     30      // 轮子半径 (mm)

/* 内部计算用缩放因子 */
#define ACKerman_K          809      // (TREAD_WIDTH/WHEELBASE)*1000*1000 = 0.8088*1000000

/************ 阿克曼运动学状态结构体 ************/
typedef struct
{
    int16 steer_angle;        // 当前转向角 (0.01度)
    int16 speed;              // 当前基础车速（与后轮闭环目标值同量纲，0.01m/s）
    int16 left_wheel_speed;   // 左轮速度（与输入 speed 同量纲，0.01m/s）
    int16 right_wheel_speed;  // 右轮速度（与输入 speed 同量纲，0.01m/s）
} ackerman_kinematic_t;

/************ 接口函数 ************/

/**
 * @brief 初始化阿克曼运动学状态
 */
extern void ackerman_init(void);

/**
 * @brief 计算左右轮速度
 * @param speed 基础车速（与后轮闭环目标值同量纲，0.01m/s）
 * @param steer_angle 转向角 (0.01度, 左负右正)
 *
 * 阿克曼核心公式:
 *   v_dif = (tread_width / wheelbase) * tan(δ * π/180)
 *   v_left  = speed * (1 + v_dif/2)
 *   v_right = speed * (1 - v_dif/2)
 */
extern void ackerman_calc_wheel_speeds(int16 speed, int16 steer_angle);

/**
 * @brief 获取左轮速度
 * @return 左轮速度（0.01m/s）
 */
extern int16 ackerman_get_left_speed(void);

/**
 * @brief 获取右轮速度
 * @return 右轮速度（0.01m/s）
 */
extern int16 ackerman_get_right_speed(void);

/**
 * @brief 设置转向角
 * @param steer_angle 转向角 (0.01度, 左负右正)
 */
extern void ackerman_set_steer_angle(int16 steer_angle);

/**
 * @brief 获取当前转向角
 * @return 转向角 (0.01度)
 */
extern int16 ackerman_get_steer_angle(void);

/**
 * @brief 获取当前车速
 * @return 当前基础车速（0.01m/s）
 */
extern int16 ackerman_get_speed(void);

/**
 * @brief 获取阿克曼运动学状态
 * @return 指向当前状态的常量指针
 */
extern const ackerman_kinematic_t* ackerman_get_state(void);

#endif /* __ACKERMAN_H__ */
