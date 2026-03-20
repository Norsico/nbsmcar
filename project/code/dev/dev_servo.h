/*
 * dev_servo.h - 舵机串级PID控制
 *
 * 包含：角度环（外环）+ 角速度环（内环）串级PID
 * 输入：位置误差 -> 输出：舵机角度
 */

#ifndef __DEV_SERVO_H__
#define __DEV_SERVO_H__

/************ 头文件 ************/
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
} servo_pid_param_t;

/************ PID控制器结构体 ************/
typedef struct
{
    servo_pid_param_t param;    // PID参数
    float target;               // 目标值
    float current;              // 当前值
    float error;                // 当前误差
    float prev_error;           // 上一次误差
    float integral;             // 积分累加
    float output;               // PID输出
    float dt;                    // 采样周期(秒)
} servo_pid_t;

/************ 串级控制器结构体 ************/
typedef struct
{
    // 外环：角度环 (位置误差 -> 期望角速度)
    servo_pid_t angle_loop;

    // 内环：角速度环 (期望角速度 - 实测角速度 -> 舵机角度)
    servo_pid_t gyro_loop;

    // 实测角速度 (由外部输入)
    float gyro_rate;

    // 误差到期望角速度的转换系数 (预留)
    float error_to_rate_ratio;

    // 输出限幅
    float servo_max;            // 舵机最大角度
    float servo_min;            // 舵机最小角度
    float servo_center;         // 舵机中值
} servo_cascade_t;

/************ 接口函数 ************/

/**
 * @brief 初始化串级控制器
 * @param ctrl 控制器结构体指针
 */
extern void servo_cascade_init(servo_cascade_t *ctrl);

/**
 * @brief 配置外环(角度环)PID参数
 * @param ctrl 控制器结构体指针
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 * @param max_out 输出限幅最大值
 * @param min_out 输出限幅最小值
 */
extern void servo_angle_pid_config(servo_cascade_t *ctrl, float kp, float ki, float kd, float max_out, float min_out);

/**
 * @brief 配置内环(角速度环)PID参数
 * @param ctrl 控制器结构体指针
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 * @param max_out 输出限幅最大值
 * @param min_out 输出限幅最小值
 */
extern void servo_gyro_pid_config(servo_cascade_t *ctrl, float kp, float ki, float kd, float max_out, float min_out);

/**
 * @brief 配置误差到期望角速度的转换系数
 * @param ctrl 控制器结构体指针
 * @param ratio 转换系数 (如: 1.0 表示误差1度对应期望角速度1°/s)
 */
extern void servo_set_error_to_rate_ratio(servo_cascade_t *ctrl, float ratio);

/**
 * @brief 配置舵机输出限幅
 * @param ctrl 控制器结构体指针
 * @param max_angle 最大角度
 * @param min_angle 最小角度
 * @param center 中值(零点位置)
 */
extern void servo_set_output_limit(servo_cascade_t *ctrl, float max_angle, float min_angle, float center);

/**
 * @brief 设置采样周期
 * @param ctrl 控制器结构体指针
 * @param dt 采样周期(秒)
 */
extern void servo_set_dt(servo_cascade_t *ctrl, float dt);

/**
 * @brief 更新实测角速度
 * @param ctrl 控制器结构体指针
 * @param gyro_rate 实测角速度(°/s)
 */
extern void servo_update_gyro(servo_cascade_t *ctrl, float gyro_rate);

/**
 * @brief 执行串级PID计算
 * @param ctrl 控制器结构体指针
 * @param error 位置误差(目标位置 - 当前位置)
 * @return 舵机输出角度
 */
extern float servo_cascade_calc(servo_cascade_t *ctrl, float error);

/**
 * @brief 重置控制器状态
 * @param ctrl 控制器结构体指针
 */
extern void servo_cascade_reset(servo_cascade_t *ctrl);

/**
 * @brief 获取当前输出(调试用)
 * @param ctrl 控制器结构体指针
 * @return 当前舵机输出角度
 */
extern float servo_get_output(servo_cascade_t *ctrl);

/**
 * @brief 获取外环目标值(期望角速度)
 * @param ctrl 控制器结构体指针
 * @return 期望角速度(°/s)
 */
extern float servo_get_target_gyro(servo_cascade_t *ctrl);

/**
 * @brief 获取内环误差
 * @param ctrl 控制器结构体指针
 * @return 角速度误差
 */
extern float servo_get_gyro_error(servo_cascade_t *ctrl);

#endif /* __DEV_SERVO_H__ */
