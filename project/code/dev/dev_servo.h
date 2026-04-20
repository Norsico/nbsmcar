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
#include "pid_control.h"
#include "zf_driver_pwm.h"

/*********** 宏定义 *************/
#define CAR_SERVO_PWM_PIN         (PWME_CH3P_PA4)
#define CAR_SERVO_FREQ            (300)
#define CAR_SERVO_MIN_ANGLE       (70)
#define CAR_SERVO_CENTER_ANGLE    (90)
#define CAR_SERVO_MAX_ANGLE       (110)
#define CAR_SERVO_TUNE_MIN_ANGLE  (50)   /* 当前屏幕调参下限按用户要求放到 50 度。 */
#define CAR_SERVO_TUNE_MAX_ANGLE  (120)  /* 工程侧推定的屏幕调参上限，避免调参时直接把机构顶到极限。 */

/************ 串级控制器结构体 ************/
typedef struct
{
    // 外环：角度环 (位置误差 -> 期望角速度)
    pid_control_t angle_loop;

    // 内环：角速度环 (期望角速度 - 实测角速度 -> 舵机角度)
    pid_control_t gyro_loop;

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

/************ 舵机基础控制函数 ************/
extern void car_servo_init(void);
extern void car_servo_set_angle(uint8 angle);
extern void car_servo_set_center(void);
extern void car_servo_set_limit(uint8 min_angle, uint8 max_angle);
extern uint8 car_servo_get_min_angle(void);
extern uint8 car_servo_get_max_angle(void);
extern uint8 car_servo_get_current_angle(void);

#endif /* __DEV_SERVO_H__ */
