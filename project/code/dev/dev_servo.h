/*
 * dev_servo.h - 舵机串级PID控制
 *
 * 包含：角度环（外环）+ 角速度环（内环）串级PID
 * 输入：位置误差 -> 输出：舵机角度
 * 说明：调整浮点型为整型，单位为0.01°，扩大100倍
 */

#ifndef __DEV_SERVO_H__
#define __DEV_SERVO_H__

/************ 头文件 ************/
#include "zf_common_typedef.h"
#include "zf_driver_pwm.h"

/*********** 宏定义 *************/
#define CAR_SERVO_PWM_PIN         (PWME_CH3P_PA4)
#define CAR_SERVO_FREQ            (300)
#define CAR_SERVO_MIN_ANGLE       (7000)
#define CAR_SERVO_CENTER_ANGLE    (9000)
#define CAR_SERVO_MAX_ANGLE       (11000)
#define CAR_SERVO_TUNE_MIN_ANGLE  (5000)   /* 当前屏幕调参下限按用户要求放到 50 度。 */
#define CAR_SERVO_TUNE_MAX_ANGLE  (12000)  /* 工程侧推定的屏幕调参上限，避免调参时直接把机构顶到极限。 */

/************ 舵机基础控制函数 ************/
extern void car_servo_init(void);
extern void car_servo_set_angle(uint16 angle);
extern void car_servo_set_center(void);
extern void car_servo_set_limit(uint16 min_angle, uint16 max_angle);
extern uint16 car_servo_get_min_angle(void);
extern uint16 car_servo_get_max_angle(void);
extern uint16 car_servo_get_current_angle(void);

#endif /* __DEV_SERVO_H__ */
