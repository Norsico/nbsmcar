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
#include "zf_driver_pwm.h"

/*********** 宏定义 *************/
#define CAR_SERVO_PWM_PIN         (PWME_CH3P_PA4)
#define CAR_SERVO_FREQ            (50)
#define CAR_SERVO_MIN_ANGLE       (70)
#define CAR_SERVO_CENTER_ANGLE    (90)
#define CAR_SERVO_MAX_ANGLE       (110)
#define CAR_SERVO_TUNE_MIN_ANGLE  (50)   /* 当前屏幕调参下限按用户要求放到 50 度。 */
#define CAR_SERVO_TUNE_MAX_ANGLE  (120)  /* 工程侧推定的屏幕调参上限，避免调参时直接把机构顶到极限。 */

/************ 舵机基础控制函数 ************/
extern void car_servo_init(void);
extern void car_servo_set_angle(uint8 angle);
extern void car_servo_set_center(void);
extern void car_servo_set_limit(uint8 min_angle, uint8 max_angle);
extern uint8 car_servo_get_min_angle(void);
extern uint8 car_servo_get_max_angle(void);
extern uint8 car_servo_get_current_angle(void);

#endif /* __DEV_SERVO_H__ */
