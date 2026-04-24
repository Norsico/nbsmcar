/*
 * servo.h
 *
 *  Created on: 2025年1月20日
 *      Author: 15958
 */

#ifndef CODE_SERVO_H_
#define CODE_SERVO_H_
#include "zf_common_headfile.h"
#include "Platform_Types.h"                  //不加这个头文件没法使用外部声明extern 会导致识别不到
//左前   ATOM1_CH0_P21_2    170伸腿,70缩腿
//右前   ATOM1_CH2_P21_4    10伸腿,110缩腿
//左后   ATOM1_CH1_P21_3    10伸腿,110缩腿
//右后   ATOM1_CH3_P21_5    170伸腿,70缩腿
extern int  height_out,height_outl,height_outr,fuzhu_Vertical_out;      //翻滚角输出
extern int fuzhuhuan_count;
extern int g_y;              //陀螺仪翻滚角速度
extern float sv_p,sv_i;         //舵机角度环
extern int16 pwm_ph1, pwm_ph4;
extern int high_out_max;      //小车过单边桥的输出
extern float sr_p,sr_i;
extern float fz_p,fz_i;
extern float fz_p_base;
extern float fz_p_high;
extern float fzz_p,fzz_d;
extern float zx_p;
extern int compensation_out;//补偿输出
extern float zhuanxiang_out,zhuanxiang_l,zhuanxiang_r;
extern float high_out;        //小车高度
extern int high_mode;         //小车高度模式，0为低模式，1为高模式
extern float fuzhu_out;       //辅助环输出
extern float LF_SPEED_ANGLE,LB_SPEED_ANGLE,RF_SPEED_ANGLE,RB_SPEED_ANGLE;
extern float LF_SPEED;
extern float LB_SPEED;
extern float RF_SPEED;
extern float RB_SPEED;
extern int jump_flag,jump_cnt,jump_flag2,jump_flag3;
//////////左后
#define LB_SERVO_MOTOR_PWM             (ATOM1_CH1_P21_3)                           // 定义主板上舵机对应引脚
#define LB_SERVO_MOTOR_FREQ            (50 )                                       // 定义主板上舵机频率  请务必注意范围 50-300

#define LB_SERVO_MOTOR_H_MAX           (10 )                                       // 定义主板上舵机活动范围 角度
#define LB_SERVO_MOTOR_L_MAX           (110 )                                       // 定义主板上舵机活动范围 角度
#define LB_SERVO_MOTOR_DUTY(x)         ((float)PWM_DUTY_MAX/(1000.0/(float)LB_SERVO_MOTOR_FREQ)*(0.5+(float)(x)/90.0))
/////////右后
#define RB_SERVO_MOTOR_PWM             (ATOM1_CH3_P21_5)                           // 定义主板上舵机对应引脚
#define RB_SERVO_MOTOR_FREQ            (50 )                                       // 定义主板上舵机频率  请务必注意范围 50-300

#define RB_SERVO_MOTOR_H_MAX           (170 )                                       // 定义主板上舵机活动范围 角度
#define RB_SERVO_MOTOR_L_MAX           (300 )                                       // 定义主板上舵机活动范围 角度
#define RB_SERVO_MOTOR_DUTY(x)         ((float)PWM_DUTY_MAX/(1000.0/(float)RB_SERVO_MOTOR_FREQ)*(0.5+(float)(x)/90.0))
////////左前
#define LF_SERVO_MOTOR_PWM             (ATOM1_CH0_P21_2)                           // 定义主板上舵机对应引脚
#define LF_SERVO_MOTOR_FREQ            (50 )                                       // 定义主板上舵机频率  请务必注意范围 50-300

#define LF_SERVO_MOTOR_H_MAX           (170 )                                       // 定义主板上舵机活动范围 角度
#define LF_SERVO_MOTOR_L_MAX           (70 )                                       // 定义主板上舵机活动范围 角度
#define LF_SERVO_MOTOR_DUTY(x)         ((float)PWM_DUTY_MAX/(1000.0/(float)LF_SERVO_MOTOR_FREQ)*(0.5+(float)(x)/90.0))

///////
#define RF_SERVO_MOTOR_PWM             (ATOM1_CH2_P21_4)                           // 定义主板上舵机对应引脚
#define RF_SERVO_MOTOR_FREQ            (50 )                                       // 定义主板上舵机频率  请务必注意范围 50-300

#define RF_SERVO_MOTOR_H_MAX           (10 )                                       // 定义主板上舵机活动范围 角度
#define RF_SERVO_MOTOR_L_MAX           (110 )                                       // 定义主板上舵机活动范围 角度
#define RF_SERVO_MOTOR_DUTY(x)         ((float)PWM_DUTY_MAX/(1000.0/(float)RF_SERVO_MOTOR_FREQ)*(0.5+(float)(x)/90.0))

#define SERVO_MOTOR_DUTY(x)         ((float)PWM_DUTY_MAX/(1000.0/(float)SERVO_MOTOR_FREQ)*(0.5+(float)(x)/90.0))
void LB_SERVO_ANGLE(float angle);
void LF_SERVO_ANGLE(float angle);
void RF_SERVO_ANGLE(float angle);
void RB_SERVO_ANGLE(float angle);
void ALL_SERVO_ANGLE(float lbangle,float lfangle,float rbangle,float rfangle);
void servo_init(void);
int servo_Vertical(float Med,float Angle,float gyro_Y);
int servo_roll_compensation(float angle,float gyro_x);
void servo_speed(void);//舵机速度
void jump(void);
void control_s(void) ;    //舵机控制,两毫秒中断
#endif /* CODE_SERVO_H_ */
