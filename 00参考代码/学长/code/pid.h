/*
 * pid.h
 *
 *  Created on: 2025年1月23日
 *      Author: 15958
 */

#ifndef CODE_PID_H_
#define CODE_PID_H_

#include "zf_common_headfile.h"
extern int SPEED_BASE ;
extern int SPEED;
extern int Target_Speed;
extern int suduhuan_count;
extern int jiaosuduhuan_count;
extern int jiaoduhuan_count;
extern float z_p,z_d,z_i;            //直立环
extern float s_p, s_i;             //速度环
extern float p_p,p_d;                 //转向角度闭环，过单边桥和横断的时候使用。
extern float Turn_p,Turn_d;         //转向环
extern float Turn_p2,Turn_gkd;       //转向环
extern int PWM_MAX;
extern int PWM_MIN;
extern float g_x,g_z;
//extern float ;            //翻滚角速度,偏航角速度
extern float g_x_last;             //上一次俯仰角速度
extern int PWM_out;
extern int py;
extern int errorout;
extern int turn_limit;
//编码器数据变量
extern int Encoder_Left,Encoder_Right;
//闭环控制中间变量
extern int Turn_out,Target_turn,MOTO1,MOTO2;
extern int32 out_speed,out_angle,out_zxangle;//输出：直立环，速度环
extern float Med_Angle;//平衡时角度值偏移量（机械中值）
int Vertical(float Med,float Angle,float gyro_Y);
extern int Err,Err_LowOut,temp;
extern float angle;//陀螺仪的角度
int Velocity(int Target,int encoder_L,int encoder_R);
int Turn(float gyro_Z,int Target_turn);
int zx_Vertical(float Med,float Angle,float gyro_Y);
void Limit(int *motoA,int *motoB);
void Limit_1(int *limit);
int abs(int p);
extern int tracking_way;         //循迹方式，0为摄像头循迹，1为角度环循迹
void Control(void);  //每隔2ms调用一次
#endif /* CODE_PID_H_ */
