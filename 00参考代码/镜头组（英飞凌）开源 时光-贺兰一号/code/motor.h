/*
 * motor.h
 *
 *  Created on: 2024年2月27日
 *      Author: xiaoming
 */

#ifndef MOTOR_H_
#define MOTOR_H_
#include "zf_common_headfile.h"
#include "C_H.h"

#define OX  (50/ 3000.0)  //标度变换

typedef struct //PID
{
    long SumError;  //误差累计
    int LastError;    //Error[-1]
    int PrevError;    //Error[-2]
    int LastSpeed;    //Speed[-1]
        float KP;                              //比例常数 Proportional Const
        float KI;                                //积分常数 Integral Const
        float KD;                              //微分常数 Derivative Const
} PID;


//void IncPID_Init(PID *sptr,float *MOTOR_PID);                      //增量式PID参数初始化
//int PID_Realize(PID *sptr, int ActualSpeed, int SetSpeed);   //增量式PID控制
//int PID_Cascade(PID *sprt, int NowPiont, int SetPoint);      //增量式串级PID控制
//void pidinitall(void);
void Control_Speed(void);
//int range_protect(int duty, int min, int max);  //限幅
//void CS_control(void);
void speed_measure(void);
//void MOTOR_Control(void);
void MOTOR_init(void);
void Encoder_init(void);
void Speed_decision(void);
void speed_PIDL(void);
void speed_PIDR(void);
void seepd_dif(void);
void Start_gpio_init(void);
void seepd_dif_two(void);
#define S3010_MID 2660
#define S3010_LEFT 3180                 //车向左转（舵机右打） 大
#define S3010_RIGHT 2140                //车向右转（舵机左打） 小
extern int S3010_Duty;
//extern float circle_in;
//extern float circle_out;
typedef struct {
  float P;
  float I;
  float D;
  int LastError;  // Error[-1]
  int PrevError;  // Error[-2]
  int EC;
  float Kdin;   //入弯D
  float Kdout;  //出弯D
} PID_Datatypedef;

//typedef struct {
//  float nowspeed;     // pulse表示nowspeed
//  int expectspeed;    // speed表示expectspeed
//  int motor_duty;     //电机占空比
//  float Length;       //走过路程
//  int Circle_OUT_th;
//  int MinSpeed;             //最低速度
//  int MaxSpeed;             //最高速度
//  float expect_True_speed;  //实际期望速度
//  int straight_speed;       //直道速度
//} SpeedDatatypedef;
extern float Speed_P_l,Speed_I_l,Speed_D_l;
extern float Speed_P_r,Speed_I_r,Speed_D_r;
extern float Left_Speed_Co, Right_Speed_Co;
extern int run_flag;
extern float Disf;
extern PID_Datatypedef SteerPIDdata;    //舵机的PID参数
#endif /* MOTOR_H_ */
