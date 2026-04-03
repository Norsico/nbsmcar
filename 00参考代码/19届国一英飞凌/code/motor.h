/*
 * motor.h
 *
 *  Created on: 2024年3月1日
 *      Author: Kurumi
 */





#ifndef MOTOR_H_
#define MOTOR_H_

#define OX  (50/ 3000.0)  //标度变换


//uint16 fan_pwm_Straight;      //负压风扇基础pwm

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

void Control_Speed(void);
void fan_speedcontrl(void);
void speed_measure(void);
void MOTOR_init(void);
void Encoder_init(void);
void Speed_decision(void);
void speed_PIDL(void);

void key_readd (void);   //读取按键状态
void key_Start_run(void);//按键发车
void gyro_data(void); //陀螺仪数据
void seepd_dif(void);
void seepd_dif_two(void);

extern float Speed_P_l,Speed_I_l,Speed_D_l;

#define S3010_MID 2660
#define S3010_LEFT 3180                 //车向左转（舵机右打） 大
#define S3010_RIGHT 2140                //车向右转（舵机左打） 小
extern int S3010_Duty;

typedef struct {
  float P;
  float I;
  float D;
  float L_P;
  float L_D;
  int LastError;  // Error[-1]
  int PrevError;  // Error[-2]
  int EC;
  float Kdin;   //入弯D
  float Kdout;  //出弯D

} PID_Datatypedef;

typedef struct {
  float nowspeed;     // pulse表示nowspeed
  int expectspeed;    // speed表示expectspeed
  int motor_duty;     //电机占空比
  float Length;       //走过路程
  int Circle_OUT_th;
  int MinSpeed;             //最低速度
  int MaxSpeed;             //最高速度
  float expect_True_speed;  //实际期望速度
  int straight_speed;       //直道速度
} SpeedDatatypedef;



extern int R_MT_PWM_Ctrl;
extern int L_MT_PWM_Ctrl;


extern float fc; //截止频率
extern float Ts; //采样周期
extern float pi; //π
extern float alpha;     //滤波系数

extern float Left_Speed_Co, Right_Speed_Co;
extern int speeddd;
extern float  Dif_spd_rat;
extern float  tffkz;
/************************ 滤波器初始化 alpha *****************************/
void low_pass_filter_init(void);
float filter_r(float value);
float filter_l(float value);


void Dif_spd_calc(int tar_speeed,float* L_speed,float* R_speed);
void PID_play();
void fan_init();   //风扇初始化

#endif /* MOTOR_H_ */

