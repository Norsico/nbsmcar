/*
 * motor.c
 *
 *  Created on: 2024年2月27日
 *      Author: xiaoming
 */

#include "zf_common_headfile.h"
#include "C_H.h"
#include "motor.h"
#include "math.h"
#include "steer.h"
int  Angle;//以PWM差的形式表示打角，>0右转，小于0左转
int rukujishu;
int run_flag = 0;
float Disf = 0;
int   bmq_record=0;//记录编码器
int16 speed1, speed2;
//int Left_Speed;//中间变量
//int Right_Speed;//中间变量
int S3010_Duty;


#define PWM_CH1             ATOM0_CH4_P02_4 //左轮正       PWM2
#define PWM_CH2             ATOM0_CH5_P02_5 //左轮反
#define PWM_CH3             ATOM0_CH6_P02_6 //右轮正
#define PWM_CH4             ATOM0_CH7_P02_7 //右轮反       PWM1
#define DIR_CH1             P02_4                       //GPIO2
#define DIR_CH2             P02_6                       //GPIO1




#define PWM_Bang 9500
#define Divaite_Bang 20

//速度决策2
int16 Speed_Goal;

//左轮
int16 Speed_Goal_l;//目标速度
int16 Speed_Encoder_l;//当前速度
float Speed_P_l,Speed_I_l,Speed_D_l;
int16 Speed_Erro_l;//当前误差
int16 Speed_PID_OUT_l;//占空比输出
int16 Speed_Lasterro_l;//上次误差
int16 Speed_Preverro_l;//上上次误差

//右轮
int16 Speed_Goal_r;//目标速度
int16 Speed_Encoder_r;//当前速度
float Speed_P_r,Speed_I_r,Speed_D_r;
int16 Speed_Erro_r;//当前误差
int16 Speed_PID_OUT_r;//占空比输出
int16 Speed_Lasterro_r;//上次误差
int16 Speed_Preverro_r;//上上次误差


void MOTOR_init()
{
//#define PWM_CH1             ATOM0_CH4_P02_4 //左轮正       PWM2
//#define PWM_CH2             ATOM0_CH5_P02_5 //左轮反
//#define PWM_CH3             ATOM0_CH6_P02_6 //右轮正
//#define PWM_CH4             ATOM0_CH7_P02_7 //右轮反       PWM1
 //   pwm_init(ATOM0_CH0_P21_2, 50, 1000);    //新库
    pwm_init(ATOM0_CH1_P21_3 ,17000,   0);   // 左轮正
  //  pwm_init(ATOM0_CH2_P21_4 ,17000,   0);  // 左轮反
    pwm_init(ATOM0_CH3_P21_5 ,17000,   0);   // 右轮反
    //ATOM0_CH0_P21_2
    //ATOM0_CH1_P21_3
    //ATOM0_CH2_P21_4
    //ATOM0_CH3_P21_5
    gpio_init(P21_2, GPO, 0, GPO_PUSH_PULL);
    gpio_init(P21_4, GPO, 0, GPO_PUSH_PULL);
}

void Encoder_init()
{
    encoder_quad_init(TIM2_ENCODER,TIM2_ENCODER_CH1_P33_7 ,TIM2_ENCODER_CH2_P33_6);//带方向
    encoder_quad_init(TIM4_ENCODER,TIM4_ENCODER_CH1_P02_8,TIM4_ENCODER_CH2_P00_9);
   // encoder_init_dir(TIM1_ENCOEDER, TIM1_CH1_ENCOEDER_E9, TIM1_CH2_ENCOEDER_E11);  //编码器左初始化
    //encoder_init_dir(TIM9_ENCOEDER, TIM9_CH1_ENCOEDER_D9, TIM9_CH2_ENCOEDER_D11);  //编码器右初始化
}


void speed_measure()
{
          speed2 = -encoder_get_count(TIM4_ENCODER);     // 定时器取值
          encoder_clear_count(TIM4_ENCODER);                     // 定时器清空
          speed1 = encoder_get_count(TIM2_ENCODER);     // 定时器取值
          encoder_clear_count(TIM2_ENCODER);                     // 定时器清空


            Control_Speed();              //变速控制
            Speed_Goal_r=Speed_Goal;
            Speed_Goal_l=Speed_Goal;
            speed_PIDL();
            speed_PIDR();

}
void Control_Speed()
{
        Speed_Goal = 180; //150
        if(ImageStatus.Road_type ==LeftCirque
               &&ImageStatus.Road_type ==RightCirque) Speed_Goal =180; //150

        if(SystemData.Stop == 1&& ImageStatus.Road_type !=LeftCirque
                &&ImageStatus.Road_type !=RightCirque)
        {
            Speed_Goal = 0;
        }
}


void Speed_decision()
{
   // Speed_Goal = 75;
    Speed_P_l = 20; //200
    Speed_I_l = 2.5; //40
    Speed_D_l = 0; //5
    Speed_P_r = 20;//200
    Speed_I_r = 2.5; //40
    Speed_D_r = 0;  //5
}

void speed_PIDL(void)
{
    Speed_Encoder_l  = speed1;//编码器采集当前速度
   // gpt12_clear(COUNTER1_TIM);

        Speed_Erro_l     =  Speed_Goal_l - Speed_Encoder_l;   //编码器反馈目标值与当前的差值


    Speed_PID_OUT_l += (Speed_P_l * (Speed_Erro_l - Speed_Lasterro_l)  +
                      Speed_I_l *  Speed_Erro_l  +
                      Speed_D_l * (Speed_Erro_l - 2 * Speed_Lasterro_l + Speed_Preverro_l));
    if(Speed_PID_OUT_l<-9900)
        Speed_PID_OUT_l = -9900;
    if(Speed_PID_OUT_l>9900)
        Speed_PID_OUT_l = 9900;
    Speed_Preverro_l =  Speed_Lasterro_l;
    Speed_Lasterro_l =  Speed_Erro_l;
    if(Speed_PID_OUT_l>=0)
    {
        pwm_set_duty(ATOM0_CH3_P21_5,Speed_PID_OUT_l);
        gpio_set_level(P21_4,0);
    }
    else
    {
        pwm_set_duty(ATOM0_CH3_P21_5,-Speed_PID_OUT_l);
        gpio_set_level(P21_4,1);

    }
}

void speed_PIDR()
{

    Speed_Encoder_r  =  speed2;//编码器采集当前速度
//    gpt12_clear(COUNTER2_TIM);

        Speed_Erro_r     =  Speed_Goal_r - Speed_Encoder_r;   //编码器反馈目标值与当前的差值

    Speed_PID_OUT_r += (Speed_P_r * (Speed_Erro_r - Speed_Lasterro_r)  +
                      Speed_I_r *  Speed_Erro_r  +
                      Speed_D_r * (Speed_Erro_r - 2 * Speed_Lasterro_r + Speed_Preverro_r));
    if(Speed_PID_OUT_r<-9900)
        Speed_PID_OUT_r = -9900;
    if(Speed_PID_OUT_r>9900)
        Speed_PID_OUT_r = 9900;
    Speed_Preverro_r =  Speed_Lasterro_r;
    Speed_Lasterro_r =  Speed_Erro_r;
    if(Speed_PID_OUT_r>=0)
    {
         // gpio_set_level(DIR_CH2,0);
        pwm_set_duty(ATOM0_CH1_P21_3,Speed_PID_OUT_r);
        gpio_set_level(P21_2,0);
    }
    else
    {
        pwm_set_duty(ATOM0_CH1_P21_3,-Speed_PID_OUT_r);
        gpio_set_level(P21_2,1);
    }
}

void Start_gpio_init(void)
{
    gpio_init(P20_7,GPI,0,GPI_PULL_DOWN);
    gpio_init(P20_8, GPO, 0, GPO_PUSH_PULL); // P00_0初始化为GPIO功能、输出模式、输出高电平、推挽输出
}
