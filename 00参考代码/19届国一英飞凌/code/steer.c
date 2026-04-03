/*
 * steer.c
 *
 *  Created on: 2024年3月1日
 *      Author: Kurumi
 */

#include <image.h>
#include "steer.h"
#include "Fuzzy.h"
#include "motor.h"
//int steer_middle;                //舵机中值
//int steer_left;                  //左转
//int steer_right;                 //右转
PID_Datatypedef SteerPIDdata;    //舵机的PID参数
PID_Datatypedef SteerPIDdata_2;  //动态二次
Piord_PD Po_PID;
extern int16 Speed_Goal;
extern int16 Speed_Goal_l;//目标速度
extern int16 Speed_Goal_r;//目标速度
//ImageStatustypedef ImageStatus;
extern SystemDatatypdef SystemData;


int PWM_;

extern uint8 keep_rings;//圆环保持标志位
void Steer_init(void) {
    pwm_init(ATOM1_CH1_P33_9,S3010_HZ,steer_middle);
}

void SteerControl(uint32 duty)
{
  LimitLeft(duty);
  LimitRight(duty);  //限幅

//  duty = 700;

  pwm_set_duty(ATOM1_CH1_P33_9, duty);
}

    float iError,
      SteerErr,
      SteerErr_pre;
  int PWM;







void SteerPID_Realize(float offset)    //单PD控制
{

  iError = offset;  //计算当前误差
if(ImageStatus.Road_type != RightCirque&&ImageStatus.Road_type != LeftCirque)    //直道舵机PD
{
  SteerErr =
       SteerPIDdata.P * iError +
       SteerPIDdata.D * (iError - SteerPIDdata.LastError);  //位置式PID算式  单PD
  if(abs(iError)<3)
      iError = 0.3*iError;
//  if(abs(iError)>3 && abs(iError)<10)
//        iError = 15*iError;
  if(abs(iError)>15)
        iError = 1.2*iError;
//  if(abs(iError)>30)
//          iError = 0;
}
else if(ImageStatus.Road_type == RightCirque||ImageStatus.Road_type == LeftCirque)   //环岛舵机PD
{
  SteerErr =
       SteerPIDdata.L_P * iError +
       SteerPIDdata.L_D * (iError - SteerPIDdata.LastError);  //位置式PID算式  单PD
}
  SteerPIDdata.LastError = iError;  //更新上次误差
  SteerErr_pre = SteerErr;   //

  PWM = steer_middle - SteerErr;  //-+              偏差 = 舵机中值 - 位置PD输出结果
  S3010_Duty=PWM;
//  S3010_Duty = filter_l(S3010_Duty);//不知道有没有用的滤波
//   PWM = 4800 ;     //舵机中值  730  820 4380
//  if(keep_rings==1)    //固定舵机值
//  {
//        PWM_++;
//        PWM = 5100;
//        if(PWM_>50)
//        {
//            PWM_ = 0;
//            keep_rings=0;
//        }
//  }

  SteerControl(PWM);

}


