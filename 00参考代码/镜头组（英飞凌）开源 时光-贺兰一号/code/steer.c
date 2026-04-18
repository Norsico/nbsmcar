/*
 * steer.c
 *
 *  Created on: 2024年3月2日
 *      Author: xiaoming
 */
#include "steer.h"
#include "Fuzzy.h"
#include "C_H.h"
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
//extern int rukubz;


void Steer_init(void) {
   pwm_init(ATOM1_CH1_P33_9,200,steer_middle);
  //gtm_pwm_init(ATOM0_CH1_P33_9, S3010_HZ, 0);
}

void SteerControl(uint32 duty) {
  LimitLeft(duty);
  LimitRight(duty);  //限幅

  pwm_set_duty(ATOM1_CH1_P33_9, duty);
}

    float iError,    //当前误差   原来是int类型 5.23更改为float
      SteerErr,
      SteerErr_pre;
  int PWM;
void SteerPID_Realize_5(float offset)  //位置式    //模糊的偏差维度参数发生变化
{
  //当前误差，定义为寄存器变量，只能用于整型和字符型变量，提高运算速度
  iError = offset;  //计算当前误差
  SteerErr =
          SteerPIDdata.P * iError +
       SteerPIDdata.D * (iError - SteerPIDdata.LastError);  //位置式PID算式  单PD
  SteerPIDdata.LastError = iError;  //更新上次误差
  SteerErr_pre = SteerErr;

  PWM = steer_middle - SteerErr;  //-+
  S3010_Duty = PWM;
  SteerControl(PWM);
}

void SteerPID_Realize(float offset) {
  //当前误差，定义为寄存器变量，只能用于整型和字符型变量，提高运算速度

  float iError,  //当前误差
      SteerErr;  //
  int PWM;
  float kp_two = 0.3;
  //  static int err_sum = 0;
  iError = offset;  //计算当前误差
  SteerErr =
      SteerPIDdata.P * iError +
      iError*abs(iError)*kp_two+(iError - SteerPIDdata.LastError)*SteerPIDdata.D;  //位置式PID算式
  SteerPIDdata.LastError = iError;                         //更新上次误差
  PWM = steer_middle - SteerErr;

  SteerControl(PWM);
}


float Steer_pl;
float Steer_ph;
float Steer_d;
int critic_offset = 7;

void SteerPID_Realize_2(int offset) {
  //定义为寄存器变量，只能用于整型和字符型变量，提高运算速度
  int32 iError, SteerErr;  //当前误差
  static int32 LastError;  //前次误差
  int PWM;
  float Kp;  //动态P
  float Kd;
 // int postive_offset;
  iError = offset;  //计算当前误差
//  postive_offset = abs(offset);
  Kp = 0.1 * (iError * iError) / (Steer_pl) + (Steer_ph);   // P值与差值成二次函数关系

  //    //试试动态D
  //    Kd = (iError * LastError > 0) ?
  //            3+Steer_d*postive_offset/20:        // 入弯
  //            4+Steer_d*postive_offset/20;        // 出弯

  // if(postive_offset<=8)
  //    Kd=Steer_d-5;
  // else if(postive_offset<=12)
  //    Kd=Steer_d-4;
  // else if(postive_offset<=16)
  //    Kd=Steer_d-3;
  // else if(postive_offset<=20)
  //    Kd=Steer_d-2;
  // else if(postive_offset<=24)
  //    Kd=Steer_d-1;
  // else
  Kd = Steer_d;

  SteerErr = Kp * iError + (Kd) * (iError - LastError) + 0.5;  //只用PD
  SteerPIDdata.EC = iError - LastError;  // EC>0 右转  EC<左转
  LastError = iError;                    //更新上次误差
  Speed_Goal_l=Speed_Goal+SteerErr;   //speedgoal 和 steererr 中间的符号上下两个可以改，如果转弯差速反了记得改一下
  Speed_Goal_r=Speed_Goal-SteerErr;

          PWM=0;
      if(SystemData.Point>70) { PWM=steer_right;}
      if(SystemData.Point<10) { PWM=steer_left;}

//  if (PWM < 0)
//    PWM = 0;
//  LimitLeft(PWM);
//  LimitRight(PWM);  //限幅

  SteerControl(PWM);
}
