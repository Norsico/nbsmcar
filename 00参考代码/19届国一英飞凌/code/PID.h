/*
 * PID.h
 *
 *  Created on: 2024年5月5日
 *      Author: Kurumi
 */

#ifndef CODE_PID_H_
#define CODE_PID_H_

#define Limit(x,y)          (x>y? y: (x<-(y)? -(y): x))

#define Duty_max          9999
#define Duty_min         -9999

typedef struct
{
    float target_val;               //目标值
    float actual_val;               //实际值
    float integral;                 //定义积分值

    float err;                      //定义偏差值
    float err_last;                 //  e(k-1)
    float err_previous;             //  e(k-2)

    float Kp;               //定义比例、积分、微分系数
    float Ki;               //定义比例、积分、微分系数
    float Kd;               //定义比例、积分、微分系数
    float limit;            //积分限幅
    float p_result;         //比例、积分、微分运算结果
    float i_result;         //比例、积分、微分运算结果
    float d_result;         //比例、积分、微分运算结果
    float inte_exce;        //积分过度
} _pid;

extern _pid  pid_Speed_r,pid_Speed_l;

extern void PID_Motor(_pid *p,float NowPlace);//电机

#endif /* CODE_PID_H_ */
