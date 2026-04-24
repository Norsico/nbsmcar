/*
 * motor.c
 *
 *  Created on: 2025Äê1ÔÂ18ÈÕ
 *      Author: 15958
 */
#include "motor.h"
#include "zf_common_headfile.h"
int speed_l,speed_r;
void speed_ctr(int left_duty, int right_duty)
{
      small_driver_set_duty(-left_duty, right_duty);
}
/*
void speed_PID_rate(int16 GoalL, int16 GoalR)
{
    Motor_Control.P1 = 1.4;//1.4
    Motor_Control.I1 = 1.05;//1
    Speed_Control.errors_L = (float)(GoalL - Motor_Control.now_speeds_3);
    Motor_Control.pwm_left = GoalL + (Speed_Control.errors_L - Speed_Control.error_pres_L)*Motor_Control.P1 + Speed_Control.errors_L * Motor_Control.I1;
    Speed_Control.error_pres_L = Speed_Control.errors_L;
    Speed_Control.errors_R = (float)(GoalR - Motor_Control.now_speeds_4);
    Motor_Control.pwm_right = GoalR + (Speed_Control.errors_R - Speed_Control.error_pres_R)*Motor_Control.P1 + Speed_Control.errors_R * Motor_Control.I1;
    Speed_Control.error_pres_R = Speed_Control.errors_R;
}
*/




float speed_p=-1.0;
float speed_i=-1.0/200;
float speed_error_l,speed_error_r;
float speed_error_last_l,speed_error_last_r;
int current_speed_l,current_speed_r;
int output_l,output_r;
float inte_l,inte_last_l,inte_r,inte_last_r;

void speed_PID_rate(int goal_l,int goal_r)
{
    int inte_max=1000000;
    current_speed_l=motor_value.receive_left_speed_data;
    speed_error_l=goal_l-5.25*current_speed_l;
    speed_error_l=0.7*speed_error_l+0.3*speed_error_last_l;
    inte_l +=speed_error_l;

if(inte_l>inte_max)
    inte_l=inte_max;
if(inte_l<-inte_max)
    inte_l=-inte_max;

    output_l=speed_error_l*speed_p+speed_i*inte_l;
    speed_error_last_l=speed_error_l;
/*
    if(output_l>3000)
        output_l=3000;
    if(output_l<-3000)
        output_l=-3000;
*/

    current_speed_r=-motor_value.receive_right_speed_data;
    speed_error_r=goal_r-current_speed_r*5.25;
    speed_error_r=0.7*speed_error_r+0.3*speed_error_last_r;
    inte_r+=speed_error_r;
    if(inte_r>inte_max)
        inte_r=0;
    if(inte_r<-inte_max)
        inte_r=0;
    output_r=speed_error_r*speed_p+speed_i*inte_r;
    speed_error_last_r=speed_error_r;
/*    if(output_r>3000)
        output_r=3000;
    if(output_r<-3000)
        output_r=-3000;*/


}

float up_p=70.0;
float up_d=0.2;
float angle_error,angle_error_last;


int up_PID(void)
{
    int output;
    angle_error=-g_eulerAngle.roll;

    output=angle_error*up_p-imu660ra_gyro_x*up_d;
    angle_error_last=angle_error;
/*    if(output>2000)
        output=2000;
    if(output<-2000)
        output=-2000;*/
    return output;

}

