/*
 * motor.h
 *
 *  Created on: 2025Äê1ÔÂ18ÈÕ
 *      Author: 15958
 */

#ifndef CODE_MOTOR_H_
#define CODE_MOTOR_H_

#include "zf_common_headfile.h"

void speed_ctr(int left_duty, int right_duty);
extern int speed_l,speed_r;
extern float up_p;
extern float up_d;
extern float angle_error,angle_error_last;
extern float speed_p;
extern float speed_i;
extern float speed_error_l,speed_error_r;
extern float speed_error_last_l,speed_error_last_r;
extern int current_speed_l,current_speed_r;
extern int output_l,output_r;
extern float inte_l,inte_last_l,inte_r,inte_last_r;
void speed_PID_rate(int goal_l,int goal_r);
int up_PID(void);
#endif /* CODE_MOTOR_H_ */
