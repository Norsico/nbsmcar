#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "headfile.h"

typedef struct
{
    int16 target_left;
    int16 target_right;
    volatile int16 read_left;
    volatile int16 read_right;
    int16 write_left_duty;
    int16 write_right_duty;
} motor_data;

extern motor_data Motor;

void motor_init(void);
void motor_start_control(void);
uint8 motor_is_straight_enabled(void);
void motor_update_fan(void);
void motor_output(int16 left_duty, int16 right_duty);
void fan_set_duty(int16 duty);

#endif
