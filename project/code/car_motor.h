#ifndef _car_motor_h_
#define _car_motor_h_

#include "zf_common_headfile.h"

#define CAR_MOTOR_TEST_MAX_DUTY    (20)

typedef enum
{
    RIGHT_MOTOR = 0,
    LEFT_MOTOR,
    CAR_MOTOR_COUNT
} car_motor_index_enum;

void car_motor_init(void);
void car_motor_set_speed(car_motor_index_enum motor, int8 speed_percent);
void car_motor_set_dual(int8 right_motor_speed_percent, int8 left_motor_speed_percent);
void car_motor_stop_all(void);

#endif
