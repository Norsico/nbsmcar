#ifndef _car_servo_h_
#define _car_servo_h_

#include "zf_common_headfile.h"

#define CAR_SERVO_PWM_PIN         (PWME_CH3P_PA4)
#define CAR_SERVO_FREQ            (50)
#define CAR_SERVO_MIN_ANGLE       (80)
#define CAR_SERVO_CENTER_ANGLE    (90)
#define CAR_SERVO_MAX_ANGLE       (100)

void car_servo_init(void);
void car_servo_set_angle(uint8 angle);
void car_servo_set_center(void);

#endif
