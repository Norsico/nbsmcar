#include "car_servo.h"

// 前轮舵机默认只允许在 80-100 度之间
#define CAR_SERVO_DUTY(angle)     ((float)PWM_DUTY_MAX / (1000.0 / (float)CAR_SERVO_FREQ) * (0.5 + (float)(angle) / 90.0))

#if ((CAR_SERVO_FREQ < 50) || (CAR_SERVO_FREQ > 300))
    #error "CAR_SERVO_FREQ ERROR!"
#endif

static uint8 car_servo_limit_angle(uint8 angle)
{
    if(angle < CAR_SERVO_MIN_ANGLE)
    {
        return CAR_SERVO_MIN_ANGLE;
    }

    if(angle > CAR_SERVO_MAX_ANGLE)
    {
        return CAR_SERVO_MAX_ANGLE;
    }

    return angle;
}

static uint32 car_servo_angle_to_duty(uint8 angle)
{
    return (uint32)CAR_SERVO_DUTY(angle);
}

void car_servo_init(void)
{
    pwm_init(CAR_SERVO_PWM_PIN, CAR_SERVO_FREQ, car_servo_angle_to_duty(CAR_SERVO_CENTER_ANGLE));
}

void car_servo_set_angle(uint8 angle)
{
    uint8 safe_angle = 0;

    safe_angle = car_servo_limit_angle(angle);
    pwm_set_duty(CAR_SERVO_PWM_PIN, car_servo_angle_to_duty(safe_angle));
}

void car_servo_set_center(void)
{
    car_servo_set_angle(CAR_SERVO_CENTER_ANGLE);
}
