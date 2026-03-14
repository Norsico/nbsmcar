#include "car_motor.h"

#define CAR_MOTOR_PWM_FREQ         (17000)
#define RIGHT_MOTOR_DIR_PIN        (IO_P75)
#define RIGHT_MOTOR_PWM_PIN        (PWMB_CH1_P74)
#define LEFT_MOTOR_DIR_PIN         (IO_P77)
#define LEFT_MOTOR_PWM_PIN         (PWMB_CH3_P76)

static int8 car_motor_limit_speed(int8 speed_percent)
{
    if(speed_percent > 100)
    {
        return 100;
    }

    if(speed_percent < -100)
    {
        return -100;
    }

    return speed_percent;
}

static gpio_pin_enum car_motor_get_dir_pin(car_motor_index_enum motor)
{
    gpio_pin_enum pin = IO_NULL;

    switch(motor)
    {
        case RIGHT_MOTOR: pin = RIGHT_MOTOR_DIR_PIN; break;
        case LEFT_MOTOR: pin = LEFT_MOTOR_DIR_PIN; break;
        default: break;
    }

    return pin;
}

static pwm_channel_enum car_motor_get_pwm_pin(car_motor_index_enum motor)
{
    pwm_channel_enum pin = RIGHT_MOTOR_PWM_PIN;

    switch(motor)
    {
        case RIGHT_MOTOR: pin = RIGHT_MOTOR_PWM_PIN; break;
        case LEFT_MOTOR: pin = LEFT_MOTOR_PWM_PIN; break;
        default: break;
    }

    return pin;
}

void car_motor_init(void)
{
    gpio_init(RIGHT_MOTOR_DIR_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(RIGHT_MOTOR_PWM_PIN, CAR_MOTOR_PWM_FREQ, 0);

    gpio_init(LEFT_MOTOR_DIR_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(LEFT_MOTOR_PWM_PIN, CAR_MOTOR_PWM_FREQ, 0);

    car_motor_stop_all();
}

void car_motor_set_speed(car_motor_index_enum motor, int8 speed_percent)
{
    gpio_pin_enum dir_pin = IO_NULL;
    pwm_channel_enum pwm_pin = RIGHT_MOTOR_PWM_PIN;
    uint32 duty = 0;
    int8 speed = 0;

    if((RIGHT_MOTOR != motor) && (LEFT_MOTOR != motor))
    {
        return;
    }

    dir_pin = car_motor_get_dir_pin(motor);
    pwm_pin = car_motor_get_pwm_pin(motor);
    speed = car_motor_limit_speed(speed_percent);

    if(speed >= 0)
    {
        gpio_set_level(dir_pin, GPIO_HIGH);
        duty = (uint32)speed * (PWM_DUTY_MAX / 100);
    }
    else
    {
        gpio_set_level(dir_pin, GPIO_LOW);
        duty = (uint32)(-speed) * (PWM_DUTY_MAX / 100);
    }

    pwm_set_duty(pwm_pin, duty);
}

void car_motor_set_dual(int8 right_motor_speed_percent, int8 left_motor_speed_percent)
{
    car_motor_set_speed(RIGHT_MOTOR, right_motor_speed_percent);
    car_motor_set_speed(LEFT_MOTOR, left_motor_speed_percent);
}

void car_motor_stop_all(void)
{
    car_motor_set_dual(0, 0);
}
