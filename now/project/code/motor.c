#include "headfile.h"

motor_data Motor;

static int16 last_error_left = 0;
static int16 last_error_right = 0;
static volatile uint32 motor_tick_ms = 0;

#define FAN_ESC_DUTY_MIN              (1U * FAN_PWM_FREQ * 10U)
#define FAN_ESC_DUTY_MAX              (2U * FAN_PWM_FREQ * 10U)
#define FAN_ESC_DUTY_STEP             (1U * FAN_PWM_FREQ * 10U / 100U)

static int16 motor_limit(int16 value)
{
    if(value > MOTOR_DUTY_LIMIT)
    {
        return MOTOR_DUTY_LIMIT;
    }
    else if(value < 0)
    {
        return 0;
    }
    else
    {
        return value;
    }
}

static uint32 fan_transform_percent_to_duty(int16 percent)
{
    if(percent <= 0)
    {
        return FAN_ESC_DUTY_MIN;
    }

    if(percent >= 100)
    {
        return FAN_ESC_DUTY_MAX;
    }

    return FAN_ESC_DUTY_MIN + ((uint32)percent * FAN_ESC_DUTY_STEP);
}

static void motor_timer(void)
{
    int16 error_left;
    int16 error_right;
    int16 output_left;
    int16 output_right;

    buzzer_tick();

    Motor.read_left = encoder_get_count(ENCODER_LEFT);
    Motor.read_right = -encoder_get_count(ENCODER_RIGHT);

    encoder_clear_count(ENCODER_LEFT);
    encoder_clear_count(ENCODER_RIGHT);
    motor_tick_ms += MOTOR_CTRL_PERIOD_MS;

    if(CarMode == CAR_MODE_STOP)
    {
        motor_output(0, 0);
        return;
    }

    if(CarMode != CAR_MODE_RUN)
    {
        return;
    }

    error_left = Motor.target_left - Motor.read_left;
    error_right = Motor.target_right - Motor.read_right;

    output_left = Motor.write_left_duty +
                  (SmartCar.motor.left_kp * (error_left - last_error_left) +
                  SmartCar.motor.left_ki * error_left);

    output_right = Motor.write_right_duty +
                   (SmartCar.motor.right_kp * (error_right - last_error_right) +
                   SmartCar.motor.right_ki * error_right);

    last_error_left = error_left;
    last_error_right = error_right;

    motor_output(motor_limit(output_left), motor_limit(output_right));
}

void motor_init(void)
{
    Motor.target_left = 0;
    Motor.target_right = 0;
    Motor.read_left = 0;
    Motor.read_right = 0;
    Motor.write_left_duty = 0;
    Motor.write_right_duty = 0;
    last_error_left = 0;
    last_error_right = 0;

    gpio_init(MOTOR_RIGHT_DIR, GPO, GPIO_LOW, GPO_PUSH_PULL);
    pwm_init(MOTOR_RIGHT_PWM, MOTOR_PWM_FREQ, 0);
    gpio_init(MOTOR_LEFT_DIR, GPO, GPIO_LOW, GPO_PUSH_PULL);
    pwm_init(MOTOR_LEFT_PWM, MOTOR_PWM_FREQ, 0);

    pwm_init(FAN_LEFT_PWM, FAN_PWM_FREQ, fan_transform_percent_to_duty(0));
    pwm_init(FAN_RIGHT_PWM, FAN_PWM_FREQ, fan_transform_percent_to_duty(0));

    encoder_dir_init(ENCODER_LEFT, ENCODER_LEFT_CHA, ENCODER_LEFT_CHB);
    encoder_dir_init(ENCODER_RIGHT, ENCODER_RIGHT_CHA, ENCODER_RIGHT_CHB);

    pit_ms_init(TIM1_PIT, MOTOR_CTRL_PERIOD_MS, motor_timer);
    interrupt_set_priority(TIMER1_IRQn, IRQ_PRIORITY_MOTOR);

    fan_set_duty(0);
}

void motor_output(int16 left_duty, int16 right_duty)
{
    Motor.write_left_duty = left_duty;
    Motor.write_right_duty = right_duty;

    if(left_duty >= 0)
    {
        gpio_set_level(MOTOR_LEFT_DIR, GPIO_HIGH);
        pwm_set_duty(MOTOR_LEFT_PWM, left_duty);
    }
    else
    {
        gpio_set_level(MOTOR_LEFT_DIR, GPIO_LOW);
        pwm_set_duty(MOTOR_LEFT_PWM, -left_duty);
    }

    if(right_duty >= 0)
    {
        gpio_set_level(MOTOR_RIGHT_DIR, GPIO_LOW);
        pwm_set_duty(MOTOR_RIGHT_PWM, right_duty);
    }
    else
    {
        gpio_set_level(MOTOR_RIGHT_DIR, GPIO_HIGH);
        pwm_set_duty(MOTOR_RIGHT_PWM, -right_duty);
    }
}

void fan_set_duty(int16 duty)
{
    uint32 pwm_duty;

    if(duty < 0)
    {
        duty = 0;
    }
    else if(duty > 100)
    {
        duty = 100;
    }

    pwm_duty = fan_transform_percent_to_duty(duty);
    pwm_set_duty(FAN_LEFT_PWM, pwm_duty);
    pwm_set_duty(FAN_RIGHT_PWM, pwm_duty);
}

uint32 motor_get_tick_ms(void)
{
    return motor_tick_ms;
}
