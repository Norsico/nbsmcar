#include "headfile.h"

motor_data Motor;

static int16 last_error_left = 0;
static int16 last_error_right = 0;

static int16 motor_limit(int16 value)
{
    if(value > MOTOR_DUTY_LIMIT)
    {
        return MOTOR_DUTY_LIMIT;
    }
    else if(value < -MOTOR_DUTY_LIMIT)
    {
        return -MOTOR_DUTY_LIMIT;
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

    return FAN_ESC_DUTY_MIN + (percent * FAN_ESC_DUTY_STEP);
}

static void fan_write_percent(int16 duty)
{
    int16 pwm_duty;

    pwm_duty = fan_transform_percent_to_duty(duty);
    pwm_set_duty(FAN_LEFT_PWM, pwm_duty);
    pwm_set_duty(FAN_RIGHT_PWM, pwm_duty);
}

static void fan_start_ramp(int16 target_duty)
{
    int16 duty;

    for(duty = 0;
        duty <= target_duty;
        duty += 1)
    {
        fan_write_percent(duty);
        system_delay_ms(30);
    }
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

void motor_update_fan(void)
{
    static int16 last_fan_target = -1;  /* 记录上次的风扇档位 */
    static uint8 change_delay = 0;      /* 档位切换延迟计数器 */
    int16 fan_target;

    if(CarMode != CAR_MODE_RUN)
    {
        return;
    }

    /* 根据直道检测选择风扇档位 */
    if(Image.is_straight && (Image.ring == 0))
    {
        fan_target = SmartCar.motor.fan_straight_duty;  /* 直道：降低下压力 */
    }
    else
    {
        fan_target = SmartCar.motor.fan_duty;  /* 弯道/环岛：正常下压力 */
    }

    /* 档位改变时，延迟3帧再切换（防止抖动） */
    if(fan_target != last_fan_target)
    {
        change_delay++;
        if(change_delay >= 3)  /* 连续3帧确认档位改变 */
        {
            fan_write_percent(fan_target);
            last_fan_target = fan_target;
            change_delay = 0;
        }
    }
    else
    {
        change_delay = 0;  /* 档位相同，清零计数 */
    }
}

void motor_init(void)
{
    Motor.target_left = SmartCar.motor.target_speed;
    Motor.target_right = SmartCar.motor.target_speed;
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

    encoder_dir_init(ENCODER_LEFT, ENCODER_LEFT_CHA, ENCODER_LEFT_CHB);
    encoder_dir_init(ENCODER_RIGHT, ENCODER_RIGHT_CHA, ENCODER_RIGHT_CHB);

    /* 风扇和定时器中断延后启动（motor_start_control()中） */
}

void motor_start_control(void)
{
    /* 风扇初始化 + 爬坡 */
    pwm_init(FAN_LEFT_PWM, FAN_PWM_FREQ, 3000);
    pwm_init(FAN_RIGHT_PWM, FAN_PWM_FREQ, 3000);
    if(CarMode == CAR_MODE_RUN)
    {
        fan_start_ramp(SmartCar.motor.fan_duty);
    }

    /* 启动电机控制中断（所有外设初始化完成后才启动） */
    pit_ms_init(TIM1_PIT, MOTOR_CTRL_PERIOD_MS, motor_timer);
    interrupt_set_priority(TIMER1_IRQn, IRQ_PRIORITY_MOTOR);
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
