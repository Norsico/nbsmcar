#include "zf_common_headfile.h"
#include "motor.h"

motor_data_t motor_data = {0, 0, 0, 0, 0, 0};
static volatile uint8 motor_tick_ready = 0;
static int16 motor_last_error_left = 0;
static int16 motor_last_error_right = 0;

/******************** 左电机PID参数 ****************/
static int16 motor_kp_left = 19;                      /* 左kp */
static int16 motor_ki_left = 4;                       /* 左ki */

/******************** 右电机PID参数 ****************/
static int16 motor_kp_right = 19;                     /* 右kp */
static int16 motor_ki_right = 4;                      /* 右ki */

/******************** 参数倍率 ********************/
static int16 motor_param_div_left = 10;               /* 左倍率 */
static int16 motor_param_div_right = 10;              /* 右倍率 */

/******************** 输出限幅 ********************/
static int8 motor_output_limit_left = 90;             /* 左输出最大百分比 */
static int8 motor_output_limit_right = 90;            /* 右输出最大百分比 */

/* 对称限幅 */
static int16 motor_limit_value(int16 value, int16 limit)
{
    if(value > limit)
    {
        return limit;
    }

    if(value < -limit)
    {
        return -limit;
    }

    return value;
}

/* 百分比输出，输入按-100~100，实际会被左右限幅裁到当前最大值。 */
static void motor_set_output(int8 right_pwm_percent, int8 left_pwm_percent)
{
    uint32 duty = 0;

    if(right_pwm_percent >= 0)
    {
        gpio_set_level(RIGHT_MOTOR_DIR_PIN, GPIO_LOW);
        duty = (uint32)right_pwm_percent * (PWM_DUTY_MAX / 100);
    }
    else
    {
        gpio_set_level(RIGHT_MOTOR_DIR_PIN, GPIO_HIGH);
        duty = (uint32)(-right_pwm_percent) * (PWM_DUTY_MAX / 100);
    }
    pwm_set_duty(RIGHT_MOTOR_PWM_PIN, duty);

    if(left_pwm_percent >= 0)
    {
        gpio_set_level(LEFT_MOTOR_DIR_PIN, GPIO_HIGH);
        duty = (uint32)left_pwm_percent * (PWM_DUTY_MAX / 100);
    }
    else
    {
        gpio_set_level(LEFT_MOTOR_DIR_PIN, GPIO_LOW);
        duty = (uint32)(-left_pwm_percent) * (PWM_DUTY_MAX / 100);
    }
    pwm_set_duty(LEFT_MOTOR_PWM_PIN, duty);
}

/* 电机驱动初始化 */
static void motor_driver_init(void)
{
    gpio_init(RIGHT_MOTOR_DIR_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(RIGHT_MOTOR_PWM_PIN, MOTOR_PWM_FREQ, 0);

    gpio_init(LEFT_MOTOR_DIR_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(LEFT_MOTOR_PWM_PIN, MOTOR_PWM_FREQ, 0);
}

/* 编码器初始化 */
static void motor_encoder_init(void)
{
    encoder_quad_init(MOTOR_ENCODER_LEFT, MOTOR_ENCODER_LEFT_CHA, MOTOR_ENCODER_LEFT_CHB);
    encoder_quad_init(MOTOR_ENCODER_RIGHT, MOTOR_ENCODER_RIGHT_CHA, MOTOR_ENCODER_RIGHT_CHB);
}

/* 编码器采样 */
static void motor_pit_handler(void)
{
    int16 left_raw;
    int16 right_raw;

    /* 读编码器 */
    left_raw = encoder_get_count(MOTOR_ENCODER_LEFT);
    right_raw = -encoder_get_count(MOTOR_ENCODER_RIGHT);

    encoder_clear_count(MOTOR_ENCODER_LEFT);
    encoder_clear_count(MOTOR_ENCODER_RIGHT);

    motor_data.count_left = left_raw;
    motor_data.count_right = right_raw;
    motor_tick_ready = 1;
}

/* 电机初始化 */
void motor_init(void)
{
    motor_driver_init();
    motor_encoder_init();
    motor_set_output(0, 0);
    pit_ms_init(MOTOR_CTRL_PIT, MOTOR_CTRL_PERIOD_MS, motor_pit_handler);
    interrupt_set_priority(TIMER1_IRQn, 2);
}

/* 目标值 */
void motor_set_target(int16 left_target, int16 right_target)
{
    motor_data.target_left = left_target;
    motor_data.target_right = right_target;
}

/* 电机更新 */
void motor_update(void)
{
    int16 left_count;
    int16 right_count;
    int16 left_error;
    int16 right_error;
    int16 left_output;
    int16 right_output;

    // 编码器采样周期没到，直接返回
    if(!motor_tick_ready)
    {
        return;
    }

    /* 锁中断，拷贝一组一致的计数并清标志。 */
    interrupt_global_disable();
    left_count = motor_data.count_left;
    right_count = motor_data.count_right;
    motor_tick_ready = 0;
    interrupt_global_enable();

    if((0 == motor_data.target_left) && (0 == motor_data.target_right))
    {
        /* 目标为0就清状态停轮 */
        motor_last_error_left = 0;
        motor_last_error_right = 0;
        motor_data.pwm_left = 0;
        motor_data.pwm_right = 0;
        motor_set_output(0, 0);
        return;
    }

    /* 当前误差 */
    left_error = motor_data.target_left - left_count;
    right_error = motor_data.target_right - right_count;

    /* 增量PI */
    left_output = motor_data.pwm_left +
                  (int16)((motor_kp_left * (left_error - motor_last_error_left) + motor_ki_left * left_error) / motor_param_div_left);
    right_output = motor_data.pwm_right +
                   (int16)((motor_kp_right * (right_error - motor_last_error_right) + motor_ki_right * right_error) / motor_param_div_right);

    /* 输出限幅 */
    motor_data.pwm_left = (int8)motor_limit_value(left_output, motor_output_limit_left);
    motor_data.pwm_right = (int8)motor_limit_value(right_output, motor_output_limit_right);

    /* 记录上次误差 */
    motor_last_error_left = left_error;
    motor_last_error_right = right_error;

    /* 写PWM */
    motor_set_output(motor_data.pwm_right, motor_data.pwm_left);
}
