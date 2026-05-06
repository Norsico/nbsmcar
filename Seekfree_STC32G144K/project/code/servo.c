#include "zf_common_headfile.h"
#include "image.h"
#include "servo.h"

#define SERVO_PWM_DUTY(angle)         ((PWM_DUTY_MAX / 1000 * SERVO_PWM_FREQ) / 100 * (50 + (angle) / 90))

static const int16 servo_ackerman_tan_table[61] =
{
    0,10,20,30,30,40,50,60,70,80,90,
    100,110,110,120,130,140,150,160,170,180,
    190,190,200,210,220,230,240,250,260,270,
    280,290,300,310,320,320,330,340,350,360,
    370,380,390,400,410,420,440,450,460,470,
    480,490,500,510,520,530,540,550,570,580
};

static volatile uint8 servo_tick_ready = 0;
static int16 servo_p = 38;                                      /* 舵机p */
static int16 servo_d = 30;                                      /* 舵机d */
static int16 servo_err2_k = 9;                                  /* 二次误差 */
static int16 servo_imu_d = 7;                                   /* 陀螺仪d */
static int16 servo_tow_point = 17;                              /* 前瞻 */
static int16 servo_ackerman = 1285;                             /* 阿克曼 */
static float servo_last_error = 0.0f;                           /* 上次误差 */
static uint16 servo_min_angle = SERVO_ANGLE_MIN;                /* 左限幅 */
static uint16 servo_max_angle = SERVO_ANGLE_MAX;                /* 右限幅 */
static uint16 servo_current_angle = SERVO_ANGLE_CENTER;         /* 当前角度 */
static int16 servo_imu_offset_z = 0;                            /* 陀螺仪零偏 */
static uint8 servo_imu_ready = 0;                               /* 陀螺仪状态 */

/* 舵机定时器 */
static void servo_pit_handler(void)
{
    servo_tick_ready = 1;
}

/* 陀螺仪初始化 */
static uint8 servo_imu_init(void)
{
    uint8 retry;

    retry = 0;
    while(retry < SERVO_IMU_INIT_RETRY_MAX)
    {
        if(0 == imu660ra_init())
        {
            return 1;
        }

        retry++;
        if(retry < SERVO_IMU_INIT_RETRY_MAX)
        {
            system_delay_ms(SERVO_IMU_INIT_DELAY_MS);
        }
    }

    return 0;
}

/* 陀螺仪采样 */
static void servo_imu_update(void)
{
    imu660ra_get_gyro();
}

/* 陀螺仪校准 */
static void servo_imu_calibrate(void)
{
    int32 sum_z;
    uint16 i;

    sum_z = 0;
    for(i = 0; i < SERVO_IMU_CALIBRATE_SAMPLES; i++)
    {
        servo_imu_update();
        sum_z += imu660ra_gyro_z;
        system_delay_ms(5);
    }

    servo_imu_offset_z = (int16)(sum_z / SERVO_IMU_CALIBRATE_SAMPLES);
}

/* 舵机调参左限幅 */
static uint16 servo_limit_tune_min(uint16 angle)
{
    if(angle < SERVO_LIMIT_TUNE_MIN)
    {
        return SERVO_LIMIT_TUNE_MIN;
    }

    if(angle >= SERVO_ANGLE_CENTER)
    {
        return (uint16)(SERVO_ANGLE_CENTER - 100);
    }

    return angle;
}

/* 舵机调参右限幅 */
static uint16 servo_limit_tune_max(uint16 angle)
{
    if(angle > SERVO_LIMIT_TUNE_MAX)
    {
        return SERVO_LIMIT_TUNE_MAX;
    }

    if(angle <= SERVO_ANGLE_CENTER)
    {
        return (uint16)(SERVO_ANGLE_CENTER + 100);
    }

    return angle;
}

/* 舵机角度限幅 */
static uint16 servo_limit_angle(uint16 angle)
{
    if(angle < servo_min_angle)
    {
        return servo_min_angle;
    }

    if(angle > servo_max_angle)
    {
        return servo_max_angle;
    }

    return angle;
}

/* 舵机输出限幅 */
static int16 servo_limit_output_duty(int16 duty)
{
    if(duty > SERVO_STEER_LEFT)
    {
        return SERVO_STEER_LEFT;
    }

    if(duty < SERVO_STEER_RIGHT)
    {
        return SERVO_STEER_RIGHT;
    }

    return duty;
}

/* 舵机四舍五入 */
static int16 servo_round_float(float value)
{
    if(value >= 0.0f)
    {
        return (int16)(value + 0.5f);
    }

    return (int16)(0 - (int16)((-value) + 0.5f));
}

/* 阿克曼转角限幅 */
static int16 servo_limit_ackerman_angle(int16 steer_angle)
{
    if(steer_angle > SERVO_ACKERMAN_MAX_ANGLE)
    {
        return SERVO_ACKERMAN_MAX_ANGLE;
    }

    if(steer_angle < -SERVO_ACKERMAN_MAX_ANGLE)
    {
        return -SERVO_ACKERMAN_MAX_ANGLE;
    }

    return steer_angle;
}

/* 阿克曼查tan */
static int16 servo_get_ackerman_tan(int16 steer_angle)
{
    uint8 idx;
    int16 sign;
    int16 tan_value;
    int16 angle_base;

    if(0 == steer_angle)
    {
        return 0;
    }

    sign = 1;
    if(steer_angle < 0)
    {
        steer_angle = -steer_angle;
        sign = -1;
    }

    if(steer_angle >= SERVO_ACKERMAN_MAX_ANGLE)
    {
        return (int16)(sign * servo_ackerman_tan_table[60]);
    }

    for(idx = 1; idx < 61; idx++)
    {
        if(steer_angle < (int16)(idx * 50))
        {
            angle_base = (int16)((idx - 1) * 50);
            tan_value = servo_ackerman_tan_table[idx - 1] +
                        (int16)(((int32)(steer_angle - angle_base) *
                                 (servo_ackerman_tan_table[idx] - servo_ackerman_tan_table[idx - 1])) / 50);
            return (int16)(sign * tan_value);
        }
    }

    return (int16)(sign * servo_ackerman_tan_table[60]);
}

/* 设置舵机角度 */
static void servo_drive_set_angle(uint16 angle)
{
    uint16 safe_angle;

    safe_angle = servo_limit_angle(angle);
    servo_current_angle = safe_angle;
    pwm_set_duty(SERVO_PWM_PIN, SERVO_PWM_DUTY((safe_angle + 800)));
}

/* 舵机输出映射 */
static void servo_control(int16 duty)
{
    int16 command_duty;
    uint16 command_angle;
    int16 angle_tmp;
    int16 duty_offset;
    int16 duty_span;
    int16 span_work;
    int16 angle_span_gcd;
    int16 angle_span_tenth;
    int16 angle_tenth;

    command_duty = servo_limit_output_duty(duty);

    if(command_duty >= SERVO_STEER_MIDDLE)
    {
        duty_offset = (int16)(command_duty - SERVO_STEER_MIDDLE);
        angle_span_tenth = (int16)((servo_max_angle - SERVO_ANGLE_CENTER) / 10);
        angle_span_gcd = angle_span_tenth;
        span_work = (int16)(SERVO_STEER_LEFT - SERVO_STEER_MIDDLE);

        while(span_work != 0)
        {
            angle_tmp = (int16)(angle_span_gcd % span_work);
            angle_span_gcd = span_work;
            span_work = angle_tmp;
        }

        angle_span_tenth = (int16)(angle_span_tenth / angle_span_gcd);
        duty_span = (int16)((SERVO_STEER_LEFT - SERVO_STEER_MIDDLE) / angle_span_gcd);
        angle_tenth = (int16)((SERVO_ANGLE_CENTER / 10) +
                               (int16)((duty_offset * angle_span_tenth + duty_span / 2) / duty_span));
    }
    else
    {
        duty_offset = (int16)(SERVO_STEER_MIDDLE - command_duty);
        angle_span_tenth = (int16)((SERVO_ANGLE_CENTER - servo_min_angle) / 10);
        angle_span_gcd = angle_span_tenth;
        span_work = (int16)(SERVO_STEER_MIDDLE - SERVO_STEER_RIGHT);

        while(span_work != 0)
        {
            angle_tmp = (int16)(angle_span_gcd % span_work);
            angle_span_gcd = span_work;
            span_work = angle_tmp;
        }

        angle_span_tenth = (int16)(angle_span_tenth / angle_span_gcd);
        duty_span = (int16)((SERVO_STEER_MIDDLE - SERVO_STEER_RIGHT) / angle_span_gcd);
        angle_tenth = (int16)((SERVO_ANGLE_CENTER / 10) -
                               (int16)((duty_offset * angle_span_tenth + duty_span / 2) / duty_span));
    }

    command_angle = (uint16)angle_tenth * 10;
    servo_drive_set_angle(command_angle);
}

/* 舵机控制 */
static void servo_pid_realize(float offset)
{
    float error;
    float abs_error;
    float imu_feedback;
    float control_value;
    float gyro_z;
    int16 output_duty;

    error = offset;
    abs_error = (error < 0.0f) ? (-error) : error;
    gyro_z = 0.0f;
    if(servo_imu_ready)
    {
        gyro_z = (float)(imu660ra_gyro_z - servo_imu_offset_z);
    }
    imu_feedback = -((float)servo_imu_d / 100.0f) * gyro_z;

    control_value = (float)servo_p * error +
                    error * abs_error * ((float)servo_err2_k / 10.0f) +
                    (float)servo_d * (error - servo_last_error) +
                    imu_feedback;

    servo_last_error = error;
    output_duty = servo_round_float((float)SERVO_STEER_MIDDLE - control_value);
    servo_control(output_duty);
}

/* 舵机初始化 */
void servo_init(void)
{
    servo_p = 38;
    servo_d = 30;
    servo_err2_k = 9;
    servo_imu_d = 7;
    servo_tow_point = 17;
    servo_ackerman = 1285;
    servo_last_error = 0.0f;
    servo_min_angle = SERVO_ANGLE_MIN;
    servo_max_angle = SERVO_ANGLE_MAX;
    servo_current_angle = SERVO_ANGLE_CENTER;
    servo_tick_ready = 0;
    servo_imu_offset_z = 0;
    servo_imu_ready = 0;

    pwm_init(SERVO_PWM_PIN, SERVO_PWM_FREQ, SERVO_PWM_DUTY(SERVO_ANGLE_CENTER));
    servo_set_center();
    pit_ms_init(SERVO_CTRL_PIT, SERVO_CTRL_PERIOD_MS, servo_pit_handler);
    interrupt_set_priority(TIMER0_IRQn, SERVO_CTRL_PRIORITY);
    if(servo_imu_init())
    {
        servo_imu_calibrate();
        servo_imu_ready = 1;
        servo_imu_update();
    }
    ImageStatus.TowPoint = (uint8)servo_tow_point;
}

/* 舵机更新 */
void servo_update(void)
{
    uint8 tick_ready;

    if(!servo_tick_ready)
    {
        return;
    }

    interrupt_global_disable();
    tick_ready = servo_tick_ready;
    servo_tick_ready = 0;
    interrupt_global_enable();

    if(!tick_ready)
    {
        return;
    }

    if(servo_imu_ready)
    {
        servo_imu_update();
    }

    if(!image_is_ready() || !image_is_result_ready())
    {
        return;
    }

    servo_pid_realize((float)((int16)ImageStatus.Det_True - ImageSensorMid));
}

/* 舵机回中 */
void servo_set_center(void)
{
    servo_last_error = 0.0f;
    servo_drive_set_angle(SERVO_ANGLE_CENTER);
}

/* 舵机参数 */
void servo_set_pid(int16 steer_p_value, int16 steer_d_value, int16 err2_k_value, int16 imu_d_value)
{
    servo_p = steer_p_value;
    servo_d = steer_d_value;
    servo_err2_k = err2_k_value;
    servo_imu_d = imu_d_value;
}

/* 前瞻 */
void servo_set_tow_point(int16 tow_point)
{
    if(tow_point < 1)
    {
        tow_point = 1;
    }
    else if(tow_point > 49)
    {
        tow_point = 49;
    }

    servo_tow_point = tow_point;
    ImageStatus.TowPoint = (uint8)servo_tow_point;
}

/* 舵机限幅 */
void servo_set_limit(int16 min_angle, int16 max_angle)
{
    uint16 safe_min;
    uint16 safe_max;

    if(min_angle < 0)
    {
        min_angle = 0;
    }
    if(max_angle < 0)
    {
        max_angle = 0;
    }

    safe_min = servo_limit_tune_min((uint16)min_angle * 100);
    safe_max = servo_limit_tune_max((uint16)max_angle * 100);
    if(safe_min >= safe_max)
    {
        safe_min = SERVO_ANGLE_MIN;
        safe_max = SERVO_ANGLE_MAX;
    }

    servo_min_angle = safe_min;
    servo_max_angle = safe_max;
    servo_drive_set_angle(servo_current_angle);
}

/* 阿克曼参数 */
void servo_set_ackerman(int16 ackerman_value)
{
    if(ackerman_value < 0)
    {
        ackerman_value = 0;
    }

    servo_ackerman = ackerman_value;
}

/* 算左右轮目标 */
void servo_calc_motor_target(int16 speed, int16 *left_speed, int16 *right_speed)
{
    int16 steer_angle;
    int16 tan_value;
    int32 diff_scale;

    if((0 == left_speed) || (0 == right_speed))
    {
        return;
    }

    steer_angle = (int16)SERVO_ANGLE_CENTER - (int16)servo_current_angle;
    steer_angle = servo_limit_ackerman_angle(steer_angle);
    tan_value = servo_get_ackerman_tan(steer_angle);
    diff_scale = ((int32)servo_ackerman * (int32)tan_value) / 100;

    *left_speed = speed;
    *right_speed = speed;

    if(steer_angle > 0)
    {
        *right_speed = speed - (int16)(((int32)speed * diff_scale) / 10000);
    }
    else if(steer_angle < 0)
    {
        *left_speed = speed + (int16)(((int32)speed * diff_scale) / 10000);
    }
}
