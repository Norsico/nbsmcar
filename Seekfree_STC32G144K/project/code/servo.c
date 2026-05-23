#include "zf_common_headfile.h"
#include "image.h"
#include "servo.h"

static int16 servo_p = 38;                                      /* 舵机p */
static int16 servo_d = 30;                                      /* 舵机d */
static int16 servo_err2_k = 9;                                  /* 二次误差 */
static int16 servo_imu_d = 7;                                   /* 陀螺仪d */
static int16 servo_tow_point = 17;                              /* 前瞻 */
static int16 servo_ackerman = 1285;                             /* 阿克曼 */
static float servo_last_error = 0.0f;                           /* 上次误差 */
static uint16 servo_current_angle = SERVO_ANGLE_CENTER;         /* 当前角度 */
static int16 servo_imu_offset_z = 0;                            /* 陀螺仪零偏 */
static uint8 servo_imu_ready = 0;                               /* 陀螺仪状态 */
static uint32 servo_last_result_sequence = 0;                   /* 上次已消费的图像结果序号 */

/* 0.01° 角度转舵机 PWM，占空比按逐飞示例的连续浮点公式换算。 */
static uint32 servo_angle_to_pwm_duty(uint16 angle)
{
    float duty;

    duty = ((float)PWM_DUTY_MAX / (1000.0f / (float)SERVO_PWM_FREQ)) *
           (0.5f + ((float)angle / 9000.0f));

    return (uint32)(duty + 0.5f);
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

/* 舵机角度限幅 */
static uint16 servo_limit_angle(uint16 angle)
{
    if(angle < SERVO_ANGLE_MIN)
    {
        return SERVO_ANGLE_MIN;
    }

    if(angle > SERVO_ANGLE_MAX)
    {
        return SERVO_ANGLE_MAX;
    }

    return angle;
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
    int16 sign;
    float x_rad;
    float tan_value;

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

    if(steer_angle > SERVO_ACKERMAN_MAX_ANGLE)
    {
        steer_angle = SERVO_ACKERMAN_MAX_ANGLE;
    }

    /* 当前舵机软件限幅只会到约 +/-15°，三阶近似已足够平滑且误差很小。 */
    x_rad = (float)steer_angle * 0.000174532925f;
    tan_value = x_rad + (x_rad * x_rad * x_rad) / 3.0f;

    return (int16)(sign * servo_round_float(tan_value * 1000.0f));
}

/* 设置舵机角度 */
static void servo_drive_set_angle(uint16 angle)
{
    uint16 safe_angle;

    safe_angle = servo_limit_angle(angle);
    servo_current_angle = safe_angle;
    pwm_set_duty(SERVO_PWM_PIN, servo_angle_to_pwm_duty((uint16)(safe_angle - SERVO_MECHANICAL_TRIM)));
}

/* 角度直出，只保留最终限幅。 */
static void servo_control(int16 angle)
{
    if(angle < 0)
    {
        angle = 0;
    }
    servo_drive_set_angle((uint16)angle);
}

/* 舵机控制 */
static void servo_pid_realize(float offset)
{
    float error;
    float abs_error;
    float imu_feedback;
    float control_value;
    float gyro_z;
    int16 output_angle;

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
    output_angle = servo_round_float((float)SERVO_ANGLE_CENTER - control_value);
    servo_control(output_angle);
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
    servo_current_angle = SERVO_ANGLE_CENTER;
    servo_imu_offset_z = 0;
    servo_imu_ready = 0;
    servo_last_result_sequence = 0;

    pwm_init(SERVO_PWM_PIN, SERVO_PWM_FREQ, servo_angle_to_pwm_duty(SERVO_ANGLE_CENTER));
    servo_set_center();
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
    uint32 image_result_sequence;

    if(!image_is_ready() || !image_is_result_ready())
    {
        return;
    }

    image_result_sequence = image_get_result_sequence();
    if(image_result_sequence == servo_last_result_sequence)
    {
        return;
    }

    servo_last_result_sequence = image_result_sequence;

    if(servo_imu_ready)
    {
        servo_imu_update();
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

/* 阿克曼参数 */
void servo_set_ackerman(int16 ackerman_value)
{
    if(ackerman_value < 0)
    {
        ackerman_value = 0;
    }

    servo_ackerman = ackerman_value;
}

/* 当前舵角对应的左右轮差速因子。 */
static int32 servo_get_runtime_diff_scale(int16 *steer_angle_out)
{
    int16 steer_angle;
    int16 tan_value;

    steer_angle = (int16)SERVO_ANGLE_CENTER - (int16)servo_current_angle;
    steer_angle = servo_limit_ackerman_angle(steer_angle);
    if(0 != steer_angle_out)
    {
        *steer_angle_out = steer_angle;
    }

    tan_value = servo_get_ackerman_tan(steer_angle);
    return ((int32)servo_ackerman * (int32)tan_value) / 100;
}

/* 当前基础速度下实际生效的内侧减速量。
 * speed_delta > 0 表示右转，应减右轮；
 * speed_delta < 0 表示左转，应减左轮。 */
int16 servo_get_speed_delta(int16 speed)
{
    int32 diff_scale;

    diff_scale = servo_get_runtime_diff_scale(0);
    return (int16)(((int32)speed * diff_scale) / 10000);
}

/* 算左右轮目标 */
void servo_calc_motor_target(int16 speed, int16 *left_speed, int16 *right_speed)
{
    int16 speed_delta;

    if((0 == left_speed) || (0 == right_speed))
    {
        return;
    }

    *left_speed = speed;
    *right_speed = speed;
    speed_delta = servo_get_speed_delta(speed);

    /* 单减差速：
     * 左转时 speed_delta < 0，降低左轮；
     * 右转时 speed_delta > 0，降低右轮。 */
    if(speed_delta < 0)
    {
        *left_speed = (int16)(speed + speed_delta);
    }
    else if(speed_delta > 0)
    {
        *right_speed = (int16)(speed - speed_delta);
    }

    if(*left_speed < 0)
    {
        *left_speed = 0;
    }

    if(*right_speed < 0)
    {
        *right_speed = 0;
    }
}

/* 当前实际生效的差速因子。 */
int32 servo_get_diff_scale(void)
{
    return servo_get_runtime_diff_scale(0);
}
