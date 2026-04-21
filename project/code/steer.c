#include "steer.h"

#include "dev_imu.h"
#include "dev_flash.h"
#include "dev_servo.h"

static uint16 SteerP = 0;
static uint16 SteerD = 0;
static uint16 SteerErr2Tenth = 0;
static uint16 SteerImuDTenth = 0;
static float SteerLastError = 0.0f;
int S3010_Duty = steer_middle;

static int16 Steer_Round_Float(float value)
{
    if(value >= 0.0f)
    {
        return (int16)(value + 0.5f);
    }

    return (int16)(0 - (int16)((-value) + 0.5f));
}

void Steer_init(void)
{
    SteerP = FlashSteerPConfig.default_value;
    SteerD = FlashSteerDConfig.default_value;
    SteerErr2Tenth = FlashSteerErr2Config.default_value;
    SteerImuDTenth = FlashSteerImuDConfig.default_value;
    SteerLastError = 0.0f;
    S3010_Duty = steer_middle;
}

void SteerControl(int duty)
{
    int command_duty = duty;
    uint16 command_angle = 0;
    int16 angle = 0;
    int16 duty_offset = 0;
    int16 duty_span = 0;
    int16 angle_span = 0;
    int16 angle_span_gcd = 0;
    int16 angle_span_tenth = 0;
    int16 angle_tenth = 0;
    uint16 min_angle = car_servo_get_min_angle();
    uint16 max_angle = car_servo_get_max_angle();
    uint16 center_angle = CAR_SERVO_CENTER_ANGLE;
    int16 center_angle_tenth = (int16)(center_angle / 10);

    LimitLeft(command_duty);
    LimitRight(command_duty);
    S3010_Duty = command_duty;

    /* 按 0.1 度做整数映射，避免整度量化后单侧小打角出不来。 */
    if(command_duty >= steer_middle)
    {
        duty_offset = (int16)(command_duty - steer_middle);
        duty_span = (int16)(steer_left - steer_middle);
        angle_span_tenth = (int16)((max_angle - center_angle) / 10);
        angle_span_gcd = angle_span_tenth;
        angle_span = duty_span;

        while(angle_span != 0)
        {
            angle = (int16)(angle_span_gcd % angle_span);
            angle_span_gcd = angle_span;
            angle_span = angle;
        }

        angle_span_tenth = (int16)(angle_span_tenth / angle_span_gcd);
        duty_span = (int16)(duty_span / angle_span_gcd);
        angle_tenth = center_angle_tenth +
                      (int16)((duty_offset * angle_span_tenth + duty_span / 2) /
                              duty_span);
    }
    else
    {
        duty_offset = (int16)(steer_middle - command_duty);
        duty_span = (int16)(steer_middle - steer_right);
        angle_span_tenth = (int16)((center_angle - min_angle) / 10);
        angle_span_gcd = angle_span_tenth;
        angle_span = duty_span;

        while(angle_span != 0)
        {
            angle = (int16)(angle_span_gcd % angle_span);
            angle_span_gcd = angle_span;
            angle_span = angle;
        }

        angle_span_tenth = (int16)(angle_span_tenth / angle_span_gcd);
        duty_span = (int16)(duty_span / angle_span_gcd);
        angle_tenth = center_angle_tenth -
                      (int16)((duty_offset * angle_span_tenth + duty_span / 2) /
                              duty_span);
    }

    command_angle = (uint16)angle_tenth * 10;
    car_servo_set_angle(command_angle);
}

void SteerPID_Realize(float offset)
{
    float iError = offset;
    float SteerErr = 0.0f;
    float abs_error = 0.0f;
    float err2_gain = 0.0f;
    float imu_d_gain = 0.0f;
    float imu_feedback = 0.0f;
    float gyro_z = 0.0f;
    int PWM = 0;

    abs_error = (iError < 0.0f) ? (-iError) : iError;
    err2_gain = (float)SteerErr2Tenth / 10.0f;
    imu_d_gain = (float)SteerImuDTenth / 10.0f;
    gyro_z = (float)(imu_get_gyro_z() - imu_get_offset_z());

    /* z 轴右转为正、左转为负 */
    imu_feedback = -imu_d_gain * gyro_z;

    // 误差
    SteerErr = (float)SteerP * iError +
               iError * abs_error * err2_gain +
               (float)SteerD * (iError - SteerLastError) +
               imu_feedback;  /* 位置式 PID 算式。 */

    SteerLastError = iError;
    PWM = Steer_Round_Float((float)steer_middle - SteerErr);
    SteerControl(PWM);
}

void Steer_Set_Pd(uint16 p, uint16 d, uint16 err2_tenth, uint16 imu_d_tenth)
{
    SteerP = p;
    SteerD = d;
    SteerErr2Tenth = err2_tenth;
    SteerImuDTenth = imu_d_tenth;
}
