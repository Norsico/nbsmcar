#include "steer.h"

#include "dev_flash.h"
#include "dev_servo.h"

static uint16 SteerP = 0;
static uint16 SteerD = 0;
static uint16 SteerErr2Tenth = 0;
static float SteerLastError = 0.0f;
int S3010_Duty = steer_middle;

static uint16 Steer_Round_Float(uint16 value)
{
    if(value >= 0)
    {
        return (value + 50);
    }

    return (value - 50);
}

void Steer_init(void)
{
    SteerP = FlashSteerPConfig.default_value;
    SteerD = FlashSteerDConfig.default_value;
    SteerErr2Tenth = FlashSteerErr2Config.default_value;
    SteerLastError = 0.0f;
    S3010_Duty = steer_middle;
}

void SteerControl(int duty)
{
    int command_duty = duty;
    uint16 command_angle = 0;
    uint16 angle = 0;
    uint16 min_angle = car_servo_get_min_angle();
    uint16 max_angle = car_servo_get_max_angle();
    uint16 center_angle = CAR_SERVO_CENTER_ANGLE;

    LimitLeft(command_duty);
    LimitRight(command_duty);
    S3010_Duty = command_duty;
 
    if(command_duty >= steer_middle)
    {
        angle = center_angle +
                (command_duty - steer_middle) *
                (max_angle - center_angle) /
                (steer_left - steer_middle);
    }
    else
    {
        angle = center_angle -
                (steer_middle - command_duty) *
                (center_angle - min_angle) /
                (steer_middle - steer_right);
    }

    command_angle = Steer_Round_Float(angle);
    car_servo_set_angle(command_angle);
}

void SteerPID_Realize(float offset)
{
    float iError = offset;
    float SteerErr = 0.0f;
    float abs_error = 0.0f;
    float err2_gain = 0.0f;
    int PWM = 0;

    abs_error = (iError < 0.0f) ? (-iError) : iError;
    if(abs_error < 3.0f)
    {
        iError = 0.3f * iError;
    }
    if(abs_error > 15.0f)
    {
        iError = 1.2f * iError;
    }

    abs_error = (iError < 0.0f) ? (-iError) : iError;
    err2_gain = (float)SteerErr2Tenth / 10.0f;

    /* 大小误差缩放要先完成，再参与当前拍 P/D 计算。 */
    SteerErr = (float)SteerP * iError +
               err2_gain * iError * abs_error +
               (float)SteerD * (iError - SteerLastError);  /* 位置式 PID 算式。 */

    SteerLastError = iError;
    PWM = Steer_Round_Float((float)steer_middle - SteerErr);
    SteerControl(PWM);
}

void Steer_Set_Pd(uint16 p, uint16 d, uint16 err2_tenth)
{
    SteerP = p;
    SteerD = d;
    SteerErr2Tenth = err2_tenth;
}
