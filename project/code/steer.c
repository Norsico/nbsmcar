#include "steer.h"

#include "dev_flash.h"
#include "dev_servo.h"

static uint16 SteerP = 0;
static uint16 SteerD = 0;
static uint16 SteerErr2Tenth = 0;
static float SteerLastError = 0.0f;
int S3010_Duty = steer_middle;

static int Steer_Round_Float(float value)
{
    if(value >= 0.0f)
    {
        return (int)(value + 0.5f);
    }

    return (int)(value - 0.5f);
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
    int command_angle = 0;
    float angle = 0.0f;
    float min_angle = (float)car_servo_get_min_angle();
    float max_angle = (float)car_servo_get_max_angle();
    float center_angle = (float)CAR_SERVO_CENTER_ANGLE;

    LimitLeft(command_duty);
    LimitRight(command_duty);
    S3010_Duty = command_duty;

    if(command_duty >= steer_middle)
    {
        angle = center_angle +
                ((float)command_duty - (float)steer_middle) *
                (max_angle - center_angle) /
                ((float)steer_left - (float)steer_middle);
    }
    else
    {
        angle = center_angle -
                ((float)steer_middle - (float)command_duty) *
                (center_angle - min_angle) /
                ((float)steer_middle - (float)steer_right);
    }

    command_angle = Steer_Round_Float(angle);
    car_servo_set_angle((uint8)command_angle);
}

void SteerPID_Realize(float offset)
{
    float iError = offset;
    float SteerErr = 0.0f;
    float abs_error = 0.0f;
    float err2_gain = 0.0f;
    float dynamic_kp = 0.0f;
    int PWM = 0;

    abs_error = (iError < 0.0f) ? (-iError) : iError;
    err2_gain = (float)SteerErr2Tenth / 10.0f;

    /* 改成动态 Kp 口径：基础 P 之外，再按误差平方抬高 Kp。
     * 额外除以 20 是为了继续沿用当前 err2 k 的调参量级，
     * 避免旧常用值（例如 3）直接把前轮打得过猛。 */
    dynamic_kp = (float)SteerP + (err2_gain * abs_error * abs_error / 20.0f);

    SteerErr = dynamic_kp * iError +
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
