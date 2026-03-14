/*********************************************************************************************************************
* 电机PID控制模块实现
*
* 功能说明：实现编码器-直流电机闭环PID控制
* 参考来源：华北理工镜头组STC全国第一开源5ms
*
* 修改记录
* 日期              备注
* 2026-03-13        创建PID控制模块
********************************************************************************************************************/

#include "motor_pid.h"
#include "encoder.h"
#include "car_motor.h"

// 左右轮PID控制器
motor_pid_t motor_pid_left = {0};
motor_pid_t motor_pid_right = {0};

// PWM限幅
#define MOTOR_PWM_MAX       (9900)
#define MOTOR_PWM_MIN       (-9900)

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     电机PID初始化
// 参数说明     void
// 返回参数     void
// 使用示例     motor_pid_init();
//-------------------------------------------------------------------------------------------------------------------
void motor_pid_init(void)
{
    // 左轮PID参数（需要根据实际调试）
    motor_pid_left.Kp = 7.0f;
    motor_pid_left.Ki = 1.16f;
    motor_pid_left.Kd = 0.0f;
    motor_pid_left.target_speed = 0;
    motor_pid_left.current_speed = 0;
    motor_pid_left.error = 0;
    motor_pid_left.last_error = 0;
    motor_pid_left.pwm_output = 0;

    // 右轮PID参数（需要根据实际调试）
    motor_pid_right.Kp = 7.0f;
    motor_pid_right.Ki = 1.16f;
    motor_pid_right.Kd = 0.0f;
    motor_pid_right.target_speed = 0;
    motor_pid_right.current_speed = 0;
    motor_pid_right.error = 0;
    motor_pid_right.last_error = 0;
    motor_pid_right.pwm_output = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     增量式PI控制器 - 左轮
// 参数说明     encoder: 当前编码器值
//              target: 目标速度
// 返回参数     void
// 使用示例     motor_pid_incremental_pi_left(encoder_value, target_speed);
//-------------------------------------------------------------------------------------------------------------------
static void motor_pid_incremental_pi_left(int16 encoder, int16 target)
{
    // 计算误差
    motor_pid_left.error = target - encoder;

    // 增量式PI算法：Δu = Kp*(e(k)-e(k-1)) + Ki*e(k)
    motor_pid_left.pwm_output += motor_pid_left.Kp * (motor_pid_left.error - motor_pid_left.last_error)
                               + motor_pid_left.Ki * motor_pid_left.error;

    // PWM限幅
    if(motor_pid_left.pwm_output > MOTOR_PWM_MAX)
    {
        motor_pid_left.pwm_output = MOTOR_PWM_MAX;
    }
    else if(motor_pid_left.pwm_output < MOTOR_PWM_MIN)
    {
        motor_pid_left.pwm_output = MOTOR_PWM_MIN;
    }

    // 保存上次误差
    motor_pid_left.last_error = motor_pid_left.error;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     增量式PI控制器 - 右轮
// 参数说明     encoder: 当前编码器值
//              target: 目标速度
// 返回参数     void
// 使用示例     motor_pid_incremental_pi_right(encoder_value, target_speed);
//-------------------------------------------------------------------------------------------------------------------
static void motor_pid_incremental_pi_right(int16 encoder, int16 target)
{
    // 计算误差
    motor_pid_right.error = target - encoder;

    // 增量式PI算法：Δu = Kp*(e(k)-e(k-1)) + Ki*e(k)
    motor_pid_right.pwm_output += motor_pid_right.Kp * (motor_pid_right.error - motor_pid_right.last_error)
                                + motor_pid_right.Ki * motor_pid_right.error;

    // PWM限幅
    if(motor_pid_right.pwm_output > MOTOR_PWM_MAX)
    {
        motor_pid_right.pwm_output = MOTOR_PWM_MAX;
    }
    else if(motor_pid_right.pwm_output < MOTOR_PWM_MIN)
    {
        motor_pid_right.pwm_output = MOTOR_PWM_MIN;
    }

    // 保存上次误差
    motor_pid_right.last_error = motor_pid_right.error;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     电机PWM输出控制 - 左轮
// 参数说明     void
// 返回参数     void
// 使用示例     motor_pwm_output_left();
//-------------------------------------------------------------------------------------------------------------------
static void motor_pwm_output_left(void)
{
    int16 pwm = motor_pid_left.pwm_output;
    int8 speed_percent = 0;

    if(pwm >= 0)
    {
        // 正转
        // speed_percent = (int8)(pwm * 100 / MOTOR_PWM_MAX);          // 逆天，这样写是错的
        speed_percent = (int8)((float)pwm * 100.0f / MOTOR_PWM_MAX);   // 这样才能算出来
        
        // 调试限幅
        if(speed_percent > 40)
        {
            speed_percent = 40;
        }
        car_motor_set_speed(LEFT_MOTOR, speed_percent);
    }
    else
    {
        // 反转
        speed_percent = (int8)((float)(-pwm) * 100.0f / MOTOR_PWM_MAX);
        
        // 调试限幅
        if(speed_percent > 40)
        {
            speed_percent = 40;
        }
        car_motor_set_speed(LEFT_MOTOR, -speed_percent);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     电机PWM输出控制 - 右轮
// 参数说明     void
// 返回参数     void
// 使用示例     motor_pwm_output_right();
//-------------------------------------------------------------------------------------------------------------------
static void motor_pwm_output_right(void)
{
    int16 pwm = motor_pid_right.pwm_output;
    int8 speed_percent = 0;

    if(pwm >= 0)
    {
        // 正转 - 使用浮点计算避免精度丢失
        speed_percent = (int8)((float)pwm * 100.0f / MOTOR_PWM_MAX);

        // 调试限幅
        if(speed_percent > 40)
        {
            speed_percent = 40;
        }

        car_motor_set_speed(RIGHT_MOTOR, speed_percent);
    }
    else
    {
        // 反转 - 使用浮点计算避免精度丢失
        speed_percent = (int8)((float)(-pwm) * 100.0f / MOTOR_PWM_MAX);

        // 调试限幅
        if(speed_percent > 40)
        {
            speed_percent = 40;
        }

        car_motor_set_speed(RIGHT_MOTOR, -speed_percent);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置目标速度
// 参数说明     left_speed: 左轮目标速度（编码器脉冲数/10ms）
//              right_speed: 右轮目标速度（编码器脉冲数/10ms）
// 返回参数     void
// 使用示例     motor_pid_set_speed(50, 50);
//-------------------------------------------------------------------------------------------------------------------
void motor_pid_set_speed(int16 left_speed, int16 right_speed)
{
    motor_pid_left.target_speed = left_speed;
    motor_pid_right.target_speed = right_speed;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     PID更新（在中断中调用）
// 参数说明     void
// 返回参数     void
// 使用示例     motor_pid_update();
//-------------------------------------------------------------------------------------------------------------------
void motor_pid_update(void)
{
    // 获取当前编码器速度
    motor_pid_left.current_speed = encoder_get_left();
    motor_pid_right.current_speed = encoder_get_right();

    // 增量式PI控制
    motor_pid_incremental_pi_left(motor_pid_left.current_speed, motor_pid_left.target_speed);
    motor_pid_incremental_pi_right(motor_pid_right.current_speed, motor_pid_right.target_speed);

    // 输出PWM到电机
    motor_pwm_output_left();
    motor_pwm_output_right();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取左轮PWM值
// 参数说明     void
// 返回参数     int16: 左轮PWM值
// 使用示例     int16 pwm = motor_pid_get_pwm_left();
//-------------------------------------------------------------------------------------------------------------------
int16 motor_pid_get_pwm_left(void)
{
    return motor_pid_left.pwm_output;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取右轮PWM值
// 参数说明     void
// 返回参数     int16: 右轮PWM值
// 使用示例     int16 pwm = motor_pid_get_pwm_right();
//-------------------------------------------------------------------------------------------------------------------
int16 motor_pid_get_pwm_right(void)
{
    return motor_pid_right.pwm_output;
}