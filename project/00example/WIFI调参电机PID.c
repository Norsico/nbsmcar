#include "zf_common_headfile.h"
#include "encoder.h"
#include "car_init.h"
#include "tuning_param.h"
#include "motor_pid.h"


// **************************** 代码区域 ****************************
#define PIT_CH                          (TIM1_PIT )                 // 使用的周期中断编号

uint8 encoder_time = 0;                                              // 编码器控制计数器

void pit_handler(void);                                             // 定时器中断处理函数声明

void main(void)
{
    int16 target_speed = 0;  // 变量定义必须在函数最开头

    clock_init(SYSTEM_CLOCK_96M); 				// 时钟配置及系统初始化<务必保留>
    debug_init();                       		// 调试串口信息初始化

    // 车辆初始化（包含舵机、电机、编码器）并启用 WiFi
    car_init_with_default_wifi();                                        // 使用默认WiFi配置的车辆初始化

    // 设置5ms的周期定时器，用于编码器和电机控制
    pit_ms_init(PIT_CH, 5, pit_handler);

    // 使用高速 WIFI SPI模块时无法使用屏幕（因为引脚有共用）
    // 使用高速 WIFI SPI模块时无法使用屏幕（因为引脚有共用）
    // 使用高速 WIFI SPI模块时无法使用屏幕（因为引脚有共用）

    // 此处编写用户代码 例如外设初始化代码等
    while(1)
    {
        // 更新调参参数（自动解析上位机数据）
        tuning_param_update();

        // ========== PID参数调试 ==========
        // 从上位机获取右轮PID参数
        motor_pid_right.Kp = tuning_param_get(1);  // 通道1：Kp参数
        motor_pid_right.Ki = tuning_param_get(2);  // 通道2：Ki参数

        // 设置右轮目标速度（通道0）
        target_speed = (int16)tuning_param_get(0);
        motor_pid_set_speed(0, target_speed);      // 左轮设为0，右轮设为目标值

        // ========== 示波器发送数据 ==========
        // 通道0：目标速度
        seekfree_assistant_oscilloscope_data.dat[0] = tuning_param_get(0);
        // 通道1：右轮实际速度（编码器值）
        seekfree_assistant_oscilloscope_data.dat[1] = encoder_get_right();
        // 通道2：右轮PWM输出百分比（-100% ~ 100%）
        seekfree_assistant_oscilloscope_data.dat[2] = (int8)((float)motor_pid_right.pwm_output * 100.0f / 9900);
        // (int8)((float)pwm * 100.0f / MOTOR_PWM_MAX)
        // 设置本次需要发送几个通道的数据
        seekfree_assistant_oscilloscope_data.channel_num = 3;

        // 发送3个通道的数据
        seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);

        system_delay_ms(20);
        // 有可能会在逐飞助手软件上看到波形更新不够连续，这是因为使用WIFI有不确定的延迟导致的

        // 此处编写需要循环执行的代码
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     定时器中断处理函数
// 参数说明     void
// 返回参数     void
// 使用示例     pit_handler();
//-------------------------------------------------------------------------------------------------------------------
void pit_handler(void)
{
    encoder_time++;

    if(encoder_time == 2)
    {
        encoder_update();           // 更新编码器数据
        motor_pid_update();         // 更新电机PID控制
        encoder_time = 0;
    }
}
// **************************** 代码区域 ****************************