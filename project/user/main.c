#include "zf_common_headfile.h"
#include "encoder.h"
#include "car_init.h"


// **************************** 代码区域 ****************************
#define PIT_CH                          (TIM1_PIT )                 // 使用的周期中断编号 如果修改 需要同步对应修改周期中断编号与 isr.c 中的调用
//#define PIT_PRIORITY                    (TIM1_IRQn)               TIM1的中断优先级默认最低，不可修改，具体看手册。


void pit_handler (void);

void main(void)
{
    clock_init(SYSTEM_CLOCK_96M); 				// 时钟配置及系统初始化<务必保留>
    debug_init();                       		// 调试串口信息初始化

    // STC32G144K只有PWM接口支持正交解码编码器.定时器接口不支持正交编码器。

    // 车辆初始化（包含舵机、电机、编码器）
    car_init_no_wifi();                                                     // 不启用WiFi的车辆初始化
    // car_init_with_default_wifi();                                        // 使用默认WiFi配置的车辆初始化
    // car_init_with_wifi("自定义SSID", "自定义密码", "自定义IP");              // 使用自定义WiFi配置的车辆初始化

    
	// 设置100ms的周期定时器
    pit_ms_init(PIT_CH, 100, pit_handler);                                      // 初始化 PIT 为周期中断 100ms 周期

//    interrupt_set_priority(PIT_PRIORITY, 0);                                  // TIM1的中断优先级默认最低，不可修改，具体看手册。
    // 此处编写用户代码 例如外设初始化代码等

    while(1)
    {
        // 此处编写需要循环执行的代码
        printf("encoder_left counter %d .\r\n", encoder_get_left());       // 输出左轮编码器计数信息
        printf("encoder_right counter %d .\r\n", encoder_get_right());     // 输出右轮编码器计数信息
        system_delay_ms(500);
        // 此处编写需要循环执行的代码
    }
}

void pit_handler (void)
{
    encoder_update();                                                       // 更新编码器数据
}
// **************************** 代码区域 ****************************
