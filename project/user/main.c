/*********************************************************************************************************************
* STC32G144K Opensourec Library
* Copyright (c) 2025 SEEKFREE
*
********************************************************************************************************************/
#include "zf_common_headfile.h"
#include "system_state.h"
#include "dev_key.h"
#include "dev_other.h"
#include "dev_servo.h"
#include "dev_adc.h"
#include "dev_motor.h"
void main(void)
{
    // 系统初始化
    clock_init(SYSTEM_CLOCK_96M);
    debug_init();
    gpio_init(LED_DEBUG, GPO, GPIO_HIGH, GPO_PUSH_PULL);

    // ----- 安全模式 ------
    if(WDT_FLAG)
    {
        WDT_FLAG = 0;
        while(1)
        {
            gpio_toggle_level(LED_DEBUG);
            system_delay_ms(100);
        }
    }

    // ----- 正常启动 ------
		
    key_init(); // 按键
		other_init(); // 蜂鸣器激光笔
		power_adc_init(); // 电池电量ADC
		car_servo_init(); // 舵机初始化并回中
		bldc_motor_init(); // 无刷电机
		
		
		if(power_adc_judge()){
			// 低电量报警
			buzzer_on();
			system_delay_ms(20);
			buzzer_off();
		}
		
		
    pit_ms_init(TIM2_PIT, TICKS_MS, system_tick_handler);
		// 检查是否为初始化状态（无错误）
    if(g_system_state == SYS_INIT)
    {
        g_system_state = SYS_PREPARE;
    }

    while(1)
    {
			// 使用状态机进行管理，不同状态循环不同功能（可扩展）
        if(system_error) g_system_state = SYS_EMERGENCY;

        switch(g_system_state){
            case SYS_PREPARE:
                if(g_flag_imu){
                    g_flag_imu = 0;
                }
                if(g_flag_key){
                    g_flag_key = 0;
                    key_update();
                }
                if(g_flag_display){
                    g_flag_display = 0;
                }
                if(g_flag_wifi){
                    g_flag_wifi = 0;
                }
                break;
            case SYS_RUNNING:
                break;
            case SYS_STOPED:
                break;
            case SYS_EMERGENCY:
                break;
            default:
                break;
        }
    }
}
