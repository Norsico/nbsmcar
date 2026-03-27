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
#include "dev_wheel.h"
#include "dev_encoder.h" 
#include "dev_wifi.h"
#include "dev_imu.h"
#include "ackerman.h"

#include "app_key.h"
#include "app_line.h"
#include "app_display.h"
#include "my_delay.h"

/*********** 调试变量 *****************/
const char ssid[] = "xyh";
const char password[] = "1261340160xyh";
const char targetIP[] = "192.168.43.236";


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
    car_wheel_init(); // 直流电机
    car_wheel_pid_init(); // 电机pid结构体初始化
    encoder_init(); // 编码器初始化
    key_event_init(); // 按键事件初始化
    ackerman_init(); // 阿克曼运动学初始化

    if(power_adc_judge()){
        // 低电量报警
        buzzer_on();
        my_delay_s(1);
        buzzer_off();
        // 启用会直接进入紧急状态
        // g_system_state = SYS_EMERGENCY;
    }

    // 调试器件初始化
#if WIFI_ENABLE
    if(wifi_init(ssid,password,targetIP)){
        // 1代表失败
    }
#endif
#if IPS_ENABLE
    display_init();
#endif
    line_app_init();

    // 陀螺仪初始化并调零
    if(!imu_init_with_retry()){
        // 初始化成功
        imu_calibrate(100); // 100 次采样计算零偏
    }

    pit_ms_init(TIM2_PIT, TICKS_MS, system_tick_handler);
    // 检查是否为初始化状态（无错误）
    if(g_system_state == SYS_INIT)
    {
        //g_system_state = SYS_RUNNING;
        g_system_state = SYS_PREPARE;
    }
    car_wheel_set_target(50,0);

    while(1)
    {
        // 使用状态机进行管理，不同状态循环不同功能（可扩展）
        if(system_error) g_system_state = SYS_EMERGENCY;

        switch(g_system_state){
            case SYS_PREPARE: // 准备状态
                g_ips_enable = 1;
                if(g_flag_imu){
                    // IMU
                    g_flag_imu = 0;
                }
                if(g_flag_key){
                    // 按键
                    g_flag_key = 0;
                    key_update(); // 按键
                    key_event_poll();
										switch_update(); // 拨码开关
                }
                if(g_flag_center){
                    // 搜线算法
                    g_flag_center = 0;
                    line_app_process_frame();
                }
#if IPS_ENABLE
                if(g_flag_display){
                    // 屏幕
                    g_flag_display = 0;
                }
#endif
#if WIFI_ENABLE
                if(g_flag_wifi){
                    // WiFi
                    g_flag_wifi = 0;
                }
#endif

                break;
            case SYS_RUNNING: // 运行状态
                g_ips_enable = 0;
                if(g_flag_center){
                    // 搜线算法
                    g_flag_center = 0;
                    line_app_process_frame();
                }
                if(g_flag_encoder){
                    // 编码器
                    g_flag_encoder = 0;
                    encoder_update();
                    // 更新后调用PID控制电机速度
                    car_wheel_update();
                    //printf("left %d ; right %d\n",encoder_get_left(),encoder_get_right());
                }
                if(g_flag_imu){
                    // IMU
                    g_flag_imu = 0;
                }
                if(g_flag_key){
                    // 按键
                    g_flag_key = 0;
                    key_update();
                    key_event_poll();
                }
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
