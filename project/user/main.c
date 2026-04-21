#include "zf_common_headfile.h"
#include "system_state.h"
#include "dev_key.h"
#include "dev_other.h"
#include "dev_servo.h"
#include "dev_adc.h"
#include "dev_motor.h"
#include "dev_wheel.h"
#include "dev_encoder.h"
#include "dev_imu.h"
#include "dev_flash.h"
#include "dev_wifi.h"
#include "ackerman.h"
#include "tuning_param.h"

#include "app_key.h"
#include "app_line.h"
#include "app_ui_display.h"

/* 主循环入口。 */
void main(void)
{

/********************************************** 初始化开始 ***************************************************/
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

    /* 先拉起基础外设，并锁存当前拨码模式。 */
    other_init();       // 蜂鸣器激光笔、拨码开关
    switch_update();    // 上电时锁存当前拨码模式
    flash_store_init(); // 掉电保存参数初始化

    /* 底层硬件初始化。 */
#if IPS_ENABLE
    // 如果开关打开了UI
    if(switch_ui_enabled())
    {
        // 初始化屏幕
        display_init();
    }
#endif
    key_init();          // 按键
    power_adc_init();    // 电池电量ADC
    car_servo_init();    // 舵机初始化并回中
    bldc_motor_init();   // 无刷电机
    car_wheel_init();    // 直流电机
    car_wheel_pid_init(); // 电机pid结构体初始化
    encoder_init();      // 编码器初始化
    key_event_init();    // 按键事件初始化
    ackerman_init();     // 阿克曼运动学初始化

    /* WiFi 拨码打开时，先准备遥测配置。 */
#if WIFI_ENABLE
    if(switch_wifi_enabled())
    {
        tuning_param_boot_init(); // 根据拨码结果决定是否进入WiFi调参模式
    }
#endif

    /* 上电默认先清执行器输出。 */  
    display_menu_init(); // 屏幕参数初始化，flash初始化取值
    /* 相机链路统一在这里初始化。 */
    line_app_init();

    /* 开屏时补一帧初始界面。 */
#if IPS_ENABLE
    if(switch_ui_enabled())
    {
        display_menu_render();
    }
#endif

    /* WiFi 拨码打开时，继续完成连接。 */
#if WIFI_ENABLE
    if(switch_wifi_enabled())
    {
        if(tuning_param_start_transport()){
            /* WiFi 初始化失败时直接转急停，避免还没连上就进入运行态。 */
            system_error = 1;
            g_system_state = SYS_EMERGENCY;
        }
    }
#endif

    if(power_adc_judge())
    {
        /* 低电量报警。 */
        buzzer_on();
        system_delay_ms(200);
        buzzer_off();
    }

    /* 暂不用陀螺仪，先跳过 IMU 初始化。 */

    /* 最后打开系统节拍。 */
    pit_ms_init(TIM2_PIT, TICKS_MS, system_tick_handler);

/********************************************** 初始化结束 ***************************************************/

    printf("Init OK\n");

    // 检查是否为初始化状态（无错误）
    if(g_system_state == SYS_INIT)
    {
        g_system_state = SYS_RUNNING;
    }


    while(1)
    {
        // 系统错误时进入紧急状态
        if(system_error) g_system_state = SYS_EMERGENCY;

        
        switch(g_system_state){
            
            // 正常运行
            case SYS_RUNNING:
            {
                
                /****************** 预判断开始 ******************/
                if(switch_ui_enabled())
                {
                    // UI打开，电机和风扇停止
                    bldc_motor_stop();
                    car_wheel_stop_all();
                    if(!display_menu_in_camera_view())
                    {
                        car_servo_set_center();
                    }
                }
                else if(switch_wifi_enabled() && !wifi_is_initialized())
                {
                    // WIFI开启，但未连接成功时，不开启风扇、电机，舵机居中
                    bldc_motor_stop();
                    car_wheel_stop_all();
                    car_servo_set_center();
                }
                else
                {
                    // 关屏打开风扇跑
                    bldc_motor_set_duty(20, 20);
                    if(!bldc_motor_is_ready())
                    {
                        /* 负压风扇没起稳前，后轮先待转。 */
                        car_wheel_hold();
                    }
                }
                /****************** 预判断结束 ******************/
                
                if(g_flag_buzzer){
                    /* 蜂鸣器 */
                    g_flag_buzzer = 0;
                    buzzer_task();
                }
                if(g_flag_imu){
                    // 陀螺仪 10ms
                    g_flag_imu = 0;
                }
                if(g_flag_steer){
                    // 舵机控制 10ms
                    g_flag_steer = 0;
                    // 开屏时，仅View页面允许舵机跟随图像输出
                    // 关屏时，WiFi准备好就允许舵机更新
                    if((((switch_ui_enabled()) && display_menu_in_camera_view()) ||
                        ((!switch_ui_enabled()) &&
                         ((!switch_wifi_enabled()) || wifi_is_initialized()))))
                    {
                        // 前轮PD控制
                        line_app_process_steer();
                    }
                }
                if(g_flag_key){
                    // 按键 20ms
                    g_flag_key = 0;
                    key_update();
                    key_event_poll();
                }
                if(g_flag_encoder){
                    // 编码器 5ms
                    g_flag_encoder = 0;
                    if(switch_ui_enabled() ||
                       (switch_wifi_enabled() && !wifi_is_initialized()) ||
                       !bldc_motor_is_ready())
                    {
                        // 屏幕打开 or WiFi没连成功 or 负压未起稳，不开后轮
                        car_wheel_hold();
                    }
                    else
                    {
                        // 更新编码器
                        encoder_update();
                        // 更新电机
                        car_wheel_update();
                    }
                }
                if(g_flag_center){
                    // 图像处理 10ms
                    g_flag_center = 0;
                    if(switch_ui_enabled())
                    {
                        // UI界面View模式，即图像预览要处理图像
                        if(display_menu_in_camera_view())
                        {
                            // 图像处理
                            line_app_process_frame();
                        }
                    }
                    else
                    {
                        // 关屏状态直接处理
                        line_app_process_frame();
                    }
                }
#if IPS_ENABLE
                if(switch_ui_enabled() && g_flag_display){
                    // 屏幕 100ms
                    g_flag_display = 0;
                    if(display_menu_in_camera_view())
                    {
                        line_app_render_frame();
                    }
                    else
                    {
                        display_menu_render();
                    }
                }
#endif
#if WIFI_ENABLE
                if(switch_wifi_enabled()){
                    // WiFi 10ms
                    if(g_flag_wifi)
                    {
                        g_flag_wifi = 0;
                        tuning_param_task();
                    }
                }
#endif
                break;
            }

            // 紧急状态
            case SYS_EMERGENCY:
                // 停风扇
                bldc_motor_stop();
                // 停直流电机
                car_wheel_control_reset();
                // 停舵机
                car_servo_set_center();

                // 清空标志位
                g_flag_buzzer = 0;
                g_flag_encoder = 0;
                g_flag_steer = 0;
                g_flag_center = 0;
#if IPS_ENABLE
                g_flag_display = 0;
#endif
#if WIFI_ENABLE
                g_flag_wifi = 0;
#endif
                break;
            default:
                break;
        }
    }
}
