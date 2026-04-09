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
#include "ackerman.h"
#include "tuning_param.h"

#include "app_key.h"
#include "app_line.h"
#include "app_ui_display.h"
#include "my_delay.h"

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
    other_init(); // 蜂鸣器激光笔、拨码开关
    switch_update(); // 上电时锁存当前拨码模式
    flash_store_init(); // 掉电保存参数初始化

#if IPS_ENABLE
    if(switch_ui_enabled())
    {
        display_init();
    }
#endif
    key_init(); // 按键
    power_adc_init(); // 电池电量ADC
    car_servo_init(); // 舵机初始化并回中
    bldc_motor_init(); // 无刷电机
    car_wheel_init(); // 直流电机
    car_wheel_pid_init(); // 电机pid结构体初始化
    encoder_init(); // 编码器初始化
    key_event_init(); // 按键事件初始化
    ackerman_init(); // 阿克曼运动学初始化
#if WIFI_ENABLE
    if(switch_wifi_enabled())
    {
        tuning_param_boot_init(); // 根据拨码结果决定是否进入WiFi调参模式
    }
#endif
    bldc_motor_stop();
    car_wheel_stop_all();
    car_wheel_set_target(0.0f);
    car_servo_set_center();
    display_menu_init(); // 无论是否开屏，都先恢复 Start 配置

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
    if(switch_wifi_enabled())
    {
        if(tuning_param_start_transport()){
            // 1代表失败
        }
    }
#endif
#if IPS_ENABLE
    if(switch_ui_enabled())
    {
        /* UI 模式下也提前拉起摄像头链路，进入第一页即可直接看图像。 */
        line_app_init();
        display_menu_render();
    }
    else
    {
#else
    {
#endif
        if(!switch_wifi_enabled() || !tuning_param_should_skip_line_init()){
            line_app_init();
        }
    }

    // 陀螺仪初始化并调零
#if IPS_ENABLE
    if(!switch_ui_enabled())
    {
#else
    {
#endif
        if(!imu_init_with_retry()){
            // 初始化成功
            imu_calibrate(100); // 100 次采样计算零偏
        }
    }

    pit_ms_init(TIM2_PIT, TICKS_MS, system_tick_handler);
    // 检查是否为初始化状态（无错误）
    if(g_system_state == SYS_INIT)
    {
        g_system_state = display_menu_start_is_enabled() ? SYS_RUNNING : SYS_PREPARE;
        //g_system_state = SYS_PREPARE;
    }
    while(1)
    {
        // 使用状态机进行管理，不同状态循环不同功能（可扩展）
        if(system_error) g_system_state = SYS_EMERGENCY;

        switch(g_system_state){
            case SYS_PREPARE: // 准备状态
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
                if(g_flag_encoder){
                    /* 准备态不跑后轮闭环，直接丢掉累计采样，避免待处理周期在后台越堆越多。 */
                    g_flag_encoder = 0;
                    encoder_clear();
                }
                if(g_flag_center){
                    // 图像处理
                    g_flag_center = 0;
                    if(switch_ui_enabled())
                    {
                        if(display_menu_in_camera_view())
                        {
                            /* 相机页刷新图像处理结果。 */
                            line_app_process_frame();
                        }
                    }
                    else
                    {
                        if(!switch_wifi_enabled() || !tuning_param_should_pause_line_app()){
                            line_app_process_frame();
                        }
                    }
                }
#if IPS_ENABLE
                if(switch_ui_enabled() && g_flag_display){
                    // 屏幕
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
                if(switch_wifi_enabled() && g_flag_wifi){
                    // WiFi
                    g_flag_wifi = 0;
                    tuning_param_task();
                }
#endif

                break;
            case SYS_RUNNING: // 运行状态
                if(g_flag_encoder){
                    // 编码器
                    g_flag_encoder = 0;
                    encoder_update(1);
                    // 更新后调用PID控制电机速度
                    car_wheel_update();
                    //printf("left %d ; right %d\n",encoder_get_left(),encoder_get_right());
                }
                if(g_flag_center){
                    // 图像处理
                    g_flag_center = 0;
                    if(switch_ui_enabled())
                    {
                        if(display_menu_in_camera_view())
                        {
                            line_app_process_frame();
                        }
                    }
                    else
                    {
                        if(!switch_wifi_enabled() || !tuning_param_should_pause_line_app()){
                            line_app_process_frame();
                        }
                    }
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
#if IPS_ENABLE
                if(switch_ui_enabled() && g_flag_display){
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
                if(switch_wifi_enabled() && g_flag_wifi){
                    g_flag_wifi = 0;
                    tuning_param_task();
                }
#endif
                break;
            case SYS_STOPED:
                break;
            case SYS_EMERGENCY:
                /* 进入急停后持续断开执行器输出，当前重新上电前不再恢复。 */
                bldc_motor_stop();
                car_wheel_control_reset();
                car_servo_set_center();
                g_flag_encoder = 0;
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
