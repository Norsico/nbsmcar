#include "system_state.h"
#include "dev_encoder.h"
#include "dev_wifi.h"
#include "dev_wheel.h"
#include "dev_other.h"

/************ 全局变量定义 ************/
volatile system_state_t g_system_state = SYS_INIT;  // 系统状态：INIT/RUNNING/STOPED/EMERGENCY
uint8 system_error = 0;                               // 系统错误标志：非0表示发生错误，进入紧急状态

vuint32 g_system_ticks = 0;                           // 系统Tick计数器，每1ms加1
vuint32 g_key_ticks = 0;                              // 按键扫描计时器
vuint32 g_buzzer_ticks = 0;                           // 蜂鸣器计时器
vuint32 g_imu_ticks = 0;                              // IMU读取计时器
vuint32 g_steer_ticks = 0;                            // 舵机控制计时器
vuint32 g_encoder_ticks = 0;													// 编码器采样计时器
vuint32 g_center_ticks = 0;
#if IPS_ENABLE
vuint32 g_display_ticks = 0;                          // 显示刷新计时器
#endif
#if WIFI_ENABLE
vuint32 g_wifi_ticks = 0;                             // WiFi任务计时器
#endif
	
vuint8 g_flag_key = 0;                                // 按键扫描标志：1表示需要执行按键扫描
vuint8 g_flag_buzzer = 0;                             // 蜂鸣器轮询标志
vuint8 g_flag_imu = 0;                               // IMU读取标志：1表示需要读取IMU数据
vuint8 g_flag_steer = 0;                             // 舵机控制标志
vuint8 g_flag_encoder = 0;														// 编码器处理标志
vuint8 g_flag_center = 0;                            // 图像处理标志位
#if IPS_ENABLE
vuint8 g_flag_display = 0;                            // 显示刷新标志：1表示需要刷新显示
#endif
#if WIFI_ENABLE
vuint8 g_flag_wifi = 0;                               // WiFi任务标志：1表示需要执行WiFi任务
#endif



/************ 定时器回调函数 ************/
void system_tick_handler(void)
{
    g_system_ticks++;

    // 按键扫描 20ms
    if(g_system_ticks - g_key_ticks >= KEY_SCAN_PERIOD){
        g_key_ticks = g_system_ticks;
        g_flag_key = 1;
    }

    /* 蜂鸣器独立按 20ms 节拍轮询，不占主链图像处理。 */
    if(g_system_ticks - g_buzzer_ticks >= BUZZER_PERIOD){
        g_buzzer_ticks = g_system_ticks;
        g_flag_buzzer = 1;
    }

    // 陀螺仪读取 10ms
    if(g_system_ticks - g_imu_ticks >= IMU_PERIOD){
        g_imu_ticks = g_system_ticks;
        g_flag_imu = 1;
    }
    // 舵机控制 10ms
    if(g_system_ticks - g_steer_ticks >= STEER_PERIOD){
        g_steer_ticks = g_system_ticks;
        g_flag_steer = 1;
    }
    // 编码器采样 5ms
    if(g_system_ticks - g_encoder_ticks >= ENCODER_PERIOD){
        g_encoder_ticks = g_system_ticks;
        g_flag_encoder = 1;
    }
	// 图像处理
	if(g_system_ticks - g_center_ticks >=  CENTER_PERIOD){
		g_center_ticks = g_system_ticks;
		g_flag_center = 1;
	}
#if IPS_ENABLE
    // 显示 100ms
    if(g_system_ticks - g_display_ticks >= DISPLAY_PERIOD){
        g_display_ticks = g_system_ticks;
        g_flag_display = 1;
    }
#endif
#if WIFI_ENABLE
    /* WiFi 调参按 10ms 节拍取包，避免下行积压过久。 */
    if(g_system_ticks - g_wifi_ticks >= WIFI_PERIOD){
        g_wifi_ticks = g_system_ticks;
        g_flag_wifi = 1;
    }
#endif
}
