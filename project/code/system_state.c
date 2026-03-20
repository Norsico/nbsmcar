#include "system_state.h"

/************ 全局变量定义 ************/
volatile system_state_t g_system_state = SYS_INIT;  // 系统状态：INIT/PREPARE/RUNNING/STOPED/EMERGENCY
uint8 system_error = 0;                               // 系统错误标志：非0表示发生错误，进入紧急状态

vuint32 g_system_ticks = 0;                           // 系统Tick计数器，每1ms加1
vuint32 g_key_ticks = 0;                              // 按键扫描计时器
vuint32 g_imu_ticks = 0;                              // IMU读取计时器
vuint32 g_display_ticks = 0;                          // 显示刷新计时器
vuint32 g_wifi_ticks = 0;                             // WiFi任务计时器

vuint8 g_flag_key = 0;                                // 按键扫描标志：1表示需要执行按键扫描
vuint8 g_flag_imu = 0;                               // IMU读取标志：1表示需要读取IMU数据
vuint8 g_flag_display = 0;                            // 显示刷新标志：1表示需要刷新显示
vuint8 g_flag_wifi = 0;                               // WiFi任务标志：1表示需要执行WiFi任务

uint8 imu_retry = 0;                                 // IMU初始化重试计数器
const uint8 IMU_MAX_RETRY = 50;                      // IMU初始化最大重试次数
const uint8 IMU_CALIBRATE_SAMPLES = 50;              // IMU零偏校准采样次数

/************ 定时器回调函数 ************/
void system_tick_handler(void)
{
    g_system_ticks++;

    // 按键扫描 20ms
    if(g_system_ticks - g_key_ticks >= KEY_SCAN_PERIOD){
        g_key_ticks = g_system_ticks;
        g_flag_key = 1;
    }

    // 陀螺仪读取 10ms
    if(g_system_ticks - g_imu_ticks >= IMU_PERIOD){
        g_imu_ticks = g_system_ticks;
        g_flag_imu = 1;
    }

    // 显示 100ms
    if(g_system_ticks - g_display_ticks >= DISPLAY_PERIOD){
        g_display_ticks = g_system_ticks;
        g_flag_display = 1;
    }

    // WiFi任务 50ms
    if(g_system_ticks - g_wifi_ticks >= WIFI_PERIOD){
        g_wifi_ticks = g_system_ticks;
        g_flag_wifi = 1;
    }
}
