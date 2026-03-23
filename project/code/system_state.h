/*
 * system_state.h - 系统状态定义
 *
 * 定义系统状态、全局变量和定时器
 */

#ifndef __SYSTEM_STATE_H__
#define __SYSTEM_STATE_H__

#include "zf_common_typedef.h"

/************ 系统状态枚举 ************/
typedef enum {
    SYS_INIT = 0,      // 上电初始化
    SYS_PREPARE,       // 准备就绪，可运行
    SYS_RUNNING,       // 赛道运行
    SYS_STOPED,        // 停止
    SYS_EMERGENCY      // 紧急情况
} system_state_t;

/************ 宏定义 ************/
#define TICKS_MS 1                     // 系统tick 1ms
#define KEY_SCAN_PERIOD 20              // 按键扫描周期 20ms (50Hz)
#define IMU_PERIOD 10                   // IMU读取周期 10ms (100Hz)
#define ENCODER_PERIOD 10               // 编码器采样周期 10ms (100Hz)
#define DISPLAY_PERIOD 100              // 显示刷新周期 100ms (10Hz)
#define WIFI_PERIOD 50                  // WiFi任务周期 50ms (20Hz)

#define LED_DEBUG (IO_P52)              // 调试LED引脚

/************ WDT 宏定义 ************/
#define WDT_PRESCALER  0x07            // 看门狗预分频：~1049ms@96MHz
#define wdt_enable()   (WDT_CONTR = 0x20 | 0x10 | WDT_PRESCALER)  // 使能看门狗
#define wdt_feed()     (WDT_CONTR |= 0x10)   // 喂狗

/************ 全局变量 ************/
// 系统状态
extern volatile system_state_t g_system_state;  // 系统当前状态
extern uint8 system_error;                       // 系统错误标志

// 任务计时器
extern vuint32 g_system_ticks;           // 系统Tick计数器
extern vuint32 g_key_ticks;              // 按键扫描计时器
extern vuint32 g_imu_ticks;              // IMU读取计时器
extern vuint32 g_encoder_ticks;          // 编码器采样计时器
extern vuint32 g_display_ticks;          // 显示刷新计时器
extern vuint32 g_wifi_ticks;             // WiFi任务计时器

// 任务标志位
extern vuint8 g_flag_key;                         // 按键扫描标志
extern vuint8 g_flag_imu;                          // IMU读取标志
extern vuint8 g_flag_encoder;                     // 编码器采样标志
extern vuint8 g_flag_display;                      // 显示刷新标志
extern vuint8 g_flag_wifi;                         // WiFi任务标志

// IMU相关
extern uint8 imu_retry;                          // IMU初始化重试计数
extern const uint8 IMU_MAX_RETRY;                // IMU最大重试次数
extern const uint8 IMU_CALIBRATE_SAMPLES;         // 零偏校准采样次数

/************ 定时器回调函数 ************/
extern void system_tick_handler(void);            // 系统Tick中断处理，每1ms调用一次

#endif /* __SYSTEM_STATE_H__ */
