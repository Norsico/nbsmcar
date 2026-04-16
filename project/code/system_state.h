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
    SYS_RUNNING,       // 主循环运行
    SYS_EMERGENCY      // 紧急情况
} system_state_t;

/************ 宏定义 ************/
#define TICKS_MS 1                     // 系统tick 1ms
#define KEY_SCAN_PERIOD 20              // 按键扫描周期 20ms (50Hz)
#define IMU_PERIOD 10                   // IMU读取周期 10ms (100Hz)
#define ENCODER_PERIOD 5                // 编码器采样周期 5ms (200Hz)，对齐 19 国一电机闭环节拍。
#define CENTER_PERIOD 10				// 搜索中心周期 10ms (100Hz)
#define DISPLAY_PERIOD 100              // 显示刷新周期 100ms (10Hz)
#define WIFI_PERIOD 10                  // WiFi任务周期 10ms，调参收包按更快节拍处理。

#define LED_DEBUG (IO_P52)              // 调试LED引脚

/************ WDT 宏定义 ************/
#define WDT_PRESCALER  0x07            // 看门狗预分频：~1049ms@96MHz
#define wdt_enable()   (WDT_CONTR = 0x20 | 0x10 | WDT_PRESCALER)  // 使能看门狗
#define wdt_feed()     (WDT_CONTR |= 0x10)   // 喂狗

/*********** 运行设定 ****************/
/* WiFi 功能编译开关：打开后可通过拨码切到 WiFi 调参模式。 */
#define WIFI_ENABLE (1)
/* 屏幕功能编译开关：打开后可通过拨码切到 UI 屏幕模式。 */
#define IPS_ENABLE  (1)
/* WiFi 参数下行编译开关：0 只发示波器，1 额外打开调参下行。 */
#define WIFI_TUNING_PARAM_ENABLE (1)
/************ 全局变量 ************/
// 系统状态
extern volatile system_state_t g_system_state;  // 系统当前状态
extern uint8 system_error;                       // 系统错误标志

// 任务计时器
extern vuint32 g_system_ticks;           // 系统Tick计数器
extern vuint32 g_key_ticks;              // 按键扫描计时器
extern vuint32 g_imu_ticks;              // IMU读取计时器
extern vuint32 g_encoder_ticks;          // 编码器采样计时器
extern vuint32 g_center_ticks;					 // 搜索中心计时器
#if IPS_ENABLE
extern vuint32 g_display_ticks;          // 显示刷新计时器
#endif
#if WIFI_ENABLE
extern vuint32 g_wifi_ticks;             // WiFi任务计时器
#endif
// 任务标志位
extern vuint8 g_flag_key;                         // 按键扫描标志
extern vuint8 g_flag_imu;                          // IMU读取标志
extern vuint8 g_flag_encoder;                     // 编码器处理标志
extern vuint8 g_flag_center;											// 搜索中心标志
#if IPS_ENABLE
extern vuint8 g_flag_display;                      // 显示刷新标志
#endif
#if WIFI_ENABLE
extern vuint8 g_flag_wifi;                         // WiFi任务标志
#endif


/************ 定时器回调函数 ************/
extern void system_tick_handler(void);            // 系统Tick中断处理，每1ms调用一次

#endif /* __SYSTEM_STATE_H__ */
