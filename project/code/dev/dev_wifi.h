/*********************************************************************************************************************
* STC32G144 Opensource Library - WiFi SPI Device Driver
* Copyright (c) 2025 SEEKFREE
*
********************************************************************************************************************/
#ifndef __DEV_WIFI_H__
#define __DEV_WIFI_H__

#include "zf_common_typedef.h"
#include "zf_device_wifi_spi.h"
#include "seekfree_assistant.h"

/************ WiFi配置 ************/
#define WIFI_SSID     "nova 11"      // WiFi名称
#define WIFI_PASSWORD "6bmxppq525m2jgx"  // WiFi密码

/************ 接口函数 ************/
// WiFi模块初始化
extern uint8 wifi_init(void);

// WiFi任务处理（需在主循环中调用）
extern void wifi_task(void);

// 发送示波器数据
extern void wifi_send_oscilloscope(float ch1, float ch2, float ch3, float ch4);

// 检查参数是否更新
extern uint8 wifi_param_updated(uint8 index);

// 获取参数值
extern float wifi_get_param(uint8 index);

#endif
