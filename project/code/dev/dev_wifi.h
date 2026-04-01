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
#define DEFAULT_WIFI_SSID     "QQ"      // WiFi名称
#define DEFAULT_WIFI_PASSWORD "1234567890xia"  // WiFi密码
#define DEFAULT_TARGET_IP 		"192.168.43.236"					// IP地址

/************ 接口函数 ************/
// WiFi模块初始化
uint8 wifi_init(const char* wifi_ssid,const char* wifi_password,const char* target_ip);
uint8 wifi_is_initialized(void); // 检查函数

#endif
