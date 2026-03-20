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
#define DEFAULT_WIFI_SSID     "nova 11"      // WiFi名称
#define DEFAULT_WIFI_PASSWORD "6bmxppq525m2jgx"  // WiFi密码
#define DEFAULT_TARGET_IP 		""					// IP地址

/************ 接口函数 ************/
// WiFi模块初始化
uint8 wifi_init(const char* wifi_ssid,const char* wifi_password,const char* target_ip);


#endif
