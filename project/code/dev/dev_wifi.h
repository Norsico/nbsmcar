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

/************ 接口函数 ************/
// WiFi模块初始化
uint8 wifi_init(const char* wifi_ssid,
                const char* wifi_password,
                const char* target_ip,
                const char* target_port,
                const char* local_port);
uint8 wifi_is_initialized(void);


#endif
