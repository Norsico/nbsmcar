#ifndef _car_init_h_
#define _car_init_h_

#include "zf_common_headfile.h"

// 车辆初始化函数
void car_init(uint8 enable_wifi, const char *wifi_ssid, const char *wifi_password, const char *target_ip);

// 简化调用宏定义
#define car_init_no_wifi()                  car_init(0, NULL, NULL, NULL)
#define car_init_with_default_wifi()        car_init(1, NULL, NULL, NULL)
#define car_init_with_wifi(ssid, pwd, ip)   car_init(1, ssid, pwd, ip)

#endif
