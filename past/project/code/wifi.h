#ifndef _WIFI_H_
#define _WIFI_H_

#include "zf_common_headfile.h"

#define WIFI_PARAM_ENABLE           (0)                  /* 调参开关 */
#define WIFI_TASK_PIT               (TIM3_PIT)           /* WiFi定时器 */
#define WIFI_TASK_PERIOD_MS         (20)                 /* WiFi周期 */
#define WIFI_OSC_CHANNEL_NUM        (3)                  /* 示波器通道数 */

#define WIFI_NAME                   "OnePlus"            /* WiFi名称 */
#define WIFI_PASSWORD               "1234567890xia"     /* WiFi密码 */
#define WIFI_TARGET_IP              "10.55.36.9"        /* 上位机IP */

void wifi_update(void);

#endif
