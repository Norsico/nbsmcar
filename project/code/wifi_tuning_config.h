#ifndef _WIFI_TUNING_CONFIG_H_
#define _WIFI_TUNING_CONFIG_H_

#include "zf_driver_gpio.h"

/*
 * WiFi 调参总开关。
 * 置 1 表示工程保留这套调参能力；置 0 则整套调参逻辑不启用。
 */
#define WIFI_TUNING_ENABLE                     (1)

/*
 * 调参模式下的行为开关。
 * 进入调参模式后，可按需关闭屏幕、跳过巡线初始化、暂停巡线处理。
 */
#define WIFI_TUNING_DISABLE_IPS_WHEN_ACTIVE    (1)
#define WIFI_TUNING_SKIP_LINE_INIT_WHEN_ACTIVE (0)
#define WIFI_TUNING_PAUSE_LINE_APP_WHEN_ACTIVE (0)

/* 示波器数据发送周期，单位 ms。 */
#define WIFI_TUNING_OSC_PERIOD_MS              (50)
/* WiFi 未连上时的重试周期，单位 ms。 */
#define WIFI_TUNING_CONNECT_RETRY_MS           (1000)

/* WiFi 和上位机网络参数。 */
#define WIFI_TUNING_WIFI_SSID                  "OnePlus"
#define WIFI_TUNING_WIFI_PASSWORD              "1234567890xia"          /* 热点密码 */
#define WIFI_TUNING_TARGET_IP                  "10.11.37.9"
#define WIFI_TUNING_TARGET_PORT                "8086"
#define WIFI_TUNING_LOCAL_PORT                 "8086"

/*
 * 调参触发钩子。
 * 默认行为：只要 WIFI_TUNING_ENABLE 为 1，就直接进入调参模式。
 * 同时后台也会持续轮询这个条件，只要后续变为真，也会自动锁存进入调参模式。
 *
 * 拨码开关示例：
 *   #define WIFI_TUNING_TRIGGER_INIT() gpio_init(IO_P33, GPI, GPIO_HIGH, GPI_PULL_UP)
 *   #define WIFI_TUNING_TRIGGER_ACTIVE() (0 == gpio_get_level(IO_P33))
 */
#ifndef WIFI_TUNING_TRIGGER_INIT
#define WIFI_TUNING_TRIGGER_INIT()             do { } while(0)
#endif

#ifndef WIFI_TUNING_TRIGGER_ACTIVE
#define WIFI_TUNING_TRIGGER_ACTIVE()           (1)
#endif

#ifdef WIFI_TUNING_PROFILE_IMPLEMENTATION

#include "dev_encoder.h"
#include "dev_wheel.h"
#include "searchLine.h"

/* 当前加权 det。 */
static float wifi_tuning_read_det_true(void)
{
    return (float)SearchLine_GetDetTrue();
}

/* 当前舵机命令角。 */
static float wifi_tuning_read_steer_command(void)
{
    return (float)SearchLine_GetSteerCommand();
}

/* 当前直道判定。 */
static float wifi_tuning_read_straight_acc(void)
{
    return (float)SearchLine_GetStraightAcc();
}

/* 当前左轮编码器脉冲。 */
static float wifi_tuning_read_left_encoder(void)
{
    return (float)encoder_get_left();
}

/* 当前右轮编码器脉冲。 */
static float wifi_tuning_read_right_encoder(void)
{
    return (float)encoder_get_right();
}

/* 当前 WiFi 只保留示波器上行，不再接参数下行。 */
#define WIFI_TUNING_OSC_TABLE(APPLY) \
    APPLY(0, 0, wifi_tuning_read_det_true) \
    APPLY(1, 0, wifi_tuning_read_steer_command) \
    APPLY(2, 0, wifi_tuning_read_straight_acc) \
    APPLY(3, &car_wheel_target_speed, 0) \
    APPLY(4, 0, wifi_tuning_read_left_encoder) \
    APPLY(5, 0, wifi_tuning_read_right_encoder) \
    APPLY(6, &ref_left_target, 0) \
    APPLY(7, &ref_right_target, 0)

#endif

#endif
