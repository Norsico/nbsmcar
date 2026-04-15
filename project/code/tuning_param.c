/*********************************************************************************************************************
 * WiFi 遥测模块实现
 *
 * 功能说明：按当前工程配置完成 WiFi 连接、参数下行和示波器发送
*
* 修改记录
* 日期              备注
* 2026-03-13        创建调参模块
 * 2026-03-25        改为配置驱动的 WiFi 调参服务
********************************************************************************************************************/

#include "tuning_param.h"
#include "dev_encoder.h"
#include "dev_wifi.h"
#include "dev_wheel.h"
#include "system_state.h"

typedef float (*tuning_param_reader_t)(void);

typedef struct
{
    tuning_param_reader_t reader;
} tuning_osc_binding_t;

static tuning_osc_binding_t g_osc_binding[TUNING_PARAM_CHANNEL_MAX] = {0};
static uint8 g_tuning_active = 0;
static uint8 g_osc_channel_count = 0;
static uint32 g_last_osc_ticks = 0;
static uint32 g_last_connect_attempt_ticks = 0;
static uint8 g_has_connect_attempt = 0;

/* 通道 0：当前目标速度。 */
static float tuning_param_read_target_speed(void)
{
    return car_wheel_target_speed;
}

/* 通道 1：左轮 kp。 */
static float tuning_param_read_left_kp(void)
{
    return wheel_pid_left.param.kp;
}

/* 通道 2：左轮 ki。 */
static float tuning_param_read_left_ki(void)
{
    return wheel_pid_left.param.ki;
}

/* 通道 3：右轮 kp。 */
static float tuning_param_read_right_kp(void)
{
    return wheel_pid_right.param.kp;
}

/* 通道 4：右轮 ki。 */
static float tuning_param_read_right_ki(void)
{
    return wheel_pid_right.param.ki;
}

/* 通道 5：左轮编码器。 */
static float tuning_param_read_left_encoder(void)
{
    return (float)encoder_get_left();
}

/* 通道 6：右轮编码器。 */
static float tuning_param_read_right_encoder(void)
{
    return (float)encoder_get_right();
}

/* 通道 7：当前预留为空。 */
static float tuning_param_read_reserved(void)
{
    return 0.0f;
}

/* WiFi 下行与 5ms 后轮闭环共用同一组运行量，写 float 时需要避开中断读半截值。 */
static void tuning_param_apply_target_speed_atomic(float target_speed)
{
    uint8 interrupt_state = EA;

    EA = 0;
    car_wheel_set_target(target_speed);
    EA = interrupt_state;
}

/* 左轮 PID 运行参数更新。 */
static void tuning_param_apply_left_pid_atomic(float kp, float ki)
{
    uint8 interrupt_state = EA;

    EA = 0;
    wheel_pid_left.param.kp = kp;
    wheel_pid_left.param.ki = ki;
    EA = interrupt_state;
}

/* 右轮 PID 运行参数更新。 */
static void tuning_param_apply_right_pid_atomic(float kp, float ki)
{
    uint8 interrupt_state = EA;

    EA = 0;
    wheel_pid_right.param.kp = kp;
    wheel_pid_right.param.ki = ki;
    EA = interrupt_state;
}

static void tuning_param_reset_profile(void)
{
    uint8 i = 0;

    for(i = 0; i < TUNING_PARAM_CHANNEL_MAX; i++)
    {
        g_osc_binding[i].reader = 0;

        seekfree_assistant_parameter[i] = 0.0f;
        seekfree_assistant_parameter_update_flag[i] = 0;
    }

    g_osc_channel_count = 0;
    g_last_osc_ticks = 0;
    g_last_connect_attempt_ticks = 0;
    g_has_connect_attempt = 0;
}

static void tuning_osc_bind_slot(uint8 slot, tuning_param_reader_t reader)
{
    if(slot >= TUNING_PARAM_CHANNEL_MAX)
    {
        return;
    }

    g_osc_binding[slot].reader = reader;

    if((slot + 1) > g_osc_channel_count)
    {
        g_osc_channel_count = slot + 1;
    }
}

static void tuning_param_build_profile(void)
{
    /* 示波器上行通道与调参通道保持同一顺序，便于直接观察改参结果。 */
    tuning_osc_bind_slot(0, tuning_param_read_target_speed);
    tuning_osc_bind_slot(1, tuning_param_read_left_kp);
    tuning_osc_bind_slot(2, tuning_param_read_left_ki);
    tuning_osc_bind_slot(3, tuning_param_read_right_kp);
    tuning_osc_bind_slot(4, tuning_param_read_right_ki);
    tuning_osc_bind_slot(5, tuning_param_read_left_encoder);
    tuning_osc_bind_slot(6, tuning_param_read_right_encoder);
    tuning_osc_bind_slot(7, tuning_param_read_reserved);
}

static float tuning_osc_read_value(uint8 slot)
{
    if(slot >= TUNING_PARAM_CHANNEL_MAX)
    {
        return 0.0f;
    }

    return g_osc_binding[slot].reader ? g_osc_binding[slot].reader() : 0.0f;
}

static void tuning_param_activate_if_needed(void)
{
    if(g_tuning_active || !WIFI_TUNING_ENABLE)
    {
        return;
    }

    tuning_param_reset_profile();
    g_tuning_active = 1;
    tuning_param_build_profile();
}

static void tuning_param_send_oscilloscope(void)
{
    uint8 channel = 0;

    if(!g_tuning_active || !wifi_is_initialized() || (0 == g_osc_channel_count))
    {
        return;
    }

    if((g_system_ticks - g_last_osc_ticks) < WIFI_TUNING_OSC_PERIOD_MS)
    {
        return;
    }

    g_last_osc_ticks = g_system_ticks;

    for(channel = 0; channel < g_osc_channel_count; channel++)
    {
        seekfree_assistant_oscilloscope_data.dat[channel] = tuning_osc_read_value(channel);
    }

    seekfree_assistant_oscilloscope_data.channel_num = g_osc_channel_count;
    seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);
}

void tuning_param_boot_init(void)
{
    tuning_param_reset_profile();
    g_tuning_active = 0;
    tuning_param_activate_if_needed();
}

uint8 tuning_param_start_transport(void)
{
    tuning_param_activate_if_needed();

    if(!g_tuning_active)
    {
        return 0;
    }

    g_last_connect_attempt_ticks = g_system_ticks;
    g_has_connect_attempt = 1;

    return wifi_init(WIFI_TUNING_WIFI_SSID,
                     WIFI_TUNING_WIFI_PASSWORD,
                     WIFI_TUNING_TARGET_IP,
                     WIFI_TUNING_TARGET_PORT,
                     WIFI_TUNING_LOCAL_PORT);
}

void tuning_param_task(void)
{
    tuning_param_activate_if_needed();

    if(g_tuning_active && !wifi_is_initialized())
    {
        if(!g_has_connect_attempt ||
           ((g_system_ticks - g_last_connect_attempt_ticks) >= WIFI_TUNING_CONNECT_RETRY_MS))
        {
            tuning_param_start_transport();
        }
    }

    if(!wifi_is_initialized())
    {
        return;
    }

#if WIFI_TUNING_PARAM_ENABLE
    seekfree_assistant_data_analysis();
    tuning_param_update();
#endif
    tuning_param_send_oscilloscope();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     更新调参参数
// 参数说明     void
// 返回参数     void
// 使用示例     tuning_param_update();
//-------------------------------------------------------------------------------------------------------------------
void tuning_param_update(void)
{
#if WIFI_TUNING_PARAM_ENABLE
    float left_kp = 0.0f;
    float left_ki = 0.0f;
    float right_kp = 0.0f;
    float right_ki = 0.0f;
    uint8 update_left_pid = 0;
    uint8 update_right_pid = 0;

    if(seekfree_assistant_parameter_update_flag[0])
    {
        /* 通道 0：整车目标速度。 */
        seekfree_assistant_parameter_update_flag[0] = 0;
        tuning_param_apply_target_speed_atomic(seekfree_assistant_parameter[0]);
    }

    if(seekfree_assistant_parameter_update_flag[1])
    {
        /* 通道 1：左轮 kp。 */
        seekfree_assistant_parameter_update_flag[1] = 0;
        left_kp = seekfree_assistant_parameter[1];
        left_ki = wheel_pid_left.param.ki;
        update_left_pid = 1;
    }

    if(seekfree_assistant_parameter_update_flag[2])
    {
        /* 通道 2：左轮 ki。 */
        seekfree_assistant_parameter_update_flag[2] = 0;
        if(!update_left_pid)
        {
            left_kp = wheel_pid_left.param.kp;
        }
        left_ki = seekfree_assistant_parameter[2];
        update_left_pid = 1;
    }

    if(seekfree_assistant_parameter_update_flag[3])
    {
        /* 通道 3：右轮 kp。 */
        seekfree_assistant_parameter_update_flag[3] = 0;
        right_kp = seekfree_assistant_parameter[3];
        right_ki = wheel_pid_right.param.ki;
        update_right_pid = 1;
    }

    if(seekfree_assistant_parameter_update_flag[4])
    {
        /* 通道 4：右轮 ki。 */
        seekfree_assistant_parameter_update_flag[4] = 0;
        if(!update_right_pid)
        {
            right_kp = wheel_pid_right.param.kp;
        }
        right_ki = seekfree_assistant_parameter[4];
        update_right_pid = 1;
    }

    if(update_left_pid)
    {
        /* 左轮 kp/ki 成对更新，避免 5ms 节拍撞上半次写入。 */
        tuning_param_apply_left_pid_atomic(left_kp, left_ki);
    }

    if(update_right_pid)
    {
        /* 右轮 kp/ki 成对更新，避免 5ms 节拍撞上半次写入。 */
        tuning_param_apply_right_pid_atomic(right_kp, right_ki);
    }
#else
    /* 当前编译配置只保留示波器上行，不处理参数下行。 */
#endif
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取指定通道参数值
// 参数说明     channel: 通道号（0-7）
// 返回参数     float: 参数值
// 使用示例     float kp = tuning_param_get(0);
//-------------------------------------------------------------------------------------------------------------------
float tuning_param_get(uint8 channel)
{
    if(channel >= TUNING_PARAM_CHANNEL_MAX)
    {
        return 0.0f;
    }

    return seekfree_assistant_parameter[channel];
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置参数变化回调函数
// 参数说明     channel: 通道号（0-7）
//              callback: 回调函数指针
// 返回参数     void
// 使用示例     tuning_param_set_callback(0, motor_pid_set_kp);
//-------------------------------------------------------------------------------------------------------------------
void tuning_param_set_callback(uint8 channel, void (*callback)(float))
{
    (void)channel;
    (void)callback;
}

uint8 tuning_param_is_active(void)
{
    return g_tuning_active;
}

uint8 tuning_param_should_disable_display(void)
{
    return (g_tuning_active && WIFI_TUNING_DISABLE_IPS_WHEN_ACTIVE) ? 1 : 0;
}

uint8 tuning_param_should_skip_line_init(void)
{
    return (g_tuning_active && WIFI_TUNING_SKIP_LINE_INIT_WHEN_ACTIVE) ? 1 : 0;
}

uint8 tuning_param_should_pause_line_app(void)
{
    return (g_tuning_active && WIFI_TUNING_PAUSE_LINE_APP_WHEN_ACTIVE) ? 1 : 0;
}
