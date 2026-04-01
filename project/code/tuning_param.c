/*********************************************************************************************************************
* 在线调参模块实现
*
* 功能说明：按配置自动完成 WiFi 连接、参数接收和示波器发送
*
* 修改记录
* 日期              备注
* 2026-03-13        创建调参模块
* 2026-03-25        改为配置驱动的 WiFi 调参服务
********************************************************************************************************************/

#include "tuning_param.h"
#include "dev_wifi.h"
#include "system_state.h"

#define WIFI_TUNING_PROFILE_IMPLEMENTATION
#include "wifi_tuning_config.h"

typedef float (*tuning_param_reader_t)(void);
typedef void (*tuning_param_writer_t)(float);

typedef struct
{
    uint8 enabled;
    float default_value;
    float min_value;
    float max_value;
    float *direct_value;
    tuning_param_writer_t writer;
    tuning_param_reader_t reader;
} tuning_param_binding_t;

typedef struct
{
    uint8 enabled;
    float *direct_value;
    tuning_param_reader_t reader;
} tuning_osc_binding_t;

static tuning_param_binding_t g_param_binding[TUNING_PARAM_CHANNEL_MAX] = {0};
static tuning_osc_binding_t g_osc_binding[TUNING_PARAM_CHANNEL_MAX] = {0};
static void (*param_callback[TUNING_PARAM_CHANNEL_MAX])(float) = {0};
static uint8 g_tuning_active = 0;
static uint8 g_osc_channel_count = 0;
static uint32 g_last_osc_ticks = 0;
static uint32 g_last_connect_attempt_ticks = 0;
static uint8 g_has_connect_attempt = 0;

static float tuning_param_limit(float value, float min_value, float max_value)
{
    if(value > max_value)
    {
        return max_value;
    }

    if(value < min_value)
    {
        return min_value;
    }

    return value;
}

static void tuning_param_reset_profile(void)
{
    uint8 i = 0;

    for(i = 0; i < TUNING_PARAM_CHANNEL_MAX; i++)
    {
        g_param_binding[i].enabled = 0;
        g_param_binding[i].default_value = 0.0f;
        g_param_binding[i].min_value = 0.0f;
        g_param_binding[i].max_value = 0.0f;
        g_param_binding[i].direct_value = 0;
        g_param_binding[i].writer = 0;
        g_param_binding[i].reader = 0;

        g_osc_binding[i].enabled = 0;
        g_osc_binding[i].direct_value = 0;
        g_osc_binding[i].reader = 0;

        seekfree_assistant_parameter[i] = 0.0f;
        seekfree_assistant_parameter_update_flag[i] = 0;
    }

    g_osc_channel_count = 0;
    g_last_osc_ticks = 0;
    g_last_connect_attempt_ticks = 0;
    g_has_connect_attempt = 0;
}

static void tuning_param_bind_slot(uint8 channel,
                                   float *direct_value,
                                   tuning_param_writer_t writer,
                                   float default_value,
                                   float min_value,
                                   float max_value)
{
    if(channel >= TUNING_PARAM_CHANNEL_MAX)
    {
        return;
    }

    g_param_binding[channel].enabled = 1;
    g_param_binding[channel].default_value = default_value;
    g_param_binding[channel].min_value = min_value;
    g_param_binding[channel].max_value = max_value;
    g_param_binding[channel].direct_value = direct_value;
    g_param_binding[channel].writer = writer;
}

static void tuning_osc_bind_slot(uint8 slot, float *direct_value, tuning_param_reader_t reader)
{
    if(slot >= TUNING_PARAM_CHANNEL_MAX)
    {
        return;
    }

    g_osc_binding[slot].enabled = 1;
    g_osc_binding[slot].direct_value = direct_value;
    g_osc_binding[slot].reader = reader;

    if((slot + 1) > g_osc_channel_count)
    {
        g_osc_channel_count = slot + 1;
    }
}

static void tuning_param_build_profile(void)
{
#define APPLY_PARAM_BINDING(channel, direct_ptr, writer_fn, default_value, min_value, max_value) \
    tuning_param_bind_slot((channel), (direct_ptr), (tuning_param_writer_t)(writer_fn), (default_value), (min_value), (max_value));

#define APPLY_OSC_BINDING(slot, direct_ptr, reader_fn) \
    tuning_osc_bind_slot((slot), (direct_ptr), (tuning_param_reader_t)(reader_fn));

    WIFI_TUNING_PARAM_TABLE(APPLY_PARAM_BINDING)
    WIFI_TUNING_OSC_TABLE(APPLY_OSC_BINDING)

#undef APPLY_OSC_BINDING
#undef APPLY_PARAM_BINDING
}

static uint8 tuning_param_trigger_requested(void)
{
    return (WIFI_TUNING_ENABLE && WIFI_TUNING_TRIGGER_ACTIVE()) ? 1 : 0;
}

static float tuning_param_read_value(uint8 channel)
{
    if(channel >= TUNING_PARAM_CHANNEL_MAX)
    {
        return 0.0f;
    }

    if(g_param_binding[channel].reader)
    {
        return g_param_binding[channel].reader();
    }

    if(g_param_binding[channel].direct_value)
    {
        return *(g_param_binding[channel].direct_value);
    }

    return seekfree_assistant_parameter[channel];
}

static float tuning_osc_read_value(uint8 slot)
{
    if(slot >= TUNING_PARAM_CHANNEL_MAX)
    {
        return 0.0f;
    }

    if(g_osc_binding[slot].reader)
    {
        return g_osc_binding[slot].reader();
    }

    if(g_osc_binding[slot].direct_value)
    {
        return *(g_osc_binding[slot].direct_value);
    }

    return 0.0f;
}

static float tuning_param_apply_value(uint8 channel, float value)
{
    float applied_value = value;

    if(channel >= TUNING_PARAM_CHANNEL_MAX)
    {
        return 0.0f;
    }

    if(g_param_binding[channel].enabled)
    {
        applied_value = tuning_param_limit(value,
                                           g_param_binding[channel].min_value,
                                           g_param_binding[channel].max_value);

        if(g_param_binding[channel].writer)
        {
            g_param_binding[channel].writer(applied_value);
        }
        else if(g_param_binding[channel].direct_value)
        {
            *(g_param_binding[channel].direct_value) = applied_value;
        }
    }

    if(param_callback[channel])
    {
        param_callback[channel](applied_value);
    }

    return applied_value;
}

static void tuning_param_apply_default_values(void)
{
    uint8 channel = 0;

    for(channel = 0; channel < TUNING_PARAM_CHANNEL_MAX; channel++)
    {
        if(!g_param_binding[channel].enabled)
        {
            continue;
        }

        seekfree_assistant_parameter[channel] = tuning_param_apply_value(channel, g_param_binding[channel].default_value);
        seekfree_assistant_parameter_update_flag[channel] = 0;
    }
}

static void tuning_param_activate_if_needed(void)
{
    if(g_tuning_active || !tuning_param_trigger_requested())
    {
        return;
    }

    tuning_param_reset_profile();
    g_tuning_active = 1;
    tuning_param_build_profile();
    tuning_param_apply_default_values();
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
    WIFI_TUNING_TRIGGER_INIT();
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

    tuning_param_update();
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
    uint8 channel = 0;

    if(!g_tuning_active || !wifi_is_initialized())
    {
        return;
    }

    // 解析上位机发送的参数
    seekfree_assistant_data_analysis();

    // 检查每个通道是否有参数更新
    for(channel = 0; channel < TUNING_PARAM_CHANNEL_MAX; channel++)
    {
        if(seekfree_assistant_parameter_update_flag[channel])
        {
            seekfree_assistant_parameter[channel] = tuning_param_apply_value(channel, seekfree_assistant_parameter[channel]);
            seekfree_assistant_parameter_update_flag[channel] = 0;
        }
    }
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

    return tuning_param_read_value(channel);
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
    if(channel >= TUNING_PARAM_CHANNEL_MAX)
    {
        return;
    }

    param_callback[channel] = callback;
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
