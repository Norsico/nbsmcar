/*********************************************************************************************************************
 * WiFi 遥测模块实现
 *
 * 功能说明：按配置自动完成 WiFi 连接和示波器发送，不再接收上位机参数写入
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

typedef struct
{
    uint8 enabled;
    float *direct_value;
    tuning_param_reader_t reader;
} tuning_osc_binding_t;

static tuning_osc_binding_t g_osc_binding[TUNING_PARAM_CHANNEL_MAX] = {0};
static uint8 g_tuning_active = 0;
static uint8 g_osc_channel_count = 0;
static uint32 g_last_osc_ticks = 0;
static uint32 g_last_connect_attempt_ticks = 0;
static uint8 g_has_connect_attempt = 0;

static void tuning_param_reset_profile(void)
{
    uint8 i = 0;

    for(i = 0; i < TUNING_PARAM_CHANNEL_MAX; i++)
    {
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
#define APPLY_OSC_BINDING(slot, direct_ptr, reader_fn) \
    tuning_osc_bind_slot((slot), (direct_ptr), (tuning_param_reader_t)(reader_fn));

    WIFI_TUNING_OSC_TABLE(APPLY_OSC_BINDING)

#undef APPLY_OSC_BINDING
}

static uint8 tuning_param_trigger_requested(void)
{
    return (WIFI_TUNING_ENABLE && WIFI_TUNING_TRIGGER_ACTIVE()) ? 1 : 0;
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

static void tuning_param_activate_if_needed(void)
{
    if(g_tuning_active || !tuning_param_trigger_requested())
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
    /* 当前 WiFi 只保留示波器上行，这里不再解析上位机参数。 */
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取指定通道参数值
// 参数说明     channel: 通道号（0-7）
// 返回参数     float: 参数值
// 使用示例     float kp = tuning_param_get(0);
//-------------------------------------------------------------------------------------------------------------------
float tuning_param_get(uint8 channel)
{
    (void)channel;
    return 0.0f;
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
