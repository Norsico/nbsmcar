#include "dev_other.h"
#include "system_state.h"

// 蜂鸣器时间
#define BUZZER_SHORT_MS (100)
#define BUZZER_LONG_MS  (500)
// 激光笔时间
#define LASER_SHORT_MS (2)

// 配置蜂鸣器、激光笔初始化
static uint8 g_switch_debug_enable = 1;
static uint8 g_switch_ui_enable = 0;
static uint8 g_switch_wifi_enable = 0;
static uint8 g_buzzer_busy = 0;
static uint32 g_buzzer_stop_tick = 0;
static uint8 g_laser_remain_ms = 0;

/* 蜂鸣器响铃期间不重复受理新的短响长响请求。 */
static void buzzer_start(uint16 duration_ms)
{
    if(0 != g_buzzer_busy)
    {
        return;
    }

    buzzer_on();
    g_buzzer_busy = 1;
    g_buzzer_stop_tick = g_system_ticks + duration_ms;
}

static void laser_start(uint16 duration_ms)
{
    if(0 == duration_ms)
    {
        g_laser_remain_ms = 0;
        laser_off();
        return;
    }

    laser_on();
    g_laser_remain_ms = (duration_ms > 255U) ? 255U : (uint8)duration_ms;
}

void other_init(void)
{
	gpio_init(BUZZER_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
	gpio_init(LASER_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
	gpio_init(SWITCH_PIN1, GPI, GPIO_HIGH, GPI_PULL_UP);
	gpio_init(SWITCH_PIN2, GPI, GPIO_HIGH, GPI_PULL_UP);
}

// 蜂鸣器响
void buzzer_on(void)
{
	gpio_set_level(BUZZER_PIN,1); // 50%占空比，均匀方波
}
// 蜂鸣器灭
void buzzer_off(void)
{
	gpio_set_level(BUZZER_PIN,0);
}

void buzzer_short(void)
{
    buzzer_start(BUZZER_SHORT_MS);
}

void buzzer_long(void)
{
    buzzer_start(BUZZER_LONG_MS);
}

/* 蜂鸣器按独立节拍轮询，到时自动关断。 */
void buzzer_task(void)
{
    if(0 == g_buzzer_busy)
    {
        return;
    }

    if(g_system_ticks >= g_buzzer_stop_tick)
    {
        buzzer_off();
        g_buzzer_busy = 0;
        g_buzzer_stop_tick = 0;
    }
}
// 激光笔亮
void laser_on(void)
{
	gpio_set_level(LASER_PIN,GPIO_HIGH);
}
// 激光笔灭
void laser_off(void)
{
	gpio_set_level(LASER_PIN,GPIO_LOW);
}
void laser_short(void)
{
    laser_start(LASER_SHORT_MS);
}
void laser_task(void)
{
    if(0 == g_laser_remain_ms)
    {
        laser_off();
        return;
    }

    g_laser_remain_ms--;
    if(0 == g_laser_remain_ms)
    {
        laser_off();
    }
}

void switch_update(void)
{
    uint8 screen_pin = 0;
    uint8 wifi_pin = 0;

    /* 拨码往下拨时为高电平。 */
    screen_pin = gpio_get_level(SWITCH_PIN1) ? 1 : 0;
    wifi_pin = gpio_get_level(SWITCH_PIN2) ? 1 : 0;

#if IPS_ENABLE
    /* PB0 打开时走屏幕 UI 模式。 */
    g_switch_ui_enable = screen_pin;
#else
    g_switch_ui_enable = 0;
#endif

#if WIFI_ENABLE
    /* PB1 打开且屏幕关闭时，走无屏 WiFi 调参模式。 */
    g_switch_wifi_enable = (uint8)(wifi_pin && !g_switch_ui_enable);
#else
    g_switch_wifi_enable = 0;
#endif

    /* 统一保留一个调试总状态，当前等价于屏幕或 WiFi 任一模式被拨开。 */
    g_switch_debug_enable = (uint8)(g_switch_ui_enable || g_switch_wifi_enable);
}

uint8 switch_debug_enabled(void)
{
    return g_switch_debug_enable;
}

uint8 switch_ui_enabled(void)
{
    return g_switch_ui_enable;
}

uint8 switch_wifi_enabled(void)
{
    return g_switch_wifi_enable;
}
