#include "dev_other.h"
#include "system_state.h"

// 配置蜂鸣器、激光笔初始化

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

void switch_update(void)
{
    uint8 screen_pin = 0;
    uint8 wifi_pin = 0;

    /* 拨码往下拨时为高电平。 */
    screen_pin = gpio_get_level(SWITCH_PIN1) ? 1 : 0;
    wifi_pin = gpio_get_level(SWITCH_PIN2) ? 1 : 0;

#if IPS_ENABLE
    /* PB0 打开时走屏幕 UI 模式。 */
    g_ips_enable = screen_pin;
#else
    g_ips_enable = 0;
#endif

#if WIFI_ENABLE
    /* PB1 打开且屏幕关闭时，走无屏 WiFi 调参模式。 */
    g_wifi_enable = (uint8)(wifi_pin && !g_ips_enable);
#else
    g_wifi_enable = 0;
#endif
}
