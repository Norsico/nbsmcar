#include "dev_other.h"
#include "system_state.h"

// 配置蜂鸣器、激光笔初始化

void other_init(void)
{
	gpio_init(BUZZER_PIN,GPO,GPIO_LOW,GPO_PUSH_PULL);
	gpio_init(LASER_PIN,GPO,GPIO_LOW,GPO_PUSH_PULL);
	gpio_init(SWITCH_PIN1,GPI,0,GPI_PULL_UP); //上拉
	gpio_init(SWITCH_PIN2,GPI,0,GPI_PULL_UP);
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

// 拨码开发更新
void switch_update(void){
	uint8 pin1,pin2;
	pin1 = gpio_get_level(SWITCH_PIN1); // 内开关
	pin2 = gpio_get_level(SWITCH_PIN2);
	if(pin1){
		// 往下拨是高
		g_debug_enable = 1; // 启用
	} else {
		g_debug_enable = 0; // 禁用
	}
	if(pin2){
		g_ips_enable = 1;
		g_wifi_enable = 0;
	} else {
		g_ips_enable = 0;
		g_wifi_enable = 1;
	}
}