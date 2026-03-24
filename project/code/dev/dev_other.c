#include "dev_other.h"

// 配置蜂鸣器、激光笔初始化

void other_init(void)
{
	gpio_init(BUZZER_PIN,GPO,GPIO_LOW,GPO_PUSH_PULL);
	gpio_init(LASER_PIN,GPO,GPIO_LOW,GPO_PUSH_PULL);
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