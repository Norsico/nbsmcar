#include "app_key.h"
#include "system_state.h" // 按键扫描间隔宏定义

/* 宏定义 */
#define LONG_PRESS_COUNT (800/KEY_SCAN_PERIOD) // 800ms 以上为长按

/* 全局函数 */
key_info_t keys_info[KEY_MAX];

/**********************************************************
 * @brief  按键初始化
 * @param  None
 * @return None
 * @note   外部上拉，按下为低电平
 **********************************************************/
void key_init(void)
{
		uint8 i;
    /* 初始化4个按键为输入模式，外部上拉 */
    gpio_init(KEY1_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(KEY2_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(KEY3_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(KEY4_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
		/* 初始化结构体 */
		for(i=0;i<KEY_MAX;i++){
			keys_info[i].state = KEY_IDLE;
			keys_info[i].interval = 0;
		}
}

/**********************************************************
 * @brief  按键扫描
 * @param  None
 * @return None
 * @note   每次扫描将状态写入结构体数据
 **********************************************************/
static void key_scan(void)
{
   /* 读取4个按键状态，按下为低电平 */
	keys_info[KEY1].level = gpio_get_level(KEY1_PIN) ? 1 : 0; 
	keys_info[KEY2].level = gpio_get_level(KEY2_PIN) ? 1 : 0;
	keys_info[KEY3].level = gpio_get_level(KEY3_PIN) ? 1 : 0;
	keys_info[KEY4].level = gpio_get_level(KEY4_PIN) ? 1 : 0;
}
/***********************************************************
 * @brief 按键状态更新
 * @note 空闲、短按、长按，每次抬起更新状态
 * @note 由于短按和长按状态保持只有一次更新周期，
	标志位和其他处理建议紧跟更新，或在中断中快速置位
***********************************************************/
void key_update(void)
{
    uint8 i;
    key_scan();  // 更新 keys_info[].level
    for (i = 0; i < KEY_MAX; i++) {
			// 空闲状态： IDLE且间隔为0
			if(keys_info[i].state == KEY_IDLE){
				// 如果按下，计数
				if(keys_info[i].level == 0){
					keys_info[i].interval++; // 开始计数
				}
				// 如果抬起，结算
				else
				{
					// 根据间隔分长按短按
					if(keys_info[i].interval>= LONG_PRESS_COUNT) keys_info[i].state = KEY_LONG;
					else if(keys_info[i].interval>0) keys_info[i].state = KEY_SHORT;
					else; // 间隔为0不作处理 
				}
			}
			// 短按和长按状态，只做重置处理
			else {
				// 确认抬起状态再重置
				if(keys_info[i].level == 1){
					keys_info[i].state = KEY_IDLE;
					keys_info[i].interval = 0;
				}
			}
		}
}
	