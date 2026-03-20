#include "app_key.h"

/* 全局变量 */
static vint32 g_laser_tick = 0;
static uint8 g_laser_on = 0;

/**********************************************************
 * @brief  按键初始化
 * @param  None
 * @return None
 * @note   外部上拉，按下为低电平
 **********************************************************/
void key_init(void)
{
    /* 初始化4个按键为输入模式，外部上拉 */
    gpio_init(KEY1_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(KEY2_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(KEY3_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(KEY4_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);

    /* 初始化激光笔为输出，默认关闭 */
    gpio_init(LASER_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
}

/**********************************************************
 * @brief  按键扫描
 * @param  None
 * @return None
 * @note   检测任意按键按下，点亮激光笔500ms
 **********************************************************/
void key_scan(void)
{
    /* C251 语法：变量必须在函数开头声明 */
    uint8 key1;
    uint8 key2;
    uint8 key3;
    uint8 key4;
    static uint8 key_last = 0;  /* 上一次按键状态 */

    /* 读取4个按键状态，按下为低电平 */
    key1 = gpio_get_level(KEY1_PIN);
    key2 = gpio_get_level(KEY2_PIN);
    key3 = gpio_get_level(KEY3_PIN);
    key4 = gpio_get_level(KEY4_PIN);

    /* 合并4个按键状态 */
    uint8 key_cur = (key1 & key2 & key3 & key4);

    /* 检测按键下降沿（从高电平变为低电平，即按下瞬间） */
    if(key_cur == KEY_PRESS && key_last != KEY_PRESS)
    {
        laser_on(LASER_ON_TIME);
    }

    /* 保存当前按键状态 */
    key_last = key_cur;
}

/**********************************************************
 * @brief  启动激光笔
 * @param  ms: 点亮时间(毫秒)
 * @return None
 **********************************************************/
void laser_on(uint32 ms)
{
    gpio_set_level(LASER_PIN, GPIO_HIGH);  // 点亮激光笔
    g_laser_tick = 0;                       // 计时器清零
    g_laser_on = 1;                          // 标记激光笔已启动
}

/**********************************************************
 * @brief  激光笔处理
 * @param  None
 * @return None
 * @note   在主循环中调用，自动关闭到期的激光笔
 **********************************************************/
void laser_process(void)
{
    if(g_laser_on)
    {
        g_laser_tick++;
        if(g_laser_tick >= LASER_ON_TIME)
        {
            gpio_set_level(LASER_PIN, GPIO_LOW);  // 关闭激光笔
            g_laser_on = 0;
        }
    }
}
