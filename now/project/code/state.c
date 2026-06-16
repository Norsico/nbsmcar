#include "headfile.h"

car_mode CarMode = CAR_MODE_STOP;

void state_init(void)
{
    uint8 switch1;
    uint8 switch2;

    gpio_init(SWITCH_MODE1, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(SWITCH_MODE2, GPI, GPIO_HIGH, GPI_PULL_UP);

    /* 检测看门狗复位标志（死机保护） */
    if(WDT_CONTR & 0x80)  /* WDT_FLAG 位被置1 = 看门狗复位 */
    {
        WDT_CONTR &= ~0x80;  /* 清除标志位 */
        CarMode = CAR_MODE_STOP;  /* 强制停车 */
        return;
    }

    /* 正常启动：根据开关状态决定模式 */
    switch1 = gpio_get_level(SWITCH_MODE1) ? 1 : 0;
    switch2 = gpio_get_level(SWITCH_MODE2) ? 1 : 0;

    if(switch1 && switch2)
    {
        CarMode = CAR_MODE_UI;
    }
    else if((switch1==0) && (switch2==0))
    {
        CarMode = CAR_MODE_RUN;
    }
    else
    {
        CarMode = CAR_MODE_STOP;
    }
}
