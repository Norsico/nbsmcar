#include "headfile.h"

static uint16 BuzzerTime = 0;

void buzzer_init(void)
{
    BuzzerTime = 0;
    gpio_init(BUZZER, GPO, GPIO_LOW, GPO_PUSH_PULL);
}

void buzzer_short(void)
{
    if(BuzzerTime == 0)
    {
        BuzzerTime = BUZZER_SHORT_MS;
        gpio_set_level(BUZZER, GPIO_HIGH);
    }
}

void buzzer_tick(void)
{
    if(BuzzerTime == 0)
    {
        return;
    }

    if(BuzzerTime <= MOTOR_CTRL_PERIOD_MS)
    {
        BuzzerTime = 0;
        gpio_set_level(BUZZER, GPIO_LOW);
    }
    else
    {
        BuzzerTime -= MOTOR_CTRL_PERIOD_MS;
    }
}
