#include "headfile.h"

#define STATE_BLIND_BOX_DELAY_MS     (1000)
#define STATE_BLIND_BOX_DELAY_TICKS  ((STATE_BLIND_BOX_DELAY_MS + MOTOR_CTRL_PERIOD_MS - 1) / MOTOR_CTRL_PERIOD_MS)

volatile car_mode CarMode = CAR_MODE_STOP;
volatile blind_box_phase BlindBoxPhase = BLIND_BOX_OFF;
static volatile uint16 BlindBoxDelayTicks = 0;

void state_init(void)
{
    uint8 switch1;
    uint8 switch2;

    gpio_init(SWITCH_MODE1, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(SWITCH_MODE2, GPI, GPIO_HIGH, GPI_PULL_UP);
    BlindBoxPhase = BLIND_BOX_OFF;
    BlindBoxDelayTicks = 0;

    /* 根据开关状态决定模式 */
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

void state_start_blind_box_delay(void)
{
    BlindBoxDelayTicks = STATE_BLIND_BOX_DELAY_TICKS;
    BlindBoxPhase = BLIND_BOX_DELAY;
}

void state_tick(void)
{
    if((CarMode != CAR_MODE_RUN) || (BlindBoxPhase != BLIND_BOX_DELAY))
    {
        return;
    }

    if(BlindBoxDelayTicks > 0)
    {
        BlindBoxDelayTicks--;
    }
    if(BlindBoxDelayTicks == 0)
    {
        BlindBoxPhase = BLIND_BOX_SPEED1;
    }
}
