#include "headfile.h"

static uint16 BuzzerTime = 0;
static uint16 BuzzerStopAlarmWait = 0;
static uint8 BuzzerStopAlarmEnabled = 0;

void buzzer_init(void)
{
    BuzzerTime = 0;
    BuzzerStopAlarmWait = 0;
    BuzzerStopAlarmEnabled = 0;
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
    if(BuzzerTime > 0)
    {
        if(BuzzerTime <= MOTOR_CTRL_PERIOD_MS)
        {
            BuzzerTime = 0;
            gpio_set_level(BUZZER, GPIO_LOW);
            if(BuzzerStopAlarmEnabled != 0)
            {
                BuzzerStopAlarmWait = BUZZER_STOP_ALARM_OFF_MS;
            }
        }
        else
        {
            BuzzerTime -= MOTOR_CTRL_PERIOD_MS;
        }
        return;
    }

    if(BuzzerStopAlarmEnabled == 0)
    {
        return;
    }

    if(BuzzerStopAlarmWait > MOTOR_CTRL_PERIOD_MS)
    {
        BuzzerStopAlarmWait -= MOTOR_CTRL_PERIOD_MS;
        return;
    }

    BuzzerStopAlarmWait = 0;
    BuzzerTime = BUZZER_STOP_ALARM_ON_MS;
    gpio_set_level(BUZZER, GPIO_HIGH);
}

void buzzer_stop_alarm_enable(uint8 enable)
{
    if(enable != 0)
    {
        BuzzerStopAlarmEnabled = 1;
    }
    else
    {
        BuzzerStopAlarmEnabled = 0;
        BuzzerStopAlarmWait = 0;
    }
}
