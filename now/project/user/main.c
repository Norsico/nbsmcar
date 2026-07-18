#include "headfile.h"

void main(void)
{
    clock_init(SYSTEM_CLOCK_114M);

    debug_init();
    para_init();
    flash_init();

    /* Challenge mode always uses motor feedback and does not use the fan. */
    CarMode = CAR_MODE_RUN;
    BlindBoxPhase = BLIND_BOX_OFF;
    SmartCar.motor.fan_en = 0;

    buzzer_init();
    servo_init();
    motor_init();
    direction_gate_init();
    motor_start_control();

    while(1)
    {
        direction_gate_update();
    }
}
