//
// Created by 31663 on 2026/7/3.
//

#include "Laser.h"

gpio_pin_enum laser[] = {IO_P91, IO_P67, IO_PB5, IO_PB4, IO_PB3, IO_P44, IO_PB2, IO_P65, IO_P17, IO_P14, IO_P65, IO_P45};
static uint8 time, i, send_buff[32];
extern int16 currentDuty;
extern uint8 canNotInRamp;
uint8 canStartLaser = 91;

void dismissLaser() {
    for (i = 0; i < 12; i++)
        gpio_set_level(laser[i], 0);
    gpio_set_level(IO_P90, 0);
}

void timer() {
    if (time) {
        if (time == 1)
            dismissLaser();
        time--;
    } else if (canStartLaser <= 90) {
        canStartLaser++;
    }
    if (canNotInRamp) canNotInRamp--;
}

void laserInit() {
    pit_ms_init(TIM5_PIT, 1, timer);
    for (i = 0; i < 12; i++)
        gpio_init(laser[i], GPO, 1, GPO_PUSH_PULL);
}

void startLaser(uint8 i) {
    if (canStartLaser < 90 || i <= 78 || i >= 113) return;
    gpio_set_level(IO_P90, 1);
    canStartLaser = 0;
    if (i <= 82) gpio_set_level(laser[0], 1);
    else if (i <= 85) gpio_set_level(laser[1], 1);
    else if (i <= 89) gpio_set_level(laser[2], 1); //1
    else if (i <= 92) gpio_set_level(laser[3], 1); //2
    else if (i <= 96) gpio_set_level(laser[4], 1); //3
    else if (i <= 100) gpio_set_level(laser[5], 1); //4
    else if (i <= 104) gpio_set_level(laser[6], 1); //5
    else if (i <= 108) gpio_set_level(laser[7], 1);
    else gpio_set_level(laser[8], 1);
    time = 6;
#if (DEBUG_MODE)
    sprintf(send_buff, "tar: %d,%d,%d,%d\n", currentDuty, encoderL, encoderR, i);
    wireless_uart_send_buffer(send_buff, 32);
#endif
}