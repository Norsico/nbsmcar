//
// Created by 31663 on 2026/4/22.
//

#include "RAMP.h"

extern int16 RUN_MODE;
extern Point centerLine[expRoadCount], LRoad[expRoadCount], RRoad[expRoadCount];
extern int16 centerLineCount, LRoadCount, RRoadCount;
extern const uint8 reflection[120][188][2];
uint8 canEnd, canNotInRamp;

void runRamp() {
    int8 data i;
    gpio_set_level(IO_P90, 1);
    centerLineDistance = 18;
    if (pitch <= -5) {
        canEnd = 1;
        setSpeed = 350;
    } else if (pitch > -2 && pitch < 2 && canEnd && dl1b_distance_mm > 500) {
        RUN_MODE = FORWARD;
        //wireless_uart_send_string("exit\r\n");
        canEnd = handle = 0;
        canNotInRamp = 50;
        enableDifSpeed = 1;
        gpio_set_level(IO_P90, 0);
    } else if (!canEnd) {
        handle = 1;
        //wireless_uart_send_string("ramp\n");
        setSpeed = 200;
        enableDifSpeed = 0;
    }
    centerLineCount = func_min(LRoadCount, RRoadCount);
    for (i = 0; i < centerLineCount; i++) {
        centerLine[i].x = LRoad[i].x + RRoad[i].x >> 1;
        centerLine[i].y = LRoad[i].y + RRoad[i].y >> 1;
    }
    if (func_abs(pitch) > 3) alongLine(centerLine, centerLineCount, 10);
}
