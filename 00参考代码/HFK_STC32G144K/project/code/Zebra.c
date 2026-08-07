//
// Created by 31663 on 2026/4/1.
//

#include "Zebra.h"

extern Point centerLine[expRoadCount];
extern int16 centerLineCount, RUN_MODE, LCorner, RCorner;
static uint8 count, add = 1;
extern uint8 outputImage[MT9V03X_H][MT9V03X_W];

uint8 haveLine() {
    int16 i = 0;
    while (i < LBorderCount && LBorder[i].x > ZEBRA_TRIGGER_POS) {
        if (findZebra(i)) return 1;
        i += 5;
    }
    return !LBorderCount || LBorder[i].x > ZEBRA_TRIGGER_POS;
}

void runZebra(uint8 have) {
    centerLineDistance = 18;
    alongLine(centerLine, centerLineCount, getAimPos());
    if (count == runTurn + 1) {
        end:
        handle = 1;
        setSpeed = 400;
        system_delay_ms(130);
        stop = 1;
    } else {
        if (add) {
            count++;
            if (count == runTurn + 1) goto end;
            add = 0;
        }
        if (LBorderCount && !have && !haveLine()) {
            RUN_MODE = FORWARD;
            add = 1;
        }
    }
}
