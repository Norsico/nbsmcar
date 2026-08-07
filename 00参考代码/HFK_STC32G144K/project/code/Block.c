//
// Created by 31663 on 2026/4/10.
//

#include "Block.h"
static uint8 added = 0;
extern uint8 ALONG;
static int16 fixedLCorner;
extern int16 RUN_MODE, centerLineCount;
extern Point centerLine[expRoadCount];

uint8 canEnd() {
    int16 i;
    if (fixedLCorner) {
        for (i = 0; i < RBorderCount && RBorder[i].x > 25; i += 5)
            if (mt9v03x_image[RBorder[i].x][RBorder[i].y - borderOffset[RBorder[i].x]] < blockThreshold) return 0;
    } else {
        for (i = 0; i < LBorderCount && LBorder[i].x > 25; i += 5)
            if (mt9v03x_image[LBorder[i].x][LBorder[i].y + borderOffset[LBorder[i].x]] < blockThreshold) return 0;
    }
    return 1;
}

void runBlock(int16 LCorner, int16 RCorner) {
    gpio_set_level(IO_P90, 1);
    if (!added) {
        fixedLCorner = (LCorner && !RCorner) || (LCorner < RCorner && LCorner && RCorner);
        added = 1;
    }
    ALONG = fixedLCorner ? ALONG_R : ALONG_L;
    alongLine(centerLine, centerLineCount, 15);
    if (canEnd()) {
        added = 0;
        gpio_set_level(IO_P90, 0);
        RUN_MODE = FORWARD;
        centerLineDistance = 18;
    }
}