//
// Created by 31663 on 2026/3/29.
//

#include "Cross.h"

#define CROSS_READY 1
#define CROSS_IN 2
extern uint8 outputImage[MT9V03X_H][MT9V03X_W], switch2;
extern int16 centerLineCount, RUN_MODE;
extern Point centerLine[expRoadCount];

static int8 CROSS_MODE = CROSS_READY, canEnd = 0;
static int16 alongR, crossThreshold, minMaxH;

Point getCrossRStart() {
    Point pos;
    int16 min = 187, i;
    if (RBorderCount >= 40 || !minMaxH)
        for (i = 0; i < RBorderCount; i += 2) {
            if (RBorder[i].y <= min) min = RBorder[i].y;
            else {
                pos.x = RBorder[i].x - 4;
                pos.y = minMaxH = RBorder[i].y;
                break;
            }
        }
    else {
        pos.x = 80;
        pos.y = minMaxH;
    }
    while (pos.x >= 8 && (getImagePointFilterSelf(pos.x, pos.y) > crossThreshold || getImagePointFilterSelf(pos.x - 8, pos.y) > crossThreshold)) pos.x--;
    if (pos.x <= 7)
        pos.x = pos.y = 0;
    else
        pos.x++;
    return pos;
}

Point getCrossLStart() {
    Point pos;
    int16 max = 0, i;
    if (LBorderCount >= 40 || !minMaxH)
        for (i = 0; i < LBorderCount; i += 2) {
            if (LBorder[i].y >= max) max = LBorder[i].y;
            else {
                pos.x = LBorder[i].x - 4;
                pos.y = minMaxH = LBorder[i].y;
                break;
            }
        }
    else {
        pos.x = 80;
        pos.y = minMaxH;
    }
    while (pos.x >= 8 && (getImagePointFilterSelf(pos.x, pos.y) > crossThreshold || getImagePointFilterSelf(pos.x - 8, pos.y) > crossThreshold)) pos.x--;
    if (pos.x <= 7)
        pos.x = pos.y = 0;
    else
        pos.x++;
    return pos;
}

void runCross() {
    Point start;
#if (DEBUG_MODE)
    if (switch2) {
        ips200_show_int16(0, MT9V03X_H + 1, LBorderCount);
        ips200_show_int16(MT9V03X_W, MT9V03X_H + 1, RBorderCount);
        ips200_show_int16(MT9V03X_W / 2, MT9V03X_H + 1, centerLineCount);
    }
#endif
    if (CROSS_MODE == CROSS_READY) {
        //gpio_set_level(IO_P90, 1);
        if (centerLineCount && centerLineCount < 13) {
            crossThreshold = threshold + mt9v03x_image[80][94] >> 1;
            alongR = LBorderCount < RBorderCount;
            CROSS_MODE = CROSS_IN;
        }
        alongLine(centerLine, centerLineCount, getAimPos());
    } else if (CROSS_MODE == CROSS_IN) {
        Point border[expRoadCount], farCenterLine[expRoadCount];
        int16 roadCount = expRoadCount, farCenterLineCount, corner, i, j;
        if (!LBorderCount && !RBorderCount)
            canEnd = 1;
        banUpdateThreshold = 1;
        if (alongR) {
            start = getCrossRStart();
            getRBorder(border, start, &roadCount, 0);
        } else {
            start = getCrossLStart();
            getLBorder(border, start, &roadCount, 0);
        }
        getGodRoadLine(border, roadCount);
        resampleRoad(border, &roadCount, 0, 3);
        corner = findCorner(border, roadCount, 1);
        filterRoad(border, roadCount);
        farCenterLineCount = roadCount;
        if (alongR)
            getCenterLineByRBorder(border, farCenterLine, farCenterLineCount);
        else getCenterLineByLBorder(border, farCenterLine, farCenterLineCount);
        if (corner && farCenterLine[corner].x > 50) corner = func_min(corner + 3, farCenterLineCount);
        else corner = 5;
        farCenterLine[corner].x = MT9V03X_H - 1;
        farCenterLine[corner].y = MT9V03X_W / 2 - 3;
        resampleRoad(farCenterLine, &farCenterLineCount, corner, 2);
        filterRoad(farCenterLine, farCenterLineCount);
        alongLine(farCenterLine, farCenterLineCount, getAimPos());
        if (centerLineCount > 20 && canEnd) {
            canEnd = minMaxH = 0;
            CROSS_MODE = CROSS_READY;
            RUN_MODE = FORWARD;
            gpio_set_level(IO_P90, 0);
        }
#if (DEBUG_MODE)
        if (switch2) {
            ips200_displayimage03x(mt9v03x_image, MT9V03X_W, MT9V03X_H, 0, 0);
            ips200_draw_point(start.y, start.x, RGB565_RED);
            for (i = 0; i < MT9V03X_H; i++)
                for (j = 0; j < MT9V03X_W; j++)
                    outputImage[i][j] = 0;
            for (i = 0; i < farCenterLineCount; i++)
                outputImage[func_limit_ab(farCenterLine[i].x, 0, 119)][func_limit_ab(farCenterLine[i].y, 0, 187)] = 255;
            for (i = 0; i < roadCount; i++)
                outputImage[func_limit_ab(border[i].x, 0, 119)][func_limit_ab(border[i].y, 0, 187)] = 255;
            ips200_displayimage03x(outputImage, MT9V03X_W, MT9V03X_H, 0, MT9V03X_H + 40);
        }
#endif
    }
}