//
// Created by 31663 on 2026/3/16.
//

#ifndef SEEKFREE_STC32G144K_100PIN_OPENSOURCE_LIBRARY_IMAGE_H
#define SEEKFREE_STC32G144K_100PIN_OPENSOURCE_LIBRARY_IMAGE_H
#include "zf_common_headfile.h"

#define block 5               //局域二值化边长
#define cap 6                //去噪   5
#define expRoadCount 100      //边线期望长度
#define ZEBRA_TRIGGER_POS 40
#define targetPos 78

void getLBorder(Point LRoad[], Point start, int16 *count, int8 delicate);
void getRBorder(Point RRoad[], Point start, int16 *count, int8 delicate);
void getCenterLineByLBorder(Point roadLine[], Point centerLine[], int16 count);
void getCenterLineByRBorder(Point roadLine[], Point centerLine[], int16 count);
void filterRoad(Point rodeLine[], int16 count);
void getGodRoadLine(Point roadLine[], int16 count);
void resampleRoad(Point roadLine[], int16 *count, int16 startPos, uint8 dis);
Point getLStart();
Point getRStart();
int16 findCorner(Point roadLine[], int16 count, int8 cross);
uint8 findZebra(int16 pos);
Point findTargetByL(int16 i);
extern const uint8 borderOffset[120];
uint8 getImagePointFilterSelf(uint8 i, uint8 j);
extern uint8 threshold, centerLineDistance, banUpdateThreshold;
extern int16 LBorderCount, RBorderCount;
extern Point LBorder[expRoadCount], RBorder[expRoadCount];
void displayBinarizationImage();
void displayBorderLineImage();
void displayGodImage();
void displayGodRoadLineImage();

#endif //SEEKFREE_STC32G144K_100PIN_OPENSOURCE_LIBRARY_IMAGE_H
