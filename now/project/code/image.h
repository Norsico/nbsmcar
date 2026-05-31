#ifndef _IMAGE_H_
#define _IMAGE_H_

#include "headfile.h"

#define LCDH                          (60)
#define LCDW                          (80)
#define IMAGE_H                       (LCDH)
#define IMAGE_W                       (LCDW)
#define ImageSensorMid                (39)
#define IMAGE_MID                     (ImageSensorMid)
#define IMAGE_BLACK                   (0)
#define IMAGE_WHITE                   (1)

typedef struct
{
    int point;
    uint8 type;
} JumpPointtypedef;

typedef struct
{
    uint8 IsRightFind;
    uint8 IsLeftFind;
    int Wide;
    int LeftBorder;
    int RightBorder;
    int close_LeftBorder;
    int close_RightBorder;
    int Center;
    int RightTemp;
    int LeftTemp;
    int LeftBoundary_First;
    int RightBoundary_First;
    int LeftBoundary;
    int RightBoundary;
} ImageDealDatatypedef;

typedef enum
{
    ROAD_NORMAL = 0,
    ROAD_STRAIGHT,
    ROAD_LEFT_RING,
    ROAD_RIGHT_RING
} RoadType_e;

typedef struct
{
    uint8 TowPoint;
    int TowPoint_True;
    int Det_True;
    uint8 Threshold;
    uint16 Threshold_static;
    uint8 Threshold_detach;
    uint8 Left_Line;
    uint8 Right_Line;
    uint8 OFFLine;
    uint8 WhiteLine;
    RoadType_e Road_type;
    int16 WhiteLine_L;
    int16 WhiteLine_R;
    int16 OFFLineBoundary;
    int straight_acc;
    int variance_acc;
} ImageStatustypedef;

typedef struct
{
    int16 image_element_rings;
    int16 ring_big_small;
    int16 image_element_rings_flag;
    int16 straight_long;
} ImageFlagtypedef;

extern uint8 Image_Use[LCDH][LCDW];
extern uint8 Pixle[LCDH][LCDW];
extern ImageStatustypedef ImageStatus;
extern ImageFlagtypedef ImageFlag;
extern ImageDealDatatypedef ImageDeal[LCDH];

void image_init(void);
void image_apply_camera(void);
void image_update(void);
uint8 image_is_ready(void);
uint8 image_is_result_ready(void);
uint32 image_get_result_sequence(void);
void image_show_debug_overlay(uint16 x, uint16 y, uint16 w, uint16 h);

#endif
