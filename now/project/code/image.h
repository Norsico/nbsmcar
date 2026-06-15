#ifndef __IMAGE_H__
#define __IMAGE_H__

#include "headfile.h"

#define LCDH                       (60)
#define LCDW                       (80)
#define IMAGE_H                    (LCDH)
#define IMAGE_W                    (LCDW)
#define ImageSensorMid             (39)
#define IMAGE_MID                  (ImageSensorMid)
#define IMAGE_BLACK                (0)
#define IMAGE_WHITE                (1)

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
} ImageStatustypedef;

typedef struct
{
    int16 image_element_rings;
    int16 ring_big_small;
    int16 image_element_rings_flag;
    int16 straight_long;
} ImageFlagtypedef;

typedef struct
{
    uint8 ready;              /* camera init ok */
    uint8 result_ready;       /* this frame can be used by servo */
    uint16 sequence;          /* processed frame counter */
    uint8 threshold;          /* final binary threshold */
    uint16 white_count;       /* white pixel count in ImageBin */
    uint8 tow_row;            /* actual row used as tow point */
    int16 center;             /* weighted center column */
    int16 error;              /* center - IMAGE_MID */
    uint8 valid_count;        /* valid rows from OFFLine to bottom */
    uint8 lost;               /* no reliable track */
    int16 left_ring_right_deviation_x10; /* right-side max line deviation x10 */
    int16 right_ring_left_deviation_x10; /* left-side max line deviation x10 */
    uint8 ring;               /* 0 none, 1 left ring, 2 right ring */
    uint8 ring_step;          /* ring process stage */
    uint8 zebra;              /* zebra line hit in current frame */
    uint8 zebra_count;        /* confirmed zebra hit count */
} image_data;

extern image_data Image;
extern uint8 ImageGray[IMAGE_H][IMAGE_W];
extern uint8 ImageBin[IMAGE_H][IMAGE_W];

#define Image_Use                  ImageGray
#define Pixle                      ImageBin

void image_init(void);
void image_apply_camera(void);
void image_update(void);
void image_show_debug_overlay(uint16 x, uint16 y, uint16 w, uint16 h);

#endif
