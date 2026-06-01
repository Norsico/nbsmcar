#ifndef _IMAGE_H_
#define _IMAGE_H_

#include "headfile.h"

#define LCDH                          (60)                   /* 图像高 */
#define LCDW                          (80)                   /* 图像宽 */
#define IMAGE_H                       (LCDH)
#define IMAGE_W                       (LCDW)
#define ImageSensorMid                (39)                   /* 图像中点 */
#define IMAGE_MID                     (ImageSensorMid)
#define IMAGE_BLACK                   (0)
#define IMAGE_WHITE                   (1)
#define LimitL(L)                     (L = ((L < 1) ? 1 : L))
#define LimitH(H)                     (H = ((H > 78) ? 78 : H))

#define IMAGE_COMPRESS_CUT_COL        (1)                    /* 压缩裁边 */
#define IMAGE_COMPRESS_CUT_ROW_TOP    (0)                    /* 压缩裁顶 */
#define IMAGE_COMPRESS_CUT_ROW_BOTTOM (10)                   /* 压缩裁底 */
#define IMAGE_COMPRESS_SRC_H          (MT9V03X_H - IMAGE_COMPRESS_CUT_ROW_TOP - IMAGE_COMPRESS_CUT_ROW_BOTTOM)
#define IMAGE_COMPRESS_SRC_W          (MT9V03X_W - (IMAGE_COMPRESS_CUT_COL * 2))

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
    Normol = 0,
    Straight,
    Cross,
    Ramp,
    LeftCirque,
    RightCirque,
    Forkin,
    Forkout,
    Barn_out,
    Barn_in,
    Cross_ture,
    Zebra_Flag
} RoadType_e;

typedef struct
{
    uint8 TowPoint;
    int TowPoint_True;
    int Det_True;
    uint8 Threshold;
    uint32 Threshold_static;
    uint8 Threshold_detach;
    uint8 Left_Line;
    uint8 Right_Line;
    uint8 OFFLine;
    uint8 WhiteLine;
    RoadType_e Road_type;
    uint8 IsCinqueOutIn;
    uint8 CirquePass;
    uint8 CirqueOut;
    uint8 CirqueOff;
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

typedef struct
{
    uint8 ready;
    uint8 result_ready;
    uint16 sequence;
    uint8 threshold;
    uint16 white_count;
    uint8 tow_row;
    int16 center;
    int16 error;
    uint8 valid_count;
    uint8 lost;
    uint8 ring;
    uint8 ring_step;
    uint8 zebra;
    uint8 zebra_count;
} image_data;

extern image_data Image;
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
uint16 image_get_speed_goal(void);
void image_show_debug_overlay(uint16 x, uint16 y, uint16 w, uint16 h);

#endif
