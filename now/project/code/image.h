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
    ROAD_RIGHT_RING,
    ROAD_RAMP              /* 坡道 */
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
    uint8 ready;              /* 摄像头初始化完成 */
    uint8 result_ready;       /* 本帧结果可供舵机使用 */
    volatile uint16 sequence; /* 已处理帧计数器 */
    uint8 threshold;          /* 最终二值化阈值 */
    uint16 white_count;       /* 二值图中白点数量 */
    uint8 tow_row;            /* 实际使用的前瞻点行号 */
    int16 center;             /* 加权中心列坐标 */
    int16 error;              /* center - IMAGE_MID */
    uint8 valid_count;        /* 从截止行到底部的有效行数 */
    uint8 lost;               /* 丢线标志 */
    int16 left_ring_right_deviation_x10; /* 左环岛右边线最大偏差 x10 */
    int16 right_ring_left_deviation_x10; /* 右环岛左边线最大偏差 x10 */
    uint8 ring;               /* 0=无环岛, 1=左环岛, 2=右环岛 */
    uint8 ring_step;          /* 环岛处理阶段 */
    uint8 zebra;              /* 本帧检测到斑马线 */
    uint8 zebra_count;        /* 确认检测到的斑马线次数 */
    uint8 is_straight;        /* 直道检测标志（连续2帧确认，含十字直行特征）*/
    int16 straight_left_error_x10;  /* 左边线拟合误差 x10 */
    int16 straight_right_error_x10; /* 右边线拟合误差 x10 */
    uint8 is_ramp;            /* 坡道检测标志 */
    uint8 ramp_count;         /* 检测到的坡道次数 */
} image_data;

extern image_data Image;
extern ImageStatustypedef ImageStatus;
extern ImageDealDatatypedef ImageDeal[IMAGE_H];
extern uint8 ImageGray[IMAGE_H][IMAGE_W];
extern uint8 ImageBin[IMAGE_H][IMAGE_W];

#define Image_Use                  ImageGray
#define Pixle                      ImageBin

void image_init(void);
void image_apply_camera(void);
void image_update(void);
void image_update_laser_test(void);
void image_show_debug_overlay(uint16 x, uint16 y, uint16 w, uint16 h);

#endif
