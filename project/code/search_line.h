#ifndef _SEARCH_LINE_H_
#define _SEARCH_LINE_H_

#include "zf_common_headfile.h"

#define LCDH 60
#define LCDW 80
#define LimitL(L) (L = ((L < 1) ? 1 : L))    //限制幅度
#define LimitH(H) (H = ((H > 78) ? 78 : H))  //限制幅度
#define ImageSensorMid 39                    //图像屏幕中点

typedef struct
{
    int point;
    uint8 type;
} JumpPointtypedef;

typedef struct
{
    uint8 IsRightFind;          /* 右边界标志。 */
    uint8 IsLeftFind;           /* 左边界标志。 */
    int Wide;                   /* 当前行边界宽度。 */
    int LeftBorder;             /* 左边界。 */
    int RightBorder;            /* 右边界。 */
    int close_LeftBorder;       /* 靠边左边界。 */
    int close_RightBorder;      /* 靠边右边界。 */
    int Center;                 /* 中线。 */
    int RightTemp;              /* 右边临时值。 */
    int LeftTemp;               /* 左边临时值。 */
    int LeftBoundary_First;     /* 左边界第一次八邻域结果。 */
    int RightBoundary_First;    /* 右边界第一次八邻域结果。 */
    int LeftBoundary;           /* 左边界最终八邻域结果。 */
    int RightBoundary;          /* 右边界最终八邻域结果。 */
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
    uint8 TowPoint;             /* 给定前瞻行。 */
    int TowPoint_True;          /* 当前实际前瞻行。 */
    int Det_True;               /* 当前加权偏差。 */
    uint8 Threshold;            /* 当前工作阈值。 */
    uint32 Threshold_static;    /* 二值化静态下限。 */
    uint8 Threshold_detach;     /* 大津法截断上限。 */
    uint8 Left_Line;            /* 左边丢线数。 */
    uint8 Right_Line;           /* 右边丢线数。 */
    uint8 OFFLine;              /* 图像顶边。 */
    uint8 WhiteLine;            /* 双边丢线数。 */
    RoadType_e Road_type;       /* 元素类型。 */
    uint8 IsCinqueOutIn;        /* 进出圆环标志。 */
    uint8 CirquePass;           /* 圆环中标志。 */
    uint8 CirqueOut;            /* 出圆环标志。 */
    uint8 CirqueOff;            /* 圆环结束标志。 */
    int16 WhiteLine_L;          /* 左边丢线数。 */
    int16 WhiteLine_R;          /* 右边丢线数。 */
    int16 OFFLineBoundary;      /* 八邻域截止行。 */
    int straight_acc;           /* 直道标志位。 */
    int variance_acc;           /* 直道方差累计。 */
} ImageStatustypedef;

typedef struct
{
    int16 image_element_rings;       /* 0:无圆环 1:左圆环 2:右圆环 */
    int16 ring_big_small;            /* 0:无 1:大圆环 2:小圆环 */
    int16 image_element_rings_flag;  /* 圆环进程 */
    int16 straight_long;             /* 长直道标志位 */
} ImageFlagtypedef;

extern uint8 Image_Use[LCDH][LCDW];
extern uint8 Pixle[LCDH][LCDW];
extern ImageStatustypedef ImageStatus;
extern ImageFlagtypedef ImageFlag;
extern ImageDealDatatypedef ImageDeal[LCDH];
extern int ImageScanInterval;        //扫边范围    上一行的边界+-ImageScanInterval
extern int ImageScanInterval_Cross;  //十字扫线范围

void compressimage(void);
void Get01change_dajin(void);
void ImageProcess(void);
void SearchLine_Process(void);
void SearchLine_DrawBinaryPreview(void);
void SearchLine_DrawRawPreview(void);
void SearchLine_ResetPreviewOverlay(void);
uint8 SearchLine_GetOtsuThreshold(void);
uint8 SearchLine_GetStraightAcc(void);
uint8 SearchLine_GetDetTrue(void);
uint8 SearchLine_GetLeftLine(void);
uint8 SearchLine_GetRightLine(void);

#endif
