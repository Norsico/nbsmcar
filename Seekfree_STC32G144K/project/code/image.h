#ifndef _IMAGE_H_
#define _IMAGE_H_

#include "flash.h"
#include "zf_common_headfile.h"

#define IMAGE_BUZZER_PIN              (IO_P96)                /* 蜂鸣器 */
#define IMAGE_BUZZER_PIT              (TIM7_PIT)             /* 蜂鸣器定时器 */
#define IMAGE_BUZZER_PERIOD_MS        (20)                   /* 蜂鸣器周期 */
#define IMAGE_BUZZER_PRIORITY         (0)                    /* 蜂鸣器优先级 */
#define IMAGE_BUZZER_SHORT_MS         (100)                  /* 短响时间 */

#define IMAGE_STOP_RAW_THRESHOLD      (25)                   /* 阈值停车线 */
#define IMAGE_ZEBRA_MISS_COUNT        (3)                    /* 斑马释放帧数 */
#define IMAGE_ZEBRA_COOLDOWN_FRAMES   (80)                   /* 斑马冷却帧数 */
#define IMAGE_OUTTRACK_BLACK_PERCENT  (90)                   /* 黑点占比线 */
#define IMAGE_OUTTRACK_CONFIRM_COUNT  (10)                   /* 出界确认帧数 */
#define IMAGE_OUTTRACK_SAMPLE_ROWS    (2)                    /* 出界采样行数 */

#define LCDH                          (60)                   /* 图像高 */
#define LCDW                          (80)                   /* 图像宽 */
#define ImageSensorMid                (39)                   /* 图像中点 */
#define LimitL(L)                     (L = ((L < 1) ? 1 : L))   /* 下限 */
#define LimitH(H)                     (H = ((H > 78) ? 78 : H)) /* 上限 */

#define IMAGE_COMPRESS_CUT_COL        (1)                    /* 压缩裁边 */
#define IMAGE_COMPRESS_CUT_ROW_TOP    (0)                    /* 压缩裁顶 */
#define IMAGE_COMPRESS_CUT_ROW_BOTTOM (10)                   /* 压缩裁底 */
#define IMAGE_COMPRESS_SRC_H          (MT9V03X_H - IMAGE_COMPRESS_CUT_ROW_TOP - IMAGE_COMPRESS_CUT_ROW_BOTTOM) /* 压缩源高 */
#define IMAGE_COMPRESS_SRC_W          (MT9V03X_W - (IMAGE_COMPRESS_CUT_COL * 2)) /* 压缩源宽 */

typedef struct
{
    int point;                                              /* 点位 */
    uint8 type;                                             /* 类型 */
} JumpPointtypedef;

typedef struct
{
    uint8 IsRightFind;                                      /* 右边界标志 */
    uint8 IsLeftFind;                                       /* 左边界标志 */
    int Wide;                                               /* 当前行宽度 */
    int LeftBorder;                                         /* 左边界 */
    int RightBorder;                                        /* 右边界 */
    int close_LeftBorder;                                   /* 靠边左边界 */
    int close_RightBorder;                                  /* 靠边右边界 */
    int Center;                                             /* 中线 */
    int RightTemp;                                          /* 右边临时值 */
    int LeftTemp;                                           /* 左边临时值 */
    int LeftBoundary_First;                                 /* 左边界第一次结果 */
    int RightBoundary_First;                                /* 右边界第一次结果 */
    int LeftBoundary;                                       /* 左边界最终结果 */
    int RightBoundary;                                      /* 右边界最终结果 */
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
    uint8 TowPoint;                                         /* 给定前瞻 */
    int TowPoint_True;                                      /* 当前前瞻 */
    int Det_True;                                           /* 当前偏差 */
    uint8 Threshold;                                        /* 当前阈值 */
    uint32 Threshold_static;                                /* 静态阈值 */
    uint8 Threshold_detach;                                 /* 截断上限 */
    uint8 Left_Line;                                        /* 左丢线 */
    uint8 Right_Line;                                       /* 右丢线 */
    uint8 OFFLine;                                          /* 顶边 */
    uint8 WhiteLine;                                        /* 双边丢线 */
    RoadType_e Road_type;                                   /* 元素类型 */
    uint8 IsCinqueOutIn;                                    /* 进出圆环 */
    uint8 CirquePass;                                       /* 圆环中 */
    uint8 CirqueOut;                                        /* 出圆环 */
    uint8 CirqueOff;                                        /* 圆环结束 */
    int16 WhiteLine_L;                                      /* 左白线 */
    int16 WhiteLine_R;                                      /* 右白线 */
    int16 OFFLineBoundary;                                  /* 截止行 */
    int straight_acc;                                       /* 直道标志 */
    int variance_acc;                                       /* 方差累计 */
} ImageStatustypedef;

typedef struct
{
    int16 image_element_rings;                              /* 圆环元素 */
    int16 ring_big_small;                                   /* 大小圆环 */
    int16 image_element_rings_flag;                         /* 圆环进程 */
    int16 straight_long;                                    /* 长直道 */
} ImageFlagtypedef;

extern uint8 Image_Use[LCDH][LCDW];
extern uint8 Pixle[LCDH][LCDW];
extern ImageStatustypedef ImageStatus;
extern ImageFlagtypedef ImageFlag;
extern ImageDealDatatypedef ImageDeal[LCDH];

void image_init(void);
void image_update(void);
void image_buzzer_update(void);
uint8 image_is_ready(void);
uint8 image_is_result_ready(void);
uint8 image_set_camera_value(flash_camera_slot_t slot, int16 value);
void image_reload_camera_page(void);
const uint8 *image_get_raw_buffer(void);
const uint8 *image_get_binary_buffer(void);

#endif
