#include "headfile.h"

/* =============================================================================
 * 全局图像数据
 * =============================================================================
 * 图像处理流程：
 *   摄像头原始图 -> ImageGray (80x60灰度图) -> ImageBin (二值图)
 *                -> ImageDeal (边线提取) -> Image (最终结果)
 *
 * 本模块功能：
 *   - 基础巡线和边线检测
 *   - 十字线填补和路径滤波
 *   - 环岛识别和处理
 *   - 斑马线检测
 *
 * 不包含：
 *   - 坡道检测（单独处理）
 *   - 目标环岛激光逻辑
 * ============================================================================= */

image_data Image;
uint8 ImageGray[IMAGE_H][IMAGE_W];
uint8 ImageBin[IMAGE_H][IMAGE_W];

/* 图像压缩设置 */
#define IMAGE_COMPRESS_CUT_COL         (1)     /* 左右各裁剪1列 */
#define IMAGE_COMPRESS_CUT_ROW_TOP     (0)     /* 上方裁剪行数 */
#define IMAGE_COMPRESS_CUT_ROW_BOTTOM  (1)     /* 下方裁剪1行 */
#define IMAGE_COMPRESS_SRC_H           (MT9V03X_H - IMAGE_COMPRESS_CUT_ROW_TOP - IMAGE_COMPRESS_CUT_ROW_BOTTOM)
#define IMAGE_COMPRESS_SRC_W           (MT9V03X_W - (IMAGE_COMPRESS_CUT_COL * 2))

/* 阈值和检测参数 */
#define IMAGE_THRESHOLD_DETACH         (150)   /* 大津法扫描上限（防止过亮区域干扰） */
#define IMAGE_THRESHOLD_STATIC         (40)    /* 二值化阈值下限（保证最小对比度） */
#define IMAGE_STOP_RAW_THRESHOLD       (25)    /* 原始阈值低于此值判定为丢线 */
#define IMAGE_OFFLINE_INIT             (2)     /* 初始有效行起始位置 */
#define IMAGE_SCAN_INTERVAL            (3)     /* 边线搜索时上下行的搜索范围 */

/* 斑马线检测参数 */
#define IMAGE_ZEBRA_MISS_COUNT         (3)     /* 斑马线消失多少帧后解锁 */
#define IMAGE_ZEBRA_COOLDOWN_FRAMES    (80)    /* 两次斑马线检测之间的冷却帧数 */
#define IMAGE_ZEBRA_EDGE_MIN           (4)     /* 确认斑马线需要的最少黑白跳变次数 */
#define IMAGE_ZEBRA_STOP_COUNT         (2)     /* 检测到几次斑马线后停车 */

/* 运行时安全参数 */
#define IMAGE_LOST_STOP_COUNT          (4)     /* 连续丢线多少帧后停车 */
#define IMAGE_RUN_START_IGNORE_FRAMES  (3)     /* 启动后忽略丢线的帧数（避免误判） */

/* 打靶检测参数 */
#define IMAGE_TARGET_LASER_PIT         (TIM0_PIT)
#define IMAGE_TARGET_LASER_IRQ         (TIMER0_IRQn)
#define IMAGE_TARGET_LASER_PRIORITY    (0)
#define IMAGE_TARGET_LASER_PERIOD_US   (500)
#define IMAGE_TARGET_LASER_FIRE_US     (3000)
#define IMAGE_TARGET_FIRE_INTERVAL     (10)
#define IMAGE_TARGET_SCAN_ROWS         (3)
#define IMAGE_LASER_95_MAX_COL         (24)
#define IMAGE_LASER_94_MAX_COL         (34)
#define IMAGE_LASER_92_MAX_COL         (44)
#define IMAGE_LASER_93_MAX_COL         (54)
#define IMAGE_LASER_TEST_OFF           (0)
#define IMAGE_LASER_TEST_ALL           (6)

/* 边界限幅宏（有效列范围 1~78） */
#define LimitL(L)                      (L = ((L < 1) ? 1 : L))
#define LimitH(H)                      (H = ((H > 78) ? 78 : H))
#define IMAGE_ABS(V)                   (((V) < 0) ? (-(V)) : (V))

/* =============================================================================
 * 内部数据结构
 * ============================================================================= */

/* 边线搜索时的跳变点 */
typedef struct
{
    int16 point;      /* 跳变点的列位置 */
    uint8 type;       /* 'T': 找到边线, 'W': 白色/无边线, 'H': 隐藏边线 */
} image_jump;

/* 每一行的边线处理结果 */
typedef struct
{
    uint8 IsLeftFind;              /* 左边线状态: 'T'/'W'/'H'/'F' */
    uint8 IsRightFind;             /* 右边线状态: 'T'/'W'/'H'/'F' */
    int16 LeftBorder;              /* 左边线列坐标 */
    int16 RightBorder;             /* 右边线列坐标 */
    int16 Center;                  /* 中心线列坐标 = (左+右)/2 */
    int16 Wide;                    /* 赛道宽度 = 右-左 */
    int16 LeftTemp;                /* 左边线临时备份 */
    int16 RightTemp;               /* 右边线临时备份 */
    int16 close_LeftBorder;        /* 近距离左边线 */
    int16 close_RightBorder;       /* 近距离右边线 */
    int16 LeftBoundary_First;      /* 边界追踪首次检测到的左边界 */
    int16 RightBoundary_First;     /* 边界追踪首次检测到的右边界 */
    int16 LeftBoundary;            /* 边界追踪最终左边界 */
    int16 RightBoundary;           /* 边界追踪最终右边界 */
} image_deal;

/* 图像处理全局状态 */
typedef struct
{
    uint8 TowPoint;                /* 配置的瞄点行（目标行） */
    int16 TowPoint_True;           /* 实际使用的瞄点（受可见范围约束） */
    int16 Det_True;                /* 加权中心列坐标（用于转向控制） */
    uint8 Threshold;               /* 最终二值化阈值（应用下限后） */
    uint16 Threshold_static;       /* 阈值下限（最小允许值） */
    uint8 Threshold_detach;        /* 大津法上限（扫描上限） */
    uint8 Left_Line;               /* 左侧白色/无边线的行数 */
    uint8 Right_Line;              /* 右侧白色/无边线的行数 */
    uint8 WhiteLine;               /* 两侧都是白色的行数 */
    uint8 OFFLine;                 /* 从上往下第一个可靠的行（地平线） */
    int16 WhiteLine_L;             /* 左侧白线指示 */
    int16 WhiteLine_R;             /* 右侧白线指示 */
    int16 OFFLineBoundary;         /* 边界追踪停止行 */
    RoadType_e Road_type;          /* 当前道路类型分类 */
} image_status;

/* 环岛识别状态机 */
typedef struct
{
    int16 image_element_rings;       /* 0:无环岛, 1:左环岛, 2:右环岛 */
    int16 ring_big_small;            /* 环岛大小标志 */
    int16 image_element_rings_flag;  /* 状态: 1/2=入口, 5/6=内部, 7/8=出口, 9=完成 */
    int16 straight_long;             /* 直道段长度计数 */
} image_flag;

/* =============================================================================
 * 静态变量
 * ============================================================================= */

/* 每一行的边线跟踪结果 */
ImageDealDatatypedef ImageDeal[IMAGE_H];

/* 全局图像处理状态 */
ImageStatustypedef ImageStatus =
{
    SERVO_POINT,              /* TowPoint */
    SERVO_POINT,              /* TowPoint_True */
    IMAGE_MID,                /* Det_True */
    0,                        /* Threshold */
    IMAGE_THRESHOLD_STATIC,   /* Threshold_static */
    IMAGE_THRESHOLD_DETACH,   /* Threshold_detach */
    0,                        /* Left_Line */
    0,                        /* Right_Line */
    0,                        /* WhiteLine */
    IMAGE_OFFLINE_INIT,       /* OFFLine */
    0,                        /* WhiteLine_L */
    0,                        /* WhiteLine_R */
    5,                        /* OFFLineBoundary */
    ROAD_NORMAL               /* Road_type */
};

/* 环岛检测状态机 */
static image_flag ImageFlag = {0};

/* 图像压缩查找表 */
static uint8 ImageRowMap[IMAGE_H];
static uint8 ImageColMap[IMAGE_W];
static uint8 ImageMapReady = 0;

/* 大津法阈值计算 */
static uint16 ImageHist[256];
static uint8 ImageRawThreshold = 0;

/* 斑马线检测 */
static uint8 ZebraHit = 0;
static uint8 ZebraDetectCount = 0;
static uint8 ZebraFrameLatch = 0;
static uint8 ZebraMissFrames = 0;
static uint8 ZebraCooldownFrames = 0;

/* 丢线检测和运行状态 */
static uint8 ImageLostCount = 0;
static uint8 ImageRunFrameCount = 0;

/* 打靶检测和激光控制 */
static uint8 TargetFound = 0;
static uint8 TargetFrameGap = IMAGE_TARGET_FIRE_INTERVAL;
static uint8 TargetCenterX = IMAGE_MID;
static uint8 TargetCenterY = 0;
static uint8 TargetLeftX = 0;
static uint8 TargetRightX = 0;
static uint8 TargetTopY = 0;
static uint8 TargetBottomY = 0;
static uint8 LaserBusy = 0;
static uint8 LaserTestLast = 0;
static uint8 LaserPitInit = 0;
static uint8 LaserTickLeft = 0;

static void image_target_laser_pit_handler(void);

/* 图像处理工作变量 */
static int16 Ysite = 0;
static int16 Xsite = 0;
static uint8 *PicTemp = 0;
static int16 IntervalLow = 0;
static int16 IntervalHigh = 0;
static int16 BottomBorderRight = 79;
static int16 BottomBorderLeft = 0;
static int16 BottomCenter = IMAGE_MID;
static uint8 ExtenLFlag = 0;
static uint8 ExtenRFlag = 0;
static int16 ytemp = 0;
static int16 TFSite = 0;
static int16 FTSite = 0;
static float DetR = 0.0f;
static float DetL = 0.0f;

/* 环岛检测工作变量 */
static uint8 Ring_Help_Flag = 0;
static int16 Left_RingsFlag_Point1_Ysite = 0;
static int16 Left_RingsFlag_Point2_Ysite = 0;
static int16 Left_Ring_Right_Deviation_X10 = 0;
static int16 Right_RingsFlag_Point1_Ysite = 0;
static int16 Right_RingsFlag_Point2_Ysite = 0;
static int16 Right_Ring_Left_Deviation_X10 = 0;
static int16 Point_Xsite = 0;
static int16 Point_Ysite = 0;
static int16 Repair_Point_Xsite = 0;
static int16 Repair_Point_Ysite = 0;

static void image_laser_all_off(void)
{
    gpio_set_level(LASER_LEFT_2, GPIO_LOW);
    gpio_set_level(LASER_LEFT_1, GPIO_LOW);
    gpio_set_level(LASER_CENTER, GPIO_LOW);
    gpio_set_level(LASER_RIGHT_1, GPIO_LOW);
    gpio_set_level(LASER_RIGHT_2, GPIO_LOW);
}

static void image_laser_all_on(void)
{
    gpio_set_level(LASER_LEFT_2, GPIO_HIGH);
    gpio_set_level(LASER_LEFT_1, GPIO_HIGH);
    gpio_set_level(LASER_CENTER, GPIO_HIGH);
    gpio_set_level(LASER_RIGHT_1, GPIO_HIGH);
    gpio_set_level(LASER_RIGHT_2, GPIO_HIGH);
}

typedef struct
{
    uint8 max_col;
    gpio_pin_enum pin;
} image_laser_pick_map_t;

static const image_laser_pick_map_t ImageLaserPickMap[] =
{
    {IMAGE_LASER_95_MAX_COL, LASER_LEFT_2},
    {IMAGE_LASER_94_MAX_COL, LASER_LEFT_1},
    {IMAGE_LASER_92_MAX_COL, LASER_CENTER},
    {IMAGE_LASER_93_MAX_COL, LASER_RIGHT_1},
};

static gpio_pin_enum image_laser_pick_pin(uint8 center_x)
{
    uint8 i;

    for(i = 0; i < (uint8)(sizeof(ImageLaserPickMap) / sizeof(ImageLaserPickMap[0])); i++)
    {
        if(center_x <= ImageLaserPickMap[i].max_col)
        {
            return ImageLaserPickMap[i].pin;
        }
    }

    return LASER_RIGHT_2;
}

static uint8 image_target_laser_test_mode(void)
{
    uint8 mode;

    if(ui_is_laser_test_active() == 0)
    {
        return IMAGE_LASER_TEST_OFF;
    }

    mode = SmartCar.camera.laser_test;
    if(mode > IMAGE_LASER_TEST_ALL)
    {
        mode = IMAGE_LASER_TEST_ALL;
    }

    return mode;
}

static void image_laser_apply_test_mode(uint8 mode)
{
    static const gpio_pin_enum TestPins[5] =
    {
        LASER_LEFT_2,
        LASER_LEFT_1,
        LASER_CENTER,
        LASER_RIGHT_1,
        LASER_RIGHT_2
    };
    uint8 i;

    image_laser_all_off();

    if(mode == IMAGE_LASER_TEST_OFF)
    {
        return;
    }

    if(mode == IMAGE_LASER_TEST_ALL)
    {
        image_laser_all_on();
        return;
    }

    if((mode >= 1) && (mode <= 5))
    {
        for(i = 0; i < 5; i++)
        {
            gpio_set_level(TestPins[i], (i == (uint8)(mode - 1)) ? GPIO_HIGH : GPIO_LOW);
        }
    }
}

static uint8 image_target_normalize_row(int16 row)
{
    if(row < 1)
    {
        return 1;
    }
    if(row > (IMAGE_H - 2))
    {
        return (IMAGE_H - 2);
    }

    return (uint8)row;
}

static uint8 image_target_normalize_gap(int16 gap)
{
    if(gap < 1)
    {
        return 1;
    }
    if(gap > 8)
    {
        return 8;
    }

    return (uint8)gap;
}

static uint8 image_target_normalize_laser_test(uint8 laser_test)
{
    if(laser_test > IMAGE_LASER_TEST_ALL)
    {
        return IMAGE_LASER_TEST_ALL;
    }

    return laser_test;
}

static void image_target_reset_result(void)
{
    TargetFound = 0;
    TargetCenterX = IMAGE_MID;
    TargetCenterY = 0;
    TargetLeftX = 0;
    TargetRightX = IMAGE_W - 1;
    TargetTopY = 0;
    TargetBottomY = 0;
}

static void image_target_laser_pit_stop(void)
{
    TR0 = 0;
    ET0 = 0;
    TIM0_CLEAR_FLAG;
}

static void image_target_laser_pit_start(void)
{
    if(LaserPitInit == 0)
    {
        /* 定时器默认关闭，只在首次打靶时初始化一次。 */
        pit_us_init(IMAGE_TARGET_LASER_PIT, IMAGE_TARGET_LASER_PERIOD_US, image_target_laser_pit_handler);
        interrupt_set_priority(IMAGE_TARGET_LASER_IRQ, IMAGE_TARGET_LASER_PRIORITY);
        LaserPitInit = 1;
    }

    TIM0_CLEAR_FLAG;
    ET0 = 1;
    TR0 = 1;
}

static void image_target_laser_pit_handler(void)
{
    TIM0_CLEAR_FLAG;

    if(LaserBusy == 0)
    {
        image_target_laser_pit_stop();
        return;
    }

    if(LaserTickLeft > 0)
    {
        LaserTickLeft--;
    }

    if(LaserTickLeft == 0)
    {
        image_laser_all_off();
        LaserBusy = 0;
        image_target_laser_pit_stop();
    }
}

static void image_target_laser_start(uint8 center_x)
{
    gpio_pin_enum laser_pin;

    laser_pin = image_laser_pick_pin(center_x);

    interrupt_global_disable();
    image_laser_all_off();
    gpio_set_level(laser_pin, GPIO_HIGH);
    /* 3ms 开火时长换算成 0.5ms 的定时器节拍数。 */
    LaserTickLeft = (uint8)((IMAGE_TARGET_LASER_FIRE_US + IMAGE_TARGET_LASER_PERIOD_US - 1) /
                            IMAGE_TARGET_LASER_PERIOD_US);
    LaserBusy = 1;
    image_target_laser_pit_start();
    interrupt_global_enable();
}

static void image_target_update_laser_mode(void)
{
    uint8 laser_test;

    laser_test = image_target_laser_test_mode();
    if(laser_test != LaserTestLast)
    {
        LaserTestLast = laser_test;
        LaserBusy = 0;
        LaserTickLeft = 0;
        image_target_laser_pit_stop();
        image_laser_all_off();
    }

    image_laser_apply_test_mode(laser_test);
}

static uint8 image_target_match_row(uint8 row, uint8 left_x, uint8 right_x)
{
    uint8 col;
    uint8 prev_pixel;
    uint8 transition_count;

    if((left_x >= right_x) || ((right_x - left_x) < 6))
    {
        return 0;
    }

    prev_pixel = ImageBin[row][left_x];
    transition_count = 0;

    for(col = (uint8)(left_x + 1); col <= right_x; col++)
    {
        if(ImageBin[row][col] != prev_pixel)
        {
            transition_count++;
            prev_pixel = ImageBin[row][col];

            if(transition_count >= 4)
            {
                if((ImageBin[row][left_x] == IMAGE_WHITE) && (ImageBin[row][right_x] == IMAGE_WHITE))
                {
                    return 1;
                }
                return 0;
            }
        }
    }

    return 0;
}

static void image_target_check(void)
{
    uint8 base_row;
    uint8 gap;
    uint8 row;
    uint8 hit_count;
    uint8 first_hit_row;
    uint8 last_hit_row;
    int16 scan_row;
    int16 left_x;
    int16 right_x;
    int16 center_x;

    image_target_reset_result();

    if(CarMode != CAR_MODE_RUN)
    {
        return;
    }

    if(ZebraHit)
    {
        return;
    }

    gap = image_target_normalize_gap(SmartCar.camera.target_gap);
    SmartCar.camera.target_gap = gap;
    base_row = image_target_normalize_row(SmartCar.camera.laser_row);
    SmartCar.camera.laser_row = base_row;

    hit_count = 0;
    first_hit_row = base_row;
    last_hit_row = base_row;
    left_x = ImageDeal[base_row].LeftBoundary;
    right_x = ImageDeal[base_row].RightBoundary;

    /* 在基准行及其上方两行内做判定，行距由 UI 的 target gap 控制。 */
    for(row = 0; row < IMAGE_TARGET_SCAN_ROWS; row++)
    {
        scan_row = (int16)base_row - (int16)row * (int16)gap;
        if(scan_row < 1)
        {
            break;
        }

        if(ImageDeal[scan_row].LeftBoundary >= ImageDeal[scan_row].RightBoundary)
        {
            continue;
        }

        if(image_target_match_row((uint8)scan_row,
                                  (uint8)ImageDeal[scan_row].LeftBoundary,
                                  (uint8)ImageDeal[scan_row].RightBoundary))
        {
            if(hit_count == 0)
            {
                first_hit_row = (uint8)scan_row;
                left_x = ImageDeal[scan_row].LeftBoundary;
                right_x = ImageDeal[scan_row].RightBoundary;
            }
            else
            {
                if(ImageDeal[scan_row].LeftBoundary > left_x)
                {
                    left_x = ImageDeal[scan_row].LeftBoundary;
                }
                if(ImageDeal[scan_row].RightBoundary < right_x)
                {
                    right_x = ImageDeal[scan_row].RightBoundary;
                }
            }
            last_hit_row = (uint8)scan_row;
            hit_count++;
        }
    }

    /* 三行里命中两行，就认为靶子成立。 */
    if(hit_count < 2)
    {
        if(TargetFrameGap < IMAGE_TARGET_FIRE_INTERVAL)
        {
            TargetFrameGap++;
        }
        return;
    }

    if(left_x >= right_x)
    {
        return;
    }

    center_x = (left_x + right_x) / 2;

    TargetFound = 1;
    TargetCenterX = (uint8)center_x;
    TargetCenterY = (uint8)((first_hit_row + last_hit_row) / 2);
    TargetLeftX = (uint8)left_x;
    TargetRightX = (uint8)right_x;
    TargetTopY = last_hit_row;
    TargetBottomY = first_hit_row;

    if(image_target_laser_test_mode() != IMAGE_LASER_TEST_OFF)
    {
        return;
    }

    if(LaserBusy)
    {
        return;
    }

    /* 同一个靶子触发后，至少间隔 10 帧再允许下一次开火。 */
    if(TargetFrameGap < IMAGE_TARGET_FIRE_INTERVAL)
    {
        TargetFrameGap++;
        return;
    }

    image_target_laser_start(TargetCenterX);
    buzzer_short();
    TargetFrameGap = 0;
}
/* =============================================================================
 * 查找表
 * ============================================================================= */

/**
 * Half_Road_Wide[row]: 每一行赛道的预期半宽
 * 用途：环岛单边巡线时，根据半宽补全另一侧边线
 * 数值：根据摄像头透视关系经验调试得出
 */
static const uint8 Half_Road_Wide[IMAGE_H] =
{
    6, 7, 7, 8, 8, 9, 9, 9, 11, 11,                       /* 第0-9行 */
    12, 12, 12, 12, 12, 13, 13, 15, 15, 16,               /* 第10-19行 */
    17, 17, 17, 18, 18, 19, 19, 19, 20, 20,               /* 第20-29行 */
    19, 20, 20, 21, 21, 22, 22, 22, 24, 24,               /* 第30-39行 */
    25, 26, 26, 27, 27, 27, 28, 28, 29, 29,               /* 第40-49行 */
    30, 30, 30, 30, 31, 31, 31, 34, 34, 35                /* 第50-59行 */
};

/**
 * WeightingX10[i]: 加权平均中心计算的权重系数（整数版 ×10）
 * 用途：计算转向控制中心时，离瞄点越近的行权重越大
 * 作用：让转向控制更平滑，不会因为单行抖动而跳变
 * @note   原 float 数组 0.96, 0.92, ... ×10 后取整；WeightingSumX10 在 image_init 中计算
 */
static const uint8 WeightingX10[10] =
{
    10, 9, 9, 8, 8, 7, 7, 6, 5, 5
};
static uint16 WeightingSumX10 = 0;     /* 启动时累加得到（= 74） */

/* =============================================================================
 * 内部辅助函数
 * ============================================================================= */

/**
 * @brief  确定实际的瞄点行（用于转向控制的目标行）
 * @return 调整后的瞄点，受可见范围和环岛状态约束
 * @note   环岛入口时固定瞄点，保证控制稳定
 */

static uint8 image_tow_point(void)
{
    int16 tow_point;

    /* 环岛入口阶段固定瞄点 */
    if((ImageFlag.image_element_rings_flag == 1) ||
       (ImageFlag.image_element_rings_flag == 2))
    {
        tow_point = SmartCar.servo.in_ring_point;
    }
    /* 环岛其他阶段使用固定瞄点 */
    else if(ImageFlag.image_element_rings != 0)
    {
        tow_point = SmartCar.servo.out_ring_point;
    }
    /* 正常巡线使用配置的瞄点 */
    else
    {
        tow_point = SmartCar.servo.tow_point;
    }

    /* 瞄点不能超过有效行 */
    if(tow_point < ImageStatus.OFFLine)
    {
        tow_point = ImageStatus.OFFLine + 1;
    }

    /* 限制瞄点范围 */
    if(tow_point >= 49)
    {
        tow_point = 49;
    }
    if(tow_point <= 0)
    {
        tow_point = 1;
    }

    return (uint8)tow_point;
}

/**
 * @brief  清空每行的跟踪结果，准备处理新的一帧
 */
static void image_clear_deal(void)
{
    ImageStatus.OFFLine = IMAGE_OFFLINE_INIT;
    ImageStatus.WhiteLine = 0;
    ImageStatus.WhiteLine_L = 0;
    ImageStatus.WhiteLine_R = 0;
    ImageStatus.OFFLineBoundary = 5;
    ImageStatus.Left_Line = 0;
    ImageStatus.Right_Line = 0;
    ImageStatus.TowPoint = (uint8)SmartCar.servo.tow_point;
    ImageStatus.TowPoint_True = image_tow_point();

    for(Ysite = IMAGE_H - 1; Ysite >= ImageStatus.OFFLine; Ysite--)
    {
        ImageDeal[Ysite].IsLeftFind = 'F';
        ImageDeal[Ysite].IsRightFind = 'F';
        ImageDeal[Ysite].LeftBorder = 0;
        ImageDeal[Ysite].RightBorder = 79;
        ImageDeal[Ysite].LeftTemp = 0;
        ImageDeal[Ysite].RightTemp = 79;
        ImageDeal[Ysite].close_LeftBorder = 0;
        ImageDeal[Ysite].close_RightBorder = 79;
        ImageDeal[Ysite].Center = IMAGE_MID;
        ImageDeal[Ysite].Wide = 79;
        ImageDeal[Ysite].LeftBoundary_First = 0;
        ImageDeal[Ysite].RightBoundary_First = 79;
        ImageDeal[Ysite].LeftBoundary = 0;
        ImageDeal[Ysite].RightBoundary = 79;
    }
}

/**
 * @brief  导出处理结果到公共Image结构体（UI和舵机使用）
 */
static void image_export_result(void)
{
    Image.threshold = ImageStatus.Threshold;
    Image.tow_row = (uint8)ImageStatus.TowPoint_True;
    Image.center = ImageStatus.Det_True;
    Image.error = (int16)(Image.center - IMAGE_MID);
    Image.valid_count = (uint8)((ImageStatus.OFFLine < IMAGE_H) ? (IMAGE_H - ImageStatus.OFFLine) : 0);
    Image.lost = 0;
    Image.left_ring_right_deviation_x10 = Left_Ring_Right_Deviation_X10;
    Image.right_ring_left_deviation_x10 = Right_Ring_Left_Deviation_X10;

    /* 判断丢线：阈值过低 */
    if(ImageRawThreshold < IMAGE_STOP_RAW_THRESHOLD)
    {
        Image.lost = 1;
    }
    /* 判断丢线：有效行过少 */
    if(ImageStatus.OFFLine > 50)
    {
        Image.lost = 1;
    }

    Image.result_ready = Image.lost ? 0 : 1;
    Image.ring = (uint8)ImageFlag.image_element_rings;
    Image.ring_step = (uint8)ImageFlag.image_element_rings_flag;
    Image.zebra = ZebraHit;
    Image.zebra_count = ZebraDetectCount;
}

/**
 * @brief  将80列图像坐标映射到预览区域的X坐标
 */
static uint16 image_debug_x(uint16 x, uint16 w, int16 col)
{
    if(col < 0)
    {
        col = 0;
    }
    if(col >= IMAGE_W)
    {
        col = IMAGE_W - 1;
    }

    return (uint16)(x + (((uint16)col * w) + (IMAGE_W / 2)) / IMAGE_W);
}

/**
 * @brief  将60行图像坐标映射到预览区域的Y坐标
 */
static uint16 image_debug_y(uint16 y, uint16 h, int16 row)
{
    if(row < 0)
    {
        row = 0;
    }
    if(row >= IMAGE_H)
    {
        row = IMAGE_H - 1;
    }

    return (uint16)(y + (((uint16)row * h) + (IMAGE_H / 2)) / IMAGE_H);
}

/**
 * @brief  在UI预览图上绘制调试信息
 * @param  x, y, w, h  预览区域的位置和大小
 * @note   绿色：边线，红色：中线，黄色：瞄点行
 */
void image_show_debug_overlay(uint16 x, uint16 y, uint16 w, uint16 h)
{
    uint8 row;
    uint8 scan_idx;
    uint8 base_row;
    uint8 gap;
    int16 scan_row;
    uint16 draw_x;
    uint16 draw_y;
    uint16 tow_y;

    if((Image.ready == 0) || (Image.sequence == 0))
    {
        return;
    }

    for(row = ImageStatus.OFFLine; row < IMAGE_H; row++)
    {
        draw_y = image_debug_y(y, h, row);

        draw_x = image_debug_x(x, w, ImageDeal[row].LeftBorder);
        ips200_draw_point(draw_x, draw_y, RGB565_GREEN);
        if(draw_y < (uint16)(y + h - 1))
        {
            ips200_draw_point(draw_x, (uint16)(draw_y + 1), RGB565_GREEN);
        }

        draw_x = image_debug_x(x, w, ImageDeal[row].RightBorder);
        ips200_draw_point(draw_x, draw_y, RGB565_GREEN);
        if(draw_y < (uint16)(y + h - 1))
        {
            ips200_draw_point(draw_x, (uint16)(draw_y + 1), RGB565_GREEN);
        }

        draw_x = image_debug_x(x, w, ImageDeal[row].Center);
        ips200_draw_point(draw_x, draw_y, RGB565_RED);
        if(draw_y < (uint16)(y + h - 1))
        {
            ips200_draw_point(draw_x, (uint16)(draw_y + 1), RGB565_RED);
        }
    }

    tow_y = image_debug_y(y, h, ImageStatus.TowPoint_True);
    for(draw_x = x; draw_x < (uint16)(x + w); draw_x++)
    {
        ips200_draw_point(draw_x, tow_y, RGB565_YELLOW);
    }

    base_row = image_target_normalize_row(SmartCar.camera.laser_row);
    gap = image_target_normalize_gap(SmartCar.camera.target_gap);
    for(scan_idx = 0; scan_idx < IMAGE_TARGET_SCAN_ROWS; scan_idx++)
    {
        scan_row = (int16)base_row - (int16)scan_idx * (int16)gap;
        if(scan_row < 0)
        {
            break;
        }

        draw_y = image_debug_y(y, h, scan_row);
        for(draw_x = x; draw_x < (uint16)(x + w); draw_x++)
        {
            ips200_draw_point(draw_x, draw_y, (scan_idx == 0) ? RGB565_YELLOW : RGB565_CYAN);
        }
    }

    if(TargetFound)
    {
        draw_x = image_debug_x(x, w, TargetCenterX);
        draw_y = image_debug_y(y, h, TargetCenterY);
        ips200_draw_point(draw_x, draw_y, RGB565_YELLOW);
    }
}

void image_apply_camera(void)
{
    uint8 i;
    short int config[MT9V03X_CONFIG_FINISH][2];

    if(Image.ready == 0)
    {
        return;
    }

    for(i = 0; i < MT9V03X_CONFIG_FINISH; i++)
    {
        config[i][0] = 0;
        config[i][1] = 0;
    }

    config[0][0] = MT9V03X_INIT;
    config[1][0] = MT9V03X_AUTO_EXP;
    config[1][1] = 0;
    config[2][0] = MT9V03X_EXP_TIME;
    config[2][1] = SmartCar.camera.exposure;
    config[3][0] = MT9V03X_FPS;
    config[3][1] = MT9V03X_FPS_DEF;
    config[4][0] = MT9V03X_SET_COL;
    config[4][1] = MT9V03X_W;
    config[5][0] = MT9V03X_SET_ROW;
    config[5][1] = MT9V03X_H;
    config[6][0] = MT9V03X_LR_OFFSET;
    config[6][1] = MT9V03X_LR_OFFSET_DEF;
    config[7][0] = MT9V03X_UD_OFFSET;
    config[7][1] = MT9V03X_UD_OFFSET_DEF;
    config[8][0] = MT9V03X_GAIN;
    config[8][1] = SmartCar.camera.gain;

    SmartCar.camera.laser_row = image_target_normalize_row(SmartCar.camera.laser_row);
    SmartCar.camera.target_gap = image_target_normalize_gap(SmartCar.camera.target_gap);
    SmartCar.camera.laser_test = image_target_normalize_laser_test(SmartCar.camera.laser_test);

    mt9v03x_sccb_set_config(config);
}

void image_init(void)
{
    uint8 retry;
    uint8 i;

    /* 初始化加权权重总和（一次累加，避免每帧重复计算） */
    WeightingSumX10 = 0;
    for(i = 0; i < 10; i++) { WeightingSumX10 += WeightingX10[i]; }

    Image.ready = 0;
    Image.sequence = 0;
    Image.threshold = 0;
    Image.white_count = 0;
    Image.center = IMAGE_MID;
    Image.error = 0;
    Image.valid_count = 0;
    Image.lost = 1;
    Image.result_ready = 0;
    Image.ring = 0;
    Image.ring_step = 0;
    Image.zebra = 0;
    Image.zebra_count = 0;
    Image.is_straight = 0;
    Image.straight_left_error_x10 = 0;
    Image.straight_right_error_x10 = 0;
    Image.is_long_straight = 0;
    Image.straight_variance_x10 = 0;
    Image.is_ramp = 0;
    Image.ramp_count = 0;
    ImageRawThreshold = 0;
    ZebraHit = 0;
    ZebraDetectCount = 0;
    ZebraFrameLatch = 0;
    ZebraMissFrames = 0;
    ZebraCooldownFrames = 0;
    ImageLostCount = 0;
    ImageRunFrameCount = 0;
    TargetFrameGap = IMAGE_TARGET_FIRE_INTERVAL;
    TargetFound = 0;
    LaserBusy = 0;
    LaserTestLast = 0;
    LaserPitInit = 0;
    LaserTickLeft = 0;

    gpio_init(LED_DEBUG, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    gpio_init(LASER_LEFT_2, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(LASER_LEFT_1, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(LASER_CENTER, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(LASER_RIGHT_1, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(LASER_RIGHT_2, GPO, GPIO_LOW, GPO_PUSH_PULL);
    image_laser_all_off();

    retry = 0;
    while(retry < CAMERA_INIT_RETRY)
    {
        if(mt9v03x_init() == 0)
        {
            Image.ready = 1;
            break;
        }
        retry++;
        system_delay_ms(CAMERA_INIT_DELAY_MS);
    }

    image_apply_camera();
    mt9v03x_finish_flag = 0;
}

/**
 * @brief  压缩原始图像到80x60灰度图
 * @note   使用预计算的查找表加速，只在第一次调用时初始化
 */
static void image_compress(void)
{
    int16 row;
    int16 col;
    uint8 src_row;
    uint8 *dst;
    uint8 *src;

    /* 第一次调用时初始化行列映射表 */
    if(ImageMapReady == 0)
    {
        for(row = 0; row < IMAGE_H; row++)
        {
            ImageRowMap[row] = (uint8)(IMAGE_COMPRESS_CUT_ROW_TOP +
                                       (((uint16)row * IMAGE_COMPRESS_SRC_H + (IMAGE_H / 2)) / IMAGE_H));
        }
        for(col = 0; col < IMAGE_W; col++)
        {
            ImageColMap[col] = (uint8)(IMAGE_COMPRESS_CUT_COL +
                                       (((uint16)col * IMAGE_COMPRESS_SRC_W + (IMAGE_W / 2)) / IMAGE_W));
        }
        ImageMapReady = 1;
    }

    /* 使用查找表快速压缩图像 */
    for(row = 0; row < IMAGE_H; row++)
    {
        src_row = ImageRowMap[row];
        dst = ImageGray[row];
        src = mt9v03x_image[src_row];
        for(col = 0; col < IMAGE_W; col++)
        {
            dst[col] = src[ImageColMap[col]];
        }
    }

    mt9v03x_finish_flag = 0;
}

/**
 * @brief  大津法自动阈值计算
 * @return 计算出的最佳阈值
 * @note   提前退出优化：假设类间方差是单峰的（对于赛道图像通常成立）
 */
static uint8 image_otsu(void)
{
    int16 row;
    int16 col;
    int16 i;
    uint16 total;
    uint16 weight_back;
    uint16 weight_front;
    uint32 sum_all;
    uint32 sum_back;
    float mean_back;
    float mean_front;
    float diff;
    float score;
    float best_score;
    uint8 threshold;

    /* 清空直方图 */
    for(i = 0; i < 256; i++)
    {
        ImageHist[i] = 0;
    }

    /* 统计灰度直方图 */
    total = IMAGE_W * IMAGE_H;
    sum_all = 0;
    for(row = 0; row < IMAGE_H; row++)
    {
        for(col = 0; col < IMAGE_W; col++)
        {
            ImageHist[ImageGray[row][col]]++;
            sum_all += ImageGray[row][col];
        }
    }

    /* 大津法：遍历阈值，找到类间方差最大的 */
    weight_back = 0;
    sum_back = 0;
    best_score = 0.0f;
    threshold = 0;

    for(i = 0; i < IMAGE_THRESHOLD_DETACH; i++)
    {
        weight_back += ImageHist[i];
        if(weight_back == 0)
        {
            continue;
        }

        weight_front = total - weight_back;
        if(weight_front == 0)
        {
            break;
        }

        sum_back += (uint32)i * ImageHist[i];
        mean_back = (float)sum_back / weight_back;
        mean_front = (float)(sum_all - sum_back) / weight_front;
        diff = mean_back - mean_front;
        score = (float)weight_back * weight_front * diff * diff;

        if(score > best_score)
        {
            best_score = score;
            threshold = (uint8)i;
        }

        /* 提前退出：类间方差开始下降 */
        if(score < best_score)
        {
            break;
        }
    }

    Image.threshold = threshold;
    ImageRawThreshold = threshold;
    Image.lost = 0;
    if(threshold < IMAGE_STOP_RAW_THRESHOLD)
    {
        Image.lost = 1;
    }

    return threshold;
}

/**
 * @brief  二值化处理：将灰度图转换为黑白图
 * @param  threshold  大津法计算出的阈值
 * @note   边缘区域降低阈值，增强边线检测
 */
static void image_binarize(uint8 threshold)
{
    uint8 row;
    uint8 col;
    uint8 thre;
    uint16 threshold_value;

    /* 应用阈值偏移 */
    threshold_value = (uint16)threshold + SmartCar.camera.threshold_offset;
    if(threshold_value > 255)
    {
        threshold_value = 255;
    }

    threshold = (uint8)threshold_value;
    ImageRawThreshold = threshold;

    /* 应用阈值下限 */
    if(threshold < ImageStatus.Threshold_static)
    {
        threshold = (uint8)ImageStatus.Threshold_static;
    }

    ImageStatus.Threshold = threshold;
    Image.white_count = 0;

    /* 二值化：边缘区域用更低的阈值 */
    for(row = 0; row < IMAGE_H; row++)
    {
        for(col = 0; col < IMAGE_W; col++)
        {
            /* 左右边缘区域阈值降低10，增强边线检测 */
            if((col <= 15) || ((col > 70) && (col <= 75)) || (col >= 65))
            {
                thre = (uint8)(threshold - 10);
            }
            else
            {
                thre = threshold;
            }

            if(ImageGray[row][col] > thre)
            {
                ImageBin[row][col] = IMAGE_WHITE;
                Image.white_count++;
            }
            else
            {
                ImageBin[row][col] = IMAGE_BLACK;
            }
        }
    }
}

/**
 * @brief  初始化底部5行的边线作为跟踪起点
 * @note   从最底行开始，向上搜索5行，建立初始边界
 */
static void image_draw_bottom(void)
{
    PicTemp = ImageBin[59];
    BottomBorderLeft = 0;
    BottomBorderRight = 79;

    if(*(PicTemp + IMAGE_MID) == 0)
    {
        for(Xsite = 0; Xsite < IMAGE_MID; Xsite++)
        {
            if(*(PicTemp + IMAGE_MID - Xsite) != 0)
            {
                break;
            }
            if(*(PicTemp + IMAGE_MID + Xsite) != 0)
            {
                break;
            }
        }

        if(*(PicTemp + IMAGE_MID - Xsite) != 0)
        {
            BottomBorderRight = IMAGE_MID - Xsite + 1;
            for(Xsite = BottomBorderRight; Xsite > 0; Xsite--)
            {
                if((*(PicTemp + Xsite) == 0) && (*(PicTemp + Xsite - 1) == 0))
                {
                    BottomBorderLeft = Xsite;
                    break;
                }
                if(Xsite == 1)
                {
                    BottomBorderLeft = 0;
                    break;
                }
            }
        }
        else if(*(PicTemp + IMAGE_MID + Xsite) != 0)
        {
            BottomBorderLeft = IMAGE_MID + Xsite - 1;
            for(Xsite = BottomBorderLeft; Xsite < 79; Xsite++)
            {
                if((*(PicTemp + Xsite) == 0) && (*(PicTemp + Xsite + 1) == 0))
                {
                    BottomBorderRight = Xsite;
                    break;
                }
                if(Xsite == 78)
                {
                    BottomBorderRight = 79;
                    break;
                }
            }
        }
    }
    else
    {
        for(Xsite = 79; Xsite > IMAGE_MID; Xsite--)
        {
            if((*(PicTemp + Xsite) == 1) && (*(PicTemp + Xsite - 1) == 1))
            {
                BottomBorderRight = Xsite;
                break;
            }
            if(Xsite == 40)
            {
                BottomBorderRight = 39;
                break;
            }
        }

        for(Xsite = 0; Xsite < IMAGE_MID; Xsite++)
        {
            if((*(PicTemp + Xsite) == 1) && (*(PicTemp + Xsite + 1) == 1))
            {
                BottomBorderLeft = Xsite;
                break;
            }
            if(Xsite == 38)
            {
                BottomBorderLeft = 39;
                break;
            }
        }
    }

    BottomCenter = (BottomBorderLeft + BottomBorderRight) / 2;
    ImageDeal[59].LeftBorder = BottomBorderLeft;
    ImageDeal[59].RightBorder = BottomBorderRight;
    ImageDeal[59].Center = BottomCenter;
    ImageDeal[59].Wide = BottomBorderRight - BottomBorderLeft;
    ImageDeal[59].IsLeftFind = 'T';
    ImageDeal[59].IsRightFind = 'T';

    for(Ysite = 58; Ysite > 54; Ysite--)
    {
        PicTemp = ImageBin[Ysite];

        for(Xsite = 79; Xsite > ImageDeal[Ysite + 1].Center; Xsite--)
        {
            if((*(PicTemp + Xsite) == 1) && (*(PicTemp + Xsite - 1) == 1))
            {
                ImageDeal[Ysite].RightBorder = Xsite;
                break;
            }
            if(Xsite == (ImageDeal[Ysite + 1].Center + 1))
            {
                ImageDeal[Ysite].RightBorder = ImageDeal[Ysite + 1].Center;
                break;
            }
        }

        for(Xsite = 0; Xsite < ImageDeal[Ysite + 1].Center; Xsite++)
        {
            if((*(PicTemp + Xsite) == 1) && (*(PicTemp + Xsite + 1) == 1))
            {
                ImageDeal[Ysite].LeftBorder = Xsite;
                break;
            }
            if(Xsite == (ImageDeal[Ysite + 1].Center - 1))
            {
                ImageDeal[Ysite].LeftBorder = ImageDeal[Ysite + 1].Center;
                break;
            }
        }

        ImageDeal[Ysite].IsLeftFind = 'T';
        ImageDeal[Ysite].IsRightFind = 'T';
        ImageDeal[Ysite].Center = (ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) / 2;
        ImageDeal[Ysite].Wide = ImageDeal[Ysite].RightBorder - ImageDeal[Ysite].LeftBorder;
    }
}

static void image_search_bottom_boundary(uint8 bottom_row)
{
    ImageDeal[bottom_row].LeftBoundary = 0;
    ImageDeal[bottom_row].RightBoundary = IMAGE_W - 1;

    for(Xsite = (IMAGE_W / 2 - 2); Xsite > 1; Xsite--)
    {
        if((ImageBin[bottom_row][Xsite] == IMAGE_WHITE) &&
           (ImageBin[bottom_row][Xsite - 1] == IMAGE_BLACK))
        {
            ImageDeal[bottom_row].LeftBoundary = Xsite;
            break;
        }
    }

    for(Xsite = (IMAGE_W / 2 + 2); Xsite < (IMAGE_W - 1); Xsite++)
    {
        if((ImageBin[bottom_row][Xsite] == IMAGE_WHITE) &&
           (ImageBin[bottom_row][Xsite + 1] == IMAGE_BLACK))
        {
            ImageDeal[bottom_row].RightBoundary = Xsite;
            break;
        }
    }
}

static uint8 image_pixel(int16 row, int16 col)
{
    if((row < 0) || (row >= IMAGE_H) || (col < 0) || (col >= IMAGE_W))
    {
        return IMAGE_BLACK;
    }

    return ImageBin[row][col];
}

static int16 image_limit_col(int16 col)
{
    if(col < 0)
    {
        return 0;
    }
    if(col >= IMAGE_W)
    {
        return IMAGE_W - 1;
    }

    return col;
}

static void image_save_left_boundary(int16 row, int16 col)
{
    if((row < 0) || (row >= IMAGE_H))
    {
        return;
    }

    col = image_limit_col(col);
    if(ImageDeal[row].LeftBoundary_First == 0)
    {
        ImageDeal[row].LeftBoundary_First = col;
    }
    ImageDeal[row].LeftBoundary = col;
}

static void image_save_right_boundary(int16 row, int16 col)
{
    if((row < 0) || (row >= IMAGE_H))
    {
        return;
    }

    col = image_limit_col(col);
    if(ImageDeal[row].RightBoundary_First == (IMAGE_W - 1))
    {
        ImageDeal[row].RightBoundary_First = col;
    }
    ImageDeal[row].RightBoundary = col;
}

static void image_search_left_right_boundary(uint8 bottom_row)
{
    static const int8 left_rule[2][8] =
    {
        {0, -1, 1, 0, 0, 1, -1, 0},
        {-1, -1, 1, -1, 1, 1, -1, 1}
    };
    static const int8 right_rule[2][8] =
    {
        {0, -1, 1, 0, 0, 1, -1, 0},
        {1, -1, 1, 1, -1, 1, -1, -1}
    };
    int16 count;
    int16 scan_y;
    int16 left_y;
    int16 left_x;
    int16 left_dir;
    int16 pixel_left_y;
    int16 pixel_left_x;
    int16 right_y;
    int16 right_x;
    int16 right_dir;
    int16 pixel_right_y;
    int16 pixel_right_x;

    count = 0;
    left_y = bottom_row;
    left_x = ImageDeal[bottom_row].LeftBoundary;
    left_dir = 0;
    pixel_left_y = bottom_row;
    pixel_left_x = left_x;
    right_y = bottom_row;
    right_x = ImageDeal[bottom_row].RightBoundary;
    right_dir = 0;
    pixel_right_y = bottom_row;
    pixel_right_x = right_x;
    scan_y = bottom_row;
    ImageStatus.OFFLineBoundary = 5;

    while(1)
    {
        count++;
        if(count > 400)
        {
            ImageStatus.OFFLineBoundary = scan_y;
            break;
        }

        if((scan_y >= pixel_left_y) && (scan_y >= pixel_right_y))
        {
            if(scan_y < ImageStatus.OFFLineBoundary)
            {
                ImageStatus.OFFLineBoundary = scan_y;
                break;
            }
            scan_y--;
        }

        if((pixel_left_y > scan_y) || (scan_y == ImageStatus.OFFLineBoundary))
        {
            pixel_left_y = left_y + left_rule[0][2 * left_dir + 1];
            pixel_left_x = left_x + left_rule[0][2 * left_dir];

            if(image_pixel(pixel_left_y, pixel_left_x) == IMAGE_BLACK)
            {
                left_dir = (left_dir == 3) ? 0 : (left_dir + 1);
            }
            else
            {
                pixel_left_y = left_y + left_rule[1][2 * left_dir + 1];
                pixel_left_x = left_x + left_rule[1][2 * left_dir];

                if(image_pixel(pixel_left_y, pixel_left_x) == IMAGE_BLACK)
                {
                    left_y = left_y + left_rule[0][2 * left_dir + 1];
                    left_x = left_x + left_rule[0][2 * left_dir];
                }
                else
                {
                    left_y = left_y + left_rule[1][2 * left_dir + 1];
                    left_x = left_x + left_rule[1][2 * left_dir];
                    left_dir = (left_dir == 0) ? 3 : (left_dir - 1);
                }

                /* Match the 19th reference trace: store the walked point itself. */
                image_save_left_boundary(left_y, left_x);
            }
        }

        if((pixel_right_y > scan_y) || (scan_y == ImageStatus.OFFLineBoundary))
        {
            pixel_right_y = right_y + right_rule[0][2 * right_dir + 1];
            pixel_right_x = right_x + right_rule[0][2 * right_dir];

            if(image_pixel(pixel_right_y, pixel_right_x) == IMAGE_BLACK)
            {
                right_dir = (right_dir == 0) ? 3 : (right_dir - 1);
            }
            else
            {
                pixel_right_y = right_y + right_rule[1][2 * right_dir + 1];
                pixel_right_x = right_x + right_rule[1][2 * right_dir];

                if(image_pixel(pixel_right_y, pixel_right_x) == IMAGE_BLACK)
                {
                    right_y = right_y + right_rule[0][2 * right_dir + 1];
                    right_x = right_x + right_rule[0][2 * right_dir];
                }
                else
                {
                    right_y = right_y + right_rule[1][2 * right_dir + 1];
                    right_x = right_x + right_rule[1][2 * right_dir];
                    right_dir = (right_dir == 3) ? 0 : (right_dir + 1);
                }

                /* Match the 19th reference trace: store the walked point itself. */
                image_save_right_boundary(right_y, right_x);
            }
        }

        if(IMAGE_ABS(pixel_right_x - pixel_left_x) < 3)
        {
            ImageStatus.OFFLineBoundary = scan_y;
            break;
        }
    }
}

/* Trace raw binary boundaries used by ring element detection. */
static void image_search_border(uint8 bottom_row)
{
    for(Xsite = 0; Xsite < IMAGE_W; Xsite++)
    {
        ImageBin[0][Xsite] = IMAGE_BLACK;
        if((bottom_row + 1) < IMAGE_H)
        {
            ImageBin[bottom_row + 1][Xsite] = IMAGE_BLACK;
        }
    }

    for(Ysite = 0; Ysite < IMAGE_H; Ysite++)
    {
        ImageDeal[Ysite].LeftBoundary_First = 0;
        ImageDeal[Ysite].RightBoundary_First = IMAGE_W - 1;
        ImageDeal[Ysite].LeftBoundary = 0;
        ImageDeal[Ysite].RightBoundary = IMAGE_W - 1;
        ImageBin[Ysite][0] = IMAGE_BLACK;
        ImageBin[Ysite][IMAGE_W - 1] = IMAGE_BLACK;
    }

    ImageStatus.WhiteLine_L = 0;
    ImageStatus.WhiteLine_R = 0;
    image_search_bottom_boundary(bottom_row);
    image_search_left_right_boundary(bottom_row);

    for(Ysite = bottom_row; Ysite > (ImageStatus.OFFLineBoundary + 1); Ysite--)
    {
        if(ImageDeal[Ysite].LeftBoundary < 3)
        {
            ImageStatus.WhiteLine_L++;
        }
        if(ImageDeal[Ysite].RightBoundary > (IMAGE_W - 3))
        {
            ImageStatus.WhiteLine_R++;
        }
    }
}

/**
 * @brief  在单行中搜索边线跳变点
 * @param  line      当前行的指针
 * @param  type      搜索类型：'L'=左边线, 'R'=右边线
 * @param  low/high  搜索范围
 * @param  jump      输出的跳变点结果
 * @note   跳变点类型：'T'=找到边线, 'W'=全白无边线, 'H'=全黑隐藏
 */
static void image_get_jump(uint8 *line, uint8 type, int16 low, int16 high, image_jump *jump)
{
    int16 i;

    /* 限制搜索范围 */
    LimitL(low);
    LimitH(low);
    LimitL(high);
    LimitH(high);

    if(low > high)
    {
        i = low;
        low = high;
        high = i;
    }

    if(type == 'L')
    {
        /* 从右向左搜索左边线（白到黑的跳变） */
        for(i = high; i >= low; i--)
        {
            if((*(line + i) == 1) && (*(line + i - 1) != 1))
            {
                jump->point = i;
                jump->type = 'T';
                break;
            }
            if(i == (low + 1))
            {
                if(*(line + (low + high) / 2) != 0)
                {
                    jump->point = (low + high) / 2;
                    jump->type = 'W';  /* 全白 */
                }
                else
                {
                    jump->point = high;
                    jump->type = 'H';  /* 全黑 */
                }
                break;
            }
        }
    }
    else
    {
        /* 从左向右搜索右边线（白到黑的跳变） */
        for(i = low; i <= high; i++)
        {
            if((*(line + i) == 1) && (*(line + i + 1) != 1))
            {
                jump->point = i;
                jump->type = 'T';
                break;
            }
            if(i == (high - 1))
            {
                if(*(line + (low + high) / 2) != 0)
                {
                    jump->point = (low + high) / 2;
                    jump->type = 'W';  /* 全白 */
                }
                else
                {
                    jump->point = low;
                    jump->type = 'H';  /* 全黑 */
                }
                break;
            }
        }
    }
}

/**
 * @brief  从底部向上逐行跟踪左右边线
 * @note   核心巡线算法：在上一行边线附近搜索当前行边线
 */
static void image_draw_lines(void)
{
    uint8 L_Found_T;
    uint8 Get_L_line;
    uint8 R_Found_T;
    uint8 Get_R_line;
    float D_L;
    float D_R;
    int16 ytemp_W_L;
    int16 ytemp_W_R;
    int16 ysite;
    uint8 L_found_point;
    uint8 R_found_point;
    image_jump JumpPoint[2];

    L_Found_T = 'F';
    Get_L_line = 'F';
    R_Found_T = 'F';
    Get_R_line = 'F';
    D_L = 0.0f;
    D_R = 0.0f;
    ytemp_W_L = 0;
    ytemp_W_R = 0;
    L_found_point = 0;
    R_found_point = 0;
    ExtenRFlag = 0;
    ExtenLFlag = 0;
    ImageStatus.Left_Line = 0;
    ImageStatus.WhiteLine = 0;
    ImageStatus.Right_Line = 0;

    for(Ysite = 54; Ysite > ImageStatus.OFFLine; Ysite--)
    {
        PicTemp = ImageBin[Ysite];

        IntervalLow = ImageDeal[Ysite + 1].RightBorder - IMAGE_SCAN_INTERVAL;
        IntervalHigh = ImageDeal[Ysite + 1].RightBorder + IMAGE_SCAN_INTERVAL;
        image_get_jump(PicTemp, 'R', IntervalLow, IntervalHigh, &JumpPoint[1]);

        IntervalLow = ImageDeal[Ysite + 1].LeftBorder - IMAGE_SCAN_INTERVAL;
        IntervalHigh = ImageDeal[Ysite + 1].LeftBorder + IMAGE_SCAN_INTERVAL;
        image_get_jump(PicTemp, 'L', IntervalLow, IntervalHigh, &JumpPoint[0]);

        if(JumpPoint[0].type == 'W')
        {
            ImageDeal[Ysite].LeftBorder = ImageDeal[Ysite + 1].LeftBorder;
        }
        else
        {
            ImageDeal[Ysite].LeftBorder = JumpPoint[0].point;
        }

        if(JumpPoint[1].type == 'W')
        {
            ImageDeal[Ysite].RightBorder = ImageDeal[Ysite + 1].RightBorder;
        }
        else
        {
            ImageDeal[Ysite].RightBorder = JumpPoint[1].point;
        }

        ImageDeal[Ysite].IsLeftFind = JumpPoint[0].type;
        ImageDeal[Ysite].IsRightFind = JumpPoint[1].type;

        if((ImageDeal[Ysite].IsLeftFind == 'H') || (ImageDeal[Ysite].IsRightFind == 'H'))
        {
            if(ImageDeal[Ysite].IsLeftFind == 'H')
            {
                for(Xsite = (ImageDeal[Ysite].LeftBorder + 1);
                    Xsite <= (ImageDeal[Ysite].RightBorder - 1);
                    Xsite++)
                {
                    if((*(PicTemp + Xsite) == 0) && (*(PicTemp + Xsite + 1) != 0))
                    {
                        ImageDeal[Ysite].LeftBorder = Xsite;
                        ImageDeal[Ysite].IsLeftFind = 'T';
                        break;
                    }
                    if(*(PicTemp + Xsite) != 0)
                    {
                        break;
                    }
                    if(Xsite == (ImageDeal[Ysite].RightBorder - 1))
                    {
                        ImageDeal[Ysite].LeftBorder = Xsite;
                        ImageDeal[Ysite].IsLeftFind = 'T';
                        break;
                    }
                }
            }

            if((ImageDeal[Ysite].RightBorder - ImageDeal[Ysite].LeftBorder) <= 7)
            {
                ImageStatus.OFFLine = Ysite + 1;
                break;
            }

            if(ImageDeal[Ysite].IsRightFind == 'H')
            {
                for(Xsite = (ImageDeal[Ysite].RightBorder - 1);
                    Xsite >= (ImageDeal[Ysite].LeftBorder + 1);
                    Xsite--)
                {
                    if((*(PicTemp + Xsite) == 0) && (*(PicTemp + Xsite - 1) != 0))
                    {
                        ImageDeal[Ysite].RightBorder = Xsite;
                        ImageDeal[Ysite].IsRightFind = 'T';
                        break;
                    }
                    if(*(PicTemp + Xsite) != 0)
                    {
                        break;
                    }
                    if(Xsite == (ImageDeal[Ysite].LeftBorder + 1))
                    {
                        ImageDeal[Ysite].RightBorder = Xsite;
                        ImageDeal[Ysite].IsRightFind = 'T';
                        break;
                    }
                }
            }
        }

        if((ImageDeal[Ysite].IsRightFind == 'W') && (Ysite > 10) && (Ysite < 50))
        {
            if(Get_R_line == 'F')
            {
                Get_R_line = 'T';
                ytemp_W_R = Ysite + 2;
                for(ysite = Ysite + 1; (ysite < Ysite + 15) && (ysite < IMAGE_H); ysite++)
                {
                    if(ImageDeal[ysite].IsRightFind == 'T')
                    {
                        R_found_point++;
                    }
                }
                if(R_found_point > 8)
                {
                    D_R = ((float)(ImageDeal[Ysite + R_found_point].RightBorder -
                                   ImageDeal[Ysite + 3].RightBorder)) /
                          ((float)(R_found_point - 3));
                    if(D_R > 0.0f)
                    {
                        R_Found_T = 'T';
                    }
                    else
                    {
                        R_Found_T = 'F';
                        if(D_R < 0.0f)
                        {
                            ExtenRFlag = 'F';
                        }
                    }
                }
            }
            if(R_Found_T == 'T')
            {
                ImageDeal[Ysite].RightBorder =
                    ImageDeal[ytemp_W_R].RightBorder - (int16)(D_R * (float)(ytemp_W_R - Ysite));
            }

            LimitL(ImageDeal[Ysite].RightBorder);
            LimitH(ImageDeal[Ysite].RightBorder);
        }

        if((ImageDeal[Ysite].IsLeftFind == 'W') && (Ysite > 10) && (Ysite < 50))
        {
            if(Get_L_line == 'F')
            {
                Get_L_line = 'T';
                ytemp_W_L = Ysite + 2;
                for(ysite = Ysite + 1; (ysite < Ysite + 15) && (ysite < IMAGE_H); ysite++)
                {
                    if(ImageDeal[ysite].IsLeftFind == 'T')
                    {
                        L_found_point++;
                    }
                }
                if(L_found_point > 8)
                {
                    D_L = ((float)(ImageDeal[Ysite + 3].LeftBorder -
                                   ImageDeal[Ysite + L_found_point].LeftBorder)) /
                          ((float)(L_found_point - 3));
                    if(D_L > 0.0f)
                    {
                        L_Found_T = 'T';
                    }
                    else
                    {
                        L_Found_T = 'F';
                        if(D_L < 0.0f)
                        {
                            ExtenLFlag = 'F';
                        }
                    }
                }
            }
            if(L_Found_T == 'T')
            {
                ImageDeal[Ysite].LeftBorder =
                    ImageDeal[ytemp_W_L].LeftBorder + (int16)(D_L * (float)(ytemp_W_L - Ysite));
            }

            LimitL(ImageDeal[Ysite].LeftBorder);
            LimitH(ImageDeal[Ysite].LeftBorder);
        }

        if((ImageDeal[Ysite].IsLeftFind == 'W') && (ImageDeal[Ysite].IsRightFind == 'W'))
        {
            ImageStatus.WhiteLine++;
        }
        if((ImageDeal[Ysite].IsLeftFind == 'W') && (Ysite < 55))
        {
            ImageStatus.Left_Line++;
        }
        if((ImageDeal[Ysite].IsRightFind == 'W') && (Ysite < 55))
        {
            ImageStatus.Right_Line++;
        }

        LimitL(ImageDeal[Ysite].LeftBorder);
        LimitH(ImageDeal[Ysite].LeftBorder);
        LimitL(ImageDeal[Ysite].RightBorder);
        LimitH(ImageDeal[Ysite].RightBorder);

        ImageDeal[Ysite].Wide = ImageDeal[Ysite].RightBorder - ImageDeal[Ysite].LeftBorder;
        ImageDeal[Ysite].Center = (ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) / 2;
        ImageDeal[Ysite].LeftTemp = ImageDeal[Ysite].LeftBorder;
        ImageDeal[Ysite].RightTemp = ImageDeal[Ysite].RightBorder;
        ImageDeal[Ysite].close_LeftBorder = ImageDeal[Ysite].LeftBorder;
        ImageDeal[Ysite].close_RightBorder = ImageDeal[Ysite].RightBorder;

        if(ImageDeal[Ysite].Wide <= 7)
        {
            ImageStatus.OFFLine = Ysite + 1;
            break;
        }
        if((ImageDeal[Ysite].RightBorder <= 10) || (ImageDeal[Ysite].LeftBorder >= 70))
        {
            ImageStatus.OFFLine = Ysite + 1;
            break;
        }
    }
}

/**
 * @brief  判断是否可以进行延长线补线
 * @return 1=允许补线, 0=不允许（环岛期间禁止补线）
 */
static uint8 image_can_draw_extension_line(void)
{
    if((ImageStatus.Road_type == ROAD_LEFT_RING) ||
       (ImageStatus.Road_type == ROAD_RIGHT_RING))
    {
        return 0;
    }

    if((ImageFlag.image_element_rings != 0) ||
       (ImageFlag.image_element_rings_flag != 0))
    {
        return 0;
    }

    return 1;
}

/**
 * @brief  补全丢失的边线（延长线法）
 * @note   当边线中断时，用线性延长填补断点
 */
static void image_draw_extension_line(void)
{
    if(image_can_draw_extension_line())
    {
        TFSite = 55;
        FTSite = 0;

        if(ImageStatus.WhiteLine >= ImageStatus.TowPoint_True - 15)
        {
            TFSite = 55;
        }

        if(ExtenLFlag != 'F')
        {
            for(Ysite = 54; Ysite >= (ImageStatus.OFFLine + 4); Ysite--)
            {
                if(ImageDeal[Ysite].IsLeftFind == 'W')
                {
                    if(ImageDeal[Ysite + 1].LeftBorder >= 70)
                    {
                        ImageStatus.OFFLine = Ysite + 1;
                        break;
                    }

                    while(Ysite >= (ImageStatus.OFFLine + 4))
                    {
                        Ysite--;
                        if((ImageDeal[Ysite].IsLeftFind == 'T') &&
                           (ImageDeal[Ysite - 1].IsLeftFind == 'T') &&
                           (ImageDeal[Ysite - 2].IsLeftFind == 'T') &&
                           (ImageDeal[Ysite - 2].LeftBorder > 0) &&
                           (ImageDeal[Ysite - 2].LeftBorder < 70))
                        {
                            FTSite = Ysite - 2;
                            break;
                        }
                    }

                    if(FTSite > ImageStatus.OFFLine)
                    {
                        DetL = ((float)(ImageDeal[FTSite].LeftBorder -
                                        ImageDeal[TFSite].LeftBorder)) /
                               ((float)(FTSite - TFSite));
                        for(ytemp = TFSite; ytemp >= FTSite; ytemp--)
                        {
                            ImageDeal[ytemp].LeftBorder =
                                (int16)(DetL * (float)(ytemp - TFSite)) + ImageDeal[TFSite].LeftBorder;
                        }
                    }
                }
                else
                {
                    TFSite = Ysite + 2;
                }
            }
        }

        if(ImageStatus.WhiteLine >= ImageStatus.TowPoint_True - 15)
        {
            TFSite = 55;
        }

        if(ExtenRFlag != 'F')
        {
            FTSite = 0;
            for(Ysite = 54; Ysite >= (ImageStatus.OFFLine + 4); Ysite--)
            {
                if(ImageDeal[Ysite].IsRightFind == 'W')
                {
                    if(ImageDeal[Ysite + 1].RightBorder <= 10)
                    {
                        ImageStatus.OFFLine = Ysite + 1;
                        break;
                    }

                    while(Ysite >= (ImageStatus.OFFLine + 4))
                    {
                        Ysite--;
                        if((ImageDeal[Ysite].IsRightFind == 'T') &&
                           (ImageDeal[Ysite - 1].IsRightFind == 'T') &&
                           (ImageDeal[Ysite - 2].IsRightFind == 'T') &&
                           (ImageDeal[Ysite - 2].RightBorder < 70) &&
                           (ImageDeal[Ysite - 2].RightBorder > 10))
                        {
                            FTSite = Ysite - 2;
                            break;
                        }
                    }

                    if(FTSite > ImageStatus.OFFLine)
                    {
                        DetR = ((float)(ImageDeal[FTSite].RightBorder -
                                        ImageDeal[TFSite].RightBorder)) /
                               ((float)(FTSite - TFSite));
                        for(ytemp = TFSite; ytemp >= FTSite; ytemp--)
                        {
                            ImageDeal[ytemp].RightBorder =
                                (int16)(DetR * (float)(ytemp - TFSite)) + ImageDeal[TFSite].RightBorder;
                        }
                    }
                }
                else
                {
                    TFSite = Ysite + 2;
                }
            }
        }
    }

    for(Ysite = 59; Ysite >= ImageStatus.OFFLine; Ysite--)
    {
        LimitL(ImageDeal[Ysite].LeftBorder);
        LimitH(ImageDeal[Ysite].LeftBorder);
        LimitL(ImageDeal[Ysite].RightBorder);
        LimitH(ImageDeal[Ysite].RightBorder);
        ImageDeal[Ysite].Center = (ImageDeal[Ysite].LeftBorder + ImageDeal[Ysite].RightBorder) / 2;
        ImageDeal[Ysite].Wide = ImageDeal[Ysite].RightBorder - ImageDeal[Ysite].LeftBorder;
    }
}

/**
 * @brief  路径滤波：平滑中线，减少抖动
 * @note   对应参考代码的 RouteFilter 函数
 */
static void image_route_filter(void)
{
    int16 center_temp;
    int16 line_temp;

    center_temp = 0;
    line_temp = 0;

    for(Ysite = 58; Ysite >= (ImageStatus.OFFLine + 5); Ysite--)
    {
        /* 连续两行都是全白时，插值填补中线 */
        if((ImageDeal[Ysite].IsLeftFind == 'W') &&
           (ImageDeal[Ysite].IsRightFind == 'W') &&
           (Ysite <= 45) &&
           (ImageDeal[Ysite - 1].IsLeftFind == 'W') &&
           (ImageDeal[Ysite - 1].IsRightFind == 'W'))
        {
            ytemp = Ysite;
            while(ytemp >= (ImageStatus.OFFLine + 5))
            {
                ytemp--;
                if((ImageDeal[ytemp].IsLeftFind == 'T') &&
                   (ImageDeal[ytemp].IsRightFind == 'T'))
                {
                    /* 线性插值填补 */
                    DetR = ((float)(ImageDeal[ytemp - 1].Center - ImageDeal[Ysite + 2].Center)) /
                           ((float)(ytemp - 1 - Ysite - 2));
                    center_temp = ImageDeal[Ysite + 2].Center;
                    line_temp = Ysite + 2;
                    while(Ysite >= ytemp)
                    {
                        ImageDeal[Ysite].Center =
                            (int16)((float)center_temp + DetR * (float)(Ysite - line_temp));
                        Ysite--;
                    }
                    break;
                }
            }
        }

        /* 简单滤波：当前行中线 = (上一行 + 当前行*2) / 3 */
        ImageDeal[Ysite].Center = (ImageDeal[Ysite - 1].Center + 2 * ImageDeal[Ysite].Center) / 3;
    }
}

/**
 * @brief  直线度判断（边线线性拟合的均方误差，整数版 ×10）
 * @param  dir    1=判断左边线, 2=判断右边线
 * @param  start  起始行
 * @param  end    结束行
 * @return 均方误差 ×10（越小越直），单位 = 像素² ×10
 * @note   等价公式 S*10 = Σ(err_num²) * 10 / (k_den² * count)
 *         其中 err_num = border_start*k_den + k_num*i - actual*k_den
 *         加 ±2048 钳位防溢出；阈值对照：<1.0f → <10, >50.0f → >500
 */
static int16 Straight_Judge(uint8 dir, uint8 start, uint8 end)
{
    int i;
    int16 count;
    int32 k_num;
    int32 k_den;
    int32 border_start;
    int32 err_num;
    int32 sum2 = 0;
    int32 k_den_sq;
    int32 S_x10;

    count = (int16)end - (int16)start;
    if(count <= 0) { return 0; }

    if(dir == 1)  /* 左边线 */
    {
        border_start = ImageDeal[start].LeftBorder;
        k_num = (int32)ImageDeal[start].LeftBorder - (int32)ImageDeal[end].LeftBorder;
        k_den = (int32)((int16)start - (int16)end);

        for(i = 0; i < count; i++)
        {
            err_num = border_start * k_den
                    + k_num * i
                    - (int32)ImageDeal[i + start].LeftBorder * k_den;
            /* 钳位到 ±2048，保证极端情况下不溢出 int32 */
            if(err_num >  2047) err_num =  2047;
            else if(err_num < -2048) err_num = -2048;
            sum2 += err_num * err_num;
        }
    }
    else if(dir == 2)  /* 右边线 */
    {
        border_start = ImageDeal[start].RightBorder;
        k_num = (int32)ImageDeal[start].RightBorder - (int32)ImageDeal[end].RightBorder;
        k_den = (int32)((int16)start - (int16)end);

        for(i = 0; i < count; i++)
        {
            err_num = border_start * k_den
                    + k_num * i
                    - (int32)ImageDeal[i + start].RightBorder * k_den;
            if(err_num >  2047) err_num =  2047;
            else if(err_num < -2048) err_num = -2048;
            sum2 += err_num * err_num;
        }
    }
    else
    {
        return 0;
    }

    k_den_sq = k_den * k_den;
    S_x10 = (int32)((sum2 * 10L) / (k_den_sq * count));
    if(S_x10 > 32767) S_x10 = 32767;
    return (int16)S_x10;
}

/**
 * @brief  计算右边线的最大偏离度（整数版 ×10）
 * @param  start  起始行
 * @param  end    结束行
 * @return 最大偏离值 ×10（绝对值，整数）
 * @note   循环内只累加 |err_num|，最后除 k_den 得到最大像素偏差 ×10
 */
static int16 Right_Border_Max_Deviation(uint8 start, uint8 end)
{
    uint8 row;
    int32 k_num;
    int32 k_den;
    int32 border_start;
    int32 err_num;
    int32 max_abs_err_num = 0;
    int32 ret;

    if(end <= start)
    {
        return 0;
    }

    k_num = (int32)ImageDeal[end].RightBorder - (int32)ImageDeal[start].RightBorder;
    k_den = (int32)end - (int32)start;
    border_start = ImageDeal[start].RightBorder;

    /* 计算每一行的偏离度（整数：err_num = border_start*k_den + k_num*Δrow - actual*k_den） */
    for(row = start; row <= end; row++)
    {
        err_num = border_start * k_den
                + k_num * ((int32)row - (int32)start)
                - (int32)ImageDeal[row].RightBorder * k_den;
        if(err_num < 0) err_num = -err_num;
        if(err_num > max_abs_err_num) max_abs_err_num = err_num;
    }

    /* max_err = max_abs_err_num / k_den，再 ×10 */
    ret = (max_abs_err_num * 10L) / k_den;
    if(ret > 32767) ret = 32767;
    return (int16)ret;
}

/**
 * @brief  计算左边线的最大偏离度
 * @param  start  起始行
 * @param  end    结束行
 * @return 最大偏离值（绝对值）
 */
static int16 Left_Border_Max_Deviation(uint8 start, uint8 end)
{
    uint8 row;
    int32 k_num;
    int32 k_den;
    int32 border_start;
    int32 err_num;
    int32 max_abs_err_num = 0;
    int32 ret;

    if(end <= start)
    {
        return 0;
    }

    k_num = (int32)ImageDeal[end].LeftBorder - (int32)ImageDeal[start].LeftBorder;
    k_den = (int32)end - (int32)start;
    border_start = ImageDeal[start].LeftBorder;

    for(row = start; row <= end; row++)
    {
        err_num = border_start * k_den
                + k_num * ((int32)row - (int32)start)
                - (int32)ImageDeal[row].LeftBorder * k_den;
        if(err_num < 0) err_num = -err_num;
        if(err_num > max_abs_err_num) max_abs_err_num = err_num;
    }

    ret = (max_abs_err_num * 10L) / k_den;
    if(ret > 32767) ret = 32767;
    return (int16)ret;
}

/* =============================================================================
 * 环岛识别和处理
 * ============================================================================= */

/**
 * @brief  左环岛判断
 * @note   通过边界拐点和边线特征识别左环岛入口
 */
static void image_judge_left_ring(void)
{
    Left_RingsFlag_Point1_Ysite = 0;
    Left_RingsFlag_Point2_Ysite = 0;

    /* 计算右边线偏离度（左环岛时右边线应该比较直，已 ×10） */
    Left_Ring_Right_Deviation_X10 =
        Right_Border_Max_Deviation(20, 55);

    /* 左环岛前置条件检查（任一条件不满足则直接返回） */
    if((ImageStatus.Right_Line > 7) ||           /* 右侧丢线过多 */
       (ImageStatus.Left_Line < 13) ||           /* 左侧丢线太少 */
       (ImageStatus.OFFLine > 10) ||             /* 有效行太少 */
       (Left_Ring_Right_Deviation_X10 >= 15) ||  /* 右边线不够直 */
       (ImageStatus.WhiteLine > 15) ||           /* 全白行过多 */
       (ImageDeal[52].IsLeftFind == 'W') ||      /* 底部左边线不能丢失 */
       (ImageDeal[53].IsLeftFind == 'W') ||
       (ImageDeal[54].IsLeftFind == 'W') ||
       (ImageDeal[55].IsLeftFind == 'W') ||
       (ImageDeal[56].IsLeftFind == 'W') ||
       (ImageDeal[57].IsLeftFind == 'W') ||
       (ImageDeal[58].IsLeftFind == 'W'))
    {
        return;
    }

    /* 找左边界第一个拐点（LeftBoundary_First突然增大） */
    for(Ysite = 58; Ysite > 25; Ysite--)
    {
        if((ImageDeal[Ysite].LeftBoundary_First - ImageDeal[Ysite - 1].LeftBoundary_First) > 4)
        {
            Left_RingsFlag_Point1_Ysite = Ysite;
            break;
        }
    }

    /* 找左边界第二个拐点（LeftBoundary突然增大） */
    for(Ysite = 58; Ysite > 25; Ysite--)
    {
        if((ImageDeal[Ysite + 1].LeftBoundary - ImageDeal[Ysite].LeftBoundary) > 4)
        {
            Left_RingsFlag_Point2_Ysite = Ysite;
            break;
        }
    }

    /* 寻找左边线的凸起特征（环岛特有的弧形） */
    for(Ysite = Left_RingsFlag_Point1_Ysite; Ysite > ImageStatus.OFFLine; Ysite--)
    {
        if((ImageDeal[Ysite + 6].LeftBorder < ImageDeal[Ysite + 3].LeftBorder) &&
           (ImageDeal[Ysite + 5].LeftBorder < ImageDeal[Ysite + 3].LeftBorder) &&
           (ImageDeal[Ysite + 3].LeftBorder > ImageDeal[Ysite + 2].LeftBorder) &&
           (ImageDeal[Ysite + 3].LeftBorder > ImageDeal[Ysite + 1].LeftBorder))
        {
            Ring_Help_Flag = 1;
            break;
        }
    }

    /* 第二确认路径：两个拐点间隔足够大 */
    if((Left_RingsFlag_Point2_Ysite > (Left_RingsFlag_Point1_Ysite + 3)) &&
       (Ring_Help_Flag == 0))
    {
        if(ImageStatus.Left_Line > 13)
        {
            Ring_Help_Flag = 1;
        }
    }

    /* 确认左环岛：满足所有条件 */
    if((Left_RingsFlag_Point2_Ysite > (Left_RingsFlag_Point1_Ysite + 3)) &&
       (Ring_Help_Flag == 1) &&
       (ImageFlag.image_element_rings_flag == 0))
    {
        ImageFlag.image_element_rings = 1;
        ImageFlag.image_element_rings_flag = 1;
        ImageFlag.ring_big_small = 1;
        ImageStatus.Road_type = ROAD_LEFT_RING;
        buzzer_short();
    }

    Ring_Help_Flag = 0;
}

/**
 * @brief  右环岛判断
 * @note   通过边界拐点和边线特征识别右环岛入口
 */
static void image_judge_right_ring(void)
{
    Right_RingsFlag_Point1_Ysite = 0;
    Right_RingsFlag_Point2_Ysite = 0;

    /* 计算左边线偏离度（右环岛时左边线应该比较直，已 ×10） */
    Right_Ring_Left_Deviation_X10 =
        Left_Border_Max_Deviation(20, 55);

    /* 右环岛前置条件检查 */
    if((ImageStatus.Left_Line > 7) ||            /* 左侧丢线过多 */
       (ImageStatus.Right_Line < 13) ||          /* 右侧丢线太少 */
       (ImageStatus.OFFLine > 10) ||             /* 有效行太少 */
       (Straight_Judge(1, 25, 45) > 500) ||     /* 左边线不够直 (原 50.0f, ×10) */
       (Right_Ring_Left_Deviation_X10 >= 15) ||  /* 左边线偏离太大 */
       (ImageStatus.WhiteLine > 15) ||           /* 全白行过多 */
       (ImageDeal[52].IsRightFind == 'W') ||     /* 底部右边线不能丢失 */
       (ImageDeal[53].IsRightFind == 'W') ||
       (ImageDeal[54].IsRightFind == 'W') ||
       (ImageDeal[55].IsRightFind == 'W') ||
       (ImageDeal[56].IsRightFind == 'W') ||
       (ImageDeal[57].IsRightFind == 'W') ||
       (ImageDeal[58].IsRightFind == 'W'))
    {
        return;
    }

    /* 找右边界第一个拐点（RightBoundary_First突然减小） */
    for(Ysite = 58; Ysite > 25; Ysite--)
    {
        if((ImageDeal[Ysite - 1].RightBoundary_First - ImageDeal[Ysite].RightBoundary_First) > 4)
        {
            Right_RingsFlag_Point1_Ysite = Ysite;
            break;
        }
    }

    /* 找右边界第二个拐点（RightBoundary突然减小） */
    for(Ysite = 58; Ysite > 25; Ysite--)
    {
        if((ImageDeal[Ysite].RightBoundary - ImageDeal[Ysite + 1].RightBoundary) > 4)
        {
            Right_RingsFlag_Point2_Ysite = Ysite;
            break;
        }
    }

    /* 寻找右边线的凹陷特征（环岛特有的弧形） */
    for(Ysite = Right_RingsFlag_Point1_Ysite; Ysite > 10; Ysite--)
    {
        if((ImageDeal[Ysite + 6].RightBorder > ImageDeal[Ysite + 3].RightBorder) &&
           (ImageDeal[Ysite + 5].RightBorder > ImageDeal[Ysite + 3].RightBorder) &&
           (ImageDeal[Ysite + 3].RightBorder < ImageDeal[Ysite + 2].RightBorder) &&
           (ImageDeal[Ysite + 3].RightBorder < ImageDeal[Ysite + 1].RightBorder))
        {
            Ring_Help_Flag = 1;
            break;
        }
    }

    /*
     * 右环第二确认路径：
     *
     * Right_RingsFlag_Point1_Ysite:
     *   由 RightBoundary_First 找到的右侧上方拐点
     * Right_RingsFlag_Point2_Ysite:
     *   由 RightBoundary 找到的右侧下方拐点
     *
     * Right_Ring_Left_Deviation_X10:
     *   右环入口要求左侧边界整体接近直线
     *   显示值 RLD10 必须小于 15 才允许继续判断右环
     *
     * Point2 >= Point1 + 1:
     *   两个右侧拐点至少相差 1 行，说明右环入口形状已经出现
     *
     * Ring_Help_Flag == 0:
     *   前面的 RightBorder 形态扫描没有确认右环入口
     *   所以这里再用 Right_Line 做一次确认
     *
     * ImageStatus.Right_Line:
     *   右侧丢边行数。右环入口出现时右侧开口变大
     *   这个值通常会变大。当前第二确认阈值是 > 7
     */
    if((Right_RingsFlag_Point2_Ysite >= (Right_RingsFlag_Point1_Ysite + 1)) &&
       (Ring_Help_Flag == 0))
    {
        if(ImageStatus.Right_Line > 7)
        {
            Ring_Help_Flag = 1;
        }
    }

    /*
     * 右环状态成立条件：
     *   1. RLD10 小于 15，左侧边界整体偏离不大
     *   2. Point2 至少比 Point1 低 1 行
     *   3. Ring_Help_Flag 已经为 1，来源可能是：
     *      - 前面的 RightBorder 形态扫描
     *      - 上面这段 Right_Line > 7 的第二确认路径
     *   4. image_element_rings_flag 仍然为 0，表示环岛流程尚未开始
     */
    if((Right_RingsFlag_Point2_Ysite >= (Right_RingsFlag_Point1_Ysite + 1)) &&
       (Ring_Help_Flag == 1) &&
       (ImageFlag.image_element_rings_flag == 0))
    {
        ImageFlag.image_element_rings = 2;
        ImageFlag.image_element_rings_flag = 1;
        ImageFlag.ring_big_small = 1;
        ImageStatus.Road_type = ROAD_RIGHT_RING;
        buzzer_short();
    }

    Ring_Help_Flag = 0;
}

/**
 * @brief  元素识别入口
 * @note   目前只识别左右环岛
 */
static void image_element_test(void)
{
    if((ImageStatus.Road_type != ROAD_LEFT_RING) &&
       (ImageStatus.Road_type != ROAD_RIGHT_RING))
    {
        ImageStatus.Road_type = ROAD_NORMAL;
    }

    if(ImageStatus.Road_type != ROAD_LEFT_RING &&
       ImageStatus.Road_type != ROAD_RIGHT_RING)
    {
        image_judge_left_ring();
        image_judge_right_ring();
    }
}

/**
 * @brief  左环岛处理（补线和状态机推进）
 * @note   状态机：1/2=入口 -> 5/6=内部 -> 7/8=出口 -> 9=完成
 */
static void image_handle_left_ring(void)
{
    int16 num;
    int16 flag_x;
    int16 flag_y;
    float slope;

    num = 0;
    flag_x = 0;
    flag_y = 0;
    slope = 0.0f;

    for(Ysite = 55; Ysite > 30; Ysite--)
    {
        if(ImageDeal[Ysite].IsLeftFind == 'W')
        {
            num++;
        }
        if((ImageDeal[Ysite + 3].IsLeftFind == 'W') &&
           (ImageDeal[Ysite + 2].IsLeftFind == 'W') &&
           (ImageDeal[Ysite + 1].IsLeftFind == 'W') &&
           (ImageDeal[Ysite].IsLeftFind == 'T'))
        {
            break;
        }
    }

    if((ImageFlag.image_element_rings_flag == 1) && (num > 10))
    {
        ImageFlag.image_element_rings_flag = 2;
    }
    if((ImageFlag.image_element_rings_flag == 2) && (num < 8))
    {
        ImageFlag.image_element_rings_flag = 5;
    }
    if((ImageFlag.image_element_rings_flag == 5) && (ImageStatus.Right_Line > 15))
    {
        ImageFlag.image_element_rings_flag = 6;
    }
    if((ImageFlag.image_element_rings_flag == 6) && (ImageStatus.Right_Line < 3))
    {
        ImageFlag.image_element_rings_flag = 7;
        buzzer_short();
    }
    if((ImageFlag.ring_big_small == 1) && (ImageFlag.image_element_rings_flag == 7))
    {
        Point_Ysite = 0;
        Point_Xsite = 0;
        for(Ysite = 50; Ysite > (ImageStatus.OFFLine + 3); Ysite--)
        {
            if((ImageDeal[Ysite].RightBorder <= ImageDeal[Ysite + 2].RightBorder) &&
               (ImageDeal[Ysite].RightBorder <= ImageDeal[Ysite - 2].RightBorder) &&
               (ImageDeal[Ysite].RightBorder <= ImageDeal[Ysite + 1].RightBorder) &&
               (ImageDeal[Ysite].RightBorder <= ImageDeal[Ysite - 1].RightBorder) &&
               (ImageDeal[Ysite].RightBorder <= ImageDeal[Ysite + 4].RightBorder) &&
               (ImageDeal[Ysite].RightBorder <= ImageDeal[Ysite - 4].RightBorder))
            {
                Point_Xsite = ImageDeal[Ysite].RightBorder;
                Point_Ysite = Ysite;
                break;
            }
        }
        if(Point_Ysite > 24)
        {
            ImageFlag.image_element_rings_flag = 8;
        }
    }
    if(ImageFlag.image_element_rings_flag == 8)
    {
        if((ImageStatus.Right_Line < 9) && (ImageStatus.OFFLine < 10))
        {
            ImageFlag.image_element_rings_flag = 9;
            buzzer_short();
        }
    }
    if(ImageFlag.image_element_rings_flag == 9)
    {
        num = 0;
        for(Ysite = 40; Ysite > 10; Ysite--)
        {
            if(ImageDeal[Ysite].IsLeftFind == 'W')
            {
                num++;
            }
        }
        if(num < 5)
        {
            ImageStatus.Road_type = ROAD_NORMAL;
            ImageFlag.image_element_rings_flag = 0;
            ImageFlag.image_element_rings = 0;
            ImageFlag.ring_big_small = 0;
        }
    }

    if((ImageFlag.image_element_rings_flag == 1) ||
       (ImageFlag.image_element_rings_flag == 2) ||
       (ImageFlag.image_element_rings_flag == 3) ||
       (ImageFlag.image_element_rings_flag == 4))
    {
        for(Ysite = 57; Ysite > ImageStatus.OFFLine; Ysite--)
        {
            ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBorder - Half_Road_Wide[Ysite] - 5;
        }
    }
    if((ImageFlag.image_element_rings_flag == 5) ||
       (ImageFlag.image_element_rings_flag == 6))
    {
        for(Ysite = 55; Ysite > ImageStatus.OFFLine; Ysite--)
        {
            for(Xsite = ImageDeal[Ysite].LeftBorder + 1; Xsite < (ImageDeal[Ysite].RightBorder - 1); Xsite++)
            {
                if((ImageBin[Ysite][Xsite] == 1) && (ImageBin[Ysite][Xsite + 1] == 0))
                {
                    flag_y = Ysite;
                    flag_x = Xsite;
                    slope = (float)(79 - flag_x) / (float)(59 - flag_y);
                    break;
                }
            }
            if(flag_y != 0)
            {
                break;
            }
        }

        if(flag_y == 0)
        {
            for(Ysite = ImageStatus.OFFLine + 1; Ysite < 30; Ysite++)
            {
                if((ImageDeal[Ysite].IsLeftFind == 'T') &&
                   (ImageDeal[Ysite + 1].IsLeftFind == 'T') &&
                   (ImageDeal[Ysite + 2].IsLeftFind == 'W') &&
                   (IMAGE_ABS(ImageDeal[Ysite].LeftBorder - ImageDeal[Ysite + 2].LeftBorder) > 10))
                {
                    flag_y = Ysite;
                    flag_x = ImageDeal[flag_y].LeftBorder;
                    ImageStatus.OFFLine = Ysite;
                    slope = (float)(79 - flag_x) / (float)(59 - flag_y);
                    break;
                }
            }
        }

        if(flag_y != 0)
        {
            for(Ysite = flag_y; Ysite < 60; Ysite++)
            {
                ImageDeal[Ysite].RightBorder = flag_x + (int16)(slope * (Ysite - flag_y));
                ImageDeal[Ysite].Center = (ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) / 2;
                if(ImageDeal[Ysite].Center < 4)
                {
                    ImageDeal[Ysite].Center = 4;
                }
            }
            ImageDeal[flag_y].RightBorder = flag_x;
            for(Ysite = flag_y - 1; Ysite > 10; Ysite--)
            {
                for(Xsite = ImageDeal[Ysite + 1].RightBorder - 10; Xsite < (ImageDeal[Ysite + 1].RightBorder + 2); Xsite++)
                {
                    if((ImageBin[Ysite][Xsite] == 1) && (ImageBin[Ysite][Xsite + 1] == 0))
                    {
                        ImageDeal[Ysite].RightBorder = Xsite;
                        ImageDeal[Ysite].Center = (ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) / 2;
                        if(ImageDeal[Ysite].Center < 4)
                        {
                            ImageDeal[Ysite].Center = 4;
                        }
                        ImageDeal[Ysite].Wide = ImageDeal[Ysite].RightBorder - ImageDeal[Ysite].LeftBorder;
                        break;
                    }
                }

                if((ImageDeal[Ysite].Wide > 8) &&
                   (ImageDeal[Ysite].RightBorder < ImageDeal[Ysite + 2].RightBorder))
                {
                    continue;
                }
                else
                {
                    ImageStatus.OFFLine = Ysite + 2;
                    break;
                }
            }
        }
    }
    if((ImageFlag.image_element_rings_flag == 8) && (ImageFlag.ring_big_small == 1))
    {
        Repair_Point_Xsite = 20;
        Repair_Point_Ysite = 0;
        for(Ysite = 40; Ysite > 5; Ysite--)
        {
            if((ImageBin[Ysite][20] == 1) && (ImageBin[Ysite - 1][20] == 0))
            {
                Repair_Point_Xsite = 20;
                Repair_Point_Ysite = Ysite - 1;
                ImageStatus.OFFLine = Ysite + 1;
                break;
            }
        }

        if(Repair_Point_Ysite > 0)
        {
            for(Ysite = 57; Ysite > (Repair_Point_Ysite - 3); Ysite--)
            {
                ImageDeal[Ysite].RightBorder =
                    (ImageDeal[58].RightBorder - Repair_Point_Xsite) * (Ysite - 58) /
                    (58 - Repair_Point_Ysite) + ImageDeal[58].RightBorder;
                ImageDeal[Ysite].Center = (ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) / 2;
            }
        }
    }
    if((ImageFlag.image_element_rings_flag == 9) ||
       (ImageFlag.image_element_rings_flag == 10))
    {
        for(Ysite = 59; Ysite > ImageStatus.OFFLine; Ysite--)
        {
            ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBorder - Half_Road_Wide[Ysite];
        }
    }
}

/**
 * @brief  右环岛处理（补线和状态机推进）
 * @note   状态机：1/2=入口 -> 5/6=内部 -> 7/8=出口 -> 9=完成
 */
static void image_handle_right_ring(void)
{
    int16 num;
    int16 flag_x;
    int16 flag_y;
    float slope;

    num = 0;
    flag_x = 0;
    flag_y = 0;
    slope = 0.0f;

    for(Ysite = 55; Ysite > 30; Ysite--)
    {
        if(ImageDeal[Ysite].IsRightFind == 'W')
        {
            num++;
        }
        if((ImageDeal[Ysite + 3].IsRightFind == 'W') &&
           (ImageDeal[Ysite + 2].IsRightFind == 'W') &&
           (ImageDeal[Ysite + 1].IsRightFind == 'W') &&
           (ImageDeal[Ysite].IsRightFind == 'T'))
        {
            break;
        }
    }
    if((ImageFlag.image_element_rings_flag == 1) && (num > 10))
    {
        ImageFlag.image_element_rings_flag = 2;
    }
    if((ImageFlag.image_element_rings_flag == 2) && (num < 8))
    {
        ImageFlag.image_element_rings_flag = 5;
    }
    if((ImageFlag.image_element_rings_flag == 5) && (ImageStatus.Left_Line > 15))
    {
        ImageFlag.image_element_rings_flag = 6;
    }
    if((ImageFlag.image_element_rings_flag == 6) && (ImageStatus.Left_Line < 4))
    {
        ImageFlag.image_element_rings_flag = 7;
        buzzer_short();
    }
    if(ImageFlag.image_element_rings_flag == 7)
    {
        Point_Xsite = 0;
        Point_Ysite = 0;
        for(Ysite = 45; Ysite > (ImageStatus.OFFLine + 3); Ysite--)
        {
            if((ImageDeal[Ysite].LeftBorder >= ImageDeal[Ysite + 2].LeftBorder) &&
               (ImageDeal[Ysite].LeftBorder >= ImageDeal[Ysite - 2].LeftBorder) &&
               (ImageDeal[Ysite].LeftBorder >= ImageDeal[Ysite + 1].LeftBorder) &&
               (ImageDeal[Ysite].LeftBorder >= ImageDeal[Ysite - 1].LeftBorder) &&
               (ImageDeal[Ysite].LeftBorder >= ImageDeal[Ysite + 4].LeftBorder) &&
               (ImageDeal[Ysite].LeftBorder >= ImageDeal[Ysite - 4].LeftBorder))
            {
                Point_Xsite = ImageDeal[Ysite].LeftBorder;
                Point_Ysite = Ysite;
                break;
            }
        }
        if(Point_Ysite > 22)
        {
            ImageFlag.image_element_rings_flag = 8;
        }
    }
    if(ImageFlag.image_element_rings_flag == 8)
    {
        if((Straight_Judge(1, ImageStatus.OFFLine + 10, 45) < 10) &&    /* 原 < 1.0f, ×10 */
           (ImageStatus.Left_Line < 9) &&
           (ImageStatus.OFFLine < 20))
        {
            ImageFlag.image_element_rings_flag = 9;
            buzzer_short();
        }
    }
    if(ImageFlag.image_element_rings_flag == 9)
    {
        num = 0;
        for(Ysite = 40; Ysite > 10; Ysite--)
        {
            if(ImageDeal[Ysite].IsRightFind == 'W')
            {
                num++;
            }
        }
        if(num < 5)
        {
            ImageStatus.Road_type = ROAD_NORMAL;
            ImageFlag.image_element_rings_flag = 0;
            ImageFlag.image_element_rings = 0;
            ImageFlag.ring_big_small = 0;
        }
    }
    if((ImageFlag.image_element_rings_flag == 1) ||
       (ImageFlag.image_element_rings_flag == 2) ||
       (ImageFlag.image_element_rings_flag == 3) ||
       (ImageFlag.image_element_rings_flag == 4))
    {
        for(Ysite = 59; Ysite > ImageStatus.OFFLine; Ysite--)
        {
            ImageDeal[Ysite].Center = ImageDeal[Ysite].LeftBorder + Half_Road_Wide[Ysite];
        }
    }
    if((ImageFlag.image_element_rings_flag == 5) ||
       (ImageFlag.image_element_rings_flag == 6))
    {
        for(Ysite = 55; Ysite > ImageStatus.OFFLine; Ysite--)
        {
            for(Xsite = ImageDeal[Ysite].LeftBorder + 1; Xsite < (ImageDeal[Ysite].RightBorder - 1); Xsite++)
            {
                if((ImageBin[Ysite][Xsite] == 1) && (ImageBin[Ysite][Xsite + 1] == 0))
                {
                    flag_y = Ysite;
                    flag_x = Xsite;
                    slope = (float)(0 - flag_x) / (float)(59 - flag_y);
                    break;
                }
            }
            if(flag_y != 0)
            {
                break;
            }
        }
        if(flag_y == 0)
        {
            for(Ysite = ImageStatus.OFFLine + 5; Ysite < 30; Ysite++)
            {
                if((ImageDeal[Ysite].IsRightFind == 'T') &&
                   (ImageDeal[Ysite + 1].IsRightFind == 'T') &&
                   (ImageDeal[Ysite + 2].IsRightFind == 'W') &&
                   (IMAGE_ABS(ImageDeal[Ysite].RightBorder - ImageDeal[Ysite + 2].RightBorder) > 10))
                {
                    flag_y = Ysite;
                    flag_x = ImageDeal[flag_y].RightBorder;
                    ImageStatus.OFFLine = Ysite;
                    slope = (float)(0 - flag_x) / (float)(59 - flag_y);
                    break;
                }
            }
        }
        if(flag_y != 0)
        {
            for(Ysite = flag_y; Ysite < 58; Ysite++)
            {
                ImageDeal[Ysite].LeftBorder = flag_x + (int16)(slope * (Ysite - flag_y));
                ImageDeal[Ysite].Center = (ImageDeal[Ysite].LeftBorder + ImageDeal[Ysite].RightBorder) / 2;
                if(ImageDeal[Ysite].Center > 79)
                {
                    ImageDeal[Ysite].Center = 79;
                }
            }
            ImageDeal[flag_y].LeftBorder = flag_x;
            for(Ysite = flag_y - 1; Ysite > 10; Ysite--)
            {
                for(Xsite = ImageDeal[Ysite + 1].LeftBorder + 8; Xsite > (ImageDeal[Ysite + 1].LeftBorder - 4); Xsite--)
                {
                    if((ImageBin[Ysite][Xsite] == 1) && (ImageBin[Ysite][Xsite - 1] == 0))
                    {
                        ImageDeal[Ysite].LeftBorder = Xsite;
                        ImageDeal[Ysite].Wide = ImageDeal[Ysite].RightBorder - ImageDeal[Ysite].LeftBorder;
                        ImageDeal[Ysite].Center = (ImageDeal[Ysite].LeftBorder + ImageDeal[Ysite].RightBorder) / 2;
                        if(ImageDeal[Ysite].Center > 79)
                        {
                            ImageDeal[Ysite].Center = 79;
                        }
                        if(ImageDeal[Ysite].Center < 5)
                        {
                            ImageDeal[Ysite].Center = 5;
                        }
                        break;
                    }
                }
                if((ImageDeal[Ysite].Wide > 8) &&
                   (ImageDeal[Ysite].LeftBorder > ImageDeal[Ysite + 2].LeftBorder))
                {
                    continue;
                }
                else
                {
                    ImageStatus.OFFLine = Ysite + 2;
                    break;
                }
            }
        }
    }
    if(ImageFlag.image_element_rings_flag == 8)
    {
        Repair_Point_Xsite = 60;
        Repair_Point_Ysite = 0;
        for(Ysite = 40; Ysite > 5; Ysite--)
        {
            if((ImageBin[Ysite][60] == 1) && (ImageBin[Ysite - 1][60] == 0))
            {
                Repair_Point_Xsite = 60;
                Repair_Point_Ysite = Ysite - 1;
                ImageStatus.OFFLine = Ysite + 1;
                break;
            }
        }
        if(Repair_Point_Ysite > 0)
        {
            for(Ysite = 57; Ysite > (Repair_Point_Ysite - 3); Ysite--)
            {
                ImageDeal[Ysite].LeftBorder =
                    (ImageDeal[58].LeftBorder - Repair_Point_Xsite) * (Ysite - 58) /
                    (58 - Repair_Point_Ysite) + ImageDeal[58].LeftBorder;
                ImageDeal[Ysite].Center = (ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) / 2;
            }
        }
    }
    if(ImageFlag.image_element_rings_flag == 9)
    {
        for(Ysite = 59; Ysite > ImageStatus.OFFLine; Ysite--)
        {
            ImageDeal[Ysite].Center = ImageDeal[Ysite].LeftBorder + Half_Road_Wide[Ysite];
        }
    }
}

/**
 * @brief  元素处理分发
 */
static void image_element_handle(void)
{
    if(ImageFlag.image_element_rings == 1)
    {
        image_handle_left_ring();
    }
    else if(ImageFlag.image_element_rings == 2)
    {
        image_handle_right_ring();
    }
}

/**
 * @brief  扫描斑马线（通过统计黑白跳变次数）
 * @return 1=检测到斑马线, 0=未检测到
 */
static uint8 image_zebra_scan(void)
{
    int16 row;
    int16 col;
    int16 left_limit;
    int16 right_limit;
    uint8 edge_count;

    /* 环岛期间不检测斑马线 */
    if((ImageStatus.Road_type == ROAD_LEFT_RING) ||
       (ImageStatus.Road_type == ROAD_RIGHT_RING))
    {
        return 0;
    }

    /* 在45-55行范围内扫描 */
    for(row = 45; row < 55; row++)
    {
        edge_count = 0;
        left_limit = ImageDeal[row].LeftBoundary - 5;
        right_limit = ImageDeal[row].RightBoundary + 5;

        /* 限制扫描范围 */
        if(left_limit < 0)
        {
            left_limit = 0;
        }
        if(left_limit > 77)
        {
            left_limit = 77;
        }
        if(right_limit < 1)
        {
            right_limit = 1;
        }
        if(right_limit > 78)
        {
            right_limit = 78;
        }
        if(left_limit >= right_limit)
        {
            continue;
        }

        /* 统计黑到白的跳变次数 */
        for(col = left_limit; col < right_limit; col++)
        {
            if((ImageBin[row][col] == IMAGE_BLACK) &&
               (ImageBin[row][col + 1] == IMAGE_WHITE))
            {
                edge_count++;
                if(edge_count > IMAGE_ZEBRA_EDGE_MIN)
                {
                    return 1;
                }
            }
        }
    }

    return 0;
}

/**
 * @brief  斑马线检测和停车控制
 * @note   带锁存和冷却机制，防止误触发
 */
static void image_check_zebra(void)
{
    ZebraHit = image_zebra_scan();

    if(CarMode != CAR_MODE_RUN)
    {
        return;
    }

    /* 冷却倒计时 */
    if(ZebraCooldownFrames > 0)
    {
        ZebraCooldownFrames--;
    }

    if(ZebraHit)
    {
        ZebraMissFrames = 0;
        if(ZebraFrameLatch == 0)
        {
            ZebraFrameLatch = 1;
            if(ZebraCooldownFrames == 0)
            {
                if(ZebraDetectCount < IMAGE_ZEBRA_STOP_COUNT)
                {
                    ZebraDetectCount++;
                }

                buzzer_short();
                if(ZebraDetectCount >= IMAGE_ZEBRA_STOP_COUNT)
                {
                    CarMode = CAR_MODE_STOP;
                }

                ZebraCooldownFrames = IMAGE_ZEBRA_COOLDOWN_FRAMES;
            }
        }
    }
    else
    {
        if(ZebraMissFrames < IMAGE_ZEBRA_MISS_COUNT)
        {
            ZebraMissFrames++;
        }
        if(ZebraMissFrames >= IMAGE_ZEBRA_MISS_COUNT)
        {
            ZebraFrameLatch = 0;
        }
    }
}

/**
 * @brief  直道检测（环岛用）
 * @note   使用左右边线拟合误差判断，连续3帧确认
 */
static void image_check_straight(void)
{
    static uint8 straight_count = 0;
    int16 left_err_x10;     /* 整数版，Straight_Judge 已返回 ×10 */
    int16 right_err_x10;

    /* 计算左右边线的直线度（均方误差 ×10） */
    left_err_x10  = Straight_Judge(1, 25, 45);
    right_err_x10 = Straight_Judge(2, 25, 45);

    /* 导出误差值到Image结构体（已 ×10，直接赋值） */
    Image.straight_left_error_x10  = left_err_x10;
    Image.straight_right_error_x10 = right_err_x10;

    /* 左右边线均为直线，连续3帧确认 (原 < 1.0f, ×10 后 < 10) */
    if((left_err_x10 < 10) && (right_err_x10 < 10))
    {
        straight_count++;
        if(straight_count >= 3)
        {
            Image.is_straight = 1;
        }
    }
    else
    {
        straight_count = 0;
        Image.is_straight = 0;
    }
}

/**
 * @brief  长直道加速检测（改进版：覆盖中、大等级直道）
 * @note   使用中线方差 + 可视距离 + 丢线判断，区分长短直道
 *         - 中线方差：衡量中线的平直程度，比边线拟合更稳定
 *         - OFFLine <= 12：可视距离较远（覆盖中、大等级直道）
 *         - Left_Line/Right_Line < 3：允许少量丢线（放宽条件）
 */
static void image_check_long_straight(void)
{
    static uint8 long_straight_count = 0;
    static float variance_acc_threshold = 80.0f;  /* 方差阈值（放宽到80） */
    float variance_acc;
    int32 sum;
    int16 valid_rows;

    sum = 0;

    /* 计算中线方差（衡量中线偏离中心的程度） */
    for(Ysite = 55; Ysite > (ImageStatus.OFFLine + 1); Ysite--)
    {
        sum += (ImageDeal[Ysite].Center - IMAGE_MID) *
               (ImageDeal[Ysite].Center - IMAGE_MID);
    }

    valid_rows = 54 - ImageStatus.OFFLine;
    if(valid_rows > 0)
    {
        variance_acc = (float)sum / (float)valid_rows;
    }
    else
    {
        variance_acc = 9999.0f;  /* 无效值 */
    }

    /* 导出方差值供UI显示 */
    Image.straight_variance_x10 = (int16)(variance_acc * 10.0f);

    /* 中长直道判断：方差较小 + 可视距离较远 + 少量丢线允许 */
    if(  variance_acc < variance_acc_threshold      /* 中线够直（放宽到80）*/
      && ImageStatus.OFFLine <= 12                  /* 可视距离较远（覆盖中、大直道）*/
      && ImageStatus.Left_Line < 3                  /* 左侧少量丢线允许 */
      && ImageStatus.Right_Line < 3                 /* 右侧少量丢线允许 */
      )
    {
        long_straight_count++;
        if(long_straight_count >= 3)  /* 连续3帧确认 */
        {
            Image.is_long_straight = 1;
        }
    }
    else
    {
        long_straight_count = 0;
        Image.is_long_straight = 0;
    }
}

/**
 * @brief  坡道检测（参考时光-贺兰一号算法）
 * @note   坡道特征：可视距离很近（OFFLine=2）+ 赛道很宽且居中
 *         检测到坡道后将 Road_type 设为 ROAD_RAMP，触发降速
 */
static void image_check_ramp(void)
{
    static uint8 ramp_detected_flag = 0;    /* 已检测到坡道标志 */
    static uint8 ramp_exit_stable = 0;      /* 退出稳定计数（连续N帧Diff>35才退出）*/
    int valid_count;
    int16 width_bottom;   /* 底部宽度 */
    int16 width_top;      /* 顶部宽度 */
    int16 width_diff;     /* 宽度差 */
    ImageDealDatatypedef xdata *p;   /* xdata 指针（2字节，比 generic 3字节省 1 字节加载） */

    /* 环岛期间不检测坡道 */
    if((ImageStatus.Road_type == ROAD_LEFT_RING) ||
       (ImageStatus.Road_type == ROAD_RIGHT_RING))
    {
        return;
    }

    /* 计算宽度差（用于判断和UI显示）*/
    width_bottom = ImageDeal[55].Wide;
    width_top = ImageDeal[10].Wide;
    width_diff = width_bottom - width_top;

    /* 坡道检测：三角形变钝 + 左右居中 + 边线直度 + 对称性 */
    if(ramp_detected_flag == 0)
    {
        /* 判断条件（超严格）：
         * 1. 底部宽度 > 58（赛道很宽）
         * 2. 宽度差在 24~29 之间（收窄范围，更精确）
         * 3. 左右边线拟合误差都很小（< 3，即 < 0.3）
         * 4. 左右边线误差接近（差值 < 2），高度对称
         * 5. 中线偏差很小（|Image.error| < 2），几乎居中
         * 6. 底部居中更严格（LeftBorder < 38 && RightBorder > 42）
         */
        if((width_bottom > 58)
           && (width_diff >= 24) && (width_diff <= 29)
           && (Image.straight_left_error_x10 < 3)
           && (Image.straight_right_error_x10 < 3)
           && (IMAGE_ABS(Image.straight_left_error_x10 - Image.straight_right_error_x10) < 2)  /* 左右高度对称 */
           && (IMAGE_ABS(Image.error) < 2))  /* 几乎居中 */
        {
            valid_count = 0;

            /* 检查第10-55行，至少35行满足：找到边线 + 严格居中
             * 优化：p++ 一次加 sizeof(struct)=58 字节（编译器常量），比 Ysite*58+offset 省
             */
            for(p = &ImageDeal[10]; p <= &ImageDeal[55]; p++)
            {
                if(  (p->IsLeftFind == 'T')
                  && (p->IsRightFind == 'T')
                  && (p->LeftBorder < 38)    /* 更严格的居中判断 */
                  && (p->RightBorder > 42)
                  )
                {
                    valid_count++;
                }
            }

            /* 至少35行满足条件，判定为坡道 */
            if(valid_count >= 35)
            {
                ImageStatus.Road_type = ROAD_RAMP;
                Image.is_ramp = 1;
                Image.ramp_count++;
                ramp_detected_flag = 1;
                ramp_exit_stable = 0;  /* 重置退出计数 */
                buzzer_short();
            }
        }
    }

    /* 退出条件：宽度差恢复正常（> 35），且连续稳定10帧 */
    if(ramp_detected_flag == 1)
    {
        /* 宽度差大于35，说明可能离开坡道了 */
        if(width_diff > 35)
        {
            ramp_exit_stable++;

            /* 连续10帧都满足，确认退出 */
            if(ramp_exit_stable >= 10)
            {
                ImageStatus.Road_type = ROAD_NORMAL;
                Image.is_ramp = 0;
                ramp_detected_flag = 0;
                ramp_exit_stable = 0;
            }
        }
        else
        {
            /* 如果宽度差又变小了，重置退出计数（还在坡道上）*/
            ramp_exit_stable = 0;
        }
    }
}

/**
 * @brief  计算加权中心（用于转向控制）
 * @param  tow_point  瞄点行
 * @note   在瞄点附近多行加权平均，增加稳定性
 */
static void image_get_det(uint8 tow_point)
{
    int32 acc = 0;
    uint16 unit_all_x10 = 0;
    int16 row;

    /* 情况1：瞄点上下各5行都可见 */
    if((tow_point - 5) >= ImageStatus.OFFLine)
    {
        for(row = (int16)(tow_point - 5); row < tow_point; row++)
        {
            uint8 w = WeightingX10[tow_point - row - 1];
            acc += (int32)w * ImageDeal[row].Center;
            unit_all_x10 += w;
        }
        for(row = (int16)(tow_point + 5); row > tow_point; row--)
        {
            uint8 w = WeightingX10[row - tow_point - 1];
            acc += (int32)w * ImageDeal[row].Center;
            unit_all_x10 += w;
        }
        /* 当前行权重 = 1.0 → ×10 = 10 */
        acc += (int32)ImageDeal[tow_point].Center * 10;
        unit_all_x10 += 10;
    }
    /* 情况2：瞄点上方被遮挡，只用下方 */
    else if(tow_point > ImageStatus.OFFLine)
    {
        for(row = ImageStatus.OFFLine; row < tow_point; row++)
        {
            uint8 w = WeightingX10[tow_point - row - 1];
            acc += (int32)w * ImageDeal[row].Center;
            unit_all_x10 += w;
        }
        for(row = (int16)(tow_point + tow_point - ImageStatus.OFFLine); row > tow_point; row--)
        {
            uint8 w = WeightingX10[row - tow_point - 1];
            acc += (int32)w * ImageDeal[row].Center;
            unit_all_x10 += w;
        }
        acc += (int32)ImageDeal[tow_point].Center * 10;
        unit_all_x10 += 10;
    }
    /* 情况3：有效行太少，用 OFFLine 附近 */
    else if(ImageStatus.OFFLine < 49)
    {
        for(row = (int16)(ImageStatus.OFFLine + 3); row > ImageStatus.OFFLine; row--)
        {
            uint8 w = WeightingX10[row - tow_point - 1];
            acc += (int32)w * ImageDeal[row].Center;
            unit_all_x10 += w;
        }
        acc += (int32)ImageDeal[ImageStatus.OFFLine].Center * 10;
        unit_all_x10 += 10;
    }
    /* 情况4：严重丢线，保持上次值，直接返回 */
    else
    {
        return;
    }

    if(unit_all_x10 == 0) { return; }
    ImageStatus.Det_True = (int16)(acc / unit_all_x10);
    ImageStatus.TowPoint_True = tow_point;
}

/* =============================================================================
 * 完整帧处理流程
 * ============================================================================= */

/**
 * @brief  完整的图像处理流程
 * @note   从压缩到最终结果的完整流程
 */
static void image_process(void)
{
    gpio_set_level(LED_DEBUG, GPIO_LOW);

    image_compress();              /* 1. 压缩原始图像到80x60 */
    image_clear_deal();            /* 2. 清空上一帧结果 */
    image_binarize(image_otsu());  /* 3. 大津法阈值+二值化 */
    image_draw_bottom();           /* 4. 初始化底部5行 */
    image_draw_lines();            /* 5. 从下往上跟踪边线 */
    image_search_border(IMAGE_H - 2); /* 6. 边界追踪（用于环岛识别） */
    image_element_test();          /* 7. 元素识别（环岛） */
    image_draw_extension_line();   /* 8. 延长线补线 */
    image_route_filter();          /* 9. 路径滤波 */
    image_element_handle();        /* 10. 元素处理（环岛补线） */
    image_check_zebra();           /* 11. 斑马线检测 */
    image_target_check();          /* 12. 打靶检测 + 激光触发 */
    image_check_ramp();            /* 12. 坡道检测 */
    image_check_straight();        /* 13. 直道检测（环岛用） */
    image_check_long_straight();   /* 14. 长直道加速检测 */
    image_get_det(image_tow_point()); /* 15. 计算加权中心 */
    image_export_result();         /* 16. 导出结果 */

    gpio_set_level(LED_DEBUG, GPIO_HIGH);
}

/* =============================================================================
 * 公共接口函数
 * ============================================================================= */

/**
 * @brief  图像处理更新（主循环调用）
 */
void image_update(void)
{
    image_target_update_laser_mode();

    if(Image.ready == 0)
    {
        return;
    }

    if(mt9v03x_finish_flag == 0)
    {
        return;
    }

    image_process();
    Image.sequence++;

    /* 非运行模式不检测丢线停车 */
    if(CarMode != CAR_MODE_RUN)
    {
        ImageLostCount = 0;
        ImageRunFrameCount = 0;
        if(image_target_laser_test_mode() == IMAGE_LASER_TEST_OFF)
        {
            LaserBusy = 0;
            LaserTickLeft = 0;
            image_target_laser_pit_stop();
            image_laser_all_off();
        }
        return;
    }

    /* 启动后前几帧忽略丢线 */
    if(ImageRunFrameCount < IMAGE_RUN_START_IGNORE_FRAMES)
    {
        ImageRunFrameCount++;
        ImageLostCount = 0;
        return;
    }

    /* 连续丢线停车保护 */
    if(Image.lost)
    {
        if(ImageLostCount < IMAGE_LOST_STOP_COUNT)
        {
            ImageLostCount++;
        }
        if(ImageLostCount >= IMAGE_LOST_STOP_COUNT)
        {
            CarMode = CAR_MODE_STOP;
        }
    }
    else
    {
        ImageLostCount = 0;
    }
}
