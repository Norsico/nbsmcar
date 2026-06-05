#include "headfile.h"

image_data Image;
uint8 ImageGray[IMAGE_H][IMAGE_W];
uint8 ImageBin[IMAGE_H][IMAGE_W];

/* Frame flow:
 * mt9v03x_image -> ImageGray -> ImageBin -> ImageDeal -> Image.
 * This file keeps base tracking, cross line fill, ring and zebra handling.
 * Ramp and target-ring laser logic are not included here.
 */
#define IMAGE_COMPRESS_CUT_COL         (1)
#define IMAGE_COMPRESS_CUT_ROW_TOP     (0)
#define IMAGE_COMPRESS_CUT_ROW_BOTTOM  (1)
#define IMAGE_COMPRESS_SRC_H           (MT9V03X_H - IMAGE_COMPRESS_CUT_ROW_TOP - IMAGE_COMPRESS_CUT_ROW_BOTTOM)
#define IMAGE_COMPRESS_SRC_W           (MT9V03X_W - (IMAGE_COMPRESS_CUT_COL * 2))

#define IMAGE_THRESHOLD_DETACH         (150)//二值化大阈值
#define IMAGE_THRESHOLD_STATIC         (40)	//二值化小阈值
#define IMAGE_STOP_RAW_THRESHOLD       (25)
#define IMAGE_OFFLINE_INIT             (2)
#define IMAGE_SCAN_INTERVAL            (3)
#define IMAGE_STRAIGHT_VARIANCE_MAX    (25)
#define IMAGE_ZEBRA_MISS_COUNT         (3)
#define IMAGE_ZEBRA_COOLDOWN_FRAMES    (80)
#define IMAGE_ZEBRA_EDGE_MIN           (4)
#define IMAGE_ZEBRA_STOP_COUNT         (2)
#define IMAGE_LOST_STOP_COUNT          (4)
#define IMAGE_RUN_START_IGNORE_FRAMES  (3)

#define LimitL(L)                      (L = ((L < 1) ? 1 : L))
#define LimitH(H)                      (H = ((H > 78) ? 78 : H))
#define IMAGE_ABS(V)                   (((V) < 0) ? (-(V)) : (V))


typedef struct
{
    int16 point;
    uint8 type;          /* T: edge found, W: white/no edge, H: hidden */
} image_jump;

typedef struct
{
    uint8 IsRightFind;   /* right edge flag */
    uint8 IsLeftFind;    /* left edge flag */
    int16 Wide;          /* RightBorder - LeftBorder */
    int16 LeftBorder;    /* left edge column */
    int16 RightBorder;   /* right edge column */
    int16 close_LeftBorder;
    int16 close_RightBorder;
    int16 Center;        /* row center column */
    int16 RightTemp;
    int16 LeftTemp;
    int16 LeftBoundary_First;
    int16 RightBoundary_First;
    int16 LeftBoundary;
    int16 RightBoundary;
} image_deal;

typedef struct
{
    uint8 TowPoint;          /* configured tow point */
    int16 TowPoint_True;     /* tow point after visible-range limit */
    int16 Det_True;          /* weighted center column */
    uint8 Threshold;         /* final threshold after lower limit */
    uint16 Threshold_static; /* threshold lower limit */
    uint8 Threshold_detach;  /* otsu scan upper limit */
    uint8 Left_Line;         /* left side white/no-edge rows */
    uint8 Right_Line;        /* right side white/no-edge rows */
    uint8 OFFLine;           /* first reliable row from top */
    uint8 WhiteLine;         /* both sides white/no-edge rows */
    RoadType_e Road_type;
    int16 WhiteLine_L;
    int16 WhiteLine_R;
    int16 OFFLineBoundary;
    int16 straight_acc;
    int16 variance_acc;
} image_status;

typedef struct
{
    int16 image_element_rings;       /* 0:none, 1:left ring, 2:right ring */
    int16 ring_big_small;
    int16 image_element_rings_flag;  /* 1/2: mouth found, 5/6: enter fill, 7/8: inside/exit, 9: finish */
    int16 straight_long;
} image_flag;

static image_deal ImageDeal[IMAGE_H];
static image_status ImageStatus =
{
    SERVO_POINT,
    SERVO_POINT,
    IMAGE_MID,
    0,
    IMAGE_THRESHOLD_STATIC,
    IMAGE_THRESHOLD_DETACH,
    0,
    0,
    IMAGE_OFFLINE_INIT,
    0,
    ROAD_NORMAL,
    0,
    0,
    5,
    0,
    0
};

static image_flag ImageFlag = {0};

static uint8 ImageRowMap[IMAGE_H];
static uint8 ImageColMap[IMAGE_W];
static uint8 ImageMapReady = 0;
static uint16 ImageHist[256];
static uint8 ImageRawThreshold = 0;
static uint8 ZebraHit = 0;
static uint8 ZebraDetectCount = 0;
static uint8 ZebraFrameLatch = 0;
static uint8 ZebraMissFrames = 0;
static uint8 ZebraCooldownFrames = 0;
static uint8 ImageLostCount = 0;
static uint8 ImageRunFrameCount = 0;

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
static uint8 Ring_Help_Flag = 0;
static int16 Left_RingsFlag_Point1_Ysite = 0;
static int16 Left_RingsFlag_Point2_Ysite = 0;
static int16 Right_RingsFlag_Point1_Ysite = 0;
static int16 Right_RingsFlag_Point2_Ysite = 0;
static int16 Point_Xsite = 0;
static int16 Point_Ysite = 0;
static int16 Repair_Point_Xsite = 0;
static int16 Repair_Point_Ysite = 0;

static const uint8 Half_Road_Wide[IMAGE_H] =
{
    6, 7, 7, 8, 8, 9, 9, 9, 10, 10,
    11, 11, 11, 11, 11, 12, 12, 13, 13, 14,
    14, 14, 14, 15, 15, 16, 16, 16, 17, 17,
    17, 18, 18, 19, 19, 20, 20, 20, 21, 21,
    21, 22, 22, 23, 23, 23, 24, 24, 25, 25,
    26, 26, 26, 26, 27, 27, 27, 28, 28, 30
};

static const float Weighting[10] =
{
    0.96f, 0.92f, 0.88f, 0.83f, 0.77f,
    0.71f, 0.65f, 0.59f, 0.53f, 0.47f
};

static uint8 image_tow_point(void)
{
    int16 tow_point;

    if((ImageFlag.image_element_rings_flag == 1) ||
       (ImageFlag.image_element_rings_flag == 2))
    {
        tow_point = 30;
    }
    else if(ImageFlag.image_element_rings != 0)
    {
        tow_point = 28;
    }
    else
    {
        tow_point = SmartCar.servo.tow_point;
    }
    if(tow_point < ImageStatus.OFFLine)
    {
        tow_point = ImageStatus.OFFLine + 1;
    }
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

/* Clear per-row tracking result before processing a new frame. */
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

/* Copy internal result to the public Image structure used by UI/servo. */
static void image_export_result(void)
{
    Image.threshold = ImageStatus.Threshold;
    Image.tow_row = (uint8)ImageStatus.TowPoint_True;
    Image.center = ImageStatus.Det_True;
    Image.error = (int16)(Image.center - IMAGE_MID);
    Image.valid_count = (uint8)((ImageStatus.OFFLine < IMAGE_H) ? (IMAGE_H - ImageStatus.OFFLine) : 0);
    Image.lost = 0;

    if(ImageRawThreshold < IMAGE_STOP_RAW_THRESHOLD)
    {
        Image.lost = 1;
    }
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

/* Map 80-column image coordinate to the preview area. */
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

/* Map 60-row image coordinate to the preview area. */
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

/* Draw green borders, red center line and yellow tow row on UI preview. */
void image_show_debug_overlay(uint16 x, uint16 y, uint16 w, uint16 h)
{
    uint8 row;
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

    mt9v03x_sccb_set_config(config);
}

void image_init(void)
{
    uint8 retry;

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
    ImageRawThreshold = 0;
    ZebraHit = 0;
    ZebraDetectCount = 0;
    ZebraFrameLatch = 0;
    ZebraMissFrames = 0;
    ZebraCooldownFrames = 0;
    ImageLostCount = 0;
    ImageRunFrameCount = 0;

    gpio_init(LED_DEBUG, GPO, GPIO_HIGH, GPO_PUSH_PULL);

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

/* Compress MT9V03X raw frame to 80x60 grayscale buffer. */
static void image_compress(void)
{
    int16 row;
    int16 col;
    uint8 src_row;
    uint8 *dst;
    uint8 *src;

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

/* Otsu threshold on ImageGray. */
static uint8 image_otsu(void)
{
    int16 row;
    int16 col;
    int16 i;
    uint16 total;
    uint32 gray_sum;
    float pixel_sum;
    float gray_average;
    float w0;
    float w1;
    float u0tmp;
    float u1tmp;
    float u0;
    float u1;
    float delta_tmp;
    float delta_max;
    float diff0;
    float diff1;
    uint8 threshold;

    for(i = 0; i < 256; i++)
    {
        ImageHist[i] = 0;
    }

    total = IMAGE_W * IMAGE_H;
    pixel_sum = (float)total;
    gray_sum = 0;
    for(row = 0; row < IMAGE_H; row++)
    {
        for(col = 0; col < IMAGE_W; col++)
        {
            ImageHist[ImageGray[row][col]]++;
            gray_sum += ImageGray[row][col];
        }
    }

    gray_average = (float)gray_sum / pixel_sum;
    w0 = 0.0f;
    w1 = 0.0f;
    u0tmp = 0.0f;
    u1tmp = 0.0f;
    u0 = 0.0f;
    u1 = 0.0f;
    delta_tmp = 0.0f;
    delta_max = 0.0f;
    threshold = 0;

    for(i = 0; i < ImageStatus.Threshold_detach; i++)
    {
        w0 += (float)ImageHist[i] / pixel_sum;
        u0tmp += (float)i * (float)ImageHist[i] / pixel_sum;
        if(w0 <= 0.0f)
        {
            continue;
        }

        w1 = 1.0f - w0;
        if(w1 <= 0.0f)
        {
            break;
        }

        u1tmp = gray_average - u0tmp;
        u0 = u0tmp / w0;
        u1 = u1tmp / w1;
        diff0 = u0 - gray_average;
        diff1 = u1 - gray_average;
        delta_tmp = (w0 * diff0 * diff0) + (w1 * diff1 * diff1);

        if(delta_tmp > delta_max)
        {
            delta_max = delta_tmp;
            threshold = (uint8)i;
        }

        /* Same threshold peak rule as the 19th reference code. */
        if(delta_tmp < delta_max)
        {
            break;
        }
    }

    return threshold;
}

/* Convert ImageGray to 0/1 ImageBin. */
static void image_binarize(uint8 threshold)
{
    uint8 row;
    uint8 col;
    uint8 thre;
    uint16 threshold_value;

    threshold_value = (uint16)threshold + SmartCar.camera.threshold_offset;
    if(threshold_value > 255)
    {
        threshold_value = 255;
    }

    threshold = (uint8)threshold_value;
    ImageRawThreshold = threshold;
    if(threshold < ImageStatus.Threshold_static)
    {
        threshold = (uint8)ImageStatus.Threshold_static;
    }

    ImageStatus.Threshold = threshold;
    Image.white_count = 0;

    for(row = 0; row < IMAGE_H; row++)
    {
        for(col = 0; col < IMAGE_W; col++)
        {
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

/* Find the first five bottom rows as the seed for edge tracking. */
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

/* Search one row around previous edge and classify the jump point. */
static void image_get_jump(uint8 *line, uint8 type, int16 low, int16 high, image_jump *jump)
{
    int16 i;

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
                    jump->type = 'W';
                }
                else
                {
                    jump->point = high;
                    jump->type = 'H';
                }
                break;
            }
        }
    }
    else
    {
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
                    jump->type = 'W';
                }
                else
                {
                    jump->point = low;
                    jump->type = 'H';
                }
                break;
            }
        }
    }
}

/* Track left/right borders from bottom to top. */
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

/* Fill missing edges and refresh center line. */
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

/* Match RouteFilter from the 19th reference code. */
static void image_route_filter(void)
{
    int16 center_temp;
    int16 line_temp;

    center_temp = 0;
    line_temp = 0;
    for(Ysite = 58; Ysite >= (ImageStatus.OFFLine + 5); Ysite--)
    {
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
        ImageDeal[Ysite].Center = (ImageDeal[Ysite - 1].Center + 2 * ImageDeal[Ysite].Center) / 3;
    }
}

static void image_straight_acc_test(void)
{
    int16 sum;
    int16 row_count;

    sum = 0;
    row_count = 54 - ImageStatus.OFFLine;
    if(row_count <= 0)
    {
        ImageStatus.straight_acc = 0;
        return;
    }

    for(Ysite = 55; Ysite > (ImageStatus.OFFLine + 1); Ysite--)
    {
        sum += (ImageDeal[Ysite].Center - IMAGE_MID) *
               (ImageDeal[Ysite].Center - IMAGE_MID);
    }

    ImageStatus.variance_acc = sum / row_count;
    if((ImageStatus.variance_acc < IMAGE_STRAIGHT_VARIANCE_MAX) &&
       (ImageStatus.OFFLine <= 7) &&
       (ImageStatus.Left_Line < 2) &&
       (ImageStatus.Right_Line < 2))
    {
        ImageStatus.straight_acc = 1;
        ImageStatus.Road_type = ROAD_STRAIGHT;
    }
    else
    {
        ImageStatus.straight_acc = 0;
    }
}

static uint16 image_ring_outer_white_count(uint8 type, uint8 startline)
{
    uint16 count;
    int16 row;
    int16 row_end;
    int16 col;

    count = 0;
    row = (int16)startline;
    if(row < 0)
    {
        row = 0;
    }
    if(row >= IMAGE_H)
    {
        return 0;
    }

    row_end = row + 10;
    if(row_end > IMAGE_H)
    {
        row_end = IMAGE_H;
    }

    if(type == 1)
    {
        for(; row < row_end; row++)
        {
            for(col = ImageDeal[row].LeftBorder; col > 1; col--)
            {
                if(ImageBin[row][col] != IMAGE_BLACK)
                {
                    count++;
                }
            }
        }
    }
    else
    {
        for(; row < row_end; row++)
        {
            for(col = ImageDeal[row].RightBorder; col < (IMAGE_W - 2); col++)
            {
                if(ImageBin[row][col] != IMAGE_BLACK)
                {
                    count++;
                }
            }
        }
    }

    return count;
}

static uint8 image_ring_bottom_ready(uint8 type)
{
    int16 row;

    for(row = 52; row <= 58; row++)
    {
        if(type == 1)
        {
            if(ImageDeal[row].IsLeftFind == 'W')
            {
                return 0;
            }
        }
        else
        {
            if(ImageDeal[row].IsRightFind == 'W')
            {
                return 0;
            }
        }
    }

    return 1;
}

static void image_judge_left_ring(void)
{
    Left_RingsFlag_Point1_Ysite = 0;
    Left_RingsFlag_Point2_Ysite = 0;
    for(Ysite = 58; Ysite > 3; Ysite--)
    {
        if((ImageDeal[Ysite].LeftBoundary_First -
            ImageDeal[Ysite - 1].LeftBoundary_First) > 4)
        {
            Left_RingsFlag_Point1_Ysite = Ysite;
            break;
        }
    }

    for(Ysite = 58; Ysite > 3; Ysite--)
    {
        if((ImageDeal[Ysite + 1].LeftBoundary -
            ImageDeal[Ysite].LeftBoundary) > 4)
        {
            Left_RingsFlag_Point2_Ysite = Ysite;
            break;
        }
    }

    for(Ysite = Left_RingsFlag_Point1_Ysite; Ysite > 10; Ysite--)
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

    if((Left_RingsFlag_Point2_Ysite > (Left_RingsFlag_Point1_Ysite + 3)) &&
       (Ring_Help_Flag == 0) &&
       (ImageStatus.Left_Line > 6))
    {
        Ring_Help_Flag = 1;
    }

    if((Left_RingsFlag_Point2_Ysite > (Left_RingsFlag_Point1_Ysite + 3)) &&
       (Ring_Help_Flag == 1) &&
       (ImageFlag.image_element_rings_flag == 0))
    {
        ImageFlag.image_element_rings = 1;
        ImageFlag.image_element_rings_flag = 1;
        ImageFlag.ring_big_small = 1;
        ImageStatus.Road_type = ROAD_LEFT_RING;
    }

    Ring_Help_Flag = 0;
}

static void image_judge_right_ring(void)
{
    Right_RingsFlag_Point1_Ysite = 0;
    Right_RingsFlag_Point2_Ysite = 0;
    for(Ysite = 58; Ysite > 3; Ysite--)
    {
        if((ImageDeal[Ysite - 1].RightBoundary_First -
            ImageDeal[Ysite].RightBoundary_First) > 4)
        {
            Right_RingsFlag_Point1_Ysite = Ysite;
            break;
        }
    }

    for(Ysite = 58; Ysite > 3; Ysite--)
    {
        if((ImageDeal[Ysite].RightBoundary -
            ImageDeal[Ysite + 1].RightBoundary) > 4)
        {
            Right_RingsFlag_Point2_Ysite = Ysite;
            break;
        }
    }

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

    if((Right_RingsFlag_Point2_Ysite > (Right_RingsFlag_Point1_Ysite + 3)) &&
       (Ring_Help_Flag == 0) &&
       (ImageStatus.Right_Line > 7))
    {
        Ring_Help_Flag = 1;
    }

    if((Right_RingsFlag_Point2_Ysite > (Right_RingsFlag_Point1_Ysite + 3)) &&
       (Ring_Help_Flag == 1) &&
       (ImageFlag.image_element_rings_flag == 0))
    {
        ImageFlag.image_element_rings = 2;
        ImageFlag.image_element_rings_flag = 1;
        ImageFlag.ring_big_small = 1;
        ImageStatus.Road_type = ROAD_RIGHT_RING;
    }

    Ring_Help_Flag = 0;
}

static void image_element_test(void)
{
    if((ImageStatus.Road_type != ROAD_LEFT_RING) &&
       (ImageStatus.Road_type != ROAD_RIGHT_RING))
    {
        ImageStatus.Road_type = ROAD_NORMAL;
        image_straight_acc_test();
    }

    if((ImageStatus.OFFLine < 5) && (ImageStatus.WhiteLine < 3))
    {
        if((ImageStatus.Right_Line < 2) &&
           (ImageStatus.Left_Line > 13) &&
           image_ring_bottom_ready(1) &&
           (image_ring_outer_white_count(1, ImageStatus.Left_Line) > 70))
        {
            image_judge_left_ring();
        }

        if((ImageStatus.Left_Line < 2) &&
           (ImageStatus.Right_Line > 17) &&
           image_ring_bottom_ready(2) &&
           (image_ring_outer_white_count(2, ImageStatus.Right_Line) > 120))
        {
            image_judge_right_ring();
        }
    }
}

static void image_handle_left_ring(void)
{
    int16 num;
    int16 flag_x;
    int16 flag_y;
    int16 scan_start;
    int16 scan_end;
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
        /* buzzer means step 5: enter-ring border fill starts */
        buzzer_short();
    }
    if((ImageFlag.image_element_rings_flag == 5) && (ImageStatus.Right_Line > 15))
    {
        ImageFlag.image_element_rings_flag = 6;
    }
    if((ImageFlag.image_element_rings_flag == 6) && (ImageStatus.Right_Line < 4))
    {
        ImageFlag.image_element_rings_flag = 7;
    }

    if((ImageFlag.ring_big_small == 1) && (ImageFlag.image_element_rings_flag == 7))
    {
        Point_Ysite = 0;
        Point_Xsite = 0;
        for(Ysite = 45; Ysite > (ImageStatus.OFFLine + 3); Ysite--)
        {
            if((ImageDeal[Ysite].RightBorder <= ImageDeal[Ysite + 1].RightBorder) &&
               (ImageDeal[Ysite].RightBorder <= ImageDeal[Ysite - 1].RightBorder))
            {
                Point_Xsite = ImageDeal[Ysite].RightBorder;
                Point_Ysite = Ysite;
                break;
            }
        }
        if(Point_Ysite > 22)
        {
            ImageFlag.image_element_rings_flag = 8;
        }
    }

    if((ImageFlag.image_element_rings_flag == 8) &&
       (ImageStatus.Right_Line < 9) &&
       (ImageStatus.OFFLine < 10))
    {
        ImageFlag.image_element_rings_flag = 9;
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

    if((ImageFlag.image_element_rings_flag >= 1) &&
       (ImageFlag.image_element_rings_flag <= 4))
    {
        for(Ysite = 57; Ysite > ImageStatus.OFFLine; Ysite--)
        {
            ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBorder - Half_Road_Wide[Ysite];
        }
    }

    if((ImageFlag.image_element_rings_flag == 5) ||
       (ImageFlag.image_element_rings_flag == 6))
    {
        for(Ysite = 55; Ysite > ImageStatus.OFFLine; Ysite--)
        {
            for(Xsite = (ImageDeal[Ysite].LeftBorder + 1);
                Xsite < (ImageDeal[Ysite].RightBorder - 1);
                Xsite++)
            {
                if((ImageBin[Ysite][Xsite] == IMAGE_WHITE) &&
                   (ImageBin[Ysite][Xsite + 1] == IMAGE_BLACK))
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
            for(Ysite = (ImageStatus.OFFLine + 1); Ysite < 30; Ysite++)
            {
                if((ImageDeal[Ysite].IsLeftFind == 'T') &&
                   (ImageDeal[Ysite + 1].IsLeftFind == 'T') &&
                   (ImageDeal[Ysite + 2].IsLeftFind == 'W') &&
                   (IMAGE_ABS(ImageDeal[Ysite].LeftBorder -
                              ImageDeal[Ysite + 2].LeftBorder) > 10))
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
            for(Ysite = flag_y; Ysite < IMAGE_H; Ysite++)
            {
                ImageDeal[Ysite].RightBorder = flag_x + (int16)(slope * (float)(Ysite - flag_y));
                ImageDeal[Ysite].Center = (ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) / 2;
                if(ImageDeal[Ysite].Center < 4)
                {
                    ImageDeal[Ysite].Center = 4;
                }
            }

            ImageDeal[flag_y].RightBorder = flag_x;
            for(Ysite = flag_y - 1; Ysite > 10; Ysite--)
            {
                scan_start = ImageDeal[Ysite + 1].RightBorder - 10;
                scan_end = ImageDeal[Ysite + 1].RightBorder + 2;
                LimitL(scan_start);
                LimitH(scan_end);
                for(Xsite = scan_start; Xsite < scan_end; Xsite++)
                {
                    if((ImageBin[Ysite][Xsite] == IMAGE_WHITE) &&
                       (ImageBin[Ysite][Xsite + 1] == IMAGE_BLACK))
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

                ImageStatus.OFFLine = Ysite + 2;
                break;
            }
        }
    }

    if((ImageFlag.image_element_rings_flag == 8) && (ImageFlag.ring_big_small == 1))
    {
        Repair_Point_Xsite = 20;
        Repair_Point_Ysite = 0;
        for(Ysite = 40; Ysite > 8; Ysite--)
        {
            if((ImageBin[Ysite][28] == IMAGE_WHITE) &&
               (ImageBin[Ysite - 1][28] == IMAGE_BLACK))
            {
                Repair_Point_Xsite = 28;
                Repair_Point_Ysite = Ysite - 1;
                ImageStatus.OFFLine = Ysite + 1;
                break;
            }
        }

        if(Repair_Point_Ysite < 57)
        {
            for(Ysite = 57; Ysite > (Repair_Point_Ysite - 3); Ysite--)
            {
                ImageDeal[Ysite].RightBorder =
                    (ImageDeal[58].RightBorder - Repair_Point_Xsite) * (Ysite - 58) /
                    (58 - Repair_Point_Ysite) + ImageDeal[58].RightBorder;
                ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBorder - Half_Road_Wide[Ysite];
                if(ImageDeal[Ysite].Center <= ImageDeal[Ysite].LeftBorder)
                {
                    ImageDeal[Ysite].Center = ImageDeal[Ysite].LeftBorder + 1;
                }
            }
        }
    }

    if((ImageFlag.image_element_rings_flag == 9) ||
       (ImageFlag.image_element_rings_flag == 10))
    {
        for(Ysite = 59; Ysite > ImageStatus.OFFLine; Ysite--)
        {
            ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBorder - Half_Road_Wide[Ysite];
            if(ImageDeal[Ysite].Center <= ImageDeal[Ysite].LeftBorder)
            {
                ImageDeal[Ysite].Center = ImageDeal[Ysite].LeftBorder + 1;
            }
        }
    }
}

static void image_handle_right_ring(void)
{
    int16 num;
    int16 flag_x;
    int16 flag_y;
    int16 scan_start;
    int16 scan_end;
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
        /* buzzer means step 5: enter-ring border fill starts */
        buzzer_short();
    }
    if((ImageFlag.image_element_rings_flag == 5) && (ImageStatus.Left_Line > 15))
    {
        ImageFlag.image_element_rings_flag = 6;
    }
    if((ImageFlag.image_element_rings_flag == 6) && (ImageStatus.Left_Line < 4))
    {
        ImageFlag.image_element_rings_flag = 7;
    }

    if(ImageFlag.image_element_rings_flag == 7)
    {
        Point_Xsite = 0;
        Point_Ysite = 0;
        for(Ysite = 45; Ysite > (ImageStatus.OFFLine + 3); Ysite--)
        {
            if((ImageDeal[Ysite].LeftBorder >= ImageDeal[Ysite + 1].LeftBorder) &&
               (ImageDeal[Ysite].LeftBorder >= ImageDeal[Ysite - 1].LeftBorder))
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

    if((ImageFlag.image_element_rings_flag == 8) &&
       (ImageStatus.Left_Line < 9) &&
       (ImageStatus.OFFLine < 10))
    {
        ImageFlag.image_element_rings_flag = 9;
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

    if((ImageFlag.image_element_rings_flag >= 1) &&
       (ImageFlag.image_element_rings_flag <= 4))
    {
        for(Ysite = 59; Ysite > ImageStatus.OFFLine; Ysite--)
        {
            ImageDeal[Ysite].Center = ImageDeal[Ysite].LeftBorder + Half_Road_Wide[Ysite];
            if(ImageDeal[Ysite].Center >= ImageDeal[Ysite].RightBorder)
            {
                ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBorder - 1;
            }
        }
    }

    if((ImageFlag.image_element_rings_flag == 5) ||
       (ImageFlag.image_element_rings_flag == 6))
    {
        for(Ysite = 55; Ysite > ImageStatus.OFFLine; Ysite--)
        {
            for(Xsite = (ImageDeal[Ysite].LeftBorder + 1);
                Xsite < (ImageDeal[Ysite].RightBorder - 1);
                Xsite++)
            {
                if((ImageBin[Ysite][Xsite] == IMAGE_WHITE) &&
                   (ImageBin[Ysite][Xsite + 1] == IMAGE_BLACK))
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
            for(Ysite = (ImageStatus.OFFLine + 5); Ysite < 30; Ysite++)
            {
                if((ImageDeal[Ysite].IsRightFind == 'T') &&
                   (ImageDeal[Ysite + 1].IsRightFind == 'T') &&
                   (ImageDeal[Ysite + 2].IsRightFind == 'W') &&
                   (IMAGE_ABS(ImageDeal[Ysite].RightBorder -
                              ImageDeal[Ysite + 2].RightBorder) > 10))
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
                ImageDeal[Ysite].LeftBorder = flag_x + (int16)(slope * (float)(Ysite - flag_y));
                ImageDeal[Ysite].Center = (ImageDeal[Ysite].LeftBorder + ImageDeal[Ysite].RightBorder) / 2;
                if(ImageDeal[Ysite].Center > 79)
                {
                    ImageDeal[Ysite].Center = 79;
                }
            }

            ImageDeal[flag_y].LeftBorder = flag_x;
            for(Ysite = flag_y - 1; Ysite > 10; Ysite--)
            {
                scan_start = ImageDeal[Ysite + 1].LeftBorder + 8;
                scan_end = ImageDeal[Ysite + 1].LeftBorder - 4;
                LimitL(scan_start);
                LimitH(scan_start);
                LimitL(scan_end);
                LimitH(scan_end);
                for(Xsite = scan_start; Xsite > scan_end; Xsite--)
                {
                    if((ImageBin[Ysite][Xsite] == IMAGE_WHITE) &&
                       (ImageBin[Ysite][Xsite - 1] == IMAGE_BLACK))
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

                ImageStatus.OFFLine = Ysite + 2;
                break;
            }
        }
    }

    if(ImageFlag.image_element_rings_flag == 8)
    {
        Repair_Point_Xsite = 20;
        Repair_Point_Ysite = 0;
        for(Ysite = 40; Ysite > 5; Ysite--)
        {
            if((ImageBin[Ysite][28] == IMAGE_WHITE) &&
               (ImageBin[Ysite - 1][28] == IMAGE_BLACK))
            {
                Repair_Point_Xsite = 28;
                Repair_Point_Ysite = Ysite - 1;
                ImageStatus.OFFLine = Ysite + 1;
                break;
            }
        }

        if(Repair_Point_Ysite < 57)
        {
            for(Ysite = 57; Ysite > (Repair_Point_Ysite - 3); Ysite--)
            {
                ImageDeal[Ysite].LeftBorder =
                    (ImageDeal[58].LeftBorder - Repair_Point_Xsite) * (Ysite - 58) /
                    (58 - Repair_Point_Ysite) + ImageDeal[58].LeftBorder;
                ImageDeal[Ysite].Center = (ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) / 2;
                if(ImageDeal[Ysite].Center >= ImageDeal[Ysite].RightBorder)
                {
                    ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBorder - 1;
                }
            }
        }
    }

    if(ImageFlag.image_element_rings_flag == 9)
    {
        for(Ysite = 59; Ysite > ImageStatus.OFFLine; Ysite--)
        {
            ImageDeal[Ysite].Center = ImageDeal[Ysite].LeftBorder + Half_Road_Wide[Ysite];
            if(ImageDeal[Ysite].Center >= ImageDeal[Ysite].RightBorder)
            {
                ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBorder - 1;
            }
        }
    }
}

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

/* Scan zebra line by counting repeated black-to-white edges. */
static uint8 image_zebra_scan(void)
{
    int16 row;
    int16 col;
    int16 left_limit;
    int16 right_limit;
    uint8 edge_count;

    if((ImageStatus.Road_type == ROAD_LEFT_RING) ||
       (ImageStatus.Road_type == ROAD_RIGHT_RING))
    {
        return 0;
    }

    for(row = 45; row < 55; row++)
    {
        edge_count = 0;
        left_limit = ImageDeal[row].LeftBoundary - 5;
        right_limit = ImageDeal[row].RightBoundary + 5;

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

static void image_check_zebra(void)
{
    ZebraHit = image_zebra_scan();

    if(CarMode != CAR_MODE_RUN)
    {
        return;
    }

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

/* Get weighted center at the current tow point. */
static void image_get_det(uint8 tow_point)
{
    float det_temp;
    float unit_all;

    det_temp = 0.0f;
    unit_all = 0.0f;

    if((tow_point - 5) >= ImageStatus.OFFLine)
    {
        for(Ysite = (int16)(tow_point - 5); Ysite < tow_point; Ysite++)
        {
            det_temp += Weighting[tow_point - Ysite - 1] * (float)ImageDeal[Ysite].Center;
            unit_all += Weighting[tow_point - Ysite - 1];
        }
        for(Ysite = (int16)(tow_point + 5); Ysite > tow_point; Ysite--)
        {
            det_temp += Weighting[Ysite - tow_point - 1] * (float)ImageDeal[Ysite].Center;
            unit_all += Weighting[Ysite - tow_point - 1];
        }
        det_temp = ((float)ImageDeal[tow_point].Center + det_temp) / (unit_all + 1.0f);
    }
    else if(tow_point > ImageStatus.OFFLine)
    {
        for(Ysite = ImageStatus.OFFLine; Ysite < tow_point; Ysite++)
        {
            det_temp += Weighting[tow_point - Ysite - 1] * (float)ImageDeal[Ysite].Center;
            unit_all += Weighting[tow_point - Ysite - 1];
        }
        for(Ysite = (int16)(tow_point + tow_point - ImageStatus.OFFLine); Ysite > tow_point; Ysite--)
        {
            det_temp += Weighting[Ysite - tow_point - 1] * (float)ImageDeal[Ysite].Center;
            unit_all += Weighting[Ysite - tow_point - 1];
        }
        det_temp = ((float)ImageDeal[tow_point].Center + det_temp) / (unit_all + 1.0f);
    }
    else if(ImageStatus.OFFLine < 49)
    {
        for(Ysite = (int16)(ImageStatus.OFFLine + 3); Ysite > ImageStatus.OFFLine; Ysite--)
        {
            det_temp += Weighting[Ysite - tow_point - 1] * (float)ImageDeal[Ysite].Center;
            unit_all += Weighting[Ysite - tow_point - 1];
        }
        det_temp = ((float)ImageDeal[ImageStatus.OFFLine].Center + det_temp) / (unit_all + 1.0f);
    }
    else
    {
        det_temp = (float)ImageStatus.Det_True;
    }

    ImageStatus.Det_True = (int16)det_temp;
    ImageStatus.TowPoint_True = tow_point;
}

/* One complete frame processing pass. */
static void image_process(void)
{
    gpio_set_level(LED_DEBUG, GPIO_LOW);

    image_compress();
    image_clear_deal();
    image_binarize(image_otsu());
    image_draw_bottom();
    image_draw_lines();
    image_search_border(IMAGE_H - 2);
    image_element_test();
    image_draw_extension_line();
    image_route_filter();
    image_element_handle();
    image_check_zebra();
    image_get_det(image_tow_point());
    image_export_result();

    gpio_set_level(LED_DEBUG, GPIO_HIGH);
}

void image_update(void)
{
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

    if(CarMode != CAR_MODE_RUN)
    {
        ImageLostCount = 0;
        ImageRunFrameCount = 0;
        return;
    }

    if(ImageRunFrameCount < IMAGE_RUN_START_IGNORE_FRAMES)
    {
        ImageRunFrameCount++;
        ImageLostCount = 0;
        return;
    }

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

