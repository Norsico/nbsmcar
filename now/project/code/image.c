#include "image.h"
#include "motor.h"
#include "state.h"
#include "ui.h"
#include "stdlib.h"

image_data Image;
uint8 Image_Use[IMAGE_H][IMAGE_W];
uint8 Pixle[IMAGE_H][IMAGE_W];

#define IMAGE_THRESHOLD_DETACH         (180)//二值化大阈值
#define IMAGE_THRESHOLD_STATIC         (70)	//二值化小阈值
#define IMAGE_STOP_RAW_THRESHOLD       (25)
#define IMAGE_OFFLINE_INIT             (2)
#define IMAGE_SCAN_INTERVAL            (2)
#define IMAGE_ZEBRA_MISS_COUNT         (3)
#define IMAGE_ZEBRA_COOLDOWN_FRAMES    (80)
#define IMAGE_ZEBRA_EDGE_MIN           (4)
#define IMAGE_ZEBRA_ROW_HIT_MIN        (2)
#define IMAGE_ZEBRA_STOP_COUNT         (2)
#define IMAGE_OUTTRACK_BLACK_PERCENT   (90)
#define IMAGE_OUTTRACK_CONFIRM_COUNT   (10)
#define IMAGE_OUTTRACK_SAMPLE_ROWS     (2)

#define IMAGE_ABS(V)                   (((V) < 0) ? (-(V)) : (V))
#define OtsuRawThreshold               ImageRawThreshold

ImageDealDatatypedef ImageDeal[IMAGE_H];
ImageStatustypedef ImageStatus =
{
    32,
    0,
    0,
    0,
    IMAGE_THRESHOLD_STATIC,
    IMAGE_THRESHOLD_DETACH
};

ImageFlagtypedef ImageFlag = {0};

static uint8 image_ready = 0;
static uint8 image_result_ready = 0;
static uint32 image_result_sequence = 0;

static uint8 CompressRowMap[IMAGE_H];
static uint8 CompressColMap[IMAGE_W];
static uint8 CompressMapReady = 0;
static uint8 ImageRawThreshold = 0;
static uint8 ZebraHit = 0;
static uint8 ZebraDetectCount = 0;
static uint8 ZebraFrameLatch = 0;
static uint8 ZebraMissFrames = 0;
static uint8 ZebraCooldownFrames = 0;
static uint8 runtime_tow_point = 0;
static uint8 OutTrackStopHitCount = 0;
static uint16 Speed_Goal = 0;

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
static int ImageScanInterval = 2;
static int ImageScanInterval_Cross = 2;
static float variance = 0.0f;
static float variance_acc = 25.0f;
static const uint8 RightRingPreEnterCenterBiasStage12 = 6;      /* 右环前两阶段中线额外右移。 */
static const uint8 RightRingPreEnterCenterBiasDefault = 2;      /* 右环准备进环阶段默认右移量。 */
static const uint8 LeftRingExitCenterBias = 3;                  /* 左环出环时中线额外左移。 */
static const uint8 RightRingExitCenterBias = 3;                 /* 右环出环时中线额外右移。 */
static const float RightRingDetectLeftEdgeStraightMax = 40.0f;  /* 右环入口前左边界直线度上限。 */
static const uint8 RightRingDetectPointSpanMax = 18;            /* 右环两个候选拐点纵向间隔上限。 */

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

static int Limit(int value, int numH, int numL)
{
    int temp;

    if(numH < numL)
    {
        temp = numH;
        numH = numL;
        numL = temp;
    }

    if(value > numH)
    {
        value = numH;
    }
    if(value < numL)
    {
        value = numL;
    }
    return value;
}

/* 将当前帧内部结果同步到老的 Image 导出结构，保持外围代码不变。 */
static void image_export_result(void)
{
    Image.ready = image_ready;
    Image.sequence = (uint16)image_result_sequence;
    Image.threshold = ImageStatus.Threshold;
    Image.white_count = 0;
    Image.tow_row = (uint8)ImageStatus.TowPoint_True;
    Image.center = (int16)ImageStatus.Det_True;
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

    Image.result_ready = image_result_ready;
    if(Image.lost || (CarMode == CAR_MODE_STOP))
    {
        Image.result_ready = 0;
    }
    Image.ring = (uint8)ImageFlag.image_element_rings;
    Image.ring_step = (uint8)ImageFlag.image_element_rings_flag;
    Image.zebra = ZebraHit;
    Image.zebra_count = ZebraDetectCount;
}

uint16 image_get_speed_goal(void)
{
    return Speed_Goal;
}

/* 按当前赛道状态和车速口径求本帧前瞻行。 */
static uint8 SearchLine_GetRuntimeTowPoint(void)
{
    int TowPoint;
    float SpeedGain;
    int speed_left;
    int speed_right;
    int speed_now;
    int speed_normal;
    int speed_straight;
    int speed_min;

    speed_normal = (int)SmartCar.motor.target_speed;
    speed_straight = (int)SmartCar.motor.straight_speed;
    speed_min = speed_normal - 10;
    if(speed_min < 0)
    {
        speed_min = 0;
    }

    speed_left = Motor.read_left;
    if(speed_left < 0)
    {
        speed_left = -speed_left;
    }

    speed_right = Motor.read_right;
    if(speed_right < 0)
    {
        speed_right = -speed_right;
    }

    speed_now = (speed_left + speed_right) / 2;
    if(ImageStatus.straight_acc == 1)
    {
        Speed_Goal = (uint16)((speed_straight < 0) ? 0 : speed_straight);
    }
    else
    {
        Speed_Goal = (uint16)((speed_normal < 0) ? 0 : speed_normal);
    }

    if((ImageStatus.Road_type == RightCirque || ImageStatus.Road_type == LeftCirque) &&
       ImageStatus.CirqueOff == 'F')
    {
        TowPoint = 28;
    }
    else if(ImageStatus.Road_type == Straight)
    {
        TowPoint = ImageStatus.TowPoint;
    }
    else if(ImageStatus.Road_type == Cross_ture)
    {
        TowPoint = 22;
    }
    else if(ImageFlag.image_element_rings_flag == 1 || ImageFlag.image_element_rings_flag == 2)
    {
        TowPoint = 30;
    }
    else
    {
        SpeedGain = ((float)(speed_now - speed_min) * 0.2f) + 0.5f;
        if(SpeedGain > 3.0f)
        {
            SpeedGain = 3.0f;
        }
        else if(SpeedGain < -1.0f)
        {
            SpeedGain = -1.0f;
        }

        TowPoint = (int)((float)ImageStatus.TowPoint - SpeedGain);
    }

    if(TowPoint < ImageStatus.OFFLine)
        TowPoint = ImageStatus.OFFLine + 1;

    if(TowPoint >= 49)
        TowPoint = 49;

    if(TowPoint <= 0)
    {
        TowPoint = 1;
    }

    return (uint8)TowPoint;
}

/* 将 80 列图像坐标映射到预览窗口 X 轴。 */
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

/* 将 60 行图像坐标映射到预览窗口 Y 轴。 */
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

/* 在摄像头预览上叠加左右边线、中线和前瞻行。 */
void image_show_debug_overlay(uint16 x, uint16 y, uint16 w, uint16 h)
{
    uint8 row;
    uint16 draw_x;
    uint16 draw_y;
    uint16 tow_y;

    if((0U == image_ready) || (0U == image_result_sequence))
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

/* 将当前曝光、增益等参数重新写入摄像头。 */
void image_apply_camera(void)
{
    uint8 i;
    short int config[MT9V03X_CONFIG_FINISH][2];

    if(0U == image_ready)
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

/* 大津法阈值计算，pixel_threshold 用来截断搜索上限。 */
uint8 Threshold_deal(uint8 *image,
                     uint16 col,
                     uint16 row,
                     uint32 pixel_threshold)
{
    uint16 width;
    uint16 height;
    int pixelCount[256];
    float pixelPro[256];
    uint16 i;
    uint16 j;
    int pixelSum;
    uint8 threshold;
    uint8 *image_data;
    uint32 gray_sum;
    float w0;
    float w1;
    float u0tmp;
    float u1tmp;
    float u0;
    float u1;
    float u;
    float deltaTmp;
    float deltaMax;
    float diff0;
    float diff1;
    float inv_pixel_sum;

    width = col;
    height = row;
    pixelSum = width * height;
    threshold = 0;
    image_data = image;
    gray_sum = 0;
    w0 = 0.0f;
    u0tmp = 0.0f;
    u = 0.0f;
    deltaMax = 0.0f;
    inv_pixel_sum = 0.0f;

    if(0 == pixelSum)
    {
        return 0;
    }

    for(i = 0; i < 256; i++)
    {
        pixelCount[i] = 0;
        pixelPro[i] = 0.0f;
    }

    for(i = 0; i < height; i++)
    {
        for(j = 0; j < width; j++)
        {
            pixelCount[(int)image_data[i * width + j]]++;
            gray_sum += (int)image_data[i * width + j];
        }
    }

    inv_pixel_sum = 1.0f / (float)pixelSum;

    for(i = 0; i < 256; i++)
    {
        pixelPro[i] = (float)pixelCount[i] * inv_pixel_sum;
    }

    u = (float)gray_sum * inv_pixel_sum;

    for(j = 0; j < (uint16)pixel_threshold; j++)
    {
        w0 += pixelPro[j];
        if(0.0f == w0)
        {
            continue;
        }

        u0tmp += j * pixelPro[j];
        w1 = 1.0f - w0;
        if(0.0f == w1)
        {
            break;
        }

        u1tmp = u - u0tmp;
        u0 = u0tmp / w0;
        u1 = u1tmp / w1;
        diff0 = u0 - u;
        diff1 = u1 - u;
        deltaTmp = w0 * diff0 * diff0 + w1 * diff1 * diff1;
        if(deltaTmp > deltaMax)
        {
            deltaMax = deltaTmp;
            threshold = (uint8)j;
        }
        if(deltaTmp < deltaMax)
        {
            break;
        }
    }

    return threshold;
}

/* 使用大津阈值完成二值化，并对左右边缘做一点补偿。 */
void Get01change_dajin(void)
{
    uint8 i = 0;
    uint8 j = 0;
    uint8 thre = 0;
    int threshold_value = 0;
    flash_camera_page_t camera_page;

    flash_get_camera_page(&camera_page);
    threshold_value = (int)Threshold_deal(Image_Use[0], LCDW, LCDH, ImageStatus.Threshold_detach) +
                      (int)camera_page.threshold_offset;
    if(threshold_value < 0)
    {
        threshold_value = 0;
    }
    else if(threshold_value > 255)
    {
        threshold_value = 255;
    }

    OtsuRawThreshold = (uint8)threshold_value;
    ImageStatus.Threshold = OtsuRawThreshold;
    if(ImageStatus.Threshold < ImageStatus.Threshold_static)
    {
        ImageStatus.Threshold = (uint8)ImageStatus.Threshold_static;
    }

    for(i = 0; i < LCDH; i++)
    {
        for(j = 0; j < LCDW; j++)
        {
            if(j <= 15)
            {
                thre = (uint8)(ImageStatus.Threshold - 10);
            }
            else if((j > 70) && (j <= 75))
            {
                thre = (uint8)(ImageStatus.Threshold - 10);
            }
            else if(j >= 65)
            {
                thre = (uint8)(ImageStatus.Threshold - 10);
            }
            else
            {
                thre = ImageStatus.Threshold;
            }

            if(Image_Use[i][j] > thre)
            {
                Pixle[i][j] = 1;
            }
            else
            {
                Pixle[i][j] = 0;
            }
        }
    }
}

/* 统计底部黑块占比，连续命中后判定出界停车。 */
static void CheckOutTrackEmergency(void)
{
    uint8 row;
    uint8 col;
    uint16 black_count = 0;

    if(CAR_MODE_RUN != CarMode)
    {
        OutTrackStopHitCount = 0;
        return;
    }

    for(row = (uint8)(LCDH - IMAGE_OUTTRACK_SAMPLE_ROWS); row < LCDH; row++)
    {
        for(col = 0; col < LCDW; col++)
        {
            if(0 == Pixle[row][col])
            {
                black_count++;
            }
        }
    }

    if(black_count > (uint16)(((uint16)(LCDW * IMAGE_OUTTRACK_SAMPLE_ROWS) * IMAGE_OUTTRACK_BLACK_PERCENT) / 100U))
    {
        if(OutTrackStopHitCount < IMAGE_OUTTRACK_CONFIRM_COUNT)
        {
            OutTrackStopHitCount++;
        }
    }
    else
    {
        OutTrackStopHitCount = 0;
    }

    if(OutTrackStopHitCount >= IMAGE_OUTTRACK_CONFIRM_COUNT)
    {
        CarMode = CAR_MODE_STOP;
    }
}

/* 初始化摄像头和图像模块状态。 */
void image_init(void)
{
    uint8 retry;

    image_ready = 0;
    image_result_ready = 0;
    image_result_sequence = 0;
    Image.ready = 0;
    Image.result_ready = 0;
    Image.sequence = 0;
    Image.threshold = 0;
    Image.white_count = 0;
    Image.tow_row = 0;
    Image.center = IMAGE_MID;
    Image.error = 0;
    Image.valid_count = 0;
    Image.lost = 1;
    Image.ring = 0;
    Image.ring_step = 0;
    Image.zebra = 0;
    Image.zebra_count = 0;
    ImageStatus.TowPoint = (uint8)SmartCar.servo.tow_point;
    ImageRawThreshold = 0;
    Speed_Goal = (uint16)((SmartCar.motor.target_speed < 0) ? 0 : SmartCar.motor.target_speed);
    ZebraHit = 0;
    ZebraDetectCount = 0;
    ZebraFrameLatch = 0;
    ZebraMissFrames = 0;
    ZebraCooldownFrames = 0;

    gpio_init(LED_DEBUG, GPO, GPIO_HIGH, GPO_PUSH_PULL);

    retry = 0;
    while(retry < CAMERA_INIT_RETRY)
    {
        if(mt9v03x_init() == 0)
        {
            image_ready = 1;
            break;
        }
        retry++;
        system_delay_ms(CAMERA_INIT_DELAY_MS);
    }

    image_apply_camera();
    mt9v03x_finish_flag = 0;
    Image.ready = image_ready;
}

/* 摄像头初始化是否完成。 */
uint8 image_is_ready(void)
{
    return image_ready;
}

/* 当前帧结果是否已经更新完成。 */
uint8 image_is_result_ready(void)
{
    return image_result_ready;
}

/* 图像结果序号，每成功处理一帧递增一次。 */
uint32 image_get_result_sequence(void)
{
    return image_result_sequence;
}

/* 将原始 MT9V03X 图像裁剪压缩到 80x60。 */
void compressimage(void)
{
    int i, j, row;
    uint8 *dst_row;
    uint8 *src_row;

    if(!CompressMapReady)
    {
        /* 当前裁剪口径固定，只在首帧生成一次压缩映射，避免每像素做浮点缩放。 */
        for(i = 0; i < LCDH; i++)
        {
            CompressRowMap[i] = (uint8)(IMAGE_COMPRESS_CUT_ROW_TOP +
                                        ((i * IMAGE_COMPRESS_SRC_H + (LCDH / 2)) / LCDH));
        }
        for(j = 0; j < LCDW; j++)
        {
            CompressColMap[j] = (uint8)(IMAGE_COMPRESS_CUT_COL +
                                        ((j * IMAGE_COMPRESS_SRC_W + (LCDW / 2)) / LCDW));
        }
        CompressMapReady = 1;
    }

    for(i = 0; i < LCDH; i++)
    {
        row = CompressRowMap[i];
        dst_row = Image_Use[i];
        src_row = mt9v03x_image[row];
        for(j = 0; j < LCDW; j++)
        {
            dst_row[j] = src_row[CompressColMap[j]];
        }
    }
    mt9v03x_finish_flag = 0;  //使用完一帧DMA传输的图像  可以开始传输下一帧
}

/* 底边初始化。 */
static uint8 DrawLinesFirst(void)
{
    PicTemp = Pixle[59];
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
        PicTemp = Pixle[Ysite];

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
    return 'T';
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//  @name           Search_Bottom_Line_OTSU
//  @brief          获取底层左右边线
//  @param          imageInput[IMAGE_ROW][IMAGE_COL]        传入的图像数组
//  @param          Row                                     图像的Ysite
//  @param          Col                                     图像的Xsite
//  @param          Bottonline                              底边行选择
//  @return         无
//  @time           2022年10月9日
//  @Author
//  Sample usage:   Search_Bottom_Line_OTSU(imageInput, Row, Col, Bottonline);
//--------------------------------------------------------------------------------------------------------------------------------------------
static void Search_Bottom_Line_OTSU(uint8 imageInput[LCDH][LCDW], uint8 Row, uint8 Col, uint8 Bottonline)
{
    if((0 == Col) || (Bottonline >= Row))
    {
        return;
    }

    //寻找左边边界
    for(Xsite = Col / 2 - 2; Xsite > 1; Xsite--)
    {
        if(imageInput[Bottonline][Xsite] == 1 && imageInput[Bottonline][Xsite - 1] == 0)
        {
            ImageDeal[Bottonline].LeftBoundary = Xsite;//获取底边左边线
            break;
        }
    }
    //寻找右边边界
    for(Xsite = Col / 2 + 2; Xsite < LCDW - 1; Xsite++)
    {
        if(imageInput[Bottonline][Xsite] == 1 && imageInput[Bottonline][Xsite + 1] == 0)
        {
            ImageDeal[Bottonline].RightBoundary = Xsite;//获取底边右边线
            break;
        }
    }
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//  @name           Search_Left_and_Right_Lines
//  @brief          通过sobel提取左右边线
//  @param          imageInput[IMAGE_ROW][IMAGE_COL]        传入的图像数组
//  @param          Row                                     图像的Ysite
//  @param          Col                                     图像的Xsite
//  @param          Bottonline                              底边行选择
//  @return         无
//  @time           2022年10月7日
//  @Author
//  Sample usage:   Search_Left_and_Right_Lines(imageInput, Row, Col, Bottonline);
//--------------------------------------------------------------------------------------------------------------------------------------------
static void Search_Left_and_Right_Lines(uint8 imageInput[LCDH][LCDW], uint8 Row, uint8 Col, uint8 Bottonline)
{
    //定义小人的当前行走状态位置为 上 左 下 右 一次要求 上：左边为黑色 左：上边为褐色 下：右边为色  右：下面有黑色
/*  前进方向定义：
                *   0
                * 3   1
                *   2
*/
/*寻左线坐标规则*/
    uint8 Left_Rule[2][8] =
    {
        {0, -1, 1, 0, 0, 1, -1, 0},
        {-1, -1, 1, -1, 1, 1, -1, 1}
    };
    /*寻右线坐标规则*/
    int Right_Rule[2][8] =
    {
        {0, -1, 1, 0, 0, 1, -1, 0},
        {1, -1, 1, 1, -1, 1, -1, -1}
    };
    int num = 0;
    uint8 Left_Ysite = Bottonline;
    uint8 Left_Xsite = ImageDeal[Bottonline].LeftBoundary;
    uint8 Left_Rirection = 0;//左边方向
    uint8 Pixel_Left_Ysite = Bottonline;
    uint8 Pixel_Left_Xsite = 0;
    uint8 Right_Ysite = Bottonline;
    uint8 Right_Xsite = ImageDeal[Bottonline].RightBoundary;
    uint8 Right_Rirection = 0;//右边方向
    uint8 Pixel_Right_Ysite = Bottonline;
    uint8 Pixel_Right_Xsite = 0;
    uint8 Ysite = Bottonline;

    if((0 == Row) || (0 == Col) || (Bottonline >= Row))
    {
        return;
    }

    ImageStatus.OFFLineBoundary = 5;
    while(1)
    {
        num++;
        if(num > 400)
        {
            ImageStatus.OFFLineBoundary = Ysite;
            break;
        }
        if(Ysite >= Pixel_Left_Ysite && Ysite >= Pixel_Right_Ysite)
        {
            if(Ysite < ImageStatus.OFFLineBoundary)
            {
                ImageStatus.OFFLineBoundary = Ysite;
                break;
            }
            else
            {
                Ysite--;
            }
        }

        /*********左边巡线*******/
        if((Pixel_Left_Ysite > Ysite) || Ysite == ImageStatus.OFFLineBoundary)//右边扫线
        {
            /*计算前方坐标*/
            Pixel_Left_Ysite = Left_Ysite + Left_Rule[0][2 * Left_Rirection + 1];
            Pixel_Left_Xsite = Left_Xsite + Left_Rule[0][2 * Left_Rirection];

            if(imageInput[Pixel_Left_Ysite][Pixel_Left_Xsite] == 0)//前方是黑色
            {
                //顺时针旋转90
                if(Left_Rirection == 3)
                    Left_Rirection = 0;
                else
                    Left_Rirection++;
            }
            else//前方是白色
            {
                /*计算左前方坐标*/
                Pixel_Left_Ysite = Left_Ysite + Left_Rule[1][2 * Left_Rirection + 1];
                Pixel_Left_Xsite = Left_Xsite + Left_Rule[1][2 * Left_Rirection];

                if(imageInput[Pixel_Left_Ysite][Pixel_Left_Xsite] == 0)//左前方为黑色
                {
                    //方向不变  Left_Rirection
                    Left_Ysite = Left_Ysite + Left_Rule[0][2 * Left_Rirection + 1];
                    Left_Xsite = Left_Xsite + Left_Rule[0][2 * Left_Rirection];
                    if(ImageDeal[Left_Ysite].LeftBoundary_First == 0)
                    {
                        ImageDeal[Left_Ysite].LeftBoundary_First = Left_Xsite;
                    }
                    ImageDeal[Left_Ysite].LeftBoundary = Left_Xsite;
                }
                else//左前方为白色
                {
                    // 方向发生改变 Left_Rirection  逆时针90度
                    Left_Ysite = Left_Ysite + Left_Rule[1][2 * Left_Rirection + 1];
                    Left_Xsite = Left_Xsite + Left_Rule[1][2 * Left_Rirection];
                    if(ImageDeal[Left_Ysite].LeftBoundary_First == 0)
                    {
                        ImageDeal[Left_Ysite].LeftBoundary_First = Left_Xsite;
                    }
                    ImageDeal[Left_Ysite].LeftBoundary = Left_Xsite;
                    if(Left_Rirection == 0)
                        Left_Rirection = 3;
                    else
                        Left_Rirection--;
                }
            }
        }

        /*********右边巡线*******/
        if((Pixel_Right_Ysite > Ysite) || Ysite == ImageStatus.OFFLineBoundary)//右边扫线
        {
            /*计算前方坐标*/
            Pixel_Right_Ysite = Right_Ysite + Right_Rule[0][2 * Right_Rirection + 1];
            Pixel_Right_Xsite = Right_Xsite + Right_Rule[0][2 * Right_Rirection];

            if(imageInput[Pixel_Right_Ysite][Pixel_Right_Xsite] == 0)//前方是黑色
            {
                //逆时针旋转90
                if(Right_Rirection == 0)
                    Right_Rirection = 3;
                else
                    Right_Rirection--;
            }
            else//前方是白色
            {
                /*计算右前方坐标*/
                Pixel_Right_Ysite = Right_Ysite + Right_Rule[1][2 * Right_Rirection + 1];
                Pixel_Right_Xsite = Right_Xsite + Right_Rule[1][2 * Right_Rirection];

                if(imageInput[Pixel_Right_Ysite][Pixel_Right_Xsite] == 0)//左前方为黑色
                {
                    //方向不变  Right_Rirection
                    Right_Ysite = Right_Ysite + Right_Rule[0][2 * Right_Rirection + 1];
                    Right_Xsite = Right_Xsite + Right_Rule[0][2 * Right_Rirection];
                    if(ImageDeal[Right_Ysite].RightBoundary_First == 79)
                    {
                        ImageDeal[Right_Ysite].RightBoundary_First = Right_Xsite;
                    }
                    ImageDeal[Right_Ysite].RightBoundary = Right_Xsite;
                }
                else//左前方为白色
                {
                    // 方向发生改变 Right_Rirection  逆时针90度
                    Right_Ysite = Right_Ysite + Right_Rule[1][2 * Right_Rirection + 1];
                    Right_Xsite = Right_Xsite + Right_Rule[1][2 * Right_Rirection];
                    if(ImageDeal[Right_Ysite].RightBoundary_First == 79)
                    {
                        ImageDeal[Right_Ysite].RightBoundary_First = Right_Xsite;
                    }
                    ImageDeal[Right_Ysite].RightBoundary = Right_Xsite;
                    if(Right_Rirection == 3)
                        Right_Rirection = 0;
                    else
                        Right_Rirection++;
                }
            }
        }

        if(abs(Pixel_Right_Xsite - Pixel_Left_Xsite) < 3)//Ysite<80是为了放在底部是斑马线扫描结束  3 && Ysite < 30
        {
            ImageStatus.OFFLineBoundary = Ysite;
            break;
        }
    }
}

/* 在给定搜索窗口内寻找左右跳变点。T 为正常找到，W 为整段偏白，H 为整段偏黑。 */
void GetJumpPointFromDet(uint8 *line, uint8 type, int16 low, int16 high, JumpPointtypedef *jump)
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

/* 边线追逐大致得到全部边线。 */
static void DrawLinesProcess(void)
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
    JumpPointtypedef JumpPoint[2];

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
        PicTemp = Pixle[Ysite];

        if(ImageStatus.Road_type != Cross_ture)
        {
            IntervalLow = ImageDeal[Ysite + 1].RightBorder - ImageScanInterval;
            IntervalHigh = ImageDeal[Ysite + 1].RightBorder + ImageScanInterval;
        }
        else
        {
            IntervalLow = ImageDeal[Ysite + 1].RightBorder - ImageScanInterval_Cross;
            IntervalHigh = ImageDeal[Ysite + 1].RightBorder + ImageScanInterval_Cross;
        }
        GetJumpPointFromDet(PicTemp, 'R', IntervalLow, IntervalHigh, &JumpPoint[1]);

        IntervalLow = ImageDeal[Ysite + 1].LeftBorder - ImageScanInterval;
        IntervalHigh = ImageDeal[Ysite + 1].LeftBorder + ImageScanInterval;
        GetJumpPointFromDet(PicTemp, 'L', IntervalLow, IntervalHigh, &JumpPoint[0]);

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

        if(ImageStatus.Road_type != Ramp)
        {
            if((ImageDeal[Ysite].IsRightFind == 'W') &&
               (Ysite > 10) &&
               (Ysite < 50) &&
               (ImageStatus.Road_type != Barn_in))
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

            if((ImageDeal[Ysite].IsLeftFind == 'W') &&
               (Ysite > 10) &&
               (Ysite < 50) &&
               (ImageStatus.Road_type != Barn_in))
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

/* 十字补线沿用当前版本实现，不回退到 past。 */
static void DrawExtensionLine(void)
{
    int16 center_temp;
    int16 line_temp;

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

    for(Ysite = 59; Ysite >= ImageStatus.OFFLine; Ysite--)
    {
        LimitL(ImageDeal[Ysite].LeftBorder);
        LimitH(ImageDeal[Ysite].LeftBorder);
        LimitL(ImageDeal[Ysite].RightBorder);
        LimitH(ImageDeal[Ysite].RightBorder);
        ImageDeal[Ysite].Center = (ImageDeal[Ysite].LeftBorder + ImageDeal[Ysite].RightBorder) / 2;
        ImageDeal[Ysite].Wide = ImageDeal[Ysite].RightBorder - ImageDeal[Ysite].LeftBorder;
    }

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

/* 用中线方差粗判当前是否为可加速直道。 */
static void Straightacc_Test(void)
{
    int sum = 0;

    for(Ysite = 55; Ysite > (ImageStatus.OFFLine + 1); Ysite--)
    {
        sum += (ImageDeal[Ysite].Center - ImageSensorMid) *
               (ImageDeal[Ysite].Center - ImageSensorMid);
    }

    variance = (float)sum / (54 - ImageStatus.OFFLine);
    ImageStatus.variance_acc = (int)variance;
    if((variance < variance_acc) &&
       (ImageStatus.OFFLine <= 7) &&
       (ImageStatus.Left_Line < 2) &&
       (ImageStatus.Right_Line < 2))
    {
        ImageStatus.straight_acc = 1;
    }
    else
    {
        ImageStatus.straight_acc = 0;
    }
}

/* 计算某一侧边线在给定行段内的直线程度。 */
static float Straight_Judge(uint8 dir, uint8 start_row, uint8 end_row)
{
    int16 row;
    int16 count;
    float sum;
    float err;
    float slope;

    if(start_row >= IMAGE_H)
    {
        start_row = IMAGE_H - 1;
    }
    if(end_row >= IMAGE_H)
    {
        end_row = IMAGE_H - 1;
    }
    if(start_row >= end_row)
    {
        return 999.0f;
    }

    count = (int16)end_row - (int16)start_row;
    if(count == 0)
    {
        return 999.0f;
    }

    sum = 0.0f;
    if(dir == 1)
    {
        slope = ((float)ImageDeal[start_row].LeftBorder -
                 (float)ImageDeal[end_row].LeftBorder) /
                (float)((int16)start_row - (int16)end_row);
        for(row = 0; row < count; row++)
        {
            err = ((float)ImageDeal[start_row].LeftBorder +
                   slope * (float)row -
                   (float)ImageDeal[start_row + row].LeftBorder);
            sum += err * err;
        }
    }
    else if(dir == 2)
    {
        slope = ((float)ImageDeal[start_row].RightBorder -
                 (float)ImageDeal[end_row].RightBorder) /
                (float)((int16)start_row - (int16)end_row);
        for(row = 0; row < count; row++)
        {
            err = ((float)ImageDeal[start_row].RightBorder +
                   slope * (float)row -
                   (float)ImageDeal[start_row + row].RightBorder);
            sum += err * err;
        }
    }
    else
    {
        return 999.0f;
    }

    return sum / (float)count;
}

/* 左环入口判定。 */
static void Element_Judgment_Left_Rings(void)
{
    if((ImageStatus.Right_Line > 7) ||
       (ImageStatus.Left_Line < 13) ||
       (ImageStatus.OFFLine > 10) ||
       (Straight_Judge(2, 25, 45) > 50.0f) ||
       (ImageStatus.WhiteLine > 15) ||
       (ImageDeal[52].IsLeftFind == 'W') ||
       (ImageDeal[53].IsLeftFind == 'W') ||
       (ImageDeal[54].IsLeftFind == 'W') ||
       (ImageDeal[55].IsLeftFind == 'W') ||
       (ImageDeal[56].IsLeftFind == 'W') ||
       (ImageDeal[57].IsLeftFind == 'W') ||
       (ImageDeal[58].IsLeftFind == 'W'))
    {
        return;
    }

    Left_RingsFlag_Point1_Ysite = 0;
    Left_RingsFlag_Point2_Ysite = 0;
    for(Ysite = 58; Ysite > 25; Ysite--)
    {
        if((ImageDeal[Ysite].LeftBoundary_First -
            ImageDeal[Ysite - 1].LeftBoundary_First) > 4)
        {
            Left_RingsFlag_Point1_Ysite = Ysite;
            break;
        }
    }

    for(Ysite = 58; Ysite > 25; Ysite--)
    {
        if((ImageDeal[Ysite + 1].LeftBoundary -
            ImageDeal[Ysite].LeftBoundary) > 4)
        {
            Left_RingsFlag_Point2_Ysite = Ysite;
            break;
        }
    }

    if(Left_RingsFlag_Point1_Ysite > 52)
    {
        Left_RingsFlag_Point1_Ysite = 52;
    }

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

    if((Left_RingsFlag_Point2_Ysite > (Left_RingsFlag_Point1_Ysite + 3)) &&
       (Ring_Help_Flag == 0) &&
       (ImageStatus.Left_Line > 13))
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
        ImageStatus.Road_type = LeftCirque;
    }

    Ring_Help_Flag = 0;
}

/* 右环入口判定。 */
static void Element_Judgment_Right_Rings(void)
{
    int16 point_span;

    if((ImageStatus.Left_Line > 7) ||
       (ImageStatus.Right_Line < 13) ||
       (ImageStatus.OFFLine > 10) ||
       (Straight_Judge(1, 25, 45) > RightRingDetectLeftEdgeStraightMax) ||
       (ImageStatus.WhiteLine > 15) ||
       (ImageDeal[52].IsRightFind == 'W') ||
       (ImageDeal[53].IsRightFind == 'W') ||
       (ImageDeal[54].IsRightFind == 'W') ||
       (ImageDeal[55].IsRightFind == 'W') ||
       (ImageDeal[56].IsRightFind == 'W') ||
       (ImageDeal[57].IsRightFind == 'W') ||
       (ImageDeal[58].IsRightFind == 'W'))
    {
        return;
    }

    Right_RingsFlag_Point1_Ysite = 0;
    Right_RingsFlag_Point2_Ysite = 0;
    for(Ysite = 58; Ysite > 25; Ysite--)
    {
        if((ImageDeal[Ysite - 1].RightBoundary_First -
            ImageDeal[Ysite].RightBoundary_First) > 4)
        {
            Right_RingsFlag_Point1_Ysite = Ysite;
            break;
        }
    }

    for(Ysite = 58; Ysite > 25; Ysite--)
    {
        if((ImageDeal[Ysite].RightBoundary -
            ImageDeal[Ysite + 1].RightBoundary) > 4)
        {
            Right_RingsFlag_Point2_Ysite = Ysite;
            break;
        }
    }

    if(Right_RingsFlag_Point1_Ysite > 52)
    {
        Right_RingsFlag_Point1_Ysite = 52;
    }

    point_span = Right_RingsFlag_Point2_Ysite - Right_RingsFlag_Point1_Ysite;
    if((Right_RingsFlag_Point1_Ysite <= 25) ||
       (Right_RingsFlag_Point2_Ysite <= 25) ||
       (point_span <= 3) ||
       (point_span > RightRingDetectPointSpanMax))
    {
        Ring_Help_Flag = 0;
        return;
    }

    for(Ysite = Right_RingsFlag_Point1_Ysite; Ysite > ImageStatus.OFFLine; Ysite--)
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

    if((Ring_Help_Flag == 0) && (ImageStatus.Right_Line > 7))
    {
        Ring_Help_Flag = 1;
    }

    if((Ring_Help_Flag == 1) && (ImageFlag.image_element_rings_flag == 0))
    {
        ImageFlag.image_element_rings = 2;
        ImageFlag.image_element_rings_flag = 1;
        ImageFlag.ring_big_small = 1;
        ImageStatus.Road_type = RightCirque;
    }

    Ring_Help_Flag = 0;
}

/* 元素总判定，当前主要负责直道和左右环入口检测。 */
void Element_Test(void)
{
    if(ImageStatus.Road_type != Cross
       &&ImageStatus.Road_type != LeftCirque
       &&ImageStatus.Road_type != RightCirque
       &&ImageStatus.Road_type != Barn_in
       &&ImageStatus.Road_type != Ramp
       &&ImageStatus.Road_type != Cross_ture)
    {
        ImageStatus.Road_type = Normol;
    }

    if(ImageStatus.Road_type != Cross
       &&ImageStatus.Road_type != LeftCirque
       &&ImageStatus.Road_type != RightCirque)
    {
        Straightacc_Test();
    }

    if(ImageStatus.Road_type != Barn_in
       &&ImageStatus.Road_type != Cross_ture
       &&ImageStatus.Road_type != Barn_out)
    {
        Element_Judgment_Left_Rings();
        Element_Judgment_Right_Rings();
    }
}

/* 左环状态机执行和补线。 */
static void Element_Handle_Left_Rings(void)
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
        /* 响一下，提示左环进入第 5 步，开始进环补线。 */
        buzzer_long();
    }
    if((ImageFlag.image_element_rings_flag == 5) && (ImageStatus.Right_Line > 15))
    {
        ImageFlag.image_element_rings_flag = 6;
    }
    if((ImageFlag.image_element_rings_flag == 6) && (ImageStatus.Right_Line < 3))
    {
        ImageFlag.image_element_rings_flag = 7;
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
            ImageStatus.Road_type = Normol;
            ImageFlag.image_element_rings_flag = 0;
            ImageFlag.image_element_rings = 0;
            ImageFlag.ring_big_small = 0;
            buzzer_short();
        }
    }

    if((ImageFlag.image_element_rings_flag >= 1) &&
       (ImageFlag.image_element_rings_flag <= 4))
    {
        for(Ysite = 57; Ysite > ImageStatus.OFFLine; Ysite--)
        {
            ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBorder - Half_Road_Wide[Ysite] - 3;
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
                if((Pixle[Ysite][Xsite] == IMAGE_WHITE) &&
                   (Pixle[Ysite][Xsite + 1] == IMAGE_BLACK))
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
                    if((Pixle[Ysite][Xsite] == IMAGE_WHITE) &&
                       (Pixle[Ysite][Xsite + 1] == IMAGE_BLACK))
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
        for(Ysite = 40; Ysite > 5; Ysite--)
        {
            if((Pixle[Ysite][28] == IMAGE_WHITE) &&
               (Pixle[Ysite - 1][28] == IMAGE_BLACK))
            {
                Repair_Point_Xsite = 28;
                Repair_Point_Ysite = Ysite - 1;
                ImageStatus.OFFLine = Ysite + 1;
                break;
            }
        }

        for(Ysite = 57; Ysite > (Repair_Point_Ysite - 3); Ysite--)
        {
            ImageDeal[Ysite].RightBorder =
                (ImageDeal[58].RightBorder - Repair_Point_Xsite) * (Ysite - 58) /
                (58 - Repair_Point_Ysite) + ImageDeal[58].RightBorder;
            ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBorder - Half_Road_Wide[Ysite] - LeftRingExitCenterBias;
            if(ImageDeal[Ysite].Center <= ImageDeal[Ysite].LeftBorder)
            {
                ImageDeal[Ysite].Center = ImageDeal[Ysite].LeftBorder + 1;
            }
        }
    }

    if((ImageFlag.image_element_rings_flag == 9) ||
       (ImageFlag.image_element_rings_flag == 10))
    {
        for(Ysite = 59; Ysite > ImageStatus.OFFLine; Ysite--)
        {
            ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBorder - Half_Road_Wide[Ysite] - LeftRingExitCenterBias;
            if(ImageDeal[Ysite].Center <= ImageDeal[Ysite].LeftBorder)
            {
                ImageDeal[Ysite].Center = ImageDeal[Ysite].LeftBorder + 1;
            }
        }
    }
}

/* 右环状态机执行和补线。 */
static void Element_Handle_Right_Rings(void)
{
    int16 num;
    int16 flag_x;
    int16 flag_y;
    int16 scan_start;
    int16 scan_end;
    int16 center_bias;
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
        /* 响一下，提示右环进入第 5 步，开始进环补线。 */
        buzzer_long();
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
            ImageStatus.Road_type = Normol;
            ImageFlag.image_element_rings_flag = 0;
            ImageFlag.image_element_rings = 0;
            ImageFlag.ring_big_small = 0;
            buzzer_short();
        }
    }

    if((ImageFlag.image_element_rings_flag >= 1) &&
       (ImageFlag.image_element_rings_flag <= 4))
    {
        center_bias = (ImageFlag.image_element_rings_flag <= 2) ?
                      RightRingPreEnterCenterBiasStage12 :
                      RightRingPreEnterCenterBiasDefault;
        for(Ysite = 59; Ysite > ImageStatus.OFFLine; Ysite--)
        {
            ImageDeal[Ysite].Center = ImageDeal[Ysite].LeftBorder + Half_Road_Wide[Ysite] + center_bias;
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
                if((Pixle[Ysite][Xsite] == IMAGE_WHITE) &&
                   (Pixle[Ysite][Xsite + 1] == IMAGE_BLACK))
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
                    if((Pixle[Ysite][Xsite] == IMAGE_WHITE) &&
                       (Pixle[Ysite][Xsite - 1] == IMAGE_BLACK))
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
        Repair_Point_Xsite = 59;
        Repair_Point_Ysite = 0;
        for(Ysite = 40; Ysite > 5; Ysite--)
        {
            if((Pixle[Ysite][51] == IMAGE_WHITE) &&
               (Pixle[Ysite - 1][51] == IMAGE_BLACK))
            {
                Repair_Point_Xsite = 51;
                Repair_Point_Ysite = Ysite - 1;
                ImageStatus.OFFLine = Ysite + 1;
                break;
            }
        }

        for(Ysite = 57; Ysite > (Repair_Point_Ysite - 3); Ysite--)
        {
            ImageDeal[Ysite].LeftBorder =
                (ImageDeal[58].LeftBorder - Repair_Point_Xsite) * (Ysite - 58) /
                (58 - Repair_Point_Ysite) + ImageDeal[58].LeftBorder;
            ImageDeal[Ysite].Center = ImageDeal[Ysite].LeftBorder + Half_Road_Wide[Ysite] + RightRingExitCenterBias;
            if(ImageDeal[Ysite].Center >= ImageDeal[Ysite].RightBorder)
            {
                ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBorder - 1;
            }
        }
    }

    if(ImageFlag.image_element_rings_flag == 9)
    {
        for(Ysite = 59; Ysite > ImageStatus.OFFLine; Ysite--)
        {
            ImageDeal[Ysite].Center = ImageDeal[Ysite].LeftBorder + Half_Road_Wide[Ysite] + RightRingExitCenterBias;
            if(ImageDeal[Ysite].Center >= ImageDeal[Ysite].RightBorder)
            {
                ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBorder - 1;
            }
        }
    }
}

/* 按当前环岛方向分发执行逻辑。 */
void Element_Handle(void)
{
    if(ImageFlag.image_element_rings == 1)
    {
        Element_Handle_Left_Rings();
    }
    else if(ImageFlag.image_element_rings == 2)
    {
        Element_Handle_Right_Rings();
    }
}

/* 扫描中下部斑马线跳变，至少两行同时命中，减少单行噪声误判。 */
static uint8 ZebraScanHit(void)
{
    int row = 0;
    int col = 0;
    int left_limit = 0;
    int right_limit = 0;
    uint8 hit_rows = 0;
    uint8 edge_count = 0;

    if((ImageStatus.Road_type == LeftCirque) ||
       (ImageStatus.Road_type == RightCirque) ||
       (ImageStatus.Road_type == Ramp))
    {
        return 0;
    }

    for(row = 45; row < 55; row++)
    {
        edge_count = 0;
        left_limit = Limit(ImageDeal[row].LeftBoundary - 5, 77, 0);
        right_limit = Limit(ImageDeal[row].RightBoundary + 5, 78, 1);
        if(left_limit >= right_limit)
        {
            continue;
        }

        for(col = left_limit; col < right_limit; col++)
        {
            if((Pixle[row][col] == IMAGE_BLACK) &&
               (Pixle[row][col + 1] == IMAGE_WHITE))
            {
                edge_count++;
                if(edge_count > IMAGE_ZEBRA_EDGE_MIN)
                {
                    hit_rows++;
                    break;
                }
            }
        }
    }

    return (hit_rows >= IMAGE_ZEBRA_ROW_HIT_MIN) ? 1U : 0U;
}

/* 第一次斑马线只记数鸣叫，第二次有效命中后停车。 */
static void CheckZebraEmergency(void)
{
    uint8 zebra_hit;

    if(CarMode != CAR_MODE_RUN)
    {
        ZebraHit = 0;
        ZebraDetectCount = 0;
        ZebraFrameLatch = 0;
        ZebraMissFrames = 0;
        ZebraCooldownFrames = 0;
        return;
    }

    if(ZebraCooldownFrames > 0)
    {
        ZebraCooldownFrames--;
    }

    zebra_hit = ZebraScanHit();
    ZebraHit = zebra_hit;
    if(zebra_hit)
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
                    Speed_Goal = 0;
                    CarMode = CAR_MODE_BRAKE_STOP;
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

/*误差按权重重新整定*/
static void GetDet(uint8 TowPoint)
{
    float DetTemp;
    float UnitAll;

    DetTemp = 0.0f;
    UnitAll = 0.0f;

    if((TowPoint - 5) >= ImageStatus.OFFLine)
    {
        for(Ysite = (int16)(TowPoint - 5); Ysite < TowPoint; Ysite++)
        {
            DetTemp += Weighting[TowPoint - Ysite - 1] * (float)ImageDeal[Ysite].Center;
            UnitAll += Weighting[TowPoint - Ysite - 1];
        }
        for(Ysite = (int16)(TowPoint + 5); Ysite > TowPoint; Ysite--)
        {
            DetTemp += Weighting[Ysite - TowPoint - 1] * (float)ImageDeal[Ysite].Center;
            UnitAll += Weighting[Ysite - TowPoint - 1];
        }
        DetTemp = ((float)ImageDeal[TowPoint].Center + DetTemp) / (UnitAll + 1.0f);
    }
    else if(TowPoint > ImageStatus.OFFLine)
    {
        for(Ysite = ImageStatus.OFFLine; Ysite < TowPoint; Ysite++)
        {
            DetTemp += Weighting[TowPoint - Ysite - 1] * (float)ImageDeal[Ysite].Center;
            UnitAll += Weighting[TowPoint - Ysite - 1];
        }
        for(Ysite = (int16)(TowPoint + TowPoint - ImageStatus.OFFLine); Ysite > TowPoint; Ysite--)
        {
            DetTemp += Weighting[Ysite - TowPoint - 1] * (float)ImageDeal[Ysite].Center;
            UnitAll += Weighting[Ysite - TowPoint - 1];
        }
        DetTemp = ((float)ImageDeal[TowPoint].Center + DetTemp) / (UnitAll + 1.0f);
    }
    else if(ImageStatus.OFFLine < 49)
    {
        for(Ysite = (int16)(ImageStatus.OFFLine + 3); Ysite > ImageStatus.OFFLine; Ysite--)
        {
            DetTemp += Weighting[Ysite - TowPoint - 1] * (float)ImageDeal[Ysite].Center;
            UnitAll += Weighting[Ysite - TowPoint - 1];
        }
        DetTemp = ((float)ImageDeal[ImageStatus.OFFLine].Center + DetTemp) / (UnitAll + 1.0f);
    }
    else
    {
        DetTemp = (float)ImageStatus.Det_True;
    }

    ImageStatus.Det_True = (int16)DetTemp;
    ImageStatus.TowPoint_True = TowPoint;
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//  @name           Search_Border_OTSU
//  @brief          通过OTSU获取边线 和信息
//  @param          imageInput[IMAGE_ROW][IMAGE_COL]        传入的图像数组
//  @param          Row                                     图像的Ysite
//  @param          Col                                     图像的Xsite
//  @param          Bottonline                              底边行选择
//  @return         无
//  @time           2022年10月7日
//  @Author
//  Sample usage:   Search_Border_OTSU(mt9v03x_image, IMAGE_ROW, IMAGE_COL, IMAGE_ROW-8);
//--------------------------------------------------------------------------------------------------------------------------------------------
static void Search_Border_OTSU(uint8 imageInput[LCDH][LCDW], uint8 Row, uint8 Col, uint8 Bottonline)
{
    ImageStatus.WhiteLine_L = 0;
    ImageStatus.WhiteLine_R = 0;
    for(Xsite = 0; Xsite < LCDW; Xsite++)
    {
        imageInput[0][Xsite] = 0;
        imageInput[Bottonline + 1][Xsite] = 0;
    }

    for(Ysite = 0; Ysite < LCDH; Ysite++)
    {
        ImageDeal[Ysite].LeftBoundary_First = 0;
        ImageDeal[Ysite].RightBoundary_First = 79;
        imageInput[Ysite][0] = 0;
        imageInput[Ysite][LCDW - 1] = 0;
    }

    Search_Bottom_Line_OTSU(imageInput, Row, Col, Bottonline);
    Search_Left_and_Right_Lines(imageInput, Row, Col, Bottonline);

    for(Ysite = Bottonline; Ysite > ImageStatus.OFFLineBoundary + 1; Ysite--)
    {
        if(ImageDeal[Ysite].LeftBoundary < 3)
        {
            ImageStatus.WhiteLine_L++;
        }
        if(ImageDeal[Ysite].RightBoundary > LCDW - 3)
        {
            ImageStatus.WhiteLine_R++;
        }
    }
}

/* 中线滤波平滑。 */
static void RouteFilter(void)
{
    int CenterTemp = 0;
    int LineTemp = 0;

    for(Ysite = 58; Ysite >= (ImageStatus.OFFLine + 5); Ysite--)
    {
        if(ImageDeal[Ysite].IsLeftFind == 'W'
           && ImageDeal[Ysite].IsRightFind == 'W'
           && Ysite <= 45
           && ImageDeal[Ysite - 1].IsLeftFind == 'W'
           && ImageDeal[Ysite - 1].IsRightFind == 'W')
        {
            ytemp = Ysite;
            while(ytemp >= (ImageStatus.OFFLine + 5))
            {
                ytemp--;
                if(ImageDeal[ytemp].IsLeftFind == 'T'
                   && ImageDeal[ytemp].IsRightFind == 'T')
                {
                    DetR = (float)(ImageDeal[ytemp - 1].Center - ImageDeal[Ysite + 2].Center) /
                           (float)(ytemp - 1 - Ysite - 2);
                    CenterTemp = ImageDeal[Ysite + 2].Center;
                    LineTemp = Ysite + 2;
                    while(Ysite >= ytemp)
                    {
                        ImageDeal[Ysite].Center = (int)(CenterTemp + DetR * (float)(Ysite - LineTemp));
                        Ysite--;
                    }
                    break;
                }
            }
        }
        ImageDeal[Ysite].Center = (ImageDeal[Ysite - 1].Center + 2 * ImageDeal[Ysite].Center) / 3;
    }
}

/* 图像主处理流程：压缩、二值化、搜线、判元素、算偏差。 */
void ImageProcess(void)
{
    gpio_set_level(IO_P52, 0);

    compressimage();          //对图像进行压缩
    ImageStatus.OFFLine = 2;  //限制图像顶端
    ImageStatus.WhiteLine = 0;
    for(Ysite = 59; Ysite >= ImageStatus.OFFLine; Ysite--)//从下往上搜线（因为第60行是最上面）
    {
        ImageDeal[Ysite].IsLeftFind = 'F';
        ImageDeal[Ysite].IsRightFind = 'F';
        ImageDeal[Ysite].LeftBorder = 0;
        ImageDeal[Ysite].RightBorder = 79;
        ImageDeal[Ysite].LeftTemp = 0;
        ImageDeal[Ysite].RightTemp = 79;
        ImageDeal[Ysite].close_LeftBorder = 0;
        ImageDeal[Ysite].close_RightBorder = 79;
    }                     //边界与标志位初始化

    // Get01change_roi_mix();  /* 图像二值化 */
    Get01change_dajin();       /* 图像二值化 */

    // 出界停车检测
    CheckOutTrackEmergency();

    DrawLinesFirst();     //绘制底边
    DrawLinesProcess();   //搜边线

    Search_Border_OTSU(Pixle, LCDH, LCDW, LCDH - 2);//58行位底行

    Element_Test();       //元素判断
    DrawExtensionLine();  /* 绘制延长线，补线。 */
    RouteFilter();        /* 中线滤波平滑。 */
    CheckZebraEmergency();  /* 斑马线第一次只记数，第二次命中进入零速闭环刹停。 */

    Element_Handle();     //环岛执行
    ImageStatus.TowPoint = (uint8)SmartCar.servo.tow_point;
    runtime_tow_point = SearchLine_GetRuntimeTowPoint();
    // SearchLine_ApplyCenterCompensation(runtime_tow_point);  // 中线压缩补偿
    GetDet(runtime_tow_point);             //获取动态前瞻  并且计算图像偏差
    
    gpio_set_level(IO_P52, 1);
}

/* 图像更新：等一帧 DMA 完成后跑完整处理流程并发布结果。 */
void image_update(void)
{
    if(0U == image_ready)
    {
        return;
    }

    if(mt9v03x_finish_flag == 0)
    {
        return;
    }

    ImageProcess();
    if(CAR_MODE_STOP == CarMode)
    {
        Speed_Goal = 0;
        image_result_ready = 0;
        image_export_result();
        return;
    }

    if((CAR_MODE_RUN == CarMode) && (ImageRawThreshold < IMAGE_STOP_RAW_THRESHOLD))
    {
        Speed_Goal = 0;
        image_result_ready = 0;
        CarMode = CAR_MODE_STOP;
        image_export_result();
        return;
    }

    image_result_ready = 1;
    image_result_sequence++;
    image_export_result();
}
