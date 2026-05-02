#include "image.h"
#include "motor.h"
#include "state.h"
#include "stdlib.h"

static uint8 image_ready = 0;
static uint8 image_result_ready = 0;
static uint8 image_buzzer_busy = 0;
static uint16 image_buzzer_time_left = 0;
static volatile uint8 image_buzzer_tick_ready = 0;


/* 蜂鸣器定时器 */
static void image_buzzer_pit_handler(void)
{
    image_buzzer_tick_ready = 1;
}

/* 蜂鸣器初始化 */
static void image_buzzer_init(void)
{
    gpio_init(IMAGE_BUZZER_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    pit_ms_init(IMAGE_BUZZER_PIT, IMAGE_BUZZER_PERIOD_MS, image_buzzer_pit_handler);
    interrupt_set_priority(TIM7_IRQn, IMAGE_BUZZER_PRIORITY);
}

/* 蜂鸣器启动 */
static void image_buzzer_start(uint16 time_ms)
{
    if(image_buzzer_busy)
    {
        return;
    }

    gpio_set_level(IMAGE_BUZZER_PIN, GPIO_HIGH);
    image_buzzer_busy = 1;
    image_buzzer_time_left = time_ms;
}

/* 蜂鸣器更新 */
void image_buzzer_update(void)
{
    uint8 tick_ready;

    interrupt_global_disable();
    tick_ready = image_buzzer_tick_ready;
    image_buzzer_tick_ready = 0;
    interrupt_global_enable();

    if(!tick_ready)
    {
        return;
    }

    if(!image_buzzer_busy)
    {
        return;
    }

    if(image_buzzer_time_left <= IMAGE_BUZZER_PERIOD_MS)
    {
        gpio_set_level(IMAGE_BUZZER_PIN, GPIO_LOW);
        image_buzzer_busy = 0;
        image_buzzer_time_left = 0;
        return;
    }

    image_buzzer_time_left = (uint16)(image_buzzer_time_left - IMAGE_BUZZER_PERIOD_MS);
}

/* 相机参数下发 */
static uint8 image_apply_camera_page(const flash_camera_page_t *page)
{
    short int config[MT9V03X_CONFIG_FINISH][2];

    if(0 == page)
    {
        return 0;
    }

    memset(config, 0, sizeof(config));
    config[0][0] = MT9V03X_INIT;
    config[0][1] = 0;
    config[1][0] = MT9V03X_AUTO_EXP;
    config[1][1] = 0;
    config[2][0] = MT9V03X_EXP_TIME;
    config[2][1] = page->exp_time;
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
    config[8][1] = page->gain;

    return (0 == mt9v03x_sccb_set_config(config)) ? 1 : 0;
}

/* 图像初始化 */
void image_init(void)
{
    image_buzzer_init();

    while(mt9v03x_init())
    {
        system_delay_ms(100);
    }

    image_reload_camera_page();
    image_ready = 1;
    image_result_ready = 0;
}

/* 图像状态 */
uint8 image_is_ready(void)
{
    return image_ready;
}

/* 结果状态 */
uint8 image_is_result_ready(void)
{
    return image_result_ready;
}

/* 设置相机参数 */
uint8 image_set_camera_value(flash_camera_slot_t slot, int16 value)
{
    flash_camera_page_t page;
    uint8 apply_ok;

    if(!image_ready)
    {
        return 0;
    }

    flash_get_camera_page(&page);

    switch(slot)
    {
        case FLASH_CAMERA_EXP_TIME:
            page.exp_time = value;
            apply_ok = (0 == mt9v03x_set_exposure_time((uint16)value)) ? 1 : 0;
            break;
        case FLASH_CAMERA_GAIN:
            page.gain = value;
            apply_ok = image_apply_camera_page(&page);
            break;
        default:
            return 0;
    }

    if(!apply_ok)
    {
        return 0;
    }

    return flash_set_camera_value(slot, value);
}

/* 重载相机参数 */
void image_reload_camera_page(void)
{
    flash_camera_page_t page;

    flash_get_camera_page(&page);
    image_apply_camera_page(&page);
}

/* 原图地址 */
const uint8 *image_get_raw_buffer(void)
{
    return Image_Use[0];
}

/* 二值图地址 */
const uint8 *image_get_binary_buffer(void)
{
    return Pixle[0];
}

/* 蜂鸣器短响 */
static void buzzer_short(void)
{
    image_buzzer_start(IMAGE_BUZZER_SHORT_MS);
}

/* 压缩灰度图和二值图直接按国一口径导出。 */
uint8 Image_Use[LCDH][LCDW] = {0};
uint8 Pixle[LCDH][LCDW] = {0};
ImageStatustypedef ImageStatus =
{
    32,  /* 当前图像先裁掉底部 20 行，给定前瞻要比参考全图口径略往下取。 */
    0,
    0,
    0,
    70,
    180
};
ImageDealDatatypedef ImageDeal[LCDH] = {0};
int ImageScanInterval = 2;
int ImageScanInterval_Cross = 2;
static int Ysite = 0, Xsite = 0;
static uint8 *PicTemp = 0;
static int IntervalLow = 0, IntervalHigh = 0;
static int ytemp = 0;
static int TFSite = 0, FTSite = 0;
static float DetR = 0, DetL = 0;
static int BottomBorderRight = 79, BottomBorderLeft = 0, BottomCenter = 0;
static uint8 ExtenLFlag = 0;
static uint8 ExtenRFlag = 0;
int Fork_dowm = 0;
ImageFlagtypedef ImageFlag = {0};
uint8 Ring_Help_Flag = 0;
int Left_RingsFlag_Point1_Ysite = 0, Left_RingsFlag_Point2_Ysite = 0;
int Right_RingsFlag_Point1_Ysite = 0, Right_RingsFlag_Point2_Ysite = 0;
int Point_Xsite = 0, Point_Ysite = 0;
int Repair_Point_Xsite = 0, Repair_Point_Ysite = 0;
static uint8 CompressRowMap[LCDH] = {0};
static uint8 CompressColMap[LCDW] = {0};
static uint8 CompressMapReady = 0;
static uint8 OtsuRawThreshold = 0;
static uint8 OutTrackStopHitCount = 0;
static uint8 ZebraDetectCount = 0;
static uint8 ZebraFrameLatch = 0;
static uint8 ZebraMissFrames = 0;
static uint8 ZebraCooldownFrames = 0;
static uint8 runtime_tow_point = 0;
static uint16 Speed_Goal = 150;
float variance = 0, variance_acc = 25;  //方差
static float Weighting[10] =
{
    0.96f, 0.92f, 0.88f, 0.83f, 0.77f,
    0.71f, 0.65f, 0.59f, 0.53f, 0.47f
};
static const uint8 Half_Road_Wide[LCDH] =
{
    6, 7, 7, 8, 8, 9, 9, 9, 10, 10,
    11, 11, 11, 11, 11, 12, 12, 13, 13, 14,
    14, 14, 14, 15, 15, 16, 16, 16, 17, 17,
    17, 18, 18, 19, 19, 20, 20, 20, 21, 21,
    21, 22, 22, 23, 23, 23, 24, 24, 25, 25,
    26, 26, 26, 26, 27, 27, 27, 28, 28, 30
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
    mt9v03x_finish_flag = 0;  //使用完一帧DMA传输的图像图像  可以开始传输下一帧
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//  @name           Search_Bottom_Line_OTSU
//  @brief          获取底层左右边线
//  @param          imageInput[IMAGE_ROW][IMAGE_COL]        传入的图像数组
//  @param          Row                                     图像的Ysite
//  @param          Col                                     图像的Xsite
//  @param          Bottonline                              底边行选择
//  @return         Bottonline                              底边行选择
//  @time           2022年10月9日
//  @Author
//  Sample usage:   Search_Bottom_Line_OTSU(imageInput, Row, Col, Bottonline);
//--------------------------------------------------------------------------------------------------------------------------------------------
void Search_Bottom_Line_OTSU(uint8 imageInput[LCDH][LCDW], uint8 Row, uint8 Col, uint8 Bottonline)
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
void Search_Left_and_Right_Lines(uint8 imageInput[LCDH][LCDW], uint8 Row, uint8 Col, uint8 Bottonline)
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
void Search_Border_OTSU(uint8 imageInput[LCDH][LCDW], uint8 Row, uint8 Col, uint8 Bottonline)
{
    ImageStatus.WhiteLine_L = 0;
    ImageStatus.WhiteLine_R = 0;
    //ImageStatus.OFFLine = 1;
    /*封上下边界处理*/
    for(Xsite = 0; Xsite < LCDW; Xsite++)
    {
        imageInput[0][Xsite] = 0;
        imageInput[Bottonline + 1][Xsite] = 0;
    }
    /*封左右边界处理*/
    for(Ysite = 0; Ysite < LCDH; Ysite++)
    {
        ImageDeal[Ysite].LeftBoundary_First = 0;
        ImageDeal[Ysite].RightBoundary_First = 79;
        imageInput[Ysite][0] = 0;
        imageInput[Ysite][LCDW - 1] = 0;
    }
    /********获取底部边线*********/
    Search_Bottom_Line_OTSU(imageInput, Row, Col, Bottonline);
    /********获取左右边线*********/
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

/* 底边初始化。 */
static uint8 DrawLinesFirst(void)
{
    PicTemp = Pixle[59];
    if(*(PicTemp + ImageSensorMid) == 0)                 //如果底边图像中点为黑，异常情况
    {
        for(Xsite = 0; Xsite < ImageSensorMid; Xsite++)  //找左右边线
        {
            if(*(PicTemp + ImageSensorMid - Xsite) != 0) //一旦找到左或右赛道到中心距离，就break
                break;                                   //并且记录Xsite
            if(*(PicTemp + ImageSensorMid + Xsite) != 0)
                break;
        }

        if(*(PicTemp + ImageSensorMid - Xsite) != 0)     //赛道如果在左边的话
        {
            BottomBorderRight = ImageSensorMid - Xsite + 1;    //59行右边线有啦
            for(Xsite = BottomBorderRight; Xsite > 0; Xsite--) //开始找59行左边线
            {
                if(*(PicTemp + Xsite) == 0 &&
                   *(PicTemp + Xsite - 1) == 0)                //连续两个黑点，滤波
                {
                    BottomBorderLeft = Xsite;                  //左边线找到
                    break;
                }
                else if(Xsite == 1)
                {
                    BottomBorderLeft = 0;                      //搜索到最后了，看不到左边线，左边线认为是0
                    break;
                }
            }
        }
        else if(*(PicTemp + ImageSensorMid + Xsite) != 0)     //赛道如果在右边的话
        {
            BottomBorderLeft = ImageSensorMid + Xsite - 1;     //59行左边线有啦
            for(Xsite = BottomBorderLeft; Xsite < 79; Xsite++) //开始找59行右边线
            {
                if(*(PicTemp + Xsite) == 0 &&
                   *(PicTemp + Xsite + 1) == 0)                //连续两个黑点，滤波
                {
                    BottomBorderRight = Xsite;                 //右边线找到
                    break;
                }
                else if(Xsite == 78)
                {
                    BottomBorderRight = 79;                    //搜索到最后了，看不到右边线，右边线认为是79
                    break;
                }
            }
        }
    }
    else                                                     //中点是白的，比较正常的情况
    {
        for(Xsite = 79; Xsite > ImageSensorMid; Xsite--)     //一个点一个点地搜索右边线
        {
            if(*(PicTemp + Xsite) == 1 &&
               *(PicTemp + Xsite - 1) == 1)                  //连续两个白点，滤波
            {
                BottomBorderRight = Xsite;                   //找到就记录
                break;
            }
            else if(Xsite == 40)
            {
                BottomBorderRight = 39;                      //找不到认为39
                break;
            }
        }
        for(Xsite = 0; Xsite < ImageSensorMid; Xsite++)      //一个点一个点地搜索左边线
        {
            if(*(PicTemp + Xsite) == 1 &&
               *(PicTemp + Xsite + 1) == 1)                  //连续两个白点，滤波
            {
                BottomBorderLeft = Xsite;                    //找到就记录
                break;
            }
            else if(Xsite == 38)
            {
                BottomBorderLeft = 39;                       //找不到认为39
                break;
            }
        }
    }

    BottomCenter = (BottomBorderLeft + BottomBorderRight) / 2;   //59行中点直接取平均
    ImageDeal[59].LeftBorder = BottomBorderLeft;                 //在数组里面记录一下信息，第一行特殊一点而已
    ImageDeal[59].RightBorder = BottomBorderRight;
    ImageDeal[59].Center = BottomCenter;                         //确定最底边
    ImageDeal[59].Wide = BottomBorderRight - BottomBorderLeft;   //存储宽度信息
    ImageDeal[59].IsLeftFind = 'T';
    ImageDeal[59].IsRightFind = 'T';

    for(Ysite = 58; Ysite > 54; Ysite--)                        //由中间向两边确定底边五行
    {
        PicTemp = Pixle[Ysite];
        for(Xsite = 79; Xsite > ImageDeal[Ysite + 1].Center; Xsite--)  //和前面一样的搜索
        {
            if(*(PicTemp + Xsite) == 1 && *(PicTemp + Xsite - 1) == 1)
            {
                ImageDeal[Ysite].RightBorder = Xsite;
                break;
            }
            else if(Xsite == (ImageDeal[Ysite + 1].Center + 1))
            {
                ImageDeal[Ysite].RightBorder = ImageDeal[Ysite + 1].Center;
                break;
            }
        }
        for(Xsite = 0; Xsite < ImageDeal[Ysite + 1].Center; Xsite++)   //和前面一样的搜索
        {
            if(*(PicTemp + Xsite) == 1 && *(PicTemp + Xsite + 1) == 1)
            {
                ImageDeal[Ysite].LeftBorder = Xsite;
                break;
            }
            else if(Xsite == (ImageDeal[Ysite + 1].Center - 1))
            {
                ImageDeal[Ysite].LeftBorder = ImageDeal[Ysite + 1].Center;
                break;
            }
        }
        ImageDeal[Ysite].IsLeftFind = 'T';                          //这些信息存储到数组里
        ImageDeal[Ysite].IsRightFind = 'T';
        ImageDeal[Ysite].Center =
            (ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) / 2; //存储中点
        ImageDeal[Ysite].Wide =
            ImageDeal[Ysite].RightBorder - ImageDeal[Ysite].LeftBorder;       //存储宽度
    }
    return 'T';
}

void GetJumpPointFromDet(uint8 *p, uint8 type, int L, int H, JumpPointtypedef *Q)  //第一个参数是要查找的数组（80个点）
                                                                                   //第二个扫左边线还是扫右边线
{                                                                                  //三四是开始和结束点
    int i = 0;

    if(type == 'L')
    {
        for(i = H; i >= L; i--)
        {
            if(*(p + i) == 1 && *(p + i - 1) != 1)
            {
                Q->point = i;
                Q->type = 'T';
                break;
            }
            else if(i == (L + 1))
            {
                if(*(p + (L + H) / 2) != 0)
                {
                    Q->point = (L + H) / 2;
                    Q->type = 'W';
                    break;
                }
                else
                {
                    Q->point = H;
                    Q->type = 'H';
                    break;
                }
            }
        }
    }
    else if(type == 'R')
    {
        for(i = L; i <= H; i++)
        {
            if(*(p + i) == 1 && *(p + i + 1) != 1)
            {
                Q->point = i;
                Q->type = 'T';
                break;
            }
            else if(i == (H - 1))
            {
                if(*(p + (L + H) / 2) != 0)
                {
                    Q->point = (L + H) / 2;
                    Q->type = 'W';
                    break;
                }
                else
                {
                    Q->point = L;
                    Q->type = 'H';
                    break;
                }
            }
        }
    }
}

/*边线追逐大致得到全部边线*/
static void DrawLinesProcess(void)
{
    uint8 L_Found_T = 'F';  //确定无边斜率的基准有边行是否被找到的标志
    uint8 Get_L_line = 'F';  //找到这一帧图像的基准左斜率
    uint8 R_Found_T = 'F';  //确定无边斜率的基准有边行是否被找到的标志
    uint8 Get_R_line = 'F';  //找到这一帧图像的基准右斜率
    float D_L = 0;           //延长线左边线斜率
    float D_R = 0;           //延长线右边线斜率
    int ytemp_W_L = 0;       //记住首次左丢边行
    int ytemp_W_R = 0;       //记住首次右丢边行
    int ysite = 0;
    uint8 L_found_point = 0;
    uint8 R_found_point = 0;
    JumpPointtypedef JumpPoint[2];  // 0左1右

    ExtenRFlag = 0;          //标志位清0
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

        LimitL(IntervalLow);   //确定左扫描区间并进行限制
        LimitH(IntervalHigh);  //确定右扫描区间并进行限制
        GetJumpPointFromDet(PicTemp, 'R', IntervalLow, IntervalHigh, &JumpPoint[1]);     //扫右边线

        IntervalLow = ImageDeal[Ysite + 1].LeftBorder - ImageScanInterval;
        IntervalHigh = ImageDeal[Ysite + 1].LeftBorder + ImageScanInterval;

        LimitL(IntervalLow);   //确定左扫描区间并进行限制
        LimitH(IntervalHigh);  //确定右扫描区间并进行限制
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

        if((ImageDeal[Ysite].IsLeftFind == 'H') ||
           (ImageDeal[Ysite].IsRightFind == 'H'))
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
                    else if(*(PicTemp + Xsite) != 0)
                    {
                        break;
                    }
                    else if(Xsite == (ImageDeal[Ysite].RightBorder - 1))
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
                    else if(*(PicTemp + Xsite) != 0)
                    {
                        break;
                    }
                    else if(Xsite == (ImageDeal[Ysite].LeftBorder + 1))
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
            if(ImageDeal[Ysite].IsRightFind == 'W' &&
               Ysite > 10 &&
               Ysite < 50 &&
               ImageStatus.Road_type != Barn_in)
            {
                if(Get_R_line == 'F')
                {
                    Get_R_line = 'T';
                    ytemp_W_R = Ysite + 2;
                    for(ysite = Ysite + 1; ysite < Ysite + 15 && ysite < LCDH; ysite++)
                    {
                        if(ImageDeal[ysite].IsRightFind == 'T')
                        {
                            R_found_point++;
                        }
                    }
                    if(R_found_point > 8)
                    {
                        D_R =
                            ((float)(ImageDeal[Ysite + R_found_point].RightBorder -
                                     ImageDeal[Ysite + 3].RightBorder)) /
                            ((float)(R_found_point - 3));
                        if(D_R > 0)
                        {
                            R_Found_T = 'T';
                        }
                        else
                        {
                            R_Found_T = 'F';
                            if(D_R < 0)
                            {
                                ExtenRFlag = 'F';
                            }
                        }
                    }
                }
                if(R_Found_T == 'T')
                {
                    ImageDeal[Ysite].RightBorder =
                        ImageDeal[ytemp_W_R].RightBorder - D_R * (ytemp_W_R - Ysite);
                }

                LimitL(ImageDeal[Ysite].RightBorder);
                LimitH(ImageDeal[Ysite].RightBorder);
            }

            if(ImageDeal[Ysite].IsLeftFind == 'W' &&
               Ysite > 10 &&
               Ysite < 50 &&
               ImageStatus.Road_type != Barn_in)
            {
                if(Get_L_line == 'F')
                {
                    Get_L_line = 'T';
                    ytemp_W_L = Ysite + 2;
                    for(ysite = Ysite + 1; ysite < Ysite + 15 && ysite < LCDH; ysite++)
                    {
                        if(ImageDeal[ysite].IsLeftFind == 'T')
                        {
                            L_found_point++;
                        }
                    }
                    if(L_found_point > 8)
                    {
                        D_L =
                            ((float)(ImageDeal[Ysite + 3].LeftBorder -
                                     ImageDeal[Ysite + L_found_point].LeftBorder)) /
                            ((float)(L_found_point - 3));
                        if(D_L > 0)
                        {
                            L_Found_T = 'T';
                        }
                        else
                        {
                            L_Found_T = 'F';
                            if(D_L < 0)
                            {
                                ExtenLFlag = 'F';
                            }
                        }
                    }
                }

                if(L_Found_T == 'T')
                {
                    ImageDeal[Ysite].LeftBorder =
                        ImageDeal[ytemp_W_L].LeftBorder + D_L * (ytemp_W_L - Ysite);
                }

                LimitL(ImageDeal[Ysite].LeftBorder);
                LimitH(ImageDeal[Ysite].LeftBorder);
            }
        }

        if(ImageDeal[Ysite].IsLeftFind == 'W' &&
           ImageDeal[Ysite].IsRightFind == 'W')
        {
            ImageStatus.WhiteLine++;
        }
        if(ImageDeal[Ysite].IsLeftFind == 'W' && Ysite < 55)
        {
            ImageStatus.Left_Line++;
        }
        if(ImageDeal[Ysite].IsRightFind == 'W' && Ysite < 55)
        {
            ImageStatus.Right_Line++;
        }

        LimitL(ImageDeal[Ysite].LeftBorder);
        LimitH(ImageDeal[Ysite].LeftBorder);
        LimitL(ImageDeal[Ysite].RightBorder);
        LimitH(ImageDeal[Ysite].RightBorder);

        ImageDeal[Ysite].Wide = ImageDeal[Ysite].RightBorder - ImageDeal[Ysite].LeftBorder;
        ImageDeal[Ysite].Center =
            (ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) / 2;
        ImageDeal[Ysite].LeftTemp = ImageDeal[Ysite].LeftBorder;
        ImageDeal[Ysite].RightTemp = ImageDeal[Ysite].RightBorder;
        ImageDeal[Ysite].close_LeftBorder = ImageDeal[Ysite].LeftBorder;
        ImageDeal[Ysite].close_RightBorder = ImageDeal[Ysite].RightBorder;

        if(ImageDeal[Ysite].Wide <= 7)
        {
            ImageStatus.OFFLine = Ysite + 1;
            break;
        }
        else if(ImageDeal[Ysite].RightBorder <= 10 ||
                ImageDeal[Ysite].LeftBorder >= 70)
        {
            ImageStatus.OFFLine = Ysite + 1;
            break;
        }
    }

    return;
}

//延长线绘制，理论上来说是很准确的
static void DrawExtensionLine(void)        //绘制延长线并重新确定中线 ，把补线补成斜线
{
    if(
        (Fork_dowm == 0
         &&ImageStatus.CirquePass == 'F'
         &&ImageStatus.IsCinqueOutIn == 'F'
         &&ImageStatus.CirqueOut == 'F'
         &&ImageStatus.Road_type != Barn_in
         &&ImageStatus.Road_type != Ramp)
        &&ImageStatus.Road_type != Cross_ture
        ||ImageStatus.CirqueOff == 'T')
    {
        if(ImageStatus.WhiteLine >= ImageStatus.TowPoint_True - 15)
            TFSite = 55;
        if(ImageStatus.CirqueOff == 'T' && ImageStatus.Road_type == LeftCirque)
            TFSite = 55;
        if(ExtenLFlag != 'F')
            for(Ysite = 54; Ysite >= (ImageStatus.OFFLine + 4); Ysite--)
            {
                PicTemp = Pixle[Ysite];
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
                        if(ImageDeal[Ysite].IsLeftFind == 'T' &&
                           ImageDeal[Ysite - 1].IsLeftFind == 'T' &&
                           ImageDeal[Ysite - 2].IsLeftFind == 'T' &&
                           ImageDeal[Ysite - 2].LeftBorder > 0 &&
                           ImageDeal[Ysite - 2].LeftBorder < 70)
                        {
                            FTSite = Ysite - 2;
                            break;
                        }
                    }

                    if(FTSite > ImageStatus.OFFLine)
                    {
                        DetL =
                            ((float)(ImageDeal[FTSite].LeftBorder -
                                     ImageDeal[TFSite].LeftBorder)) /
                            ((float)(FTSite - TFSite));
                        for(ytemp = TFSite; ytemp >= FTSite; ytemp--)
                        {
                            ImageDeal[ytemp].LeftBorder =
                                (int)(DetL * ((float)(ytemp - TFSite))) +
                                ImageDeal[TFSite].LeftBorder;
                        }
                    }
                }
                else
                {
                    TFSite = Ysite + 2;
                }
            }

        if(ImageStatus.WhiteLine >= ImageStatus.TowPoint_True - 15)
            TFSite = 55;
        if(ImageStatus.CirqueOff == 'T' && ImageStatus.Road_type == RightCirque)
            TFSite = 55;
        if(ExtenRFlag != 'F')
            for(Ysite = 54; Ysite >= (ImageStatus.OFFLine + 4); Ysite--)
            {
                PicTemp = Pixle[Ysite];

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
                        if(ImageDeal[Ysite].IsRightFind == 'T' &&
                           ImageDeal[Ysite - 1].IsRightFind == 'T' &&
                           ImageDeal[Ysite - 2].IsRightFind == 'T' &&
                           ImageDeal[Ysite - 2].RightBorder < 70 &&
                           ImageDeal[Ysite - 2].RightBorder > 10)
                        {
                            FTSite = Ysite - 2;
                            break;
                        }
                    }

                    if(FTSite > ImageStatus.OFFLine)
                    {
                        DetR =
                            ((float)(ImageDeal[FTSite].RightBorder -
                                     ImageDeal[TFSite].RightBorder)) /
                            ((float)(FTSite - TFSite));
                        for(ytemp = TFSite; ytemp >= FTSite; ytemp--)
                        {
                            ImageDeal[ytemp].RightBorder =
                                (int)(DetR * ((float)(ytemp - TFSite))) +
                                ImageDeal[TFSite].RightBorder;
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
        ImageDeal[Ysite].Wide = -ImageDeal[Ysite].LeftBorder + ImageDeal[Ysite].RightBorder;
    }
}

/* 中线滤波平滑。 */
static void RouteFilter(void)
{
    int CenterTemp = 0;
    int LineTemp = 0;

    for(Ysite = 58; Ysite >= (ImageStatus.OFFLine + 5); Ysite--)                                     //从开始位到停止位
    {
        if(ImageDeal[Ysite].IsLeftFind == 'W'
           &&ImageDeal[Ysite].IsRightFind == 'W'
           &&Ysite <= 45
           &&ImageDeal[Ysite - 1].IsLeftFind == 'W'
           &&ImageDeal[Ysite - 1].IsRightFind == 'W')  //当前行左右都无边，而且在前45行   滤波
        {
            ytemp = Ysite;
            while(ytemp >= (ImageStatus.OFFLine + 5))     // 改改试试，-6效果好一些
            {
                ytemp--;
                if(ImageDeal[ytemp].IsLeftFind == 'T'
                   &&ImageDeal[ytemp].IsRightFind == 'T')  //寻找两边都正常的，找到离本行最近的就不找了
                {
                    DetR = (float)(ImageDeal[ytemp - 1].Center - ImageDeal[Ysite + 2].Center) /
                           (float)(ytemp - 1 - Ysite - 2);          //算斜率
                    CenterTemp = ImageDeal[Ysite + 2].Center;
                    LineTemp = Ysite + 2;
                    while(Ysite >= ytemp)
                    {
                        ImageDeal[Ysite].Center = (int)(CenterTemp + DetR * (float)(Ysite - LineTemp));  //用斜率补
                        Ysite--;
                    }
                    break;
                }
            }
        }
        ImageDeal[Ysite].Center = (ImageDeal[Ysite - 1].Center + 2 * ImageDeal[Ysite].Center) / 3;      //求平均，应该会比较滑  本来是上下两点平均
    }
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
    int speed_min;

    speed_normal = (int)flash_get_motor_value(FLASH_MOTOR_TARGET_SPEED);
    speed_min = speed_normal - 10;  // 速度最小值先暂时就 -10
    if(speed_min < 0)
    {
        /* Car Speed 页速度较低时，最低速度基准不再继续下探。 */
        speed_min = 0;
    }

    speed_left = motor_data.count_left;
    if(speed_left < 0)
    {
        speed_left = -speed_left;
    }

    speed_right = motor_data.count_right;
    if(speed_right < 0)
    {
        speed_right = -speed_right;
    }

    speed_now = (speed_left + speed_right) / 2;
    if(ImageStatus.straight_acc == 1)
    {
        // 暂时关闭直道加速
        // Speed_Goal = (uint16)speed_straight;
        Speed_Goal = (uint16)speed_normal;
    }
    else
    {
        Speed_Goal = (uint16)speed_normal;
    }

    if((ImageStatus.Road_type == RightCirque || ImageStatus.Road_type == LeftCirque) && ImageStatus.CirqueOff == 'F')
        TowPoint = 30;    //圆环前瞻
    else if(ImageStatus.Road_type == Straight)
        TowPoint = ImageStatus.TowPoint;
    else if(ImageStatus.Road_type == Cross_ture)
        TowPoint = 22;
    else if(ImageFlag.image_element_rings_flag == 1 || ImageFlag.image_element_rings_flag == 2)
        TowPoint = 30;
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

/*误差按权重重新整定*/
static void GetDet(uint8 TowPoint)
{
    float DetTemp = 0;
    float UnitAll = 0;

    if((TowPoint - 5) >= ImageStatus.OFFLine) {                                             //前瞻取设定前瞻还是可视距离  需要分情况讨论
        for(Ysite = (TowPoint - 5); Ysite < TowPoint; Ysite++) {
            DetTemp = DetTemp + Weighting[TowPoint - Ysite - 1] * (ImageDeal[Ysite].Center);
            UnitAll = UnitAll + Weighting[TowPoint - Ysite - 1];
        }
        for(Ysite = (TowPoint + 5); Ysite > TowPoint; Ysite--) {
            DetTemp += Weighting[-TowPoint + Ysite - 1] * (ImageDeal[Ysite].Center);
            UnitAll += Weighting[-TowPoint + Ysite - 1];
        }
        DetTemp = (ImageDeal[TowPoint].Center + DetTemp) / (UnitAll + 1);

    } else if(TowPoint > ImageStatus.OFFLine) {
        for(Ysite = ImageStatus.OFFLine; Ysite < TowPoint; Ysite++) {
            DetTemp += Weighting[TowPoint - Ysite - 1] * (ImageDeal[Ysite].Center);
            UnitAll += Weighting[TowPoint - Ysite - 1];
        }
        for(Ysite = (TowPoint + TowPoint - ImageStatus.OFFLine); Ysite > TowPoint; Ysite--) {
            DetTemp += Weighting[-TowPoint + Ysite - 1] * (ImageDeal[Ysite].Center);
            UnitAll += Weighting[-TowPoint + Ysite - 1];
        }
        DetTemp = (ImageDeal[Ysite].Center + DetTemp) / (UnitAll + 1);
    } else if(ImageStatus.OFFLine < 49) {
        for(Ysite = (ImageStatus.OFFLine + 3); Ysite > ImageStatus.OFFLine; Ysite--) {
            DetTemp += Weighting[-TowPoint + Ysite - 1] * (ImageDeal[Ysite].Center);
            UnitAll += Weighting[-TowPoint + Ysite - 1];
        }
        DetTemp = (ImageDeal[ImageStatus.OFFLine].Center + DetTemp) / (UnitAll + 1);

    } else
        DetTemp = ImageStatus.Det_True;                                                     //如果是出现OFFLine>50情况，保持上一次的偏差值

    ImageStatus.Det_True = DetTemp;                                                         //此时的解算出来的平均图像偏差
    ImageStatus.TowPoint_True = TowPoint;                                                   //此时的前瞻
}

//用于加速的直道检测
static void Straightacc_Test(void)
{
    int sum = 0;

    for(Ysite = 55; Ysite > ImageStatus.OFFLine + 1; Ysite--)
    {
        sum += (ImageDeal[Ysite].Center - ImageSensorMid) * (ImageDeal[Ysite].Center - ImageSensorMid);
    }

    variance = (float)sum / (54 - ImageStatus.OFFLine);
    ImageStatus.variance_acc = (int)variance;
    if(variance < variance_acc &&
       ImageStatus.OFFLine <= 7 &&
       ImageStatus.Left_Line < 2 &&
       ImageStatus.Right_Line < 2)
    {
        ImageStatus.straight_acc = 1;
    }
    else
    {
        ImageStatus.straight_acc = 0;
    }
}

static float Straight_Judge(uint8 dir, uint8 start_row, uint8 end_row)
{
    int16 row;
    int16 count;
    float sum = 0.0f;
    float err;
    float slope;

    if(start_row >= LCDH)
    {
        start_row = LCDH - 1;
    }
    if(end_row >= LCDH)
    {
        end_row = LCDH - 1;
    }
    if(start_row >= end_row)
    {
        return 999.0f;
    }

    count = (int16)end_row - (int16)start_row;
    if(0 == count)
    {
        return 999.0f;
    }

    if(1 == dir)
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
    else if(2 == dir)
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

static void Element_Judgment_Left_Rings(void)
{
    if(ImageStatus.Right_Line > 7 ||
       ImageStatus.Left_Line < 13 ||
       ImageStatus.OFFLine > 10 ||
       Straight_Judge(2, 25, 45) > 50.0f ||
       ImageStatus.WhiteLine > 15 ||
       ImageDeal[52].IsLeftFind == 'W' ||
       ImageDeal[53].IsLeftFind == 'W' ||
       ImageDeal[54].IsLeftFind == 'W' ||
       ImageDeal[55].IsLeftFind == 'W' ||
       ImageDeal[56].IsLeftFind == 'W' ||
       ImageDeal[57].IsLeftFind == 'W' ||
       ImageDeal[58].IsLeftFind == 'W')
    {
        return;
    }

    Left_RingsFlag_Point1_Ysite = 0;
    Left_RingsFlag_Point2_Ysite = 0;
    for(Ysite = 58; Ysite > 25; Ysite--)
    {
        if(ImageDeal[Ysite].LeftBoundary_First - ImageDeal[Ysite - 1].LeftBoundary_First > 4)
        {
            Left_RingsFlag_Point1_Ysite = Ysite;
            break;
        }
    }
    for(Ysite = 58; Ysite > 25; Ysite--)
    {
        if(ImageDeal[Ysite + 1].LeftBoundary - ImageDeal[Ysite].LeftBoundary > 4)
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
        if(ImageDeal[Ysite + 6].LeftBorder < ImageDeal[Ysite + 3].LeftBorder &&
           ImageDeal[Ysite + 5].LeftBorder < ImageDeal[Ysite + 3].LeftBorder &&
           ImageDeal[Ysite + 3].LeftBorder > ImageDeal[Ysite + 2].LeftBorder &&
           ImageDeal[Ysite + 3].LeftBorder > ImageDeal[Ysite + 1].LeftBorder)
        {
            Ring_Help_Flag = 1;
            break;
        }
    }
    if(Left_RingsFlag_Point2_Ysite > Left_RingsFlag_Point1_Ysite + 3 && Ring_Help_Flag == 0)
    {
        if(ImageStatus.Left_Line > 13)
        {
            Ring_Help_Flag = 1;
        }
    }
    if(Left_RingsFlag_Point2_Ysite > Left_RingsFlag_Point1_Ysite + 3 &&
       Ring_Help_Flag == 1 &&
       ImageFlag.image_element_rings_flag == 0)
    {
        ImageFlag.image_element_rings = 1;
        ImageFlag.image_element_rings_flag = 1;
        ImageFlag.ring_big_small = 1;
        ImageStatus.Road_type = LeftCirque;
    }
    Ring_Help_Flag = 0;
}

static void Element_Judgment_Right_Rings(void)
{
    float straight_judge;

    straight_judge = Straight_Judge(1, 25, 45);
    if(ImageStatus.Left_Line > 7 ||
       ImageStatus.Right_Line < 13 ||
       ImageStatus.OFFLine > 10 ||
       (straight_judge > 50.0f) ||
       ImageStatus.WhiteLine > 15 ||
       ImageDeal[52].IsRightFind == 'W' ||
       ImageDeal[53].IsRightFind == 'W' ||
       ImageDeal[54].IsRightFind == 'W' ||
       ImageDeal[55].IsRightFind == 'W' ||
       ImageDeal[56].IsRightFind == 'W' ||
       ImageDeal[57].IsRightFind == 'W' ||
       ImageDeal[58].IsRightFind == 'W')
    {
        return;
    }

    Right_RingsFlag_Point1_Ysite = 0;
    Right_RingsFlag_Point2_Ysite = 0;
    for(Ysite = 58; Ysite > 25; Ysite--)
    {
        if(ImageDeal[Ysite - 1].RightBoundary_First - ImageDeal[Ysite].RightBoundary_First > 4)
        {
            Right_RingsFlag_Point1_Ysite = Ysite;
            break;
        }
    }
    for(Ysite = 58; Ysite > 25; Ysite--)
    {
        if(ImageDeal[Ysite].RightBoundary - ImageDeal[Ysite + 1].RightBoundary > 4)
        {
            Right_RingsFlag_Point2_Ysite = Ysite;
            break;
        }
    }

    

    if((Right_RingsFlag_Point1_Ysite <= 25) ||
       (Right_RingsFlag_Point2_Ysite <= 25))
    {
        /* 旧 OTSU 版这里靠前级边界更稳定，当前搜线口径下单点毛刺更多。
         * 两个候选拐点没同时成立时，直接不判右环，先压掉误触发。 */
        Ring_Help_Flag = 0;
        return;
    }

    

    for(Ysite = Right_RingsFlag_Point1_Ysite; Ysite > 10; Ysite--)
    {
        if(ImageDeal[Ysite + 6].RightBorder > ImageDeal[Ysite + 3].RightBorder &&
           ImageDeal[Ysite + 5].RightBorder > ImageDeal[Ysite + 3].RightBorder &&
           ImageDeal[Ysite + 3].RightBorder < ImageDeal[Ysite + 2].RightBorder &&
           ImageDeal[Ysite + 3].RightBorder < ImageDeal[Ysite + 1].RightBorder)
        {
            Ring_Help_Flag = 1;
            break;
        }
    }

    if(Right_RingsFlag_Point2_Ysite > Right_RingsFlag_Point1_Ysite + 3 && Ring_Help_Flag == 0)
    {
        if(ImageStatus.Right_Line > 7)
        {
            Ring_Help_Flag = 1;
        }
    }

    if(Right_RingsFlag_Point2_Ysite > Right_RingsFlag_Point1_Ysite + 3 &&
       Ring_Help_Flag == 1 &&
       ImageFlag.image_element_rings_flag == 0)
    {
        ImageFlag.image_element_rings = 2;
        ImageFlag.image_element_rings_flag = 1;
        ImageFlag.ring_big_small = 1;
        ImageStatus.Road_type = RightCirque;
        // buzzer_short();
    }
    Ring_Help_Flag = 0;
}

static void Element_Handle_Left_Rings(void)
{
    int num = 0;
    int flag_Xsite_1 = 0;
    int flag_Ysite_1 = 0;
    float Slope_Rings = 0.0f;

    /***************************************判断**************************************/
    for(Ysite = 55; Ysite > 30; Ysite--)
    {
        if(ImageDeal[Ysite].IsLeftFind == 'W')
        {
            num++;
        }
        if(ImageDeal[Ysite + 3].IsLeftFind == 'W' &&
           ImageDeal[Ysite + 2].IsLeftFind == 'W' &&
           ImageDeal[Ysite + 1].IsLeftFind == 'W' &&
           ImageDeal[Ysite].IsLeftFind == 'T')
        {
            break;
        }
    }
        //准备进环
    if(ImageFlag.image_element_rings_flag == 1 && num > 10)
    {
        ImageFlag.image_element_rings_flag = 2;
    }
    if(ImageFlag.image_element_rings_flag == 2 && num < 8)
    {
        ImageFlag.image_element_rings_flag = 5;
        buzzer_short();
    }
        //进环
    if(ImageFlag.image_element_rings_flag == 5 && ImageStatus.Right_Line > 15)
    {
        ImageFlag.image_element_rings_flag = 6;
    }
        //进环小圆环
    if(ImageFlag.image_element_rings_flag == 6 && ImageStatus.Right_Line < 3)
    {
        ImageFlag.image_element_rings_flag = 7;
    }

        //环内 大圆环判断
    if(ImageFlag.ring_big_small == 1 && ImageFlag.image_element_rings_flag == 7)
    {
        Point_Ysite = 0;
        Point_Xsite = 0;
        for(Ysite = 50; Ysite > ImageStatus.OFFLine + 3; Ysite--)
        {
            if(ImageDeal[Ysite].RightBorder <= ImageDeal[Ysite + 2].RightBorder &&
               ImageDeal[Ysite].RightBorder <= ImageDeal[Ysite - 2].RightBorder &&
               ImageDeal[Ysite].RightBorder <= ImageDeal[Ysite + 1].RightBorder &&
               ImageDeal[Ysite].RightBorder <= ImageDeal[Ysite - 1].RightBorder &&
               ImageDeal[Ysite].RightBorder <= ImageDeal[Ysite + 4].RightBorder &&
               ImageDeal[Ysite].RightBorder <= ImageDeal[Ysite - 4].RightBorder)
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

        //出环后
    if(ImageFlag.image_element_rings_flag == 8)
    {
        if(ImageStatus.Right_Line < 9 && ImageStatus.OFFLine < 10)
        {
            ImageFlag.image_element_rings_flag = 9;
        }
    }

        //结束圆环进程
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
            ImageStatus.Road_type = 0;
            ImageFlag.image_element_rings_flag = 0;
            ImageFlag.image_element_rings = 0;
            ImageFlag.ring_big_small = 0;
            ImageStatus.Road_type = Normol;
            buzzer_short();
        }
    }

    /***************************************处理**************************************/
        //准备进环  半宽处理
    if(ImageFlag.image_element_rings_flag == 1 ||
       ImageFlag.image_element_rings_flag == 2 ||
       ImageFlag.image_element_rings_flag == 3 ||
       ImageFlag.image_element_rings_flag == 4)
    {
        for(Ysite = 57; Ysite > ImageStatus.OFFLine; Ysite--)
        {
            ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBorder - Half_Road_Wide[Ysite] - 5;
        }
    }
        //进环  补线
    if(ImageFlag.image_element_rings_flag == 5 ||
       ImageFlag.image_element_rings_flag == 6)
    {
        flag_Xsite_1 = 0;
        flag_Ysite_1 = 0;
        for(Ysite = 55; Ysite > ImageStatus.OFFLine; Ysite--)
        {
            for(Xsite = ImageDeal[Ysite].LeftBorder + 1; Xsite < ImageDeal[Ysite].RightBorder - 1; Xsite++)
            {
                if(Pixle[Ysite][Xsite] == 1 && Pixle[Ysite][Xsite + 1] == 0)
                {
                    flag_Ysite_1 = Ysite;
                    flag_Xsite_1 = Xsite;
                    Slope_Rings = (float)(79 - flag_Xsite_1) / (float)(59 - flag_Ysite_1);
                    break;
                }
            }
            if(flag_Ysite_1 != 0)
            {
                break;
            }
        }
        if(flag_Ysite_1 == 0)
        {
            for(Ysite = ImageStatus.OFFLine + 1; Ysite < 30; Ysite++)
            {
                if(ImageDeal[Ysite].IsLeftFind == 'T' &&
                   ImageDeal[Ysite + 1].IsLeftFind == 'T' &&
                   ImageDeal[Ysite + 2].IsLeftFind == 'W' &&
                   abs(ImageDeal[Ysite].LeftBorder - ImageDeal[Ysite + 2].LeftBorder) > 10)
                {
                    flag_Ysite_1 = Ysite;
                    flag_Xsite_1 = ImageDeal[flag_Ysite_1].LeftBorder;
                    ImageStatus.OFFLine = Ysite;
                    Slope_Rings = (float)(79 - flag_Xsite_1) / (float)(59 - flag_Ysite_1);
                    break;
                }
            }
        }
        if(flag_Ysite_1 != 0)
        {
            for(Ysite = flag_Ysite_1; Ysite < 60; Ysite++)
            {
                ImageDeal[Ysite].RightBorder = flag_Xsite_1 + Slope_Rings * (Ysite - flag_Ysite_1);
                ImageDeal[Ysite].Center = (ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) / 2;
                if(ImageDeal[Ysite].Center < 4)
                {
                    ImageDeal[Ysite].Center = 4;
                }
            }
            ImageDeal[flag_Ysite_1].RightBorder = flag_Xsite_1;
            for(Ysite = flag_Ysite_1 - 1; Ysite > 10; Ysite--)
            {
                for(Xsite = ImageDeal[Ysite + 1].RightBorder - 10; Xsite < ImageDeal[Ysite + 1].RightBorder + 2; Xsite++)
                {
                    if(Pixle[Ysite][Xsite] == 1 && Pixle[Ysite][Xsite + 1] == 0)
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
                if(ImageDeal[Ysite].Wide > 8 && ImageDeal[Ysite].RightBorder < ImageDeal[Ysite + 2].RightBorder)
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
        //环内 小环弯道减半宽 大环不减
    if(ImageFlag.image_element_rings_flag == 7)
    {

    }
        //大圆环出环 补线
    if(ImageFlag.image_element_rings_flag == 8 && ImageFlag.ring_big_small == 1)
    {
        Repair_Point_Xsite = 20;
        Repair_Point_Ysite = 0;
        for(Ysite = 40; Ysite > 5; Ysite--)
        {
            if(Pixle[Ysite][28] == 1 && Pixle[Ysite - 1][28] == 0)
            {
                Repair_Point_Xsite = 28;
                Repair_Point_Ysite = Ysite - 1;
                ImageStatus.OFFLine = Ysite + 1;
                break;
            }
        }
        for(Ysite = 57; Ysite > Repair_Point_Ysite - 3; Ysite--)
        {
            ImageDeal[Ysite].RightBorder =
                (ImageDeal[58].RightBorder - Repair_Point_Xsite) * (Ysite - 58) /
                (58 - Repair_Point_Ysite) + ImageDeal[58].RightBorder;
            ImageDeal[Ysite].Center = (ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) / 2;
        }
    }
        //已出环 半宽处理
    if(ImageFlag.image_element_rings_flag == 9 || ImageFlag.image_element_rings_flag == 10)
    {
        for(Ysite = 59; Ysite > ImageStatus.OFFLine; Ysite--)
        {
            ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBorder - Half_Road_Wide[Ysite];
        }
    }
}

static void Element_Handle_Right_Rings(void)
{
    int num = 0;
    int flag_Xsite_1 = 0;
    int flag_Ysite_1 = 0;
    float Slope_Right_Rings = 0.0f;

    /****************判断*****************/
    for(Ysite = 55; Ysite > 30; Ysite--)
    {
        if(ImageDeal[Ysite].IsRightFind == 'W')
        {
            num++;
        }
        if(ImageDeal[Ysite + 3].IsRightFind == 'W' &&
           ImageDeal[Ysite + 2].IsRightFind == 'W' &&
           ImageDeal[Ysite + 1].IsRightFind == 'W' &&
           ImageDeal[Ysite].IsRightFind == 'T')
        {
            break;
        }
    }
        //准备进环
    if(ImageFlag.image_element_rings_flag == 1 && num > 10)
    {
        ImageFlag.image_element_rings_flag = 2;
    }
    if(ImageFlag.image_element_rings_flag == 2 && num < 8)
    {
        ImageFlag.image_element_rings_flag = 5;
        buzzer_short();
    }
        //进环
    if(ImageFlag.image_element_rings_flag == 5 && ImageStatus.Left_Line > 15)
    {
        ImageFlag.image_element_rings_flag = 6;
    }
        //进环小圆环
    if(ImageFlag.image_element_rings_flag == 6 && ImageStatus.Left_Line < 4)
    {
        ImageFlag.image_element_rings_flag = 7;
    }
        //环内 大圆环判断
    if(ImageFlag.image_element_rings_flag == 7)
    {
        Point_Xsite = 0;
        Point_Ysite = 0;
        for(Ysite = 45; Ysite > ImageStatus.OFFLine + 3; Ysite--)
        {
            if(ImageDeal[Ysite].LeftBorder >= ImageDeal[Ysite + 2].LeftBorder &&
               ImageDeal[Ysite].LeftBorder >= ImageDeal[Ysite - 2].LeftBorder &&
               ImageDeal[Ysite].LeftBorder >= ImageDeal[Ysite + 1].LeftBorder &&
               ImageDeal[Ysite].LeftBorder >= ImageDeal[Ysite - 1].LeftBorder &&
               ImageDeal[Ysite].LeftBorder >= ImageDeal[Ysite + 4].LeftBorder &&
               ImageDeal[Ysite].LeftBorder >= ImageDeal[Ysite - 4].LeftBorder)
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
        //出环后
    if(ImageFlag.image_element_rings_flag == 8)
    {
        if(ImageStatus.Left_Line < 9 && ImageStatus.OFFLine < 10)
        {
            ImageFlag.image_element_rings_flag = 9;
        }
    }
        //结束圆环进程
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
    /***************************************处理**************************************/
         //准备进环  半宽处理
    if(ImageFlag.image_element_rings_flag == 1 ||
       ImageFlag.image_element_rings_flag == 2 ||
       ImageFlag.image_element_rings_flag == 3 ||
       ImageFlag.image_element_rings_flag == 4)
    {
        for(Ysite = 59; Ysite > ImageStatus.OFFLine; Ysite--)
        {
            ImageDeal[Ysite].Center = ImageDeal[Ysite].LeftBorder + Half_Road_Wide[Ysite] + 2;
        }
    }
        //进环  补线
    if(ImageFlag.image_element_rings_flag == 5 ||
       ImageFlag.image_element_rings_flag == 6)
    {
        flag_Xsite_1 = 0;
        flag_Ysite_1 = 0;
        for(Ysite = 55; Ysite > ImageStatus.OFFLine; Ysite--)
        {
            for(Xsite = ImageDeal[Ysite].LeftBorder + 1; Xsite < ImageDeal[Ysite].RightBorder - 1; Xsite++)
            {
                if(Pixle[Ysite][Xsite] == 1 && Pixle[Ysite][Xsite + 1] == 0)
                {
                    flag_Ysite_1 = Ysite;
                    flag_Xsite_1 = Xsite;
                    Slope_Right_Rings = (float)(0 - flag_Xsite_1) / (float)(59 - flag_Ysite_1);
                    break;
                }
            }
            if(flag_Ysite_1 != 0)
            {
                break;
            }
        }
        if(flag_Ysite_1 == 0)
        {
            for(Ysite = ImageStatus.OFFLine + 5; Ysite < 30; Ysite++)
            {
                if(ImageDeal[Ysite].IsRightFind == 'T' &&
                   ImageDeal[Ysite + 1].IsRightFind == 'T' &&
                   ImageDeal[Ysite + 2].IsRightFind == 'W' &&
                   abs(ImageDeal[Ysite].RightBorder - ImageDeal[Ysite + 2].RightBorder) > 10)
                {
                    flag_Ysite_1 = Ysite;
                    flag_Xsite_1 = ImageDeal[flag_Ysite_1].RightBorder;
                    ImageStatus.OFFLine = Ysite;
                    Slope_Right_Rings = (float)(0 - flag_Xsite_1) / (float)(59 - flag_Ysite_1);
                    break;
                }
            }
        }
        if(flag_Ysite_1 != 0)
        {
            for(Ysite = flag_Ysite_1; Ysite < 58; Ysite++)
            {
                ImageDeal[Ysite].LeftBorder = flag_Xsite_1 + Slope_Right_Rings * (Ysite - flag_Ysite_1);
                ImageDeal[Ysite].Center = (ImageDeal[Ysite].LeftBorder + ImageDeal[Ysite].RightBorder) / 2;
                if(ImageDeal[Ysite].Center > 79)
                {
                    ImageDeal[Ysite].Center = 79;
                }
            }
            ImageDeal[flag_Ysite_1].LeftBorder = flag_Xsite_1;
            for(Ysite = flag_Ysite_1 - 1; Ysite > 10; Ysite--)
            {
                for(Xsite = ImageDeal[Ysite + 1].LeftBorder + 8; Xsite > ImageDeal[Ysite + 1].LeftBorder - 4; Xsite--)
                {
                    if(Pixle[Ysite][Xsite] == 1 && Pixle[Ysite][Xsite - 1] == 0)
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
                if(ImageDeal[Ysite].Wide > 8 && ImageDeal[Ysite].LeftBorder > ImageDeal[Ysite + 2].LeftBorder)
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
        //环内不处理
    if(ImageFlag.image_element_rings_flag == 7)
    {

    }
        //大圆环出环 补线
    if(ImageFlag.image_element_rings_flag == 8)
    {
        Repair_Point_Xsite = 59;
        Repair_Point_Ysite = 0;
        for(Ysite = 40; Ysite > 5; Ysite--)
        {
            if(Pixle[Ysite][51] == 1 && Pixle[Ysite - 1][51] == 0)
            {
                Repair_Point_Xsite = 51;
                Repair_Point_Ysite = Ysite - 1;
                ImageStatus.OFFLine = Ysite + 1;
                break;
            }
        }
        for(Ysite = 57; Ysite > Repair_Point_Ysite - 3; Ysite--)
        {
            ImageDeal[Ysite].LeftBorder =
                (ImageDeal[58].LeftBorder - Repair_Point_Xsite) * (Ysite - 58) /
                (58 - Repair_Point_Ysite) + ImageDeal[58].LeftBorder;
            ImageDeal[Ysite].Center = (ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) / 2;
        }
    }
        //已出环 半宽处理
    if(ImageFlag.image_element_rings_flag == 9)
    {
        for(Ysite = 59; Ysite > ImageStatus.OFFLine; Ysite--)
        {
            ImageDeal[Ysite].Center = ImageDeal[Ysite].LeftBorder + Half_Road_Wide[Ysite];
        }
    }
}

/****元素检测*****/  //圆环检测
void Element_Test(void)
{
    if(ImageStatus.Road_type != Cross  //赛道类型清0
       &&ImageStatus.Road_type != LeftCirque
       &&ImageStatus.Road_type != RightCirque
       &&ImageStatus.Road_type != Barn_in
       &&ImageStatus.Road_type != Ramp
       &&ImageStatus.Road_type != Cross_ture)
    {
        ImageStatus.Road_type = 0;
    }

    if(ImageStatus.Road_type != Cross
       &&ImageStatus.Road_type != LeftCirque
       &&ImageStatus.Road_type != RightCirque)
        Straightacc_Test();   //直道检测

    if(ImageStatus.Road_type != Barn_in  //圆环检测
       &&ImageStatus.Road_type != Cross_ture
       &&ImageStatus.Road_type != Barn_out)
    {
        Element_Judgment_Left_Rings();   //左圆环检测
        Element_Judgment_Right_Rings();  //右圆环检测
    }
}

void Element_Handle(void)
{
    if(ImageFlag.image_element_rings == 1)
        Element_Handle_Left_Rings();
    else if(ImageFlag.image_element_rings == 2)
        Element_Handle_Right_Rings();
//      Cross_Handle();
}

/* @brief      优化的大津法。
 * @param      image  图像数组。
 * @param      col    宽。
 * @param      row    高。
 * @param      pixel_threshold 阈值分离。
 * @return     uint8
 */
uint8 Threshold_deal(uint8* image,
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
    image_data = image;  /* 指向像素数据的指针。 */
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

    /* 统计灰度级中每个像素在整幅图像中的个数。 */
    for(i = 0; i < height; i++)
    {
        for(j = 0; j < width; j++)
        {
            pixelCount[(int)image_data[i * width + j]]++;  /* 将当前点的像素值作为计数数组下标。 */
            gray_sum += (int)image_data[i * width + j];    /* 灰度值总和。 */
        }
    }

    inv_pixel_sum = 1.0f / (float)pixelSum;

    /* 计算每个像素值在整幅图像中的比例。 */
    for(i = 0; i < 256; i++)
    {
        pixelPro[i] = (float)pixelCount[i] * inv_pixel_sum;
    }

    u = (float)gray_sum * inv_pixel_sum;  /* 全局平均灰度。 */

    /* 遍历灰度级 [0, pixel_threshold) 。 */
    for(j = 0; j < (uint16)pixel_threshold; j++)
    {
        w0 += pixelPro[j];  /* 背景部分每个灰度值的像素点所占比例之和。 */
        if(0.0f == w0)
        {
            continue;
        }

        u0tmp += j * pixelPro[j];  /* 背景部分 每个灰度值的点比例乘灰度值。 */
        w1 = 1.0f - w0;
        if(0.0f == w1)
        {
            break;
        }

        u1tmp = u - u0tmp;
        u0 = u0tmp / w0;    /* 背景平均灰度。 */
        u1 = u1tmp / w1;    /* 前景平均灰度。 */
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

/* 图像二值化。 */
void Get01change_dajin(void)
{
    uint8 i = 0;
    uint8 j = 0;
    uint8 thre = 0;

    OtsuRawThreshold = Threshold_deal(Image_Use[0], LCDW, LCDH, ImageStatus.Threshold_detach);
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

            /* 数值越大，显示的内容越多，较浅的图像也能显示出来。 */
            if(Image_Use[i][j] > (thre))
            {
                Pixle[i][j] = 1;  /* 白。 */
            }
            else
            {
                Pixle[i][j] = 0;  /* 黑。 */
            }
        }
    }
}

/* 图像二值化，沿用现有函数口径，内部改成参考代码的底部参考均值阈值。 */
void Get01change_roi_mix(void)
{
    uint8 i = 0;
    uint8 j = 0;
    uint8 current_threshold;
    uint16 sample_count = 0;
    uint16 threshold_value = 0;
    uint32 gray_sum = 0;

    /* 参考“路边野生”代码，直接取底部参考区均值，再乘固定倍率得到白点门槛。 */
    for(i = (uint8)(LCDH - 8); i < LCDH; i++)
    {
        for(j = 0; j < LCDW; j++)
        {
            gray_sum += Image_Use[i][j];
            sample_count++;
        }
    }

    if(0 != sample_count)
    {
        threshold_value = (uint16)((gray_sum * 11U +
                                    ((uint32)sample_count * 5U)) /
                                   ((uint32)sample_count * 10U));
    }
    if(threshold_value > 255U)
    {
        OtsuRawThreshold = 255U;
    }
    else
    {
        OtsuRawThreshold = (uint8)threshold_value;
    }
    if(threshold_value < ImageStatus.Threshold_static)
    {
        threshold_value = (uint16)ImageStatus.Threshold_static;
    }
    if(threshold_value > 255U)
    {
        threshold_value = 255U;
    }

    current_threshold = (uint8)threshold_value;
    ImageStatus.Threshold = current_threshold;

    for(i = 0; i < LCDH; i++)
    {
        for(j = 0; j < LCDW; j++)
        {
            if(Image_Use[i][j] > current_threshold)
            {
                Pixle[i][j] = 1;  /* 白。 */
            }
            else
            {
                Pixle[i][j] = 0;  /* 黑。 */
            }
        }
    }
}

/* 按 19 国一斑马线口径，扫描中下部多次黑白跳变。 */
static uint8 ZebraScanHit(void)
{
    int row = 0;
    int col = 0;
    int left_limit = 0;
    int right_limit = 0;
    uint8 edge_count = 0;

    if(ImageStatus.Road_type == LeftCirque
       || ImageStatus.Road_type == RightCirque
       || ImageStatus.Road_type == Ramp)
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
            if(Pixle[row][col] == 0 && Pixle[row][col + 1] == 1)
            {
                edge_count++;
                if(edge_count > 4)
                {
                    return 1;
                }
            }
        }
    }

    return 0;
}

/* 第一次斑马线只记数并鸣叫，第二次有效命中直接停车。 */
static void CheckZebraEmergency(void)
{
    uint8 zebra_hit;

    if(STATE_RUN != state_get_mode())
    {
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
    if(zebra_hit)
    {
        ZebraMissFrames = 0;
        if(0 == ZebraFrameLatch)
        {
            ZebraFrameLatch = 1;
            if(0 == ZebraCooldownFrames)
            {
                if(ZebraDetectCount < 2)
                {
                    ZebraDetectCount++;
                }

                buzzer_short();
                if(ZebraDetectCount >= 2)
                {
                    state_set_mode(STATE_STOP);
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

/* 检查底部黑点占比是否持续超阈值，超时后直接停车。 */
static void CheckOutTrackEmergency(void)
{
    uint8 row;
    uint8 col;
    uint16 black_count = 0;

    if(STATE_RUN != state_get_mode())
    {
        OutTrackStopHitCount = 0;
        return;
    }

    /* 统计底部 2 行黑点数。 */
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

    /* 本帧黑点占比超过阈值时，累计连续命中次数。 */
    if(black_count > (uint16)(((uint16)(LCDW * IMAGE_OUTTRACK_SAMPLE_ROWS) * IMAGE_OUTTRACK_BLACK_PERCENT) / 100U))
    {
        if(OutTrackStopHitCount < IMAGE_OUTTRACK_CONFIRM_COUNT)
        {
            OutTrackStopHitCount++;
        }
    }
    else
    {
        /* 中间只要恢复，连续命中计数立即清零。 */
        OutTrackStopHitCount = 0;
    }

    /* 连续命中达到 10 帧后切停车状态。 */
    if(OutTrackStopHitCount >= IMAGE_OUTTRACK_CONFIRM_COUNT)
    {
        state_set_mode(STATE_STOP);
    }
}

// 图像处理
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
    CheckZebraEmergency();  /* 斑马线第一次只记数，第二次命中切紧急状态。 */


    Element_Handle();     //环岛执行
    runtime_tow_point = SearchLine_GetRuntimeTowPoint();
    // SearchLine_ApplyCenterCompensation(runtime_tow_point);  // 中线压缩补偿
    GetDet(runtime_tow_point);             //获取动态前瞻  并且计算图像偏差
    gpio_set_level(IO_P52, 1);
}

/* 图像更新 */
void image_update(void)
{
    if(!image_ready)
    {
        return;
    }

    if(!mt9v03x_finish_flag)
    {
        return;
    }

    ImageProcess();

    if(STATE_STOP == state_get_mode())
    {
        image_result_ready = 0;
        return;
    }

    if((STATE_RUN == state_get_mode()) && (OtsuRawThreshold < IMAGE_STOP_RAW_THRESHOLD))
    {
        image_result_ready = 0;
        state_set_mode(STATE_STOP);
        return;
    }

    image_result_ready = 1;
}
