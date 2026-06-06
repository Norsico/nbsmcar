#include "headfile.h"
#include "math.h"

/*
 * image_process.c
 *
 *  Created on: 2025年2月25日
 *      Author: 15958
 */

/*
 * image_processing.c
 *
 *  Created on: 2025年2月26日
 *      Author: 15958
 */

/*
 * select_median.c / situation.c
 *
 * 当前工程只移植普通图像处理链路；元素处理函数不参与本工程图像结果。
 */

image_data Image;
uint8 ImageGray[IMAGE_H][IMAGE_W];     //用来存储压缩之后灰度图像的二维数组
uint8 Pixle[IMAGE_H][IMAGE_W];          //图像处理时真正处理的二值化图像数组
uint8 ImageBin[IMAGE_H][IMAGE_W];

#define TRUE                         (1)
#define FALSE                        (0)
#define Lstart                       (1)
#define Rstart                       (2)
#define Mstart                       (3)

#define IMAGE_COMPRESS_ROW_SCALE     (1.2f)
#define IMAGE_COMPRESS_COL_SCALE     (2.35f)
#define IMAGE_GRAYSCALE              (256)
#define IMAGE_THRESHOLD_DETACH       (256)
#define IMAGE_H_TO_E                 (0.65f)

#define IMAGE_SCOPE_UP               (0)
#define IMAGE_SCOPE_DOWN             (59)
#define IMAGE_SCOPE_LEFT             (5)
#define IMAGE_SCOPE_RIGHT            (74)
#define IMAGE_BASE_MID               (40)
#define SIZE_HRE                     (IMAGE_H)
#define MID_POINT                    (IMAGE_BASE_MID)
#define HtoE                         (IMAGE_H_TO_E)
#define White                        (IMAGE_WHITE)
#define Black                        (IMAGE_BLACK)
#define Pixle_hb                     ImageBin
typedef struct
{
    //图像X坐标
    uint8 ui8_ImageX;
    //图像Y坐标
    uint8 ui8_ImageY;
    //图像单边最远点
    uint8 ui8_MaxY;
    //图像最远点
    uint8 ui8_AllMaxY;
} LadderMovePoint;

typedef struct
{
    //状态判断计数
    uint16 ui16_counter[5];
    //状态标志
    int8 i8_StatusFlag[9];
    //状态处理变量
    int8 i8_StatusHandle[9];
    //图像处理范围
    uint8 ui8_DisposeScopeUp;
    uint8 ui8_DisposeScopeDown;
    uint8 ui8_DisposeScopeLeft;
    uint8 ui8_DisposeScopeRight;
    //标准直道行位置
    float f_BaseY[10];
    //标准数组
    uint8 *ui8_LineWidth;
    //标准权重
    float f_BaseLineWeight[10];
    //行权重
    float f_LineWeight[10];
    //控制量数组
    int16 *i16p_dataImage;
    //图像数组
    uint8 ui8_ImageArray[IMAGE_H][IMAGE_W];
    //左边界
    int8 ui8_LPoint[IMAGE_H];
    //右边界
    int8 ui8_RPoint[IMAGE_H];
    //扫描行距离
    uint8 ui8_ScanLineY[10];
    //扫描行左边界(补线)
    uint8 ui8_ScanLineL[10];
    //扫描行右边界(补线)
    uint8 ui8_ScanLineR[10];
    //扫描行左边界(最边界)
    uint8 ui8_ScanLineToL[10];
    //扫描行右边界(最边界)
    uint8 ui8_ScanLineToR[10];
    //扫描赛道宽度
    uint8 ui8_ScanLineWidth[10];
    //中值求取起点
    uint8 ui8_ScanDirection;
    //初始中值
    int16 i16_Mid[10];
    //最终中值
    int16 i16_FinallyMid[10];
    //最小可视距离
    uint8 ui8_MinH;
    int8 i8_MinHX;
    //反向可视距离
    uint8 VisitableScope;
    //爬梯最远点
    uint8 MaxPoint;
} Dispose_Image;

static LadderMovePoint L_Move;
static LadderMovePoint R_Move;
static Dispose_Image DI;

static uint8 Vistable_scale;
static uint8 street_len_40;
static uint8 street_len_5;
static uint8 street_len_75;
static uint8 street_len_47;
static uint8 street_len_33;
static uint8 street_len[IMAGE_W];
static uint8 Threshold;                                //通过大津法计算出来的前20行二值化阈值
static uint8 Change_time;
static uint8 L_point;
static uint8 R_point;
static int poserror;
static int poserror_array[4];
static double f_E_H;
static double d_Y;
static uint8 img_x;
static uint8 img_y;
static uint16 hangkuan60[IMAGE_H];
static uint8 ui8_LineWidth[IMAGE_H];

static void InitDisposeImageData(void);
static void Image_Compress(void);
static uint8 Get_Threshold(uint8 *image, uint16 col, uint16 row);
static void Get_BinaryImage(void);
static void Pixle_Filter(void);
static void ConstructImage(void);
static void Ladder(void);
static uint8 LeftPointLadder(uint8 *ui8p_LF);
static uint8 RightPointLadder(uint8 *ui8p_RF);
static void FitRoad(void);
static void Fit(int8 i8_X1, int8 i8_X2, uint8 ui8_Y1, uint8 ui8_Y2);
void Fits(int8 i8_X1, int8 i8_X2, uint8 ui8_Y1, uint8 ui8_Y2);
static void GetReverseVisualRange(void);
static void Get_len(void);
static void MeasureLineWidth(void);
void Seek_point(uint8 Y);
static uint8 Seek_Write_point(uint8 x1, uint8 Y);
static uint8 Seek_Black_point(uint8 x1, uint8 Y);
static void DetermineScanLine(void);
static void NormalTreatment(void);
static void SelectMid(void);
static void DetermineMid(void);
static void NormalControl(void);
static void DisposeImage(void);
static void Image_Process(void);

/*
uint8 ui8_LineWidthWaiGua[60] =
{
0,15,16,17,17,17,18,18,19,20,20,20,
22,22,23,24,24,25,25,26,27,27,28,28,
29,30,30,31,31,32,33,33,34,35,35,36,
37,37,38,38,39,39,40,41,41,42,43,43,
43,45,45,45,47,47,47,49,49,49,50,51,

};
*/

//59:67     54:61   49:57     44:52      39:47      34:40      29:36
//24:31     19:26   14:20     09:17     04:13
//59:40     54:36   49:34     44:32      39:27      34:24      29:22
//24:19     19:17   14:13     09:11     04:11
/*uint8 ui8_LineWidthWaiGua[60] =
{
        11,11,11,11,11,11,11,11,11,11,
        12,12,13,13,13,14,15,16,16,17,
        17,18,18,19,19,20,21,21,22,22,
        23,23,24,24,24,25,25,26,27,27,
        28,29,30,31,32,32,33,33,34,34,
        35,35,36,36,36,37,38,39,40,40,

};*/
/*uint8 ui8_LineWidthWaiGua[60] =
{
        13,13,13,13,13,14,15,16,16,17,
        18,18,19,20,20,21,22,23,24,26,
        27,28,29,30,31,32,33,34,35,36,
        37,38,39,40,40,41,42,44,46,47,
        48,49,50,51,52,53,54,55,56,57,
        58,59,60,61,61,62,63,64,65,67,

};*/
//80*60标定
static uint8 ui8_LineWidthWaiGua[IMAGE_H] =
{
    13,13,13,13,13,14,15,16,16,17,
    18,18,19,20,20,20,20,20,20,20,
    21,22,23,24,25,26,27,28,29,30,
    31,32,33,34,35,36,37,38,39,40,
    38,39,40,41,42,42,43,43,43,44,
    45,45,46,47,47,47,48,48,49,50
};
/*
uint8 ui8_LineWidthWaiGua[60] =
{
        13,13,13,13,13,14,15,16,16,17,
        18,18,19,20,20,20,20,20,20,20,
        21,22,23,24,25,26,27,28,29,30,
        31,32,33,34,35,36,37,38,39,40,
        55,55,55,55,55,55,55,55,55,55,
        55,55,55,55,55,55,55,55,55,55


};
*/

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

static uint8 image_limit_row(int16 row)
{
    if(row < 0)
    {
        return 0;
    }
    if(row >= IMAGE_H)
    {
        return IMAGE_H - 1;
    }
    return (uint8)row;
}

/***图像初始化***/
static void InitDisposeImageData(void)
{
    DI.ui8_DisposeScopeUp = IMAGE_SCOPE_UP;
    DI.ui8_DisposeScopeDown = IMAGE_SCOPE_DOWN;
    DI.ui8_DisposeScopeLeft = IMAGE_SCOPE_LEFT;
    DI.ui8_DisposeScopeRight = IMAGE_SCOPE_RIGHT;

    /*直道行距，十条动态横线的标准行距，明天测一下，用DI.ScanLineY[]测试*/
    DI.f_BaseY[0] = 59.0f;
    DI.f_BaseY[1] = 48.0f;
    DI.f_BaseY[2] = 38.0f;
    DI.f_BaseY[3] = 30.0f;
    DI.f_BaseY[4] = 22.0f;
    DI.f_BaseY[5] = 16.0f;
    DI.f_BaseY[6] = 11.0f;
    DI.f_BaseY[7] = 8.0f;
    DI.f_BaseY[8] = 5.0f;
    DI.f_BaseY[9] = 4.0f;

 // DI.i16p_dataImage         =       i16p_GetData();
  DI.i16_Mid[0]             =            MID_POINT;     //初始中值
  DI.i16_Mid[1]             =            MID_POINT;
  DI.i16_Mid[2]             =            MID_POINT;
  DI.i16_Mid[3]             =            MID_POINT;
  DI.i16_Mid[4]             =            MID_POINT;
  DI.i16_FinallyMid[0]      =            MID_POINT;     //最终中值
  DI.i16_FinallyMid[1]      =            MID_POINT;
  DI.i16_FinallyMid[2]      =            MID_POINT;
  DI.i16_FinallyMid[3]      =            MID_POINT;
  DI.i16_FinallyMid[4]      =            MID_POINT;
  //DI.i16p_dataImage[State]  =                FALSE;
  DI.ui8_LPoint[DI.ui8_DisposeScopeDown] =      0;
  DI.ui8_RPoint[DI.ui8_DisposeScopeDown] =      IMAGE_W - 1;
  //权重设置

  DI.f_BaseLineWeight[0] = 0;     //标准权重，基础权重,第零行是最下面一行
  DI.f_BaseLineWeight[1] = 0;//1
  DI.f_BaseLineWeight[2] = 2;//1
  DI.f_BaseLineWeight[3] = 1;//2
  DI.f_BaseLineWeight[4] = 1;//3    1
  DI.f_BaseLineWeight[5] = 0;//3    1
  DI.f_BaseLineWeight[6] = 0;//2    2
  DI.f_BaseLineWeight[7] = 0;//1    3
  DI.f_BaseLineWeight[8] = 0;//0    3
  DI.f_BaseLineWeight[9] = 0;
  DI.ui8_LineWidth = ui8_LineWidthWaiGua;  //标准数组
  DI.f_LineWeight[0] = DI.f_BaseLineWeight[0];
}

static void image_export_result(void)
{
    Image.center = (int16)(poserror + MID_POINT);
    Image.error = (int16)poserror;
    Image.valid_count = (uint8)((DI.ui8_MinH < IMAGE_H) ? (IMAGE_H - DI.ui8_MinH) : 0);
    Image.result_ready = 1;
    Image.ring = 0;
    Image.ring_step = 0;
    Image.zebra = 0;
    Image.zebra_count = 0;
}

static uint16 image_debug_x(uint16 x, uint16 w, int16 col)
{
    col = image_limit_col(col);
    return (uint16)(x + (((uint16)col * w) + (IMAGE_W / 2)) / IMAGE_W);
}

static uint16 image_debug_y(uint16 y, uint16 h, int16 row)
{
    row = image_limit_row(row);
    return (uint16)(y + (((uint16)row * h) + (IMAGE_H / 2)) / IMAGE_H);
}

void image_show_debug_overlay(uint16 x, uint16 y, uint16 w, uint16 h)
{
    uint8 row;
    uint8 i;
    uint16 draw_x;
    uint16 draw_y;
    uint16 tow_y;

    if((Image.ready == 0) || (Image.sequence == 0))
    {
        return;
    }

    MeasureLineWidth();
    for(row = 0; row < IMAGE_H; row++)
    {
        draw_y = image_debug_y(y, h, row);
        draw_x = image_debug_x(x, w, DI.ui8_LPoint[row]);
        ips200_draw_point(draw_x, draw_y, RGB565_GREEN);
        draw_x = image_debug_x(x, w, DI.ui8_RPoint[row]);
        ips200_draw_point(draw_x, draw_y, RGB565_GREEN);
    }

    for(i = 0; i < 10; i++)
    {
        draw_y = image_debug_y(y, h, DI.ui8_ScanLineY[i]);
        draw_x = image_debug_x(x, w, DI.i16_Mid[i]);
        ips200_draw_point(draw_x, draw_y, RGB565_RED);
    }

    tow_y = image_debug_y(y, h, Image.tow_row);
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
    Image.result_ready = 0;
    Image.sequence = 0;
    Image.threshold = 0;
    Image.white_count = 0;
    Image.tow_row = SERVO_POINT;
    Image.center = MID_POINT;
    Image.error = 0;
    Image.valid_count = 0;
    Image.lost = 1;
    Image.ring = 0;
    Image.ring_step = 0;
    Image.zebra = 0;
    Image.zebra_count = 0;
    InitDisposeImageData();

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

//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//  @name           Image_Compress
//  @brief          原始灰度图像压缩处理
//  @brief          作用就是将原始尺寸的灰度图像压缩成你所需要的大小，这里我是把原始80行170列的灰度图像压缩成60行80列的灰度图像。
//  @brief          为什么要压缩？因为我不需要那么多的信息，60*80图像所展示的信息原则上已经足够完成比赛任务了，当然你可以根据自己的理解修改。
//  @parameter      void
//  @return         void
//  @time           2022年1月18日
//  @Author         陈海涛
//  Sample usage:   Image_Compress();
//---------------------------------------------------------------------------------------------------------------------------------------------------------------
static void Image_Compress(void)
{
    int XSITE;
    int YSITE;

    for(YSITE = 0; YSITE < IMAGE_H; YSITE++)
    {
        for(XSITE = 0; XSITE < IMAGE_W; XSITE++)
        {
           // Image_Use[YSITE][XSITE] = mt9v03x_image[(int)(1.7*YSITE)][(int)(1.0*XSITE * 2.35)];
            Image_Use[YSITE][XSITE] = mt9v03x_image[(int)(1.0f * YSITE * IMAGE_COMPRESS_ROW_SCALE)][(int)(1.0f * XSITE * IMAGE_COMPRESS_COL_SCALE)];
        }
    }
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//  @name           Get_Threshold
//  @brief          优化之后的的大津法。大津法就是一种能够算出一幅图像最佳的那个分割阈值的一种算法。
//  @brief          这个东西你们可以如果实在不能理解就直接拿来用，什么参数都不用修改，只要没有光照影响，那么算出来的这个阈值就一定可以得到一幅效果还不错的二值化图像。
//  @parameter      image  原始的灰度图像数组
//  @parameter      clo    图像的宽（图像的列）
//  @parameter      row    图像的高（图像的行）
//  @return         uint8
//  @time           2022年1月17日
//  @Author         陈海涛
//  Sample usage:   Threshold = Threshold_deal(Image_Use[0], 80, 60); 把存放60行80列的二维图像数组Image_Use传进来，求出这幅图像的阈值，并将这个阈值赋给Threshold。
//---------------------------------------------------------------------------------------------------------------------------------------------------------------
static uint8 Get_Threshold(uint8 *image, uint16 col, uint16 row)
{
    int width;
    int height;
    int pixelCount[IMAGE_GRAYSCALE];
    float pixelPro[IMAGE_GRAYSCALE];
    int i;
    int j;
    int pixel_sum;
    uint8 threshold;
    int threshold_j;
    uint8 *gray_ptr;
    uint32 gray_sum;
    float w0;
    float w1;
    float u0tmp;
    float u1tmp;
    float u0;
    float u1;
    float u;
    float delta_tmp;
    float delta_max;

    width = (int)col;
    height = (int)row;
    pixel_sum = width * height;
    threshold = 0;
    gray_ptr = image;

    for(i = 0; i < IMAGE_GRAYSCALE; i++)
    {
        pixelCount[i] = 0;
    }

    gray_sum = 0;
    /**************************************统计每个灰度值(0-255)在整幅图像中出现的次数**************************************/
    for(i = 0; i < height; i++)
    {
        for(j = 0; j < width; j++)
        {
            pixelCount[(int)gray_ptr[i * width + j]]++;       //将当前的像素点的像素值（灰度值）作为计数数组的下标。
            gray_sum += (int)gray_ptr[i * width + j];         //计算整幅灰度图像的灰度值总和。
        }
    }
    /**************************************统计每个灰度值(0-255)在整幅图像中出现的次数**************************************/

    /**************************************计算每个像素值（灰度值）在整幅灰度图像中所占的比例*************************************************/
    for(i = 0; i < IMAGE_GRAYSCALE; i++)
    {
        pixelPro[i] = (float)pixelCount[i] / pixel_sum;
    }
    /**************************************计算每个像素值（灰度值）在整幅灰度图像中所占的比例**************************************************/

    /**************************************开始遍历整幅图像的灰度值（0-255），这一步也是大津法最难理解的一步***************************/
    /*******************为什么说他难理解？因为我也是不理解！！反正好像就是一个数学问题，你可以理解为数学公式。***************************/
    w0 = 0.0f;
    u0tmp = 0.0f;
    delta_max = 0.0f;
    for(threshold_j = 0; threshold_j < IMAGE_THRESHOLD_DETACH; threshold_j++)
    {
        w0 += pixelPro[threshold_j];                          //求出背景部分每个灰度值的像素点所占的比例之和，即背景部分的比例。
        u0tmp += threshold_j * pixelPro[threshold_j];
        w1 = 1 - w0;
        u1tmp = gray_sum / pixel_sum - u0tmp;
        u0 = u0tmp / w0;                            //背景平均灰度
        u1 = u1tmp / w1;                            //前景平均灰度
        u = u0tmp + u1tmp;                          //全局平均灰度
        delta_tmp = w0 * pow((u0 - u), 2) + w1 * pow((u1 - u), 2);
        if(delta_tmp > delta_max)
        {
            delta_max = delta_tmp;
            threshold = (uint8)threshold_j;
        }
        if(delta_tmp < delta_max)
        {
            break;
        }
    }

    return threshold;                             //把上面这么多行代码算出来的阈值给return出去。
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//  @name           Get_BinaryImage
//  @brief          灰度图像二值化处理
//  @brief          整体思路就是：先调用Get_Threshold（）函数得到阈值，然后遍历原始灰度图像的每一个像素点，用每一个像素点的灰度值来跟阈值计较。
//  @brief          大于阈值的你就把它那个像素点的值赋值为1（记为白点），否则就赋值为0（记为黑点）。当然你可以把这个赋值反过来，只要你自己清楚1和0谁代表黑谁代表白就行。
//  @brief          所以我前面提到的60*80现在你们就应该明白是什么意思了吧！就是像素点嘛，一行有80个像素点，一共60行，也就是压缩后的每一幅图像有4800个像素点。
//  @parameter      void
//  @return         void
//  @time           2022年3月21日
//  @Author
//  Sample usage:   Get_BinaryImage();
//---------------------------------------------------------------------------------------------------------------------------------------------------------------
static void Get_BinaryImage(void)
{
    uint8 i;
    uint8 j;

    Threshold = Get_Threshold(Image_Use[0], IMAGE_W, IMAGE_H);      //这里是一个函数调用，通过该函数可以计算出一个效果很不错的二值化阈值。
    Image.threshold = Threshold;
    Image.white_count = 0;

    for(i = 0; i < IMAGE_H; i++)                                //遍历二维数组的每一行
    {
        for(j = 0; j < IMAGE_W; j++)                            //遍历二维数组的每一列
        {
            if(Image_Use[i][j] > Threshold)                      //如果这个点的灰度值大于阈值Threshold
            {
                Pixle[i][j] = 1;                                  //那么这个像素点就记为白点
                DI.ui8_ImageArray[i][j] = 1;
                Pixle_hb[i][j] = 255;
                Image.white_count++;
            }
            else                                                 //如果这个点的灰度值小于阈值Threshold
            {
                Pixle[i][j] = 0;                                  //那么这个像素点就记为黑点
                DI.ui8_ImageArray[i][j] = 0;
                Pixle_hb[i][j] = 0;
            }
        }
    }
}

//像素滤波
static void Pixle_Filter(void)
{
    int nr;  //行
    int nc;  //列

    for(nr = 5; nr < 55; nr++)
    {
        for(nc = 14; nc < 79; nc++)
        {
            if((Pixle[nr][nc] == 0) &&
               ((Pixle[nr - 1][nc] + Pixle[nr + 1][nc] +
                 Pixle[nr][nc + 1] + Pixle[nr][nc - 1]) >= 3))
            {
                Pixle[nr][nc] = 1;
                DI.ui8_ImageArray[nr][nc] = 1;
                Pixle_hb[nr][nc] = 255;
            }
        }
    }
}

/************************************************************************
函数名：左点爬梯
功能：左点记录边界
返回值：能否继续移动
************************************************************************/
static uint8 LeftPointLadder(uint8 *ui8p_LF)
{
    /*爬梯第一步：移动到的点为黑色*/
    if(!DI.ui8_ImageArray[L_Move.ui8_ImageY][L_Move.ui8_ImageX])                                //黑点进入白区，当前满足条件还是黑点
    {
        while((L_Move.ui8_ImageX < DI.ui8_DisposeScopeRight) &&
              (!DI.ui8_ImageArray[L_Move.ui8_ImageY][L_Move.ui8_ImageX + 1]))                    //X点小于右边界，并有往右一个点还是黑色，这个while是为了找黑白跳变点
        {
            L_Move.ui8_ImageX++;                                                                //标定右移一位
        }
        if(L_Move.ui8_ImageX < DI.ui8_DisposeScopeRight)
        {
            L_Move.ui8_ImageX++; //标记白点的X坐标，再向外，确实移动到白点
            return TRUE;
        }
        return FALSE;
    }
    /*移动到的点为白色*/
    else if(DI.ui8_ImageArray[L_Move.ui8_ImageY - 1][L_Move.ui8_ImageX])                         //左边减一行，第一个点就是白点，向上找。白区内向上
    {
        if((L_Move.ui8_ImageY == DI.ui8_DisposeScopeDown) &&                                     //最低点
           ((DI.ui8_LPoint[L_Move.ui8_ImageY] - DI.ui8_DisposeScopeLeft) > (MID_POINT >> 2)) && //右移两位相当于除以4，即大于10
           ((DI.ui8_LPoint[L_Move.ui8_ImageY] - L_Move.ui8_ImageX) > (MID_POINT >> 2)))
        {
            L_Move.ui8_ImageX = (uint8)((DI.ui8_LPoint[L_Move.ui8_ImageY] + L_Move.ui8_ImageX) / 2); //左下角出现噪点，根据上次左点比较跳变，作为继续爬梯的初始点
        }
        else
        {
            if(!ui8p_LF[L_Move.ui8_ImageY])  //这一行左边界没确定，确定为左边界
            {
                DI.ui8_LPoint[L_Move.ui8_ImageY] = (int8)L_Move.ui8_ImageX;//找到并记录！！！
                L_Move.ui8_AllMaxY = L_Move.ui8_ImageY; //记录为最远
                ui8p_LF[L_Move.ui8_ImageY] = 1;
            }
            L_Move.ui8_ImageY--;   //处理下一行
        }
        return TRUE;
    }
    /*遇到白色噪点回爬*/
    else if((L_Move.ui8_ImageY < DI.ui8_DisposeScopeDown) &&                                     //如果处理的上一行为白，向上找不到白点，且向右找不到白点，则返回找
            DI.ui8_ImageArray[L_Move.ui8_ImageY + 1][L_Move.ui8_ImageX])
    {
        while((L_Move.ui8_ImageY < DI.ui8_DisposeScopeDown) &&
              DI.ui8_ImageArray[L_Move.ui8_ImageY + 1][L_Move.ui8_ImageX] &&
              (!DI.ui8_ImageArray[L_Move.ui8_ImageY + 1][L_Move.ui8_ImageX + 1]))                 //向下是白，向下向右为黑
        {
            L_Move.ui8_ImageY++;                                                                 //返回上一回，说明遇到噪点，回到上一行继续向右
        }
        if(L_Move.ui8_ImageY < DI.ui8_DisposeScopeDown)
        {
            L_Move.ui8_ImageY++;
            L_Move.ui8_ImageX++;
            return TRUE;
        }
        return FALSE;
    }
    /*回爬后向右找到白色*/
    else if(DI.ui8_ImageArray[L_Move.ui8_ImageY][L_Move.ui8_ImageX + 1])                          //向上是黑则向右
    {
        L_Move.ui8_ImageX++;
        return TRUE;
    }

    return FALSE;
}

/************************************************************************
函数名：右点爬梯
功能：右点记录边界
返回值：能否继续移动
************************************************************************/
static uint8 RightPointLadder(uint8 *ui8p_RF)
{
    /*爬梯第一步：移动到的点为黑色*/
    if(!DI.ui8_ImageArray[R_Move.ui8_ImageY][R_Move.ui8_ImageX])
    {
        while((R_Move.ui8_ImageX > DI.ui8_DisposeScopeLeft) &&
              (!DI.ui8_ImageArray[R_Move.ui8_ImageY][R_Move.ui8_ImageX - 1]))                    //X点大于左边界，并有往左一个点还是黑色，这个while是为了找黑白跳变点
        {
            R_Move.ui8_ImageX--;                                                                //标定左移一位
        }
        if(R_Move.ui8_ImageX > DI.ui8_DisposeScopeLeft)
        {
            R_Move.ui8_ImageX--;                                                                //标记白点的X坐标，再向外，确实移动到白点
            return TRUE;
        }
        return FALSE;
    }
    /*移动到的点为白色*/
    else if(DI.ui8_ImageArray[R_Move.ui8_ImageY - 1][R_Move.ui8_ImageX])                         //上一行同列是白点，白区内向上
    {
        if((R_Move.ui8_ImageY == DI.ui8_DisposeScopeDown) &&                                     //最低点
           ((DI.ui8_DisposeScopeRight - DI.ui8_RPoint[R_Move.ui8_ImageY]) > (MID_POINT >> 2)) &&
           ((R_Move.ui8_ImageX - DI.ui8_RPoint[R_Move.ui8_ImageY]) > (MID_POINT >> 2)))
        {
            R_Move.ui8_ImageX = (uint8)((DI.ui8_RPoint[R_Move.ui8_ImageY] + R_Move.ui8_ImageX) / 2); //右下角出现噪点，根据上次右点比较跳变，作为继续爬梯的初始点
        }
        else
        {
            if(!ui8p_RF[R_Move.ui8_ImageY])                                                       //这一行右边界没确定，确定为右边界
            {
                DI.ui8_RPoint[R_Move.ui8_ImageY] = (int8)R_Move.ui8_ImageX;
                R_Move.ui8_AllMaxY = R_Move.ui8_ImageY;                                           //记录为最远
                ui8p_RF[R_Move.ui8_ImageY] = 1;
            }
            R_Move.ui8_ImageY--;                                                                  //处理下一行
        }
        return TRUE;
    }
    /*回爬后向左找到白色*/
    else if(DI.ui8_ImageArray[R_Move.ui8_ImageY][R_Move.ui8_ImageX - 1])
    {
        R_Move.ui8_ImageX--;
        return TRUE;
    }
    /*遇到白色噪点回爬*/
    else if((R_Move.ui8_ImageY < DI.ui8_DisposeScopeDown) &&
            DI.ui8_ImageArray[R_Move.ui8_ImageY + 1][R_Move.ui8_ImageX])
    {
        while((R_Move.ui8_ImageY < DI.ui8_DisposeScopeDown) &&
              DI.ui8_ImageArray[R_Move.ui8_ImageY + 1][R_Move.ui8_ImageX] &&
              (!DI.ui8_ImageArray[R_Move.ui8_ImageY + 1][R_Move.ui8_ImageX - 1]))                 //向下是白，向下向左为黑
        {
            R_Move.ui8_ImageY++;                                                                 //返回上一回，说明遇到噪点，回到上一行继续向左
        }
        if(R_Move.ui8_ImageY < DI.ui8_DisposeScopeDown)
        {
            R_Move.ui8_ImageY++;
            R_Move.ui8_ImageX--;
            return TRUE;
        }
        return FALSE;
    }

    return FALSE;
}

static void Ladder(void)
{
    uint8 ui8_LF[IMAGE_H];
    uint8 ui8_RF[IMAGE_H];
    uint8 i;

    for(i = 0; i < IMAGE_H; i++)
    {
        ui8_LF[i] = 0;     //图像有没有计算过
        ui8_RF[i] = 0;
    }

    L_Move.ui8_ImageX = DI.ui8_DisposeScopeLeft;
    L_Move.ui8_ImageY = DI.ui8_DisposeScopeDown;
    L_Move.ui8_MaxY = DI.ui8_DisposeScopeDown;
    L_Move.ui8_AllMaxY = DI.ui8_DisposeScopeDown;

    R_Move.ui8_ImageX = DI.ui8_DisposeScopeRight;
    R_Move.ui8_ImageY = DI.ui8_DisposeScopeDown;
    R_Move.ui8_MaxY = DI.ui8_DisposeScopeDown;
    R_Move.ui8_AllMaxY = DI.ui8_DisposeScopeDown;

    //左点爬梯
    while(LeftPointLadder(ui8_LF) &&
          (L_Move.ui8_ImageY > DI.ui8_DisposeScopeUp) &&
          (L_Move.ui8_ImageX < DI.ui8_DisposeScopeRight))
    {
        if((L_Move.ui8_ImageX < MID_POINT) && (L_Move.ui8_MaxY > L_Move.ui8_ImageY))
        {
            L_Move.ui8_MaxY = L_Move.ui8_ImageY;
        }
    }

    //右点爬梯
    while(RightPointLadder(ui8_RF) &&
          (R_Move.ui8_ImageY > DI.ui8_DisposeScopeUp) &&
          (R_Move.ui8_ImageX > DI.ui8_DisposeScopeLeft))
    {
        if((R_Move.ui8_ImageX > MID_POINT) && (R_Move.ui8_MaxY > R_Move.ui8_ImageY))
        {
            R_Move.ui8_MaxY = R_Move.ui8_ImageY;
        }
    }

    //中值求取起点
    if(L_Move.ui8_MaxY < R_Move.ui8_MaxY)   //左小于右 左弯道
    {
        DI.ui8_ScanDirection = Lstart;     //左右爬梯的距离比较得出的最远方向
    }
    else if(L_Move.ui8_MaxY > R_Move.ui8_MaxY)
    {
        DI.ui8_ScanDirection = Rstart;
    }
    else
    {
        DI.ui8_ScanDirection = Mstart;
    }
}

/************************************************************************
函数名：拟合
功能：拟合赛道调整函数
************************************************************************/
static void Fit(int8 i8_X1, int8 i8_X2, uint8 ui8_Y1, uint8 ui8_Y2)
{
    float f_X;
    float f_Dx;
    uint8 ui8_Y;

    f_X = 0;
    if(i8_X1 < i8_X2)
    {
        f_Dx = (i8_X2 - i8_X1) / ((ui8_Y1 - ui8_Y2 - 1) * 1.0f);
        for(ui8_Y = (uint8)(ui8_Y1 - 1); ui8_Y > ui8_Y2; ui8_Y--)
        {
            f_X += f_Dx;
            DI.ui8_LPoint[ui8_Y] = (int)(DI.ui8_LPoint[ui8_Y1] + f_X);
        }
    }
    else
    {
        f_Dx = (i8_X1 - i8_X2) / ((ui8_Y1 - ui8_Y2 - 1) * 1.0f);
        for(ui8_Y = (uint8)(ui8_Y1 - 1); ui8_Y > ui8_Y2; ui8_Y--)
        {
            f_X += f_Dx;
            DI.ui8_RPoint[ui8_Y] = (int)(DI.ui8_RPoint[ui8_Y1] - f_X);
        }
    }
}

/************************************************************************
函数名：变换
功能：将白点变为黑点
************************************************************************/
void Fits(int8 i8_X1, int8 i8_X2, uint8 ui8_Y1, uint8 ui8_Y2)
{
    float f_X;
    float f_Dx;
    uint8 ui8_Y;
    uint8 X;

    f_X = 0;
    if(i8_X1 < i8_X2)
    {
        f_Dx = (i8_X2 - i8_X1) / ((ui8_Y1 - ui8_Y2 - 1) * 1.0f);
        for(ui8_Y = (uint8)(ui8_Y1 - 1); ui8_Y > ui8_Y2; ui8_Y--)
        {
            f_X += f_Dx;
            DI.ui8_LPoint[ui8_Y] = (int)(DI.ui8_LPoint[ui8_Y1] + f_X);
            for(X = (uint8)DI.ui8_LPoint[ui8_Y]; X > 0; X--)
            {
                DI.ui8_ImageArray[ui8_Y][X] = Black;
            }
        }
    }
    else
    {
        f_Dx = (i8_X1 - i8_X2) / ((ui8_Y1 - ui8_Y2 - 1) * 1.0f);
        for(ui8_Y = (uint8)(ui8_Y1 - 1); ui8_Y > ui8_Y2; ui8_Y--)
        {
            f_X += f_Dx;
            DI.ui8_RPoint[ui8_Y] = (int)(DI.ui8_RPoint[ui8_Y1] - f_X);
            for(X = (uint8)DI.ui8_RPoint[ui8_Y]; X < 79; X++)
            {
                DI.ui8_ImageArray[ui8_Y][X] = Black;
            }
        }
    }
}

/************************************************************************
函数名：拟合赛道
功能：调整出可行进路线   //注意：编译要用有符号char类型
************************************************************************/
static void FitRoad(void)
{
    /*横向边界************************************************************/
    int8 i8_UpLpX;
    uint8 ui8_UpLpY;
    int8 i8_EndLpX;
    uint8 ui8_EndLpY;
    int8 i8_UpRpX;
    uint8 ui8_UpRpY;
    int8 i8_EndRpX;
    uint8 ui8_EndRpY;
    uint8 ui8_Y;

    i8_UpLpX = -1;
    ui8_UpLpY = 0;
    i8_EndLpX = -1;
    ui8_EndLpY = 0;
    i8_UpRpX = -1;
    ui8_UpRpY = 0;
    i8_EndRpX = -1;
    ui8_EndRpY = 0;

    if(DI.ui8_ScanDirection == Rstart)
    {
        //右拐补左线
        for(ui8_Y = (uint8)(DI.ui8_DisposeScopeDown - 1); ui8_Y > L_Move.ui8_AllMaxY; ui8_Y--)
        {
            if((i8_UpLpX == -1) && (DI.ui8_LPoint[ui8_Y] <= DI.ui8_LPoint[ui8_Y + 1]))
            {
                ui8_UpLpY = ui8_Y + 1;
                i8_UpLpX = DI.ui8_LPoint[ui8_UpLpY];
            }
            else if((i8_UpLpX != -1) && (i8_EndLpX == -1) &&
                    (DI.ui8_LPoint[ui8_Y] > DI.ui8_LPoint[ui8_Y + 1]))
            {
                ui8_EndLpY = ui8_Y;
                i8_EndLpX = DI.ui8_LPoint[ui8_EndLpY];
            }
            else if((i8_UpLpX != -1) && (i8_EndLpX != -1))
            {
                //找到两个拐点连起来，继续寻找拐点
                Fit(i8_UpLpX, i8_EndLpX, ui8_UpLpY, ui8_EndLpY);
                i8_UpLpX = -1;
                i8_EndLpX = -1;
            }
        }
    }
    else if(DI.ui8_ScanDirection == Lstart)
    {
        //左拐补右线
        for(ui8_Y = (uint8)(DI.ui8_DisposeScopeDown - 1); ui8_Y > R_Move.ui8_AllMaxY; ui8_Y--)
        {
            if((i8_UpRpX == -1) && (DI.ui8_RPoint[ui8_Y] >= DI.ui8_RPoint[ui8_Y + 1]))
            {
                ui8_UpRpY = ui8_Y + 1;
                i8_UpRpX = DI.ui8_RPoint[ui8_UpRpY];
            }
            else if((i8_UpRpX != -1) && (i8_EndRpX == -1) &&
                    (DI.ui8_RPoint[ui8_Y] < DI.ui8_RPoint[ui8_Y + 1]))
            {
                ui8_EndRpY = ui8_Y;
                i8_EndRpX = DI.ui8_RPoint[ui8_EndRpY];
            }
            else if((i8_UpRpX != -1) && (i8_EndRpX != -1))
            {
                //
                Fit(i8_UpRpX, i8_EndRpX, ui8_UpRpY, ui8_EndRpY);
                i8_UpRpX = -1;
                i8_EndRpX = -1;
            }
        }
    }
    else
    {
        for(ui8_Y = (uint8)(DI.ui8_DisposeScopeDown - 1); ui8_Y > L_Move.ui8_AllMaxY; ui8_Y--)
        {
            if((i8_UpLpX == -1) && (DI.ui8_LPoint[ui8_Y] <= DI.ui8_LPoint[ui8_Y + 1]))
            {
                ui8_UpLpY = ui8_Y + 1;
                i8_UpLpX = DI.ui8_LPoint[ui8_UpLpY];
            }
            else if((i8_UpLpX != -1) && (i8_EndLpX == -1) &&
                    (DI.ui8_LPoint[ui8_Y] > DI.ui8_LPoint[ui8_Y + 1]))
            {
                ui8_EndLpY = ui8_Y;
                i8_EndLpX = DI.ui8_LPoint[ui8_EndLpY];
            }
            else if((i8_UpLpX != -1) && (i8_EndLpX != -1))
            {
                //
                Fit(i8_UpLpX, i8_EndLpX, ui8_UpLpY, ui8_EndLpY);
                i8_UpLpX = -1;
                i8_EndLpX = -1;
            }
        }

        for(ui8_Y = (uint8)(DI.ui8_DisposeScopeDown - 1); ui8_Y > R_Move.ui8_AllMaxY; ui8_Y--)
        {
            if((i8_UpRpX == -1) && (DI.ui8_RPoint[ui8_Y] >= DI.ui8_RPoint[ui8_Y + 1]))
            {
                ui8_UpRpY = ui8_Y + 1;
                i8_UpRpX = DI.ui8_RPoint[ui8_UpRpY];
            }
            else if((i8_UpRpX != -1) && (i8_EndRpX == -1) &&
                    (DI.ui8_RPoint[ui8_Y] < DI.ui8_RPoint[ui8_Y + 1]))
            {
                ui8_EndRpY = ui8_Y;
                i8_EndRpX = DI.ui8_RPoint[ui8_EndRpY];
            }
            else if((i8_UpRpX != -1) && (i8_EndRpX != -1))
            {
                Fit(i8_UpRpX, i8_EndRpX, ui8_UpRpY, ui8_EndRpY);
                i8_UpRpX = -1;
                i8_EndRpX = -1;
            }
        }
    }
}

/************************************************************************
函数名：Get_len()
功能：获取可视距离和圆环补线点
返回值：len
***********************************************************************/
static void Get_len(void)
{
    for(img_y = 58; img_y > 1; img_y--)
    {
        if((DI.ui8_ImageArray[img_y][40] == IMAGE_BLACK) &&
           (DI.ui8_ImageArray[img_y - 1][40] == IMAGE_BLACK))
        {
            break;
        }
    }
    street_len_40 = 59 - img_y; //道路中len_40的白色的长度，也叫可视距离

    for(img_y = 50; img_y > 1; img_y--)
    {
        if((DI.ui8_ImageArray[img_y][5] == IMAGE_BLACK) &&
           (DI.ui8_ImageArray[img_y - 1][5] == IMAGE_BLACK))
        {
            break;
        }
    }
    street_len_5 = 59 - img_y; //道路中len_5的白色长度

    for(img_y = 50; img_y > 1; img_y--)
    {
        if((DI.ui8_ImageArray[img_y][75] == IMAGE_BLACK) &&
           (DI.ui8_ImageArray[img_y - 1][75] == IMAGE_BLACK))
        {
            break;
        }
    }
    street_len_75 = 59 - img_y;  //道路中len_75的白色长度

    for(img_y = 50; img_y > 1; img_y--)
    {
        if((DI.ui8_ImageArray[img_y][47] == IMAGE_BLACK) &&
           (DI.ui8_ImageArray[img_y - 1][47] == IMAGE_BLACK))
        {
            break;
        }
    }
    street_len_47 = 59 - img_y;  //道路中len_75的白色长度

    for(img_y = 50; img_y > 1; img_y--)
    {
        if((DI.ui8_ImageArray[img_y][33] == IMAGE_BLACK) &&
           (DI.ui8_ImageArray[img_y - 1][33] == IMAGE_BLACK))
        {
            break;
        }
    }
    street_len_33 = 59 - img_y;  //道路中len_75的白色长度

    for(img_y = 50; img_y > 1; img_y--)
    {
        if((DI.ui8_ImageArray[img_y][46] == IMAGE_BLACK) &&
           (DI.ui8_ImageArray[img_y - 1][46] == IMAGE_BLACK))
        {
            break;
        }
    }
    street_len[46] = 59 - img_y;  //道路中len_75的白色长度

    for(img_y = 50; img_y > 1; img_y--)
    {
        if((DI.ui8_ImageArray[img_y][48] == IMAGE_BLACK) &&
           (DI.ui8_ImageArray[img_y - 1][48] == IMAGE_BLACK))
        {
            break;
        }
    }
    street_len[48] = 59 - img_y;  //道路中len_75的白色长度

    for(img_y = 50; img_y > 1; img_y--)
    {
        if((DI.ui8_ImageArray[img_y][51] == IMAGE_BLACK) &&
           (DI.ui8_ImageArray[img_y - 1][51] == IMAGE_BLACK))
        {
            break;
        }
    }
    street_len[51] = 59 - img_y;  //道路中len_75的白色长度

    for(img_y = 50; img_y > 1; img_y--)
    {
        if((DI.ui8_ImageArray[img_y][34] == IMAGE_BLACK) &&
           (DI.ui8_ImageArray[img_y - 1][34] == IMAGE_BLACK))
        {
            break;
        }
    }
    street_len[34] = 59 - img_y;  //道路中len_75的白色长度

    for(img_y = 50; img_y > 1; img_y--)
    {
        if((DI.ui8_ImageArray[img_y][32] == IMAGE_BLACK) &&
           (DI.ui8_ImageArray[img_y - 1][32] == IMAGE_BLACK))
        {
            break;
        }
    }
    street_len[32] = 59 - img_y;  //道路中len_75的白色长度

    for(img_y = 50; img_y > 1; img_y--)
    {
        if((DI.ui8_ImageArray[img_y][29] == IMAGE_BLACK) &&
           (DI.ui8_ImageArray[img_y - 1][29] == IMAGE_BLACK))
        {
            break;
        }
    }
    street_len[29] = 59 - img_y;  //道路中len_75的白色长度
}

/*
函数名：测量路宽
功能：根据可视距离确定路宽
*/
static void MeasureLineWidth(void)
{
    uint8 ui8_I;

    for(ui8_I = 0; ui8_I < IMAGE_H; ui8_I++)
    {
        if(DI.ui8_RPoint[ui8_I] > DI.ui8_LPoint[ui8_I])
        {
            ui8_LineWidth[ui8_I] = (uint8)(DI.ui8_RPoint[ui8_I] - DI.ui8_LPoint[ui8_I]);
        }
    }
}

/**找边界点***/
void Seek_point(uint8 Y)
{
    //uint8 x1=0,x2=0,Bx3=0,Bx4=0,Wx3=0,Wx4=0;
    uint8 x1;
    uint8 x2;
    uint8 Bx3;
    uint8 Bx4;
    //uint8 Wlength=0,WMax_length=0;   //白色区域宽度
    uint8 Blength;
    uint8 BMax_length;   //黑色区域宽度

    x1 = 0;
    x2 = 0;
    Bx3 = 0;
    Bx4 = 0;
    Blength = 0;
    BMax_length = 0;
    Change_time = 0;
    L_point = 2;
    R_point = 78;

    //遍历所取行
    while(x1 < 79)
    {
        if(!DI.ui8_ImageArray[Y][x1])   //第一点是黑色,首先开始黑色向白色移动
        {
            x2 = Seek_Write_point(x1, Y);
            Blength = x2 - x1;
            if(Blength > BMax_length)   //记录最大长度,并记录坐标
            {
                Bx4 = x2;
                Bx3 = x1;
                BMax_length = Blength;
            }
        }
        else
        {
            x2 = Seek_Black_point(x1, Y);
            //x2=Seek_Write_point(x1,Y);
            Blength = x2 - x1;
            if(Blength <= 10)
            {
                Change_time--;
            }
            if(Blength > BMax_length)   //记录最大长度,并记录坐标
            {
                Bx4 = x2;
                Bx3 = x1;
                BMax_length = Blength;
            }
        }
        if(x2 <= x1)
        {
            x2 = x1 + 1;
        }
        x1 = x2;
    }
    L_point = Bx3;
    R_point = Bx4;
}

//黑色向白色,返回白色点X2
static uint8 Seek_Write_point(uint8 x1, uint8 Y)
{
    for(img_x = x1; img_x <= 78; img_x++)
    {
        if(DI.ui8_ImageArray[Y][img_x] && DI.ui8_ImageArray[Y][img_x + 1])
        {
            Change_time++;                           //每次黑白跳变次数+1
            return img_x;
        }
    }
    return 79;
}

//白色向黑色,返回黑点X2
static uint8 Seek_Black_point(uint8 x1, uint8 Y)
{
    for(img_x = x1; img_x <= 78; img_x++)
    {
        if(!DI.ui8_ImageArray[Y][img_x] && !DI.ui8_ImageArray[Y][img_x + 1])
        {
            Change_time++;                         //每次黑白跳变次数+1
            return img_x;
        }
    }
    return 79;
}

/************************************************************************
函数名：获取反向可视距离
功能：根据中值处理计算出反向可视距离
返回值：反向可视距离（可确定小S、直道）
************************************************************************/
static void GetReverseVisualRange(void)
{
    uint16 ui16_SumY;       //扫描统计
    int16 i16_StartPointX;  //起始点X
    uint8 ui8_X;
    uint8 ui8_Y;
    uint8 ui8_LX;
    uint8 ui8_RX;

    ui16_SumY = 0;
    DI.ui8_MinH = DI.ui8_DisposeScopeUp; //最小可视距离

    if((DI.ui8_LPoint[DI.ui8_DisposeScopeDown] > DI.ui8_DisposeScopeLeft) &&
       (DI.ui8_RPoint[DI.ui8_DisposeScopeDown] < DI.ui8_DisposeScopeRight))
    {
        i16_StartPointX = (DI.ui8_LPoint[DI.ui8_DisposeScopeDown] +
                           DI.ui8_RPoint[DI.ui8_DisposeScopeDown]) >> 1;
    }
    else if(DI.ui8_LPoint[DI.ui8_DisposeScopeDown] > DI.ui8_DisposeScopeLeft)
    {
        i16_StartPointX = DI.ui8_LPoint[DI.ui8_DisposeScopeDown] +
                          (DI.ui8_LineWidth[DI.ui8_DisposeScopeDown] >> 1);
    }
    else if(DI.ui8_RPoint[DI.ui8_DisposeScopeDown] < DI.ui8_DisposeScopeRight)
    {
        i16_StartPointX = DI.ui8_RPoint[DI.ui8_DisposeScopeDown] -
                          (DI.ui8_LineWidth[DI.ui8_DisposeScopeDown] / 2);
    }
    else
    {
        i16_StartPointX = MID_POINT;
    }

    //上面的代码是确定可视距离的起点位于图像底部的什么位置

    if(i16_StartPointX > 69)
    {
        i16_StartPointX = 69;
    }
    else if(i16_StartPointX < 10)
    {
        i16_StartPointX = 10;
    }//留出十条竖线扫描

    for(ui8_X = 1; ui8_X <= 10; ui8_X++)
    {
        ui8_Y = DI.ui8_DisposeScopeDown;  //最下端
        ui8_LX = (uint8)(i16_StartPointX - ui8_X);
        ui8_RX = (uint8)(i16_StartPointX + ui8_X);

        while((--ui8_Y > (DI.ui8_DisposeScopeUp + 2)) &&
              (DI.ui8_ImageArray[ui8_Y][ui8_LX] || DI.ui8_ImageArray[ui8_Y - 1][ui8_LX]) &&
              (ui8_LX > DI.ui8_LPoint[ui8_Y]) &&
              (ui8_LX < DI.ui8_RPoint[ui8_Y]))
        {
        }
        ui16_SumY += ui8_Y;
        if((DI.ui8_MinH < ui8_Y) && (ui8_Y != 57))
        {
            DI.i8_MinHX = ui8_LX;
            DI.ui8_MinH = ui8_Y;
        }

        ui8_Y = DI.ui8_DisposeScopeDown;
        while((--ui8_Y > (DI.ui8_DisposeScopeUp + 2)) &&
              (DI.ui8_ImageArray[ui8_Y][ui8_RX] || DI.ui8_ImageArray[ui8_Y - 1][ui8_RX]) &&
              (ui8_RX > DI.ui8_LPoint[ui8_Y]) &&
              (ui8_RX < DI.ui8_RPoint[ui8_Y]))
        {
        }
        ui16_SumY += ui8_Y;
        if((DI.ui8_MinH < ui8_Y) && (ui8_Y != 57))
        {
            DI.i8_MinHX = ui8_RX;
            DI.ui8_MinH = ui8_Y;
        }
    }

    //平均值为可视距离
    Vistable_scale = (uint8)(ui16_SumY / 20);
    if(DI.ui8_MinH < Vistable_scale)
    {
        DI.ui8_MinH = Vistable_scale;
        DI.i8_MinHX = MID_POINT;
    }
}

static void ConstructImage(void)
{
    uint8 Y;

    /***主函数处理***/
    //Soft_Binarization();
    Ladder();                  //爬梯
    FitRoad();                 //补线，还没有考虑环岛。补线条件是垂直上升
    Get_len();
    GetReverseVisualRange();   //获取反向可视距离
    for(Y = 0; Y <= DI.ui8_DisposeScopeDown; Y++)
    {
        hangkuan60[Y] = (uint16)(DI.ui8_RPoint[Y] - DI.ui8_LPoint[Y]);
    }
}

/************************************************************************
函数名：确定扫描行
功能：根据可视距离来确定扫描行以确保扫描区域正确
************************************************************************/
static void DetermineScanLine(void)
{
    uint8 ui8_LineWidth;
    int8 i8_I;

    //先确定第九行参数
    if((59 - street_len_5 < Vistable_scale) && (street_len_5 > street_len_75))
    {
        DI.ui8_ScanLineY[9] = 60 - street_len_5;
    }
    else if((59 - street_len_75 < Vistable_scale) && (street_len_75 > street_len_5))
    {
        DI.ui8_ScanLineY[9] = 60 - street_len_75;
    }
    else
    {
        DI.ui8_ScanLineY[9] = Vistable_scale + 1;
    }

    DI.ui8_ScanLineL[9] = DI.ui8_LPoint[DI.ui8_ScanLineY[9]]; //由远及近
    DI.ui8_ScanLineR[9] = DI.ui8_RPoint[DI.ui8_ScanLineY[9]];
    DI.ui8_ScanLineToL[9] = (DI.ui8_ScanLineL[9] + DI.ui8_ScanLineR[9]) / 2;

    //路宽计算
    //DI.ui8_ScanLineWidth[9] = DI.ui8_ScanLineR[9] - DI.ui8_ScanLineL[9];

    DI.ui8_ScanLineToR[9] = DI.ui8_ScanLineToL[9];

    //找最边界
    while((DI.ui8_ScanLineToL[9] > DI.ui8_DisposeScopeLeft) &&
          (DI.ui8_ImageArray[DI.ui8_ScanLineY[9]][DI.ui8_ScanLineToL[9]--] ||
           DI.ui8_ImageArray[DI.ui8_ScanLineY[9]][DI.ui8_ScanLineToL[9]]))
    {
    }
    if((!DI.ui8_ImageArray[DI.ui8_ScanLineY[9]][DI.ui8_ScanLineToL[9]]) &&
       (DI.ui8_ScanLineToL[9] < DI.ui8_DisposeScopeRight))
    {
        DI.ui8_ScanLineToL[9]++; //找到边界便找下一行
    }

    while((DI.ui8_ScanLineToR[9] < DI.ui8_DisposeScopeRight) &&
          (DI.ui8_ImageArray[DI.ui8_ScanLineY[9]][DI.ui8_ScanLineToR[9]++] ||
           DI.ui8_ImageArray[DI.ui8_ScanLineY[9]][DI.ui8_ScanLineToR[9]]))
    {
    }
    if((!DI.ui8_ImageArray[DI.ui8_ScanLineY[9]][DI.ui8_ScanLineToR[9]]) &&
       (DI.ui8_ScanLineToR[9] > DI.ui8_DisposeScopeLeft))
    {
        DI.ui8_ScanLineToR[9]--;
    }
    //再次求取路宽
    //第一种2边都到白边
    //if(DI.ui8_ScanLineToR[9] < DI.ui8_DisposeScopeRight - 3 && DI.ui8_ScanLineToL[9] > DI.ui8_DisposeScopeLeft + 3
    //   && (DI.ui8_ScanLineToR[9] - DI.ui8_RPoint[DI.ui8_ScanLineY[9]] >= 3 ||
    //       DI.ui8_LPoint[DI.ui8_ScanLineY[9]] - DI.ui8_ScanLineToL[9] >= 3))
    //{
    //    DI.ui8_ScanLineWidth[9] = DI.ui8_ScanLineToR[9] - DI.ui8_ScanLineToL[9];
    //}
    //else if(DI.ui8_ScanLineToR[9] < DI.ui8_DisposeScopeRight - 3 && DI.ui8_ScanLineToL[9] > DI.ui8_DisposeScopeLeft + 3
    //        && (DI.ui8_ScanLineToR[9] - DI.ui8_RPoint[DI.ui8_ScanLineY[9]] < 3 ||
    //            DI.ui8_LPoint[DI.ui8_ScanLineY[9]] - DI.ui8_ScanLineToL[9] < 3))
    //{
    //}
    //if(DI.ui8_ScanLineToR[9] >= DI.ui8_DisposeScopeRight - 3 && DI.ui8_ScanLineToL[9] <= DI.ui8_DisposeScopeLeft + 3)
    //{
    //    DI.ui8_ScanLineWidth[9] = (uint8)(DI.ui8_LineWidth[DI.ui8_ScanLineY[9]]);
    //}

    ui8_LineWidth = DI.ui8_DisposeScopeDown - DI.ui8_ScanLineY[9];
    f_E_H = (double)ui8_LineWidth / 45.0;
    d_Y = DI.ui8_ScanLineY[9];

    for(i8_I = 8; i8_I >= 0; i8_I--)
    {
        //十等分 每分宽度为1,2,3,4,5,6,7,8,9故分为45小分
        d_Y = d_Y + (double)(9 - i8_I) * f_E_H;
        //采集扫描行位置
        DI.ui8_ScanLineY[i8_I] = (int)(d_Y + 0.5);
        //采集扫描行左边界（补线）
        DI.ui8_ScanLineL[i8_I] = DI.ui8_LPoint[DI.ui8_ScanLineY[i8_I]];
        //采集扫描行右边界（补线）
        DI.ui8_ScanLineR[i8_I] = DI.ui8_RPoint[DI.ui8_ScanLineY[i8_I]];
        //采集扫描行最边界（黑白交界处）
        DI.ui8_ScanLineToL[i8_I] = (DI.ui8_ScanLineL[i8_I] + DI.ui8_ScanLineR[i8_I]) >> 1;
        DI.ui8_ScanLineToR[i8_I] = DI.ui8_ScanLineToL[i8_I];

        while((DI.ui8_ScanLineToL[i8_I] > DI.ui8_DisposeScopeLeft) &&
              (DI.ui8_ImageArray[DI.ui8_ScanLineY[i8_I]][DI.ui8_ScanLineToL[i8_I]--] ||
               DI.ui8_ImageArray[DI.ui8_ScanLineY[i8_I]][DI.ui8_ScanLineToL[i8_I]]))
        {
        }
        if((!DI.ui8_ImageArray[DI.ui8_ScanLineY[i8_I]][DI.ui8_ScanLineToL[i8_I]]) &&
           (DI.ui8_ScanLineToL[i8_I] < DI.ui8_DisposeScopeRight))
        {
            DI.ui8_ScanLineToL[i8_I]++;
        }

        while((DI.ui8_ScanLineToR[i8_I] < DI.ui8_DisposeScopeRight) &&
              (DI.ui8_ImageArray[DI.ui8_ScanLineY[i8_I]][DI.ui8_ScanLineToR[i8_I]++] ||
               DI.ui8_ImageArray[DI.ui8_ScanLineY[i8_I]][DI.ui8_ScanLineToR[i8_I]]))
        {
        }
        if((!DI.ui8_ImageArray[DI.ui8_ScanLineY[i8_I]][DI.ui8_ScanLineToR[i8_I]]) &&
           (DI.ui8_ScanLineToR[i8_I] > DI.ui8_DisposeScopeLeft))
        {
            DI.ui8_ScanLineToR[i8_I]--;
        }
    }
}

/***********************************************************************
函数名：正常处理
***********************************************************************/
static void NormalTreatment(void)
{
    uint8 ui8_I;

    //直道
    for(ui8_I = 0; ui8_I < 10; ui8_I++)
    {
        //普通角度
        //左右有边界
        if((DI.ui8_ScanLineL[ui8_I] > (DI.ui8_DisposeScopeLeft + 10)) &&
           (DI.ui8_ScanLineR[ui8_I] < (DI.ui8_DisposeScopeRight - 10)))
        {
            DI.i16_Mid[ui8_I] = (DI.ui8_ScanLineL[ui8_I] + DI.ui8_ScanLineR[ui8_I]) >> 1;
            //左边有边界
        }
        else if(DI.ui8_ScanLineL[ui8_I] > (DI.ui8_DisposeScopeLeft + 10))
        {
            DI.i16_Mid[ui8_I] = DI.ui8_ScanLineL[ui8_I] +
                                (DI.ui8_LineWidth[DI.ui8_ScanLineY[ui8_I]] >> 1);
            //右边有边界
        }
        else if(DI.ui8_ScanLineR[ui8_I] < (DI.ui8_DisposeScopeRight - 10))
        {
            DI.i16_Mid[ui8_I] = DI.ui8_ScanLineR[ui8_I] -
                                (DI.ui8_LineWidth[DI.ui8_ScanLineY[ui8_I]] >> 1);
        }
    }
}

/***********************************************************************
函数名：提取初始中值
功能：判断道路类型分别处理得到初始中值
参数：行位置
***********************************************************************
***********************************************************************
************************************************************************/
static void SelectMid(void)
{
    NormalTreatment();
}

/************************************************************************
函数名：确定最后中值
功能：根据道路状况和中值变化确定有效性         //判断阳光进行转电磁在这里写     oo
参数：行位置
************************************************************************/
static void DetermineMid(void)
{
    uint8 ui8_I;
    double i32_Mid;
    double d_SumWeight;

    i32_Mid = 0;
    d_SumWeight = 0;
    for(ui8_I = 0; ui8_I < 10; ui8_I++)
    {
        DI.i16_FinallyMid[ui8_I] = (int)(DI.i16_Mid[ui8_I] + HtoE * (DI.i16_Mid[ui8_I] - MID_POINT)              //比值，用以判断弯道的缓急 HtoE在同样的弯道反应不同的偏差
                                                             * DI.ui8_ScanLineY[ui8_I] / DI.f_BaseY[ui8_I]); //当前动态横线/标准横线

        i32_Mid = i32_Mid + DI.i16_FinallyMid[ui8_I] * DI.f_BaseLineWeight[ui8_I];

        d_SumWeight = d_SumWeight + DI.f_BaseLineWeight[ui8_I];
    }

    i32_Mid = i32_Mid / d_SumWeight;

    //  if(IN_L==2){
    //      if (i32_Mid < DI.ui8_DisposeScopeLeft ) {
    //    i32_Mid = DI.ui8_DisposeScopeLeft ;
    //  }
    // }
    //  else if(IN_R==2){
    //    if(i32_Mid > DI.ui8_DisposeScopeRight){
    //    i32_Mid = DI.ui8_DisposeScopeRight;
    //    }
    //  }

    if(i32_Mid < DI.ui8_DisposeScopeLeft - 10)
    {
        i32_Mid = DI.ui8_DisposeScopeLeft - 10;
    }
    else if(i32_Mid > DI.ui8_DisposeScopeRight + 10)
    {
        i32_Mid = DI.ui8_DisposeScopeRight + 10;
    }

    else
    {
        poserror = (int)(i32_Mid - MID_POINT);
    }

    if(poserror > 23)//21
    {
        poserror = 23;
    }
    if(poserror < -23)
    {
        poserror = -23;
    }

    poserror_array[3] = poserror_array[2];
    poserror_array[2] = poserror_array[1];
    poserror_array[1] = poserror_array[0];
    poserror_array[0] = poserror;
}

/************************************************************************
函数名：正常控制
************************************************************************/
static void NormalControl(void)
{
//  DetermineWeight();
    SelectMid();    //提取初始中值
    DetermineMid(); //确定最后中值
}

static void DisposeImage(void)
{
/************************************************************************
函数名：图像处理函数
功能：图像处理流程
************************************************************************/
    ConstructImage();    //图像构造
    DetermineScanLine(); //确定扫描行
    NormalControl();     //正常控制
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//  @name           Image_Process
//  @brief          整个图像处理的主函数，里面包含了所有的图像处理子函数
//  @parameter      void
//  @time           2022年1月19日
//  @Author
//  Sample usage:   Image_Process();
//---------------------------------------------------------------------------------------------------------------------------------------------------------------
static void Image_Process(void)
{
    if(mt9v03x_finish_flag == 1)                         //如果一帧图像采集完了，那么就可以对这副图像处理。
    {
        Image_Compress();    //图像压缩，把原始图像压缩成60*80
        Get_BinaryImage();   //图像二值化处理，把采集到的原始灰度图像变成黑白图像。
        Pixle_Filter();      //像素滤波，结果同步给爬梯使用的 DI.ui8_ImageArray。
        mt9v03x_finish_flag = 0;
    }
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

    gpio_set_level(LED_DEBUG, GPIO_LOW);

    Image_Process();
    DisposeImage();

    Image.tow_row = image_limit_row(SmartCar.servo.tow_point);
    Image.lost = 0;     //学长主链路不按图像黑白比例停车，半十字由扫描行继续给出中值。
    image_export_result();
    Image.sequence++;

    gpio_set_level(LED_DEBUG, GPIO_HIGH);

}
