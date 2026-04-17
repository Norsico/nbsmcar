#include "search_line.h"
#include "dev_flash.h"
#include "dev_encoder.h"
#include "dev_other.h"
#include "dev_servo.h"
#include "dev_wheel.h"
#include "system_state.h"

#define SEARCH_LINE_OTSU_W                  (LCDW)
#define SEARCH_LINE_OTSU_H                  (LCDH)
#define SEARCH_LINE_OTSU_THRESHOLD_MIN      (0)
#define SEARCH_LINE_OTSU_THRESHOLD_CAP      (180)
#define SEARCH_LINE_OTSU_THRESHOLD_STATIC   (70)

/* 普通赛道基础前瞻行。 */
#define SEARCH_LINE_OTSU_DET_TOW_POINT      (27)
#define SEARCH_LINE_OTSU_DET_WINDOW         (5)
#define SEARCH_LINE_OTSU_DET_TOW_POINT_MAX  (49)
#define SEARCH_LINE_OTSU_DET_WEIGHT_COUNT   (10)
#define SEARCH_LINE_OTSU_DET_SPEED_REF      (160.0f)
#define SEARCH_LINE_OTSU_DET_SPEED_GAIN     (0.2f)
#define SEARCH_LINE_OTSU_DET_SPEED_BIAS     (0.5f)
#define SEARCH_LINE_OTSU_DET_SPEED_GAIN_MIN (-1.0f)
#define SEARCH_LINE_OTSU_DET_SPEED_GAIN_MAX (3.0f)
#define SEARCH_LINE_OTSU_VARIANCE_ACC_LIMIT (25)
#define SEARCH_LINE_OTSU_STRAIGHT_OFFLINE_MAX (7)
#define SEARCH_LINE_OTSU_STRAIGHT_LOST_LINE_MAX (0)
#define SEARCH_LINE_RING_FLAG1_BEEP_MS       (120)
#define SEARCH_LINE_STEER_REF_MIDDLE_DUTY   (4880.0f)
#define SEARCH_LINE_STEER_REF_RIGHT_DUTY    (4100.0f)

#define SEARCH_LINE_STEER_REF_LEFT_DUTY     (5520.0f)
#define SEARCH_LINE_STATE_INIT              ('F') /* 初始态，当前行未完成搜边。 */
#define SEARCH_LINE_STATE_FOUND             ('T') /* 找到跳变边界。 */
#define SEARCH_LINE_STATE_WHITE             ('W') /* 扫描窗内全白，当前侧无明确边界。 */
#define SEARCH_LINE_STATE_BLACK             ('H') /* 扫描窗内全黑，当前侧搜索失败。 */
#define SEARCH_LINE_ROAD_NORMAL             (Normol)
#define SEARCH_LINE_ROAD_STRAIGHT           (Straight)
#define SEARCH_LINE_ROAD_CROSS              (Cross)
#define SEARCH_LINE_ROAD_RAMP               (Ramp)
#define SEARCH_LINE_ROAD_LEFT_CIRQUE        (LeftCirque)
#define SEARCH_LINE_ROAD_RIGHT_CIRQUE       (RightCirque)
#define SEARCH_LINE_ROAD_FORK_IN            (Forkin)
#define SEARCH_LINE_ROAD_FORK_OUT           (Forkout)
#define SEARCH_LINE_ROAD_BARN_OUT           (Barn_out)
#define SEARCH_LINE_ROAD_BARN_IN            (Barn_in)
#define SEARCH_LINE_ROAD_CROSS_TRUE         (Cross_ture)
#define SEARCH_LINE_ROAD_ZEBRA              (Zebra_Flag)

/* 边界跟踪前进规则，口径对齐 19 国一 Search_Left_and_Right_Lines。 */
static const int8 SearchLine_Otsu_Left_Rule[2][8] =
{
    {0, -1, 1, 0, 0, 1, -1, 0},
    {-1, -1, 1, -1, 1, 1, -1, 1}
};
static const int8 SearchLine_Otsu_Right_Rule[2][8] =
{
    {0, -1, 1, 0, 0, 1, -1, 0},
    {1, -1, 1, 1, -1, 1, -1, -1}
};

/* 压缩灰度图和二值图直接按国一口径导出。 */
IFX_ALIGN(4) uint8 Image_Use[LCDH][LCDW] = {0};
IFX_ALIGN(4) uint8 Pixle[LCDH][LCDW] = {0};
ImageStatustypedef ImageStatus =
{
    SEARCH_LINE_OTSU_DET_TOW_POINT,
    0,
    0,
    0,
    SEARCH_LINE_OTSU_THRESHOLD_STATIC,
    SEARCH_LINE_OTSU_THRESHOLD_CAP
};
ImageDealDatatypedef ImageDeal[LCDH] = {0};
int ImageScanInterval = 2;
int ImageScanInterval_Cross = 2;
/* 底部初始化后的左边界。 */
static uint8 SearchLine_Otsu_Left_Border[SEARCH_LINE_OTSU_H] = {0};
/* 底部初始化后的右边界。 */
static uint8 SearchLine_Otsu_Right_Border[SEARCH_LINE_OTSU_H] = {0};
/* 当前行是否已经完成底边初始化。 */
static uint8 SearchLine_Otsu_Row_Valid[SEARCH_LINE_OTSU_H] = {0};
static int Ysite = 0, Xsite = 0;
static uint8 *PicTemp = 0;
static int IntervalLow = 0, IntervalHigh = 0;
static int ytemp = 0;
static int TFSite = 0, FTSite = 0;
static float DetR = 0, DetL = 0;
static int BottomBorderRight = 79, BottomBorderLeft = 0, BottomCenter = 0;
/* 当前阶段先把边界观测缓存名收回到国一口径。 */
static uint8 LeftBoundary_First[SEARCH_LINE_OTSU_H] = {0};
static uint8 RightBoundary_First[SEARCH_LINE_OTSU_H] = {0};
static uint8 LeftBoundary[SEARCH_LINE_OTSU_H] = {0};
static uint8 RightBoundary[SEARCH_LINE_OTSU_H] = {0};
static uint8 ExtenLFlag = 0;
static uint8 ExtenRFlag = 0;
/* 对齐参考代码的舵机位置式 PD 预览量。 */
static int16 SearchLine_Otsu_Steer_Offset = 0;
static uint8 SearchLine_Otsu_Steer_Command = CAR_SERVO_CENTER_ANGLE;
static uint16 SearchLine_Otsu_Steer_P_Tenth = FLASH_STEER_P_DEFAULT_TENTH;
static uint16 SearchLine_Otsu_Steer_D_Tenth = FLASH_STEER_D_DEFAULT_TENTH;
static float SearchLine_Otsu_Steer_Last_Error = 0.0f;
/* 当前工程未接岔路状态源。 */
static uint8 SearchLine_Otsu_Fork_Down = 0;
static uint8 SearchLine_Otsu_Ring_Element = 0;
static uint8 SearchLine_Otsu_Ring_Size = 0;
static uint8 SearchLine_Otsu_Ring_Flag = 0;
static uint8 Cirque_Left_Count = 0;
static uint8 Cirque_Right_Count = 0;
static uint16 Ring_Stage_Num = 0;
static uint16 Ring_Point_Y = 0;
static int16 Ring_Straight_Judge_Tenth = -1;
static uint32 SearchLine_Ring_Beep_Stop_Tick = 0;
static uint8 SearchLine_Preview_Label_Ready = 0;
static uint8 SearchLine_Preview_Last_Threshold = 0xFF;
static int16 SearchLine_Preview_Last_Offset = 32767;
static uint8 SearchLine_Preview_Last_Command = 0xFF;
static uint8 SearchLine_Preview_Last_Ring_Element = 0xFF;
static uint8 SearchLine_Preview_Last_Ring_Flag = 0xFF;
static uint8 SearchLine_Preview_Last_Ring_Size = 0xFF;
static uint8 SearchLine_Preview_Last_Offline_Row = 0xFF;
static uint8 SearchLine_Preview_Last_White_Line = 0xFF;
static uint8 SearchLine_Preview_Last_Cirque_Left_Count = 0xFF;
static uint8 SearchLine_Preview_Last_Cirque_Right_Count = 0xFF;
static uint8 SearchLine_Preview_Last_Ring_Left_Line = 0xFF;
static uint8 SearchLine_Preview_Last_Ring_Right_Line = 0xFF;
static uint8 SearchLine_Preview_Last_Ring_Left_Line_RightPanel = 0xFF;
static uint8 SearchLine_Preview_Last_Ring_Right_Line_RightPanel = 0xFF;
static uint16 SearchLine_Preview_Last_Ring_Stage_Num = 0xFFFF;
static uint16 SearchLine_Preview_Last_Ring_Point_Y = 0xFFFF;
static int16 SearchLine_Preview_Last_Ring_Straight_Judge_Tenth = 32767;
float Det = 0;
float Mh = MT9V03X_H;
float Lh = LCDH;
float Mw = MT9V03X_W;
float Lw = LCDW;

#define SearchLine_Otsu_Left_Boundary_First LeftBoundary_First
#define SearchLine_Otsu_Right_Boundary_First RightBoundary_First
#define SearchLine_Otsu_Left_Boundary LeftBoundary
#define SearchLine_Otsu_Right_Boundary RightBoundary
#define SearchLine_Otsu_Offline_Row ImageStatus.OFFLine
#define SearchLine_Otsu_Left_Line ImageStatus.Left_Line
#define SearchLine_Otsu_Right_Line ImageStatus.Right_Line
#define SearchLine_Otsu_White_Line ImageStatus.WhiteLine
#define SearchLine_Otsu_White_Line_Left ImageStatus.WhiteLine_L
#define SearchLine_Otsu_White_Line_Right ImageStatus.WhiteLine_R
#define SearchLine_Otsu_TowPoint_True ImageStatus.TowPoint_True
#define SearchLine_Otsu_Det_True ImageStatus.Det_True
#define SearchLine_Otsu_Cirque_Left_Count Cirque_Left_Count
#define SearchLine_Otsu_Cirque_Right_Count Cirque_Right_Count
#define SearchLine_Otsu_Ring_Stage_Num Ring_Stage_Num
#define SearchLine_Otsu_Ring_Point_Y Ring_Point_Y
#define SearchLine_Otsu_Ring_Straight_Judge_Tenth Ring_Straight_Judge_Tenth
static float Weighting[SEARCH_LINE_OTSU_DET_WEIGHT_COUNT] =
{
    0.96f, 0.92f, 0.88f, 0.83f, 0.77f,
    0.71f, 0.65f, 0.59f, 0.53f, 0.47f
};
static const uint8 Half_Road_Wide[SEARCH_LINE_OTSU_H] =
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
    int temp = 0;

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

/* 当前先按整幅相机图压缩，处理口径直接对齐国一。 */
void compressimage(void)
{
    int i, j, row, line;
    const float div_h = Mh / Lh, div_w = Mw / Lw;

    for(i = 0; i < LCDH; i++)
    {
        row = i * div_h + 0.5f;

        for(j = 0; j < LCDW; j++)
        {
            line = j * div_w + 0.5f;
            Image_Use[i][j] = mt9v03x_image[row][line];
        }
    }
    mt9v03x_finish_flag = 0;  /* 使用完一帧 DMA 图像后允许下一帧继续搬运。 */
}

static uint8 SearchLine_Get_Otsu_Binary_Pixel(int16 row, int16 col)
{
    if((row <= 0) || (row >= (59)) ||
       (col <= 0) || (col >= (SEARCH_LINE_OTSU_W - 1)))
    {
        return 0;
    }

    return Pixle[row][col];
}

/* 圆环支线缓存初始化。 */
static void SearchLine_Clear_Otsu_BorderTraceState(void)
{
    uint16 row = 0;

    ImageStatus.OFFLineBoundary = 5;

    for(row = 0; row < SEARCH_LINE_OTSU_H; row++)
    {
        SearchLine_Otsu_Left_Boundary_First[row] = 0;
        SearchLine_Otsu_Right_Boundary_First[row] = SEARCH_LINE_OTSU_W - 1;
        SearchLine_Otsu_Left_Boundary[row] = 0;
        SearchLine_Otsu_Right_Boundary[row] = SEARCH_LINE_OTSU_W - 1;
    }
}

/* 19 国一 Search_Border_OTSU 的边界跟踪支线。
 * 这一支只更新观测缓存，不改主链左右边界。
 */
static void Search_Border_OTSU(uint8 imageInput[LCDH][LCDW], uint8 Row, uint8 Col, uint8 Bottonline)
{
    uint8 bottom_row = 58;
    uint8 trace_row = 0;
    uint8 left_y = 0;
    uint8 left_x = 0;
    uint8 right_y = 0;
    uint8 right_x = 0;
    uint8 left_direction = 0;
    uint8 right_direction = 0;
    uint8 left_probe_y = 0;
    uint8 left_probe_x = 0;
    uint8 right_probe_y = 0;
    uint8 right_probe_x = 0;
    uint8 row = 0;
    uint16 guard = 0;
    int16 next_row = 0;
    int16 next_col = 0;
    int16 probe_delta = 0;
    int16 center_col = ImageSensorMid;
    int16 col = 0;

    (void)imageInput;
    (void)Row;
    (void)Col;
    bottom_row = Bottonline;

    SearchLine_Otsu_White_Line_Left = 0;
    SearchLine_Otsu_White_Line_Right = 0;
    SearchLine_Clear_Otsu_BorderTraceState();

    if(!SearchLine_Otsu_Row_Valid[bottom_row])
    {
        return;
    }

    left_y = bottom_row;
    right_y = bottom_row;
    left_x = 0;
    right_x = SEARCH_LINE_OTSU_W - 1;
    for(col = center_col - 2; col > 1; col--)
    {
        if((1 == SearchLine_Get_Otsu_Binary_Pixel(bottom_row, col)) &&
           (0 == SearchLine_Get_Otsu_Binary_Pixel(bottom_row, col - 1)))
        {
            left_x = (uint8)col;
            break;
        }
    }
    for(col = center_col + 2; col < (SEARCH_LINE_OTSU_W - 1); col++)
    {
        if((1 == SearchLine_Get_Otsu_Binary_Pixel(bottom_row, col)) &&
           (0 == SearchLine_Get_Otsu_Binary_Pixel(bottom_row, col + 1)))
        {
            right_x = (uint8)col;
            break;
        }
    }
    left_probe_y = bottom_row;
    left_probe_x = left_x;
    right_probe_y = bottom_row;
    right_probe_x = right_x;
    trace_row = bottom_row;

    SearchLine_Otsu_Left_Boundary_First[bottom_row] = left_x;
    SearchLine_Otsu_Left_Boundary[bottom_row] = left_x;
    SearchLine_Otsu_Right_Boundary_First[bottom_row] = right_x;
    SearchLine_Otsu_Right_Boundary[bottom_row] = right_x;

    while(1)
    {
        guard++;
        if(guard > 400)
        {
            ImageStatus.OFFLineBoundary = trace_row;
            break;
        }

        if((trace_row >= left_probe_y) && (trace_row >= right_probe_y))
        {
            if(trace_row < ImageStatus.OFFLineBoundary)
            {
                ImageStatus.OFFLineBoundary = trace_row;
                break;
            }
            else
            {
                trace_row--;
            }
        }

        if((left_probe_y > trace_row) || (trace_row == ImageStatus.OFFLineBoundary))
        {
            next_row = (int16)left_y + SearchLine_Otsu_Left_Rule[0][2 * left_direction + 1];
            next_col = (int16)left_x + SearchLine_Otsu_Left_Rule[0][2 * left_direction];
            left_probe_y = (uint8)Limit(next_row, 0, SEARCH_LINE_OTSU_H - 1);
            left_probe_x = (uint8)Limit(next_col, 0, SEARCH_LINE_OTSU_W - 1);

            if(0 == SearchLine_Get_Otsu_Binary_Pixel(next_row, next_col))
            {
                if(3 == left_direction)
                {
                    left_direction = 0;
                }
                else
                {
                    left_direction++;
                }
            }
            else
            {
                next_row = (int16)left_y + SearchLine_Otsu_Left_Rule[1][2 * left_direction + 1];
                next_col = (int16)left_x + SearchLine_Otsu_Left_Rule[1][2 * left_direction];
                left_probe_y = (uint8)Limit(next_row, 0, SEARCH_LINE_OTSU_H - 1);
                left_probe_x = (uint8)Limit(next_col, 0, SEARCH_LINE_OTSU_W - 1);

                if(0 == SearchLine_Get_Otsu_Binary_Pixel(next_row, next_col))
                {
                    left_y = (uint8)Limit((int16)left_y +
                                                           SearchLine_Otsu_Left_Rule[0][2 * left_direction + 1],
                                                           0,
                                                           SEARCH_LINE_OTSU_H - 1);
                    left_x = (uint8)Limit((int16)left_x +
                                                           SearchLine_Otsu_Left_Rule[0][2 * left_direction],
                                                           0,
                                                           SEARCH_LINE_OTSU_W - 1);
                    if(0 == SearchLine_Otsu_Left_Boundary_First[left_y])
                    {
                        SearchLine_Otsu_Left_Boundary_First[left_y] = left_x;
                    }
                    SearchLine_Otsu_Left_Boundary[left_y] = left_x;
                }
                else
                {
                    left_y = (uint8)Limit(next_row, 0, SEARCH_LINE_OTSU_H - 1);
                    left_x = (uint8)Limit(next_col, 0, SEARCH_LINE_OTSU_W - 1);
                    if(0 == SearchLine_Otsu_Left_Boundary_First[left_y])
                    {
                        SearchLine_Otsu_Left_Boundary_First[left_y] = left_x;
                    }
                    SearchLine_Otsu_Left_Boundary[left_y] = left_x;
                    if(0 == left_direction)
                    {
                        left_direction = 3;
                    }
                    else
                    {
                        left_direction--;
                    }
                }
            }
        }

        if((right_probe_y > trace_row) || (trace_row == ImageStatus.OFFLineBoundary))
        {
            next_row = (int16)right_y + SearchLine_Otsu_Right_Rule[0][2 * right_direction + 1];
            next_col = (int16)right_x + SearchLine_Otsu_Right_Rule[0][2 * right_direction];
            right_probe_y = (uint8)Limit(next_row, 0, SEARCH_LINE_OTSU_H - 1);
            right_probe_x = (uint8)Limit(next_col, 0, SEARCH_LINE_OTSU_W - 1);

            if(0 == SearchLine_Get_Otsu_Binary_Pixel(next_row, next_col))
            {
                if(0 == right_direction)
                {
                    right_direction = 3;
                }
                else
                {
                    right_direction--;
                }
            }
            else
            {
                next_row = (int16)right_y + SearchLine_Otsu_Right_Rule[1][2 * right_direction + 1];
                next_col = (int16)right_x + SearchLine_Otsu_Right_Rule[1][2 * right_direction];
                right_probe_y = (uint8)Limit(next_row, 0, SEARCH_LINE_OTSU_H - 1);
                right_probe_x = (uint8)Limit(next_col, 0, SEARCH_LINE_OTSU_W - 1);

                if(0 == SearchLine_Get_Otsu_Binary_Pixel(next_row, next_col))
                {
                    right_y = (uint8)Limit((int16)right_y +
                                                            SearchLine_Otsu_Right_Rule[0][2 * right_direction + 1],
                                                            0,
                                                            SEARCH_LINE_OTSU_H - 1);
                    right_x = (uint8)Limit((int16)right_x +
                                                            SearchLine_Otsu_Right_Rule[0][2 * right_direction],
                                                            0,
                                                            SEARCH_LINE_OTSU_W - 1);
                    if((SEARCH_LINE_OTSU_W - 1) == SearchLine_Otsu_Right_Boundary_First[right_y])
                    {
                        SearchLine_Otsu_Right_Boundary_First[right_y] = right_x;
                    }
                    SearchLine_Otsu_Right_Boundary[right_y] = right_x;
                }
                else
                {
                    right_y = (uint8)Limit(next_row, 0, SEARCH_LINE_OTSU_H - 1);
                    right_x = (uint8)Limit(next_col, 0, SEARCH_LINE_OTSU_W - 1);
                    if((SEARCH_LINE_OTSU_W - 1) == SearchLine_Otsu_Right_Boundary_First[right_y])
                    {
                        SearchLine_Otsu_Right_Boundary_First[right_y] = right_x;
                    }
                    SearchLine_Otsu_Right_Boundary[right_y] = right_x;
                    if(3 == right_direction)
                    {
                        right_direction = 0;
                    }
                    else
                    {
                        right_direction++;
                    }
                }
            }
        }

        probe_delta = (int16)right_probe_x - (int16)left_probe_x;
        if(probe_delta < 0)
        {
            probe_delta = -probe_delta;
        }
        if(probe_delta < 3)
        {
            ImageStatus.OFFLineBoundary = trace_row;
            break;
        }
    }

    for(row = bottom_row; row > (uint8)(ImageStatus.OFFLineBoundary + 1); row--)
    {
        if(SearchLine_Otsu_Left_Boundary[row] < 3)
        {
            SearchLine_Otsu_White_Line_Left++;
        }
        if(SearchLine_Otsu_Right_Boundary[row] > (SEARCH_LINE_OTSU_W - 3))
        {
            SearchLine_Otsu_White_Line_Right++;
        }
    }
}

static uint8 SearchLine_Clamp_Otsu_Search_Col(int16 value)
{
    return (uint8)Limit(value, 1, SEARCH_LINE_OTSU_W - 2);
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
    SearchLine_Otsu_Left_Border[59] = (uint8)ImageDeal[59].LeftBorder;
    SearchLine_Otsu_Right_Border[59] = (uint8)ImageDeal[59].RightBorder;
    SearchLine_Otsu_Row_Valid[59] = 1;

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
        SearchLine_Otsu_Left_Border[Ysite] = (uint8)ImageDeal[Ysite].LeftBorder;
        SearchLine_Otsu_Right_Border[Ysite] = (uint8)ImageDeal[Ysite].RightBorder;
        SearchLine_Otsu_Row_Valid[Ysite] = 1;
    }
    return 'T';
}

void GetJumpPointFromDet(uint8 *p, uint8 type, int L, int H, JumpPointtypedef *Q)
{
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
    uint8 L_Found_T = 'F';
    uint8 Get_L_line = 'F';
    uint8 R_Found_T = 'F';
    uint8 Get_R_line = 'F';
    float D_L = 0;
    float D_R = 0;
    int ytemp_W_L = 0;
    int ytemp_W_R = 0;
    int ysite = 0;
    uint8 L_found_point = 0;
    uint8 R_found_point = 0;
    JumpPointtypedef JumpPoint[2];

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

        LimitL(IntervalLow);
        LimitH(IntervalHigh);
        GetJumpPointFromDet(PicTemp, 'R', IntervalLow, IntervalHigh, &JumpPoint[1]);

        IntervalLow = ImageDeal[Ysite + 1].LeftBorder - ImageScanInterval;
        IntervalHigh = ImageDeal[Ysite + 1].LeftBorder + ImageScanInterval;

        LimitL(IntervalLow);
        LimitH(IntervalHigh);
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

        SearchLine_Otsu_Left_Border[Ysite] = (uint8)ImageDeal[Ysite].LeftBorder;
        SearchLine_Otsu_Right_Border[Ysite] = (uint8)ImageDeal[Ysite].RightBorder;
        SearchLine_Otsu_Row_Valid[Ysite] = 1;

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
        (SearchLine_Otsu_Fork_Down == 0
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
        SearchLine_Otsu_Left_Border[Ysite] = (uint8)ImageDeal[Ysite].LeftBorder;
        SearchLine_Otsu_Right_Border[Ysite] = (uint8)ImageDeal[Ysite].RightBorder;
        SearchLine_Otsu_Row_Valid[Ysite] = 1;
    }
}

/* 中线滤波平滑。 */
static void RouteFilter(void)
{
    int16 row = 0;
    int16 search_row = 0;
    int16 fill_row = 0;
    int16 center_temp = 0;
    int16 line_temp = 0;
    float center_slope = 0.0f;

    for(row = 58; row >= (int16)SearchLine_Otsu_Offline_Row + 5; row--)
    {
        if((SEARCH_LINE_STATE_WHITE == ImageDeal[row].IsLeftFind) &&
           (SEARCH_LINE_STATE_WHITE == ImageDeal[row].IsRightFind) &&
           (row <= 45) &&
           (SEARCH_LINE_STATE_WHITE == ImageDeal[row - 1].IsLeftFind) &&
           (SEARCH_LINE_STATE_WHITE == ImageDeal[row - 1].IsRightFind))
        {
            search_row = row;
            while(search_row >= (int16)SearchLine_Otsu_Offline_Row + 5)
            {
                search_row--;
                if((SEARCH_LINE_STATE_FOUND == ImageDeal[search_row].IsLeftFind) &&
                   (SEARCH_LINE_STATE_FOUND == ImageDeal[search_row].IsRightFind))
                {
                    center_slope = ((float)ImageDeal[search_row - 1].Center -
                                    (float)ImageDeal[row + 2].Center) /
                                   (float)((search_row - 1) - (row + 2));
                    center_temp = ImageDeal[row + 2].Center;
                    line_temp = row + 2;
                    for(fill_row = row; fill_row >= search_row; fill_row--)
                    {
                        ImageDeal[fill_row].Center =
                            Limit((int16)((float)center_temp +
                                          center_slope * (float)(fill_row - line_temp)),
                                  0,
                                  SEARCH_LINE_OTSU_W - 1);
                        SearchLine_Otsu_Row_Valid[fill_row] = 1;
                    }
                    row = search_row - 1;
                    break;
                }
            }
        }

        ImageDeal[row].Center =
            (ImageDeal[row - 1].Center + 2 * ImageDeal[row].Center) / 3;
        SearchLine_Otsu_Row_Valid[row] = 1;
    }
}

/* 固定前瞻加权中线。 */
static void GetDet(void)
{
    int16 tow_point = 0;
    int16 row = 0;
    int16 weight_index = 0;
    int16 left_speed = 0;
    int16 right_speed = 0;
    float det_temp = 0.0f;
    float unit_all = 0.0f;
    float speed_gain = 0.0f;
    float current_speed = 0.0f;
    int16 det_value = 0;

    /* 动态前瞻按左右后轮滤波编码器均值调节。 */
    left_speed = encoder_get_left();
    right_speed = encoder_get_right();
    current_speed = ((float)left_speed + (float)right_speed) * 0.5f;
    speed_gain = (current_speed - SEARCH_LINE_OTSU_DET_SPEED_REF) *
                 SEARCH_LINE_OTSU_DET_SPEED_GAIN +
                 SEARCH_LINE_OTSU_DET_SPEED_BIAS;
    if(speed_gain > SEARCH_LINE_OTSU_DET_SPEED_GAIN_MAX)
    {
        speed_gain = SEARCH_LINE_OTSU_DET_SPEED_GAIN_MAX;
    }
    else if(speed_gain < SEARCH_LINE_OTSU_DET_SPEED_GAIN_MIN)
    {
        speed_gain = SEARCH_LINE_OTSU_DET_SPEED_GAIN_MIN;
    }

    /* 圆环沿用 19 国一口径，进环和环中都按 15 行看。 */
    if((((SEARCH_LINE_ROAD_RIGHT_CIRQUE == ImageStatus.Road_type) ||
         (SEARCH_LINE_ROAD_LEFT_CIRQUE == ImageStatus.Road_type)) &&
        ('F' == ImageStatus.CirqueOff)) ||
       (1 == SearchLine_Otsu_Ring_Flag) ||
       (2 == SearchLine_Otsu_Ring_Flag))
    {
        tow_point = 15;
    }
    else
    {
        tow_point = (int16)((float)ImageStatus.TowPoint - speed_gain);
    }

    if(tow_point < ((int16)SearchLine_Otsu_Offline_Row + 1))
    {
        tow_point = (int16)SearchLine_Otsu_Offline_Row + 1;
    }
    if(tow_point > SEARCH_LINE_OTSU_DET_TOW_POINT_MAX)
    {
        tow_point = SEARCH_LINE_OTSU_DET_TOW_POINT_MAX;
    }

    SearchLine_Otsu_TowPoint_True = (uint8)tow_point;

    if((tow_point - SEARCH_LINE_OTSU_DET_WINDOW) >= (int16)SearchLine_Otsu_Offline_Row)
    {
        for(row = tow_point - SEARCH_LINE_OTSU_DET_WINDOW; row < tow_point; row++)
        {
            weight_index = tow_point - row - 1;
            det_temp += Weighting[weight_index] * (float)ImageDeal[row].Center;
            unit_all += Weighting[weight_index];
        }

        for(row = tow_point + SEARCH_LINE_OTSU_DET_WINDOW; row > tow_point; row--)
        {
            weight_index = row - tow_point - 1;
            det_temp += Weighting[weight_index] * (float)ImageDeal[row].Center;
            unit_all += Weighting[weight_index];
        }

        det_temp = ((float)ImageDeal[tow_point].Center + det_temp) / (unit_all + 1.0f);
    }
    else if(tow_point > (int16)SearchLine_Otsu_Offline_Row)
    {
        for(row = (int16)SearchLine_Otsu_Offline_Row; row < tow_point; row++)
        {
            weight_index = tow_point - row - 1;
            det_temp += Weighting[weight_index] * (float)ImageDeal[row].Center;
            unit_all += Weighting[weight_index];
        }

        for(row = tow_point + tow_point - (int16)SearchLine_Otsu_Offline_Row; row > tow_point; row--)
        {
            weight_index = row - tow_point - 1;
            det_temp += Weighting[weight_index] * (float)ImageDeal[row].Center;
            unit_all += Weighting[weight_index];
        }

        det_temp = ((float)ImageDeal[tow_point].Center + det_temp) / (unit_all + 1.0f);
    }
    else if(SearchLine_Otsu_Offline_Row < SEARCH_LINE_OTSU_DET_TOW_POINT_MAX)
    {
        for(row = (int16)SearchLine_Otsu_Offline_Row + 3; row > (int16)SearchLine_Otsu_Offline_Row; row--)
        {
            weight_index = row - tow_point - 1;
            if((weight_index >= 0) && (weight_index < SEARCH_LINE_OTSU_DET_WEIGHT_COUNT))
            {
                det_temp += Weighting[weight_index] * (float)ImageDeal[row].Center;
                unit_all += Weighting[weight_index];
            }
        }

        det_temp = ((float)ImageDeal[SearchLine_Otsu_Offline_Row].Center + det_temp) /
                   (unit_all + 1.0f);
    }
    else
    {
        det_temp = (float)SearchLine_Otsu_Det_True;
    }

    Det = det_temp;
    det_value = (int16)(Det + 0.5f);
    SearchLine_Otsu_Det_True = (uint8)Limit(det_value, 0, SEARCH_LINE_OTSU_W - 1);
}

/* 当前工程这里只保留国一 straight_speed 的直道判定部分。 */
static void straight_speed(void)
{
    int16 row = 0;
    int16 delta = 0;
    uint16 valid_count = 0;
    uint32 sum = 0;
    float variance_acc = 0.0f;

    ImageStatus.straight_acc = 0;
    ImageStatus.variance_acc = 0;

    if((ImageStatus.Road_type == SEARCH_LINE_ROAD_CROSS) ||
       (ImageStatus.Road_type == SEARCH_LINE_ROAD_LEFT_CIRQUE) ||
       (ImageStatus.Road_type == SEARCH_LINE_ROAD_RIGHT_CIRQUE))
    {
        return;
    }

    if(SearchLine_Otsu_Offline_Row >= 54)
    {
        return;
    }

    for(row = 55; row > ((int16)SearchLine_Otsu_Offline_Row + 1); row--)
    {
        delta = ImageDeal[row].Center - ImageSensorMid;
        sum += (uint32)(delta * delta);
        valid_count++;
    }

    if(0 == valid_count)
    {
        return;
    }

    variance_acc = (float)sum / (float)valid_count;
    ImageStatus.variance_acc = (uint16)(variance_acc + 0.5f);
    if((variance_acc < (float)SEARCH_LINE_OTSU_VARIANCE_ACC_LIMIT) &&
       (SearchLine_Otsu_Offline_Row <= SEARCH_LINE_OTSU_STRAIGHT_OFFLINE_MAX) &&
       (SearchLine_Otsu_Left_Line <= SEARCH_LINE_OTSU_STRAIGHT_LOST_LINE_MAX) &&
       (SearchLine_Otsu_Right_Line <= SEARCH_LINE_OTSU_STRAIGHT_LOST_LINE_MAX))
    {
        ImageStatus.straight_acc = 1;
    }
}

static float Straight_Judge(uint8 dir, uint8 start_row, uint8 end_row)
{
    int16 row = 0;
    int16 count = 0;
    float variance = 999.0f;
    float sum = 0.0f;
    float err = 0.0f;
    float slope = 0.0f;

    if(start_row >= SEARCH_LINE_OTSU_H)
    {
        start_row = SEARCH_LINE_OTSU_H - 1;
    }
    if(end_row >= SEARCH_LINE_OTSU_H)
    {
        end_row = SEARCH_LINE_OTSU_H - 1;
    }
    if(start_row >= end_row)
    {
        return variance;
    }

    count = (int16)end_row - (int16)start_row;
    if(0 == count)
    {
        return variance;
    }

    if(1 == dir)
    {
        slope = ((float)SearchLine_Otsu_Left_Border[start_row] -
                 (float)SearchLine_Otsu_Left_Border[end_row]) /
                (float)((int16)start_row - (int16)end_row);
        for(row = 0; row < count; row++)
        {
            err = ((float)SearchLine_Otsu_Left_Border[start_row] +
                   slope * (float)row -
                   (float)SearchLine_Otsu_Left_Border[start_row + row]);
            sum += err * err;
        }
    }
    else if(2 == dir)
    {
        slope = ((float)SearchLine_Otsu_Right_Border[start_row] -
                 (float)SearchLine_Otsu_Right_Border[end_row]) /
                (float)((int16)start_row - (int16)end_row);
        for(row = 0; row < count; row++)
        {
            err = ((float)SearchLine_Otsu_Right_Border[start_row] +
                   slope * (float)row -
                   (float)SearchLine_Otsu_Right_Border[start_row + row]);
            sum += err * err;
        }
    }
    else
    {
        return variance;
    }

    variance = sum / (float)count;
    return variance;
}

static uint8 Cirque_Or_Cross(uint8 type, uint8 start_row)
{
    uint8 num = 0;
    uint8 row = 0;
    uint8 end_row = 0;
    int16 col = 0;

    if(start_row >= SEARCH_LINE_OTSU_H)
    {
        return 0;
    }

    end_row = (uint8)Limit((int16)start_row + 10, 0, SEARCH_LINE_OTSU_H);
    if(1 == type)
    {
        for(row = start_row; row < end_row; row++)
        {
            for(col = SearchLine_Otsu_Left_Border[row]; col > 1; col--)
            {
                if(0 != SearchLine_Get_Otsu_Binary_Pixel(row, col))
                {
                    num++;
                }
            }
        }
    }
    else if(2 == type)
    {
        for(row = start_row; row < end_row; row++)
        {
            for(col = SearchLine_Otsu_Right_Border[row]; col < (SEARCH_LINE_OTSU_W - 2); col++)
            {
                if(0 != SearchLine_Get_Otsu_Binary_Pixel(row, col))
                {
                    num++;
                }
            }
        }
    }

    return num;
}

static void Element_Judgment_Left_Rings(void)
{
    uint8 ring_ysite = 3;
    uint8 point1_y = 0;
    uint8 point2_y = 0;
    uint8 row = 0;
    uint8 ring_help_flag = 0;

    for(row = 58; row > ring_ysite; row--)
    {
        if((int16)SearchLine_Otsu_Left_Boundary_First[row] -
           (int16)SearchLine_Otsu_Left_Boundary_First[row - 1] > 4)
        {
            point1_y = row;
            break;
        }
    }
    for(row = 58; row > ring_ysite; row--)
    {
        if((int16)SearchLine_Otsu_Left_Boundary[row + 1] -
           (int16)SearchLine_Otsu_Left_Boundary[row] > 4)
        {
            point2_y = row;
            break;
        }
    }

    for(row = point1_y; row > 10; row--)
    {
        if((SearchLine_Otsu_Left_Border[row + 6] < SearchLine_Otsu_Left_Border[row + 3]) &&
           (SearchLine_Otsu_Left_Border[row + 5] < SearchLine_Otsu_Left_Border[row + 3]) &&
           (SearchLine_Otsu_Left_Border[row + 3] > SearchLine_Otsu_Left_Border[row + 2]) &&
           (SearchLine_Otsu_Left_Border[row + 3] > SearchLine_Otsu_Left_Border[row + 1]))
        {
            ring_help_flag = 1;
            break;
        }
    }

    if((point2_y > (uint8)(point1_y + 3)) && (0 == ring_help_flag))
    {
        if(SearchLine_Otsu_Left_Line > 6)
        {
            ring_help_flag = 1;
        }
    }

    if((point2_y > (uint8)(point1_y + 3)) &&
       (1 == ring_help_flag) &&
       (0 == SearchLine_Otsu_Ring_Flag))
    {
        SearchLine_Otsu_Ring_Element = 1;
        SearchLine_Otsu_Ring_Flag = 1;
        /* 主线入口按大圆环状态进入。 */
        SearchLine_Otsu_Ring_Size = 1;
        ImageStatus.Road_type = SEARCH_LINE_ROAD_LEFT_CIRQUE;
        /* 左环入口短响。 */
        buzzer_on();
        SearchLine_Ring_Beep_Stop_Tick = g_system_ticks + SEARCH_LINE_RING_FLAG1_BEEP_MS;
    }
}

static void Element_Judgment_Right_Rings(void)
{
    uint8 ring_ysite = 25;
    uint8 point1_y = 0;
    uint8 point2_y = 0;
    uint8 row = 0;
    uint8 ring_help_flag = 0;
    float straight_judge = 0.0f;

    straight_judge = Straight_Judge(1, 25, 45);
    if((SearchLine_Otsu_Left_Line > 7) ||
       (SearchLine_Otsu_Right_Line < 13) ||
       (SearchLine_Otsu_Offline_Row > 10) ||
       (straight_judge > 50.0f) ||
       (SearchLine_Otsu_White_Line > 15) ||
       (SEARCH_LINE_STATE_WHITE == ImageDeal[52].IsRightFind) ||
       (SEARCH_LINE_STATE_WHITE == ImageDeal[53].IsRightFind) ||
       (SEARCH_LINE_STATE_WHITE == ImageDeal[54].IsRightFind) ||
       (SEARCH_LINE_STATE_WHITE == ImageDeal[55].IsRightFind) ||
       (SEARCH_LINE_STATE_WHITE == ImageDeal[56].IsRightFind) ||
       (SEARCH_LINE_STATE_WHITE == ImageDeal[57].IsRightFind) ||
       (SEARCH_LINE_STATE_WHITE == ImageDeal[58].IsRightFind))
    {
        return;
    }

    for(row = 58; row > ring_ysite; row--)
    {
        if((int16)SearchLine_Otsu_Right_Boundary_First[row - 1] -
           (int16)SearchLine_Otsu_Right_Boundary_First[row] > 4)
        {
            point1_y = row;
            break;
        }
    }
    for(row = 58; row > ring_ysite; row--)
    {
        if((int16)SearchLine_Otsu_Right_Boundary[row] -
           (int16)SearchLine_Otsu_Right_Boundary[row + 1] > 4)
        {
            point2_y = row;
            break;
        }
    }

    for(row = point1_y; row > 10; row--)
    {
        if((SearchLine_Otsu_Right_Border[row + 6] > SearchLine_Otsu_Right_Border[row + 3]) &&
           (SearchLine_Otsu_Right_Border[row + 5] > SearchLine_Otsu_Right_Border[row + 3]) &&
           (SearchLine_Otsu_Right_Border[row + 3] < SearchLine_Otsu_Right_Border[row + 2]) &&
           (SearchLine_Otsu_Right_Border[row + 3] < SearchLine_Otsu_Right_Border[row + 1]))
        {
            ring_help_flag = 1;
            break;
        }
    }

    if((point2_y > (uint8)(point1_y + 3)) && (0 == ring_help_flag))
    {
        if(SearchLine_Otsu_Right_Line > 7)
        {
            ring_help_flag = 1;
        }
    }

    if((point2_y > (uint8)(point1_y + 3)) &&
       (1 == ring_help_flag) &&
       (0 == SearchLine_Otsu_Ring_Flag))
    {
        SearchLine_Otsu_Ring_Element = 2;
        SearchLine_Otsu_Ring_Flag = 1;
        /* 主线入口按大圆环状态进入。 */
        SearchLine_Otsu_Ring_Size = 1;
        ImageStatus.Road_type = SEARCH_LINE_ROAD_RIGHT_CIRQUE;
        /* 右环入口短响。 */
        buzzer_on();
        SearchLine_Ring_Beep_Stop_Tick = g_system_ticks + SEARCH_LINE_RING_FLAG1_BEEP_MS;
    }
}

static void Element_Handle_Left_Rings(void)
{
    int16 num = 0;
    int16 row = 0;
    int16 col = 0;
    int16 point_y = 0;
    int16 repair_x = 20;
    int16 repair_y = 7;
    int16 flag_x_1 = 0;
    int16 flag_y_1 = 0;
    int16 start_x = 0;
    int16 end_x = 0;
    int16 width = 0;
    float slope_rings = 0.0f;

    SearchLine_Otsu_Ring_Stage_Num = 0;
    SearchLine_Otsu_Ring_Point_Y = 0;
    SearchLine_Otsu_Ring_Straight_Judge_Tenth = -1;

    for(row = 55; row > 30; row--)
    {
        if(SEARCH_LINE_STATE_WHITE == ImageDeal[row].IsLeftFind)
        {
            num++;
        }
        if((SEARCH_LINE_STATE_WHITE == ImageDeal[row + 3].IsLeftFind) &&
           (SEARCH_LINE_STATE_WHITE == ImageDeal[row + 2].IsLeftFind) &&
           (SEARCH_LINE_STATE_WHITE == ImageDeal[row + 1].IsLeftFind) &&
           (SEARCH_LINE_STATE_FOUND == ImageDeal[row].IsLeftFind))
        {
            break;
        }
    }
    SearchLine_Otsu_Ring_Stage_Num = (uint16)num;

    if((1 == SearchLine_Otsu_Ring_Flag) && (num > 10))
    {
        SearchLine_Otsu_Ring_Flag = 2;
    }
    if((2 == SearchLine_Otsu_Ring_Flag) && (num < 8))
    {
        SearchLine_Otsu_Ring_Flag = 5;
    }
    if((5 == SearchLine_Otsu_Ring_Flag) && (SearchLine_Otsu_Right_Line > 15))
    {
        SearchLine_Otsu_Ring_Flag = 6;
    }
    if((6 == SearchLine_Otsu_Ring_Flag) && (SearchLine_Otsu_Right_Line < 4))
    {
        SearchLine_Otsu_Ring_Flag = 7;
    }

    if(7 == SearchLine_Otsu_Ring_Flag)
    {
        point_y = 0;
        for(row = 45; row > ((int16)SearchLine_Otsu_Offline_Row + 3); row--)
        {
            if((SearchLine_Otsu_Right_Border[row] <= SearchLine_Otsu_Right_Border[row + 1]) &&
               (SearchLine_Otsu_Right_Border[row] <= SearchLine_Otsu_Right_Border[row - 1]))
            {
                point_y = row;
                break;
            }
        }
        if(point_y > 22)
        {
            SearchLine_Otsu_Ring_Flag = 8;
        }
        SearchLine_Otsu_Ring_Point_Y = (uint16)point_y;
    }

    if(8 == SearchLine_Otsu_Ring_Flag)
    {
        if((SearchLine_Otsu_Right_Line < 9) &&
           (SearchLine_Otsu_Offline_Row < 10))
        {
            SearchLine_Otsu_Ring_Flag = 9;
        }
    }

    if(9 == SearchLine_Otsu_Ring_Flag)
    {
        num = 0;
        for(row = 40; row > 10; row--)
        {
            if(SEARCH_LINE_STATE_WHITE == ImageDeal[row].IsLeftFind)
            {
                num++;
            }
        }
        if(num < 5)
        {
            ImageStatus.Road_type = SEARCH_LINE_ROAD_NORMAL;
            SearchLine_Otsu_Ring_Flag = 0;
            SearchLine_Otsu_Ring_Element = 0;
            SearchLine_Otsu_Ring_Size = 0;
        }
    }

    if((1 == SearchLine_Otsu_Ring_Flag) ||
       (2 == SearchLine_Otsu_Ring_Flag) ||
       (3 == SearchLine_Otsu_Ring_Flag) ||
       (4 == SearchLine_Otsu_Ring_Flag))
    {
        for(row = 59; row > (int16)SearchLine_Otsu_Offline_Row; row--)
        {
            ImageDeal[row].Center =
                Limit((int16)SearchLine_Otsu_Right_Border[row] -
                      (int16)Half_Road_Wide[row],
                      0,
                      SEARCH_LINE_OTSU_W - 1);
        }
    }

    if((5 == SearchLine_Otsu_Ring_Flag) || (6 == SearchLine_Otsu_Ring_Flag))
    {
        for(row = 55; row > (int16)SearchLine_Otsu_Offline_Row; row--)
        {
            for(col = (int16)SearchLine_Otsu_Left_Border[row] + 1;
                col < (int16)SearchLine_Otsu_Right_Border[row] - 1;
                col++)
            {
                if((1 == SearchLine_Get_Otsu_Binary_Pixel(row, col)) &&
                   (0 == SearchLine_Get_Otsu_Binary_Pixel(row, col + 1)))
                {
                    flag_y_1 = row;
                    flag_x_1 = col;
                    slope_rings = (float)(79 - flag_x_1) / (float)(59 - flag_y_1);
                    break;
                }
            }
            if(0 != flag_y_1)
            {
                break;
            }
        }

        if(0 == flag_y_1)
        {
            for(row = (int16)SearchLine_Otsu_Offline_Row + 1; row < 30; row++)
            {
                if((SEARCH_LINE_STATE_FOUND == ImageDeal[row].IsLeftFind) &&
                   (SEARCH_LINE_STATE_FOUND == ImageDeal[row + 1].IsLeftFind) &&
                   (SEARCH_LINE_STATE_WHITE == ImageDeal[row + 2].IsLeftFind) &&
                   (SearchLine_Otsu_Left_Border[row] > SearchLine_Otsu_Left_Border[row + 2] + 10 ||
                    SearchLine_Otsu_Left_Border[row] + 10 < SearchLine_Otsu_Left_Border[row + 2]))
                {
                    flag_y_1 = row;
                    flag_x_1 = SearchLine_Otsu_Left_Border[flag_y_1];
                    SearchLine_Otsu_Offline_Row = (uint8)row;
                    slope_rings = (float)(79 - flag_x_1) / (float)(59 - flag_y_1);
                    break;
                }
            }
        }

        if(0 != flag_y_1)
        {
            for(row = flag_y_1; row < SEARCH_LINE_OTSU_H; row++)
            {
                SearchLine_Otsu_Right_Border[row] =
                    (uint8)Limit((int16)((float)flag_x_1 +
                                                          slope_rings * (float)(row - flag_y_1)),
                                                  0,
                                                  SEARCH_LINE_OTSU_W - 1);
                ImageDeal[row].Center =
                    Limit(((int16)SearchLine_Otsu_Right_Border[row] +
                           (int16)SearchLine_Otsu_Left_Border[row]) / 2,
                          4,
                          SEARCH_LINE_OTSU_W - 1);
            }

            SearchLine_Otsu_Right_Border[flag_y_1] =
                (uint8)Limit(flag_x_1, 0, SEARCH_LINE_OTSU_W - 1);

            for(row = flag_y_1 - 1; row > 10; row--)
            {
                width = 0;
                start_x = (int16)SearchLine_Otsu_Right_Border[row + 1] - 10;
                end_x = (int16)SearchLine_Otsu_Right_Border[row + 1] + 2;
                start_x = SearchLine_Clamp_Otsu_Search_Col(start_x);
                end_x = SearchLine_Clamp_Otsu_Search_Col(end_x);
                for(col = start_x; col <= end_x; col++)
                {
                    if((1 == SearchLine_Get_Otsu_Binary_Pixel(row, col)) &&
                       (0 == SearchLine_Get_Otsu_Binary_Pixel(row, col + 1)))
                    {
                        SearchLine_Otsu_Right_Border[row] =
                            (uint8)Limit(col, 0, SEARCH_LINE_OTSU_W - 1);
                        ImageDeal[row].Center =
                            Limit(((int16)SearchLine_Otsu_Right_Border[row] +
                                   (int16)SearchLine_Otsu_Left_Border[row]) / 2,
                                  4,
                                  SEARCH_LINE_OTSU_W - 1);
                        width = (int16)SearchLine_Otsu_Right_Border[row] -
                                (int16)SearchLine_Otsu_Left_Border[row];
                        break;
                    }
                }

                if((width > 8) &&
                   (SearchLine_Otsu_Right_Border[row] < SearchLine_Otsu_Right_Border[row + 2]))
                {
                    continue;
                }
                else
                {
                    SearchLine_Otsu_Offline_Row = (uint8)(row + 2);
                    break;
                }
            }
        }
    }

    if(6 == SearchLine_Otsu_Ring_Flag)
    {
        for(row = 57; row > (int16)SearchLine_Otsu_Offline_Row; row--)
        {
            ImageDeal[row].Center = 15;
        }
    }

    if((8 == SearchLine_Otsu_Ring_Flag) && (1 == SearchLine_Otsu_Ring_Size))
    {
        for(row = 40; row > 5; row--)
        {
            if((1 == SearchLine_Get_Otsu_Binary_Pixel(row, 28)) &&
               (0 == SearchLine_Get_Otsu_Binary_Pixel(row - 1, 28)))
            {
                repair_x = 28;
                repair_y = row - 1;
                SearchLine_Otsu_Offline_Row = (uint8)(row + 1);
                break;
            }
        }
        for(row = 57; row > (repair_y - 3); row--)
        {
            SearchLine_Otsu_Right_Border[row] =
                (uint8)Limit((((int16)SearchLine_Otsu_Right_Border[58] - repair_x) *
                                               (row - 58)) /
                                              (58 - repair_y) +
                                              (int16)SearchLine_Otsu_Right_Border[58],
                                              0,
                                              SEARCH_LINE_OTSU_W - 1);
            ImageDeal[row].Center =
                ((int16)SearchLine_Otsu_Right_Border[row] +
                 (int16)SearchLine_Otsu_Left_Border[row]) / 2;
        }
    }

    if((9 == SearchLine_Otsu_Ring_Flag) || (10 == SearchLine_Otsu_Ring_Flag))
    {
        for(row = 59; row > (int16)SearchLine_Otsu_Offline_Row; row--)
        {
            ImageDeal[row].Center =
                Limit((int16)SearchLine_Otsu_Right_Border[row] -
                      (int16)Half_Road_Wide[row],
                      0,
                      SEARCH_LINE_OTSU_W - 1);
        }
    }
}

static void Element_Handle_Right_Rings(void)
{
    int16 num = 0;
    int16 row = 0;
    int16 col = 0;
    int16 point_y = 0;
    int16 repair_x = 20;
    int16 repair_y = 7;
    int16 flag_x_1 = 0;
    int16 flag_y_1 = 0;
    int16 start_x = 0;
    int16 end_x = 0;
    int16 width = 0;
    float slope_right_rings = 0.0f;
    float straight_judge = 0.0f;

    SearchLine_Otsu_Ring_Stage_Num = 0;
    SearchLine_Otsu_Ring_Point_Y = 0;
    SearchLine_Otsu_Ring_Straight_Judge_Tenth = -1;

    /* 统计右侧连续丢线段长度。 */
    for(row = 55; row > 30; row--)
    {
        if(SEARCH_LINE_STATE_WHITE == ImageDeal[row].IsRightFind)
        {
            num++;
        }
        if((SEARCH_LINE_STATE_WHITE == ImageDeal[row + 3].IsRightFind) &&
           (SEARCH_LINE_STATE_WHITE == ImageDeal[row + 2].IsRightFind) &&
           (SEARCH_LINE_STATE_WHITE == ImageDeal[row + 1].IsRightFind) &&
           (SEARCH_LINE_STATE_FOUND == ImageDeal[row].IsRightFind))
        {
            break;
        }
    }
    SearchLine_Otsu_Ring_Stage_Num = (uint16)num;

    /* 准备进环。右侧连续丢线足够长，说明开始贴着右大环外沿走。 */
    if((1 == SearchLine_Otsu_Ring_Flag) && (num > 10))
    {
        SearchLine_Otsu_Ring_Flag = 2;
    }
    /* 贴边段结束。右侧连续丢线回落后，开始切到补左边界的绕环阶段。 */
    if((2 == SearchLine_Otsu_Ring_Flag) && (num < 12))
    {
        SearchLine_Otsu_Ring_Flag = 5;
        /* 右大环真正切到环内补线时再响一下，方便区分入口判到和入环稳定。 */
        buzzer_on();
        SearchLine_Ring_Beep_Stop_Tick = g_system_ticks + SEARCH_LINE_RING_FLAG1_BEEP_MS;
    }
    /* 左边重新露出较多有效边界，说明已经绕到环内后半段。 */
    if((5 == SearchLine_Otsu_Ring_Flag) && (SearchLine_Otsu_Left_Line > 15))
    {
        SearchLine_Otsu_Ring_Flag = 6;
    }
    /* 左边再次变短，说明车头基本转回来，准备找出环点。 */
    if((6 == SearchLine_Otsu_Ring_Flag) && (SearchLine_Otsu_Left_Line < 4))
    {
        SearchLine_Otsu_Ring_Flag = 7;
    }

    /* 一号右大环这里按左边界局部峰值找出环点，不再只看相邻两行。 */
    if(7 == SearchLine_Otsu_Ring_Flag)
    {
        point_y = 0;
        for(row = 45; row > ((int16)SearchLine_Otsu_Offline_Row + 3); row--)
        {
            if((SearchLine_Otsu_Left_Border[row] >= SearchLine_Otsu_Left_Border[row + 2]) &&
               (SearchLine_Otsu_Left_Border[row] >= SearchLine_Otsu_Left_Border[row - 2]) &&
               (SearchLine_Otsu_Left_Border[row] >= SearchLine_Otsu_Left_Border[row + 1]) &&
               (SearchLine_Otsu_Left_Border[row] >= SearchLine_Otsu_Left_Border[row - 1]) &&
               (SearchLine_Otsu_Left_Border[row] >= SearchLine_Otsu_Left_Border[row + 4]) &&
               (SearchLine_Otsu_Left_Border[row] >= SearchLine_Otsu_Left_Border[row - 4]))
            {
                point_y = row;
                break;
            }
        }
        if(point_y > 22)
        {
            SearchLine_Otsu_Ring_Flag = 8;
        }
        SearchLine_Otsu_Ring_Point_Y = (uint16)point_y;
    }

    /* 出环确认。左边界足够接近直线，且前方有效视野恢复后，切到退环阶段。 */
    if(8 == SearchLine_Otsu_Ring_Flag)
    {
        straight_judge = Straight_Judge(1,
                                                        (uint8)Limit((int16)SearchLine_Otsu_Offline_Row + 10,
                                                                                      0,
                                                                                      SEARCH_LINE_OTSU_H - 1),
                                                        45);
        SearchLine_Otsu_Ring_Straight_Judge_Tenth = (int16)(straight_judge * 10.0f + 0.5f);
        if((straight_judge < 1.0f) &&
           (SearchLine_Otsu_Left_Line < 9) &&
           (SearchLine_Otsu_Offline_Row < 20))
        {
            SearchLine_Otsu_Ring_Flag = 9;
        }
    }

    /* 退环完成后，等右侧白边计数回落，再彻底清空环岛状态。 */
    if(9 == SearchLine_Otsu_Ring_Flag)
    {
        num = 0;
        for(row = 40; row > 10; row--)
        {
            if(SEARCH_LINE_STATE_WHITE == ImageDeal[row].IsRightFind)
            {
                num++;
            }
        }
        if(num < 5)
        {
            ImageStatus.Road_type = SEARCH_LINE_ROAD_NORMAL;
            SearchLine_Otsu_Ring_Flag = 0;
            SearchLine_Otsu_Ring_Element = 0;
            SearchLine_Otsu_Ring_Size = 0;
            /* 右大环出环清状态时响一下，方便确认已经彻底回到普通赛道。 */
            buzzer_on();
            SearchLine_Ring_Beep_Stop_Tick = g_system_ticks + SEARCH_LINE_RING_FLAG1_BEEP_MS;
        }
    }

    /* 1/2/3/4 阶段按左边界推中线。 */
    if((1 == SearchLine_Otsu_Ring_Flag) ||
       (2 == SearchLine_Otsu_Ring_Flag) ||
       (3 == SearchLine_Otsu_Ring_Flag) ||
       (4 == SearchLine_Otsu_Ring_Flag))
    {
        for(row = 59; row > (int16)SearchLine_Otsu_Offline_Row; row--)
        {
            ImageDeal[row].Center =
                Limit((int16)SearchLine_Otsu_Left_Border[row] +
                      (int16)Half_Road_Wide[row],
                      0,
                      SEARCH_LINE_OTSU_W - 1);
        }
    }

    /* 5/6 阶段用图内白黑跳变重建左边界，核心目的是把环内缺失的左边补出来。 */
    if((5 == SearchLine_Otsu_Ring_Flag) || (6 == SearchLine_Otsu_Ring_Flag))
    {
        for(row = 55; row > (int16)SearchLine_Otsu_Offline_Row; row--)
        {
            for(col = (int16)SearchLine_Otsu_Left_Border[row] + 1;
                col < (int16)SearchLine_Otsu_Right_Border[row] - 1;
                col++)
            {
                if((1 == SearchLine_Get_Otsu_Binary_Pixel(row, col)) &&
                   (0 == SearchLine_Get_Otsu_Binary_Pixel(row, col + 1)))
                {
                    flag_y_1 = row;
                    flag_x_1 = col;
                    slope_right_rings = (float)(0 - flag_x_1) / (float)(59 - flag_y_1);
                    break;
                }
            }
            if(0 != flag_y_1)
            {
                break;
            }
        }

        if(0 == flag_y_1)
        {
            for(row = (int16)SearchLine_Otsu_Offline_Row + 5; row < 30; row++)
            {
                if((SEARCH_LINE_STATE_FOUND == ImageDeal[row].IsRightFind) &&
                   (SEARCH_LINE_STATE_FOUND == ImageDeal[row + 1].IsRightFind) &&
                   (SEARCH_LINE_STATE_WHITE == ImageDeal[row + 2].IsRightFind) &&
                   (SearchLine_Otsu_Right_Border[row] > SearchLine_Otsu_Right_Border[row + 2] + 10 ||
                    SearchLine_Otsu_Right_Border[row] + 10 < SearchLine_Otsu_Right_Border[row + 2]))
                {
                    flag_y_1 = row;
                    flag_x_1 = SearchLine_Otsu_Right_Border[flag_y_1];
                    SearchLine_Otsu_Offline_Row = (uint8)row;
                    slope_right_rings = (float)(0 - flag_x_1) / (float)(59 - flag_y_1);
                    break;
                }
            }
        }

        if(0 != flag_y_1)
        {
            for(row = flag_y_1; row < 58; row++)
            {
                SearchLine_Otsu_Left_Border[row] =
                    (uint8)Limit((int16)((float)flag_x_1 +
                                                          slope_right_rings * (float)(row - flag_y_1)),
                                                  0,
                                                  SEARCH_LINE_OTSU_W - 1);
                ImageDeal[row].Center =
                    Limit(((int16)SearchLine_Otsu_Left_Border[row] +
                           (int16)SearchLine_Otsu_Right_Border[row]) / 2,
                          0,
                          SEARCH_LINE_OTSU_W - 1);
            }

            SearchLine_Otsu_Left_Border[flag_y_1] =
                (uint8)Limit(flag_x_1, 0, SEARCH_LINE_OTSU_W - 1);

            for(row = flag_y_1 - 1; row > 10; row--)
            {
                width = 0;
                start_x = (int16)SearchLine_Otsu_Left_Border[row + 1] + 8;
                end_x = (int16)SearchLine_Otsu_Left_Border[row + 1] - 4;
                start_x = SearchLine_Clamp_Otsu_Search_Col(start_x);
                end_x = SearchLine_Clamp_Otsu_Search_Col(end_x);
                for(col = start_x; col >= end_x; col--)
                {
                    if((1 == SearchLine_Get_Otsu_Binary_Pixel(row, col)) &&
                       (0 == SearchLine_Get_Otsu_Binary_Pixel(row, col - 1)))
                    {
                        SearchLine_Otsu_Left_Border[row] =
                            (uint8)Limit(col, 0, SEARCH_LINE_OTSU_W - 1);
                        ImageDeal[row].Center =
                            Limit(((int16)SearchLine_Otsu_Left_Border[row] +
                                   (int16)SearchLine_Otsu_Right_Border[row]) / 2,
                                  5,
                                  SEARCH_LINE_OTSU_W - 1);
                        width = (int16)SearchLine_Otsu_Right_Border[row] -
                                (int16)SearchLine_Otsu_Left_Border[row];
                        break;
                    }
                }

                if((width > 8) &&
                   (SearchLine_Otsu_Left_Border[row] > SearchLine_Otsu_Left_Border[row + 2]))
                {
                    continue;
                }
                else
                {
                    SearchLine_Otsu_Offline_Row = (uint8)(row + 2);
                    break;
                }
            }
        }
    }

    /* 6 阶段沿用补线后的中线。 */

    /* 8 阶段在固定列重新找左边修补点，把左边界拉成一条直线，准备退出圆环。 */
    if(8 == SearchLine_Otsu_Ring_Flag)
    {
        for(row = 40; row > 8; row--)
        {
            if((1 == SearchLine_Get_Otsu_Binary_Pixel(row, 28)) &&
               (0 == SearchLine_Get_Otsu_Binary_Pixel(row - 1, 28)))
            {
                repair_x = 28;
                repair_y = row - 1;
                SearchLine_Otsu_Offline_Row = (uint8)(row + 1);
                break;
            }
        }
        for(row = 57; row > (repair_y - 3); row--)
        {
            SearchLine_Otsu_Left_Border[row] =
                (uint8)Limit((((int16)SearchLine_Otsu_Left_Border[58] - repair_x) *
                                               (row - 58)) /
                                              (58 - repair_y) +
                                              (int16)SearchLine_Otsu_Left_Border[58],
                                              0,
                                              SEARCH_LINE_OTSU_W - 1);
            ImageDeal[row].Center =
                ((int16)SearchLine_Otsu_Left_Border[row] +
                 (int16)SearchLine_Otsu_Right_Border[row]) / 2;
        }
    }

    /* 9 阶段恢复普通赛道口径，中线重新按左边界加半路宽计算。 */
    if(9 == SearchLine_Otsu_Ring_Flag)
    {
        for(row = 59; row > (int16)SearchLine_Otsu_Offline_Row; row--)
        {
            ImageDeal[row].Center =
                Limit((int16)SearchLine_Otsu_Left_Border[row] +
                      (int16)Half_Road_Wide[row],
                      0,
                      SEARCH_LINE_OTSU_W - 1);
        }
    }
}

static void Element_Handle(void)
{
    if(1 == SearchLine_Otsu_Ring_Element)
    {
        Element_Handle_Left_Rings();
    }
    else if(2 == SearchLine_Otsu_Ring_Element)
    {
        Element_Handle_Right_Rings();
    }
}

/* 元素判断。 */
static void Element_Test(void)
{
    SearchLine_Otsu_Cirque_Left_Count =
        Cirque_Or_Cross(1, SearchLine_Otsu_Left_Line);
    SearchLine_Otsu_Cirque_Right_Count =
        Cirque_Or_Cross(2, SearchLine_Otsu_Right_Line);

    /* 圆环判断。 */
    if((SearchLine_Otsu_Offline_Row < 5) &&
       (SearchLine_Otsu_White_Line < 3) &&
       (SEARCH_LINE_STATE_WHITE != ImageDeal[52].IsLeftFind) &&
       (SEARCH_LINE_STATE_WHITE != ImageDeal[53].IsLeftFind) &&
       (SEARCH_LINE_STATE_WHITE != ImageDeal[54].IsLeftFind) &&
       (SEARCH_LINE_STATE_WHITE != ImageDeal[55].IsLeftFind) &&
       (SEARCH_LINE_STATE_WHITE != ImageDeal[56].IsLeftFind) &&
       (SEARCH_LINE_STATE_WHITE != ImageDeal[57].IsLeftFind) &&
       (SEARCH_LINE_STATE_WHITE != ImageDeal[58].IsLeftFind))
    {
        /* 左圆环判断。 */
        if((SearchLine_Otsu_Right_Line < 2) &&
           (SearchLine_Otsu_Left_Line > 13) &&
           (SearchLine_Otsu_Cirque_Left_Count > 70))
        {
            Element_Judgment_Left_Rings();
        }
    }

    /* 右圆环入口先过外层筛选。 */
    if((ImageStatus.Road_type != SEARCH_LINE_ROAD_BARN_IN) &&
       (ImageStatus.Road_type != SEARCH_LINE_ROAD_CROSS_TRUE) &&
       (ImageStatus.Road_type != SEARCH_LINE_ROAD_BARN_OUT))
    {
        Element_Judgment_Right_Rings();
    }
}

/* 舵机位置式 PD 预览。 */
static void SearchLine_Update_Otsu_SteerPreview(void)
{
    int16 abs_offset = 0;
    int16 command_angle = 0;
    float i_error = 0.0f;
    float steer_err = 0.0f;
    float pwm = 0.0f;
    float center_angle = 0.0f;
    float min_angle = 0.0f;
    float max_angle = 0.0f;
    float angle = 0.0f;
    float steer_p = 0.0f;
    float steer_d = 0.0f;

    SearchLine_Otsu_Steer_Offset = (int16)SearchLine_Otsu_Det_True - ImageSensorMid;
    i_error = (float)SearchLine_Otsu_Steer_Offset;
    steer_p = (float)SearchLine_Otsu_Steer_P_Tenth / 10.0f;
    steer_d = (float)SearchLine_Otsu_Steer_D_Tenth / 10.0f;
    steer_err = steer_p * i_error +
                steer_d * (i_error - SearchLine_Otsu_Steer_Last_Error);

    abs_offset = SearchLine_Otsu_Steer_Offset;
    if(abs_offset < 0)
    {
        abs_offset = (int16)(-abs_offset);
    }
    if(abs_offset < 3)
    {
        i_error = 0.3f * i_error;
    }
    if(abs_offset > 15)
    {
        i_error = 1.2f * i_error;
    }
    SearchLine_Otsu_Steer_Last_Error = i_error;

    pwm = SEARCH_LINE_STEER_REF_MIDDLE_DUTY - steer_err;
    if(pwm > SEARCH_LINE_STEER_REF_LEFT_DUTY)
    {
        pwm = SEARCH_LINE_STEER_REF_LEFT_DUTY;
    }
    if(pwm < SEARCH_LINE_STEER_REF_RIGHT_DUTY)
    {
        pwm = SEARCH_LINE_STEER_REF_RIGHT_DUTY;
    }

    min_angle = (float)car_servo_get_min_angle();
    max_angle = (float)car_servo_get_max_angle();
    center_angle = (float)CAR_SERVO_CENTER_ANGLE;
    if(pwm >= SEARCH_LINE_STEER_REF_MIDDLE_DUTY)
    {
        /* 当前前轮左右方向与参考舵机占空比方向相反，这里按本工程角度方向重映射。 */
        angle = center_angle +
                (pwm - SEARCH_LINE_STEER_REF_MIDDLE_DUTY) *
                (max_angle - center_angle) /
                (SEARCH_LINE_STEER_REF_LEFT_DUTY - SEARCH_LINE_STEER_REF_MIDDLE_DUTY);
    }
    else
    {
        angle = center_angle -
                (SEARCH_LINE_STEER_REF_MIDDLE_DUTY - pwm) *
                (center_angle - min_angle) /
                (SEARCH_LINE_STEER_REF_MIDDLE_DUTY - SEARCH_LINE_STEER_REF_RIGHT_DUTY);
    }

    command_angle = (int16)(angle + 0.5f);
    SearchLine_Otsu_Steer_Command = (uint8)Limit(command_angle,
                                                                   (int16)min_angle,
                                                                   (int16)max_angle);
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
    uint16 width = col;
    uint16 height = row;
    int pixelCount[256];
    float pixelPro[256];
    int i = 0;
    int j = 0;
    int pixelSum = width * height;
    uint8 threshold = 0;
    uint8* data = image;  /* 指向像素数据的指针。 */
    uint32 gray_sum = 0;
    float w0 = 0.0f;
    float w1 = 0.0f;
    float u0tmp = 0.0f;
    float u1tmp = 0.0f;
    float u0 = 0.0f;
    float u1 = 0.0f;
    float u = 0.0f;
    float deltaTmp = 0.0f;
    float deltaMax = 0.0f;

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
            pixelCount[(int)data[i * width + j]]++;  /* 将当前点的像素值作为计数数组下标。 */
            gray_sum += (int)data[i * width + j];    /* 灰度值总和。 */
        }
    }

    /* 计算每个像素值在整幅图像中的比例。 */
    for(i = 0; i < 256; i++)
    {
        pixelPro[i] = (float)pixelCount[i] / pixelSum;
    }

    /* 遍历灰度级 [0, pixel_threshold) 。 */
    for(j = 0; j < (int)pixel_threshold; j++)
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

        u1tmp = (float)gray_sum / pixelSum - u0tmp;
        u0 = u0tmp / w0;    /* 背景平均灰度。 */
        u1 = u1tmp / w1;    /* 前景平均灰度。 */
        u = u0tmp + u1tmp;  /* 全局平均灰度。 */
        deltaTmp = w0 * pow((u0 - u), 2) + w1 * pow((u1 - u), 2);
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

    if(threshold < SEARCH_LINE_OTSU_THRESHOLD_MIN)
    {
        threshold = SEARCH_LINE_OTSU_THRESHOLD_MIN;
    }

    return threshold;
}

/* 图像二值化。 */
void Get01change_dajin(void)
{
    uint8 i = 0;
    uint8 j = 0;
    uint8 thre = 0;

    ImageStatus.Threshold = Threshold_deal(Image_Use[0], LCDW, LCDH, ImageStatus.Threshold_detach);
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

// 图像处理
void ImageProcess(void)
{
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

    Get01change_dajin();  //图像二值化
    DrawLinesFirst();     //绘制底边
    DrawLinesProcess();   //搜边线

    Search_Border_OTSU(Pixle, LCDH, LCDW, LCDH - 2);//58行位底行
    
    Element_Test();       /* 元素判断。 */
    DrawExtensionLine();  /* 绘制延长线，补线。 */
    RouteFilter();        /* 中线滤波平滑。 */

    /* 当前工程未接斑马线与堵转积分，这里直接进入元素执行和直道判定。 */
    /* 当前工程只保留直道标志计算，不在这里直接改后轮速度。 */
    Element_Handle();     /* 环岛执行。 */
    straight_speed();     /* 直道判定。 */
    GetDet();             /* 获取动态前瞻并计算图像偏差。 */

    /* 当前工程舵机已经实接，这里继续把图像偏差映射成业务角度。 */
    SearchLine_Update_Otsu_SteerPreview();
}

/* 执行一帧搜线处理。 */
void SearchLine_Process(void)
{
    if((0 != SearchLine_Ring_Beep_Stop_Tick) &&
       (g_system_ticks >= SearchLine_Ring_Beep_Stop_Tick))
    {
        buzzer_off();
        SearchLine_Ring_Beep_Stop_Tick = 0;
    }
    gpio_set_level(IO_P52, 0);
    ImageProcess();
    gpio_set_level(IO_P52, 1);
}

uint8 SearchLine_GetOtsuThreshold(void)
{
    return ImageStatus.Threshold;
}

uint8 SearchLine_GetSteerCommand(void)
{
    return SearchLine_Otsu_Steer_Command;
}

uint8 SearchLine_GetStraightAcc(void)
{
    return ImageStatus.straight_acc;
}

uint8 SearchLine_GetDetTrue(void)
{
    return SearchLine_Otsu_Det_True;
}

uint8 SearchLine_GetLeftLine(void)
{
    return SearchLine_Otsu_Left_Line;
}

uint8 SearchLine_GetRightLine(void)
{
    return SearchLine_Otsu_Right_Line;
}

void SearchLine_SetSteerPdTenth(uint16 p_tenth, uint16 d_tenth)
{
    SearchLine_Otsu_Steer_P_Tenth = p_tenth;
    SearchLine_Otsu_Steer_D_Tenth = d_tenth;
}

static void SearchLine_DrawPreviewLabels(void)
{
    uint16 preview_h = (uint16)(LCDH * 2);

    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_show_string(0, (uint16)(preview_h + 4), "yu zhi");
    ips200_show_string(0, (uint16)(preview_h + 20), "qian zhan pian cha");
    ips200_show_string(0, (uint16)(preview_h + 36), "duo ji jiao du");

    /* 圆环条件调试量。 */
    ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
    ips200_show_string(0, (uint16)(preview_h + 52), "yuan huan");
    ips200_set_color(RGB565_CYAN, RGB565_BLACK);
    ips200_show_string(0, (uint16)(preview_h + 68), "ru huan");
    ips200_set_color(RGB565_GREEN, RGB565_BLACK);
    ips200_show_string(0, (uint16)(preview_h + 84), "zuo pan");
    ips200_set_color(RGB565_MAGENTA, RGB565_BLACK);
    ips200_show_string(0, (uint16)(preview_h + 100), "you pan");
    ips200_set_color(RGB565_BLUE, RGB565_BLACK);
    ips200_show_string(0, (uint16)(preview_h + 116), "jie duan");
    SearchLine_Preview_Label_Ready = 1;
}

static void SearchLine_FormatThresholdText(char *text, uint8 threshold)
{
    if(0 == text)
    {
        return;
    }

    text[0] = (threshold >= 100U) ? (char)('0' + threshold / 100U) : ' ';
    text[1] = (threshold >= 10U) ? (char)('0' + (threshold / 10U) % 10U) : ' ';
    text[2] = (char)('0' + threshold % 10U);
    text[3] = '\0';
}

/* 重置相机页预览缓存。 */
void SearchLine_ResetPreviewOverlay(void)
{
    /* 相机页切黑底后重置标签和数值缓存。 */
    SearchLine_Preview_Label_Ready = 0;
    SearchLine_Preview_Last_Threshold = 0xFF;
    SearchLine_Preview_Last_Offset = 32767;
    SearchLine_Preview_Last_Command = 0xFF;
    SearchLine_Preview_Last_Ring_Element = 0xFF;
    SearchLine_Preview_Last_Ring_Flag = 0xFF;
    SearchLine_Preview_Last_Ring_Size = 0xFF;
    SearchLine_Preview_Last_Offline_Row = 0xFF;
    SearchLine_Preview_Last_White_Line = 0xFF;
    SearchLine_Preview_Last_Cirque_Left_Count = 0xFF;
    SearchLine_Preview_Last_Cirque_Right_Count = 0xFF;
    SearchLine_Preview_Last_Ring_Left_Line = 0xFF;
    SearchLine_Preview_Last_Ring_Right_Line = 0xFF;
    SearchLine_Preview_Last_Ring_Left_Line_RightPanel = 0xFF;
    SearchLine_Preview_Last_Ring_Right_Line_RightPanel = 0xFF;
    SearchLine_Preview_Last_Ring_Stage_Num = 0xFFFF;
    SearchLine_Preview_Last_Ring_Point_Y = 0xFFFF;
    SearchLine_Preview_Last_Ring_Straight_Judge_Tenth = 32767;
}

static void SearchLine_DrawPreview(uint8 show_raw)
{
    char threshold_text[4];
    char offset_text[4];
    char command_text[4];
    char ring_text[12];
    char gate_text[16];
    char left_text[16];
    char right_text[16];
    char stage_text[16];
    uint16 x = 0;
    uint16 y = 0;
    uint8 row = 0;
    uint8 left_col = 0;
    uint8 right_col = 0;
    uint8 center_col = 0;
    uint8 boundary_col = 0;
    uint16 offset_abs = 0;
    uint8 command_value = 0;
    uint16 preview_w = (uint16)(LCDW * 2);
    uint16 preview_h = (uint16)(LCDH * 2);

    /* 相机页直接显示 80x60 处理图，灰度和二值都与主算法共用同一口径。 */
    if(show_raw)
    {
        ips200_show_gray_image(0,
                               0,
                               Image_Use[0],
                               LCDW,
                               LCDH,
                               preview_w,
                               preview_h,
                               0);
    }
    else
    {
        ips200_show_gray_image(0,
                               0,
                               Pixle[0],
                               LCDW,
                               LCDH,
                               preview_w,
                               preview_h,
                               1);
    }

    /* 相机页底部显示当前 OTSU 阈值。 */
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    if(!SearchLine_Preview_Label_Ready)
    {
        SearchLine_DrawPreviewLabels();
    }
    if(SearchLine_Preview_Last_Threshold != ImageStatus.Threshold)
    {
        SearchLine_FormatThresholdText(threshold_text, ImageStatus.Threshold);
        ips200_show_string(preview_w, (uint16)(preview_h + 4), threshold_text);
        SearchLine_Preview_Last_Threshold = ImageStatus.Threshold;
    }

    if(SearchLine_Otsu_Steer_Offset < 0)
    {
        offset_text[0] = '-';
        offset_abs = (uint16)(-SearchLine_Otsu_Steer_Offset);
    }
    else
    {
        offset_text[0] = '+';
        offset_abs = (uint16)SearchLine_Otsu_Steer_Offset;
    }
    offset_text[1] = (char)('0' + (offset_abs / 10U) % 10U);
    offset_text[2] = (char)('0' + offset_abs % 10U);
    offset_text[3] = '\0';

    command_value = SearchLine_Otsu_Steer_Command;
    command_text[0] = (char)('0' + command_value / 100U);
    command_text[1] = (char)('0' + (command_value / 10U) % 10U);
    command_text[2] = (char)('0' + command_value % 10U);
    command_text[3] = '\0';

    if(SearchLine_Preview_Last_Offset != SearchLine_Otsu_Steer_Offset)
    {
        ips200_show_string(preview_w, (uint16)(preview_h + 20), offset_text);
        SearchLine_Preview_Last_Offset = SearchLine_Otsu_Steer_Offset;
    }
    if(SearchLine_Preview_Last_Command != command_value)
    {
        ips200_show_string(preview_w, (uint16)(preview_h + 36), command_text);
        SearchLine_Preview_Last_Command = command_value;
    }

    if(1 == SearchLine_Otsu_Ring_Element)
    {
        ring_text[0] = 'z';
        ring_text[1] = 'u';
        ring_text[2] = 'o';
    }
    else if(2 == SearchLine_Otsu_Ring_Element)
    {
        ring_text[0] = 'y';
        ring_text[1] = 'o';
        ring_text[2] = 'u';
    }
    else
    {
        ring_text[0] = 'w';
        ring_text[1] = 'u';
        ring_text[2] = ' ';
    }
    ring_text[3] = ' ';
    ring_text[4] = (char)('0' + (SearchLine_Otsu_Ring_Flag % 10U));
    ring_text[5] = ' ';
    if(1 == SearchLine_Otsu_Ring_Size)
    {
        ring_text[6] = 'd';
        ring_text[7] = 'a';
        ring_text[8] = '\0';
    }
    else if(2 == SearchLine_Otsu_Ring_Size)
    {
        ring_text[6] = 'x';
        ring_text[7] = 'i';
        ring_text[8] = 'a';
        ring_text[9] = 'o';
        ring_text[10] = '\0';
    }
    else
    {
        ring_text[6] = '-';
        ring_text[7] = '\0';
    }

    gate_text[0] = 'o';
    gate_text[1] = (char)('0' + (SearchLine_Otsu_Offline_Row / 10U) % 10U);
    gate_text[2] = (char)('0' + SearchLine_Otsu_Offline_Row % 10U);
    gate_text[3] = ' ';
    gate_text[4] = 'w';
    gate_text[5] = (char)('0' + (SearchLine_Otsu_White_Line / 10U) % 10U);
    gate_text[6] = (char)('0' + SearchLine_Otsu_White_Line % 10U);
    gate_text[7] = '\0';

    left_text[0] = 'r';
    left_text[1] = (char)('0' + (SearchLine_Otsu_Right_Line / 10U) % 10U);
    left_text[2] = (char)('0' + SearchLine_Otsu_Right_Line % 10U);
    left_text[3] = ' ';
    left_text[4] = 'l';
    left_text[5] = (char)('0' + (SearchLine_Otsu_Left_Line / 10U) % 10U);
    left_text[6] = (char)('0' + SearchLine_Otsu_Left_Line % 10U);
    left_text[7] = ' ';
    left_text[8] = 'c';
    left_text[9] = (char)('0' + (SearchLine_Otsu_Cirque_Left_Count / 100U) % 10U);
    left_text[10] = (char)('0' + (SearchLine_Otsu_Cirque_Left_Count / 10U) % 10U);
    left_text[11] = (char)('0' + SearchLine_Otsu_Cirque_Left_Count % 10U);
    left_text[12] = '\0';

    right_text[0] = 'l';
    right_text[1] = (char)('0' + (SearchLine_Otsu_Left_Line / 10U) % 10U);
    right_text[2] = (char)('0' + SearchLine_Otsu_Left_Line % 10U);
    right_text[3] = ' ';
    right_text[4] = 'r';
    right_text[5] = (char)('0' + (SearchLine_Otsu_Right_Line / 10U) % 10U);
    right_text[6] = (char)('0' + SearchLine_Otsu_Right_Line % 10U);
    right_text[7] = ' ';
    right_text[8] = 'c';
    right_text[9] = (char)('0' + (SearchLine_Otsu_Cirque_Right_Count / 100U) % 10U);
    right_text[10] = (char)('0' + (SearchLine_Otsu_Cirque_Right_Count / 10U) % 10U);
    right_text[11] = (char)('0' + SearchLine_Otsu_Cirque_Right_Count % 10U);
    right_text[12] = '\0';

    stage_text[0] = 'n';
    stage_text[1] = (char)('0' + (SearchLine_Otsu_Ring_Stage_Num / 10U) % 10U);
    stage_text[2] = (char)('0' + SearchLine_Otsu_Ring_Stage_Num % 10U);
    stage_text[3] = ' ';
    stage_text[4] = 'p';
    stage_text[5] = (char)('0' + (SearchLine_Otsu_Ring_Point_Y / 10U) % 10U);
    stage_text[6] = (char)('0' + SearchLine_Otsu_Ring_Point_Y % 10U);
    stage_text[7] = ' ';
    stage_text[8] = 's';
    if(SearchLine_Otsu_Ring_Straight_Judge_Tenth >= 0)
    {
        stage_text[9] = (char)('0' + (SearchLine_Otsu_Ring_Straight_Judge_Tenth / 10) % 10);
        stage_text[10] = '.';
        stage_text[11] = (char)('0' + SearchLine_Otsu_Ring_Straight_Judge_Tenth % 10);
        stage_text[12] = '\0';
    }
    else
    {
        stage_text[9] = '-';
        stage_text[10] = '-';
        stage_text[11] = '\0';
    }

    ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
    if((SearchLine_Preview_Last_Ring_Element != SearchLine_Otsu_Ring_Element) ||
       (SearchLine_Preview_Last_Ring_Flag != SearchLine_Otsu_Ring_Flag) ||
       (SearchLine_Preview_Last_Ring_Size != SearchLine_Otsu_Ring_Size))
    {
        ips200_show_string(104, (uint16)(preview_h + 52), "            ");
        ips200_show_string(104, (uint16)(preview_h + 52), ring_text);
        SearchLine_Preview_Last_Ring_Element = SearchLine_Otsu_Ring_Element;
        SearchLine_Preview_Last_Ring_Flag = SearchLine_Otsu_Ring_Flag;
        SearchLine_Preview_Last_Ring_Size = SearchLine_Otsu_Ring_Size;
    }

    ips200_set_color(RGB565_CYAN, RGB565_BLACK);
    if((SearchLine_Preview_Last_Offline_Row != SearchLine_Otsu_Offline_Row) ||
       (SearchLine_Preview_Last_White_Line != SearchLine_Otsu_White_Line))
    {
        ips200_show_string(104, (uint16)(preview_h + 68), "            ");
        ips200_show_string(104, (uint16)(preview_h + 68), gate_text);
        SearchLine_Preview_Last_Offline_Row = SearchLine_Otsu_Offline_Row;
        SearchLine_Preview_Last_White_Line = SearchLine_Otsu_White_Line;
    }

    ips200_set_color(RGB565_GREEN, RGB565_BLACK);
    if((SearchLine_Preview_Last_Ring_Left_Line != SearchLine_Otsu_Left_Line) ||
       (SearchLine_Preview_Last_Ring_Right_Line != SearchLine_Otsu_Right_Line) ||
       (SearchLine_Preview_Last_Cirque_Left_Count != SearchLine_Otsu_Cirque_Left_Count))
    {
        ips200_show_string(104, (uint16)(preview_h + 84), "            ");
        ips200_show_string(104, (uint16)(preview_h + 84), left_text);
        SearchLine_Preview_Last_Cirque_Left_Count = SearchLine_Otsu_Cirque_Left_Count;
        SearchLine_Preview_Last_Ring_Left_Line = SearchLine_Otsu_Left_Line;
        SearchLine_Preview_Last_Ring_Right_Line = SearchLine_Otsu_Right_Line;
    }

    ips200_set_color(RGB565_MAGENTA, RGB565_BLACK);
    if((SearchLine_Preview_Last_Ring_Left_Line_RightPanel != SearchLine_Otsu_Left_Line) ||
       (SearchLine_Preview_Last_Ring_Right_Line_RightPanel != SearchLine_Otsu_Right_Line) ||
       (SearchLine_Preview_Last_Cirque_Right_Count != SearchLine_Otsu_Cirque_Right_Count))
    {
        ips200_show_string(104, (uint16)(preview_h + 100), "            ");
        ips200_show_string(104, (uint16)(preview_h + 100), right_text);
        SearchLine_Preview_Last_Cirque_Right_Count = SearchLine_Otsu_Cirque_Right_Count;
        SearchLine_Preview_Last_Ring_Left_Line_RightPanel = SearchLine_Otsu_Left_Line;
        SearchLine_Preview_Last_Ring_Right_Line_RightPanel = SearchLine_Otsu_Right_Line;
    }

    ips200_set_color(RGB565_BLUE, RGB565_BLACK);
    if((SearchLine_Preview_Last_Ring_Stage_Num != SearchLine_Otsu_Ring_Stage_Num) ||
       (SearchLine_Preview_Last_Ring_Point_Y != SearchLine_Otsu_Ring_Point_Y) ||
       (SearchLine_Preview_Last_Ring_Straight_Judge_Tenth != SearchLine_Otsu_Ring_Straight_Judge_Tenth))
    {
        ips200_show_string(104, (uint16)(preview_h + 116), "            ");
        ips200_show_string(104, (uint16)(preview_h + 116), stage_text);
        SearchLine_Preview_Last_Ring_Stage_Num = SearchLine_Otsu_Ring_Stage_Num;
        SearchLine_Preview_Last_Ring_Point_Y = SearchLine_Otsu_Ring_Point_Y;
        SearchLine_Preview_Last_Ring_Straight_Judge_Tenth = SearchLine_Otsu_Ring_Straight_Judge_Tenth;
    }

    for(row = SearchLine_Otsu_Offline_Row; row <= 59; row++)
    {
        if(!SearchLine_Otsu_Row_Valid[row])
        {
            continue;
        }

        left_col = ImageDeal[row].LeftBorder;
        right_col = ImageDeal[row].RightBorder;
        center_col = ImageDeal[row].Center;

        y = (uint16)(((uint32)row * (uint32)preview_h + (uint32)(LCDH / 2)) /
                     (uint32)LCDH);
        x = (uint16)(((uint32)left_col * (uint32)preview_w + (uint32)(LCDW / 2)) /
                     (uint32)LCDW);
        ips200_draw_point(x, y, RGB565_GREEN);
        ips200_draw_point(x, (uint16)Limit((int32)y + 1, preview_h - 1, 0), RGB565_GREEN);

        x = (uint16)(((uint32)right_col * (uint32)preview_w + (uint32)(LCDW / 2)) /
                     (uint32)LCDW);
        ips200_draw_point(x, y, RGB565_GREEN);
        ips200_draw_point(x, (uint16)Limit((int32)y + 1, preview_h - 1, 0), RGB565_GREEN);

        x = (uint16)(((uint32)center_col * (uint32)preview_w + (uint32)(LCDW / 2)) /
                     (uint32)LCDW);
        ips200_draw_point(x, y, RGB565_RED);
        ips200_draw_point(x, (uint16)Limit((int32)y + 1, preview_h - 1, 0), RGB565_RED);
    }

    for(row = ImageStatus.OFFLineBoundary; row <= 58; row++)
    {
        y = (uint16)(((uint32)row * (uint32)preview_h + (uint32)(LCDH / 2)) /
                     (uint32)LCDH);

        boundary_col = SearchLine_Otsu_Left_Boundary_First[row];
        x = (uint16)(((uint32)boundary_col * (uint32)preview_w + (uint32)(LCDW / 2)) /
                     (uint32)LCDW);
        ips200_draw_point(x, y, RGB565_YELLOW);

        boundary_col = SearchLine_Otsu_Right_Boundary_First[row];
        x = (uint16)(((uint32)boundary_col * (uint32)preview_w + (uint32)(LCDW / 2)) /
                     (uint32)LCDW);
        ips200_draw_point(x, y, RGB565_YELLOW);

        boundary_col = SearchLine_Otsu_Left_Boundary[row];
        x = (uint16)(((uint32)boundary_col * (uint32)preview_w + (uint32)(LCDW / 2)) /
                     (uint32)LCDW);
        ips200_draw_point(x, y, RGB565_CYAN);

        boundary_col = SearchLine_Otsu_Right_Boundary[row];
        x = (uint16)(((uint32)boundary_col * (uint32)preview_w + (uint32)(LCDW / 2)) /
                     (uint32)LCDW);
        ips200_draw_point(x, y, RGB565_CYAN);
    }
}

/* 显示压缩二值图。 */
void SearchLine_DrawBinaryPreview(void)
{
    SearchLine_DrawPreview(0);
}

/* 显示原始灰度图。 */
void SearchLine_DrawRawPreview(void)
{
    SearchLine_DrawPreview(1);
}
