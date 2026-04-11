#include "SearchLine.h"
#include "dev_flash.h"
#include "dev_servo.h"
#include "dev_wheel.h"

#define SEARCH_LINE_OTSU_W                  (80)
#define SEARCH_LINE_OTSU_H                  (60)
#define SEARCH_LINE_OTSU_BOTTOM_ROW         (SEARCH_LINE_OTSU_H - 1)
#define SEARCH_LINE_OTSU_BOTTOM_INIT_ROW    (55)
#define SEARCH_LINE_OTSU_OFFLINE_MIN        (2)
#define SEARCH_LINE_OTSU_THRESHOLD_MIN      (0)
#define SEARCH_LINE_OTSU_THRESHOLD_CAP      (180)
#define SEARCH_LINE_OTSU_THRESHOLD_STATIC   (70)
/* OTSU 阈值重算周期。
 * 1 表示每帧都重算。
 * 2-20 表示隔 N 帧重算一次，中间帧直接复用上一次阈值。
 * 当前先用 10，后续可按赛道光照稳定性手调，步进 1。
 */
#define SEARCH_LINE_OTSU_THRESHOLD_INTERVAL (10)
#define SEARCH_LINE_OTSU_PIXEL_FILTER_ENABLE (0)
#define SEARCH_LINE_OTSU_LEFT_COMP_END      (15)
#define SEARCH_LINE_OTSU_RIGHT_COMP_START   (65)
#define SEARCH_LINE_OTSU_MIN_WIDTH          (7)
#define SEARCH_LINE_OTSU_EDGE_LIMIT         (10)
#define SEARCH_LINE_OTSU_SCAN_WINDOW        (2)
#define SEARCH_LINE_OTSU_SCAN_WINDOW_CROSS  (2)
#define SEARCH_LINE_OTSU_MIDDLE_LINE        (SEARCH_LINE_OTSU_W / 2 - 1)
#define SEARCH_LINE_OTSU_SEARCH_BORDER_BOTTOM_ROW (SEARCH_LINE_OTSU_H - 2)
/* 普通赛道基础前瞻行，当前按参考常用值先收回到 27。 */
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
#define SEARCH_LINE_STEER_REF_MIDDLE_DUTY   (4880.0f)
#define SEARCH_LINE_STEER_REF_RIGHT_DUTY    (4100.0f)
#define SEARCH_LINE_STEER_REF_LEFT_DUTY     (5520.0f)

#define SEARCH_LINE_STATE_INIT              ('F')
#define SEARCH_LINE_STATE_FOUND             ('T')
#define SEARCH_LINE_STATE_WHITE             ('W')
#define SEARCH_LINE_STATE_BLACK             ('H')

typedef enum
{
    SEARCH_LINE_ROAD_NORMAL = 0,
    SEARCH_LINE_ROAD_STRAIGHT,
    SEARCH_LINE_ROAD_CROSS,
    SEARCH_LINE_ROAD_RAMP,
    SEARCH_LINE_ROAD_LEFT_CIRQUE,
    SEARCH_LINE_ROAD_RIGHT_CIRQUE,
    SEARCH_LINE_ROAD_FORKIN,
    SEARCH_LINE_ROAD_FORKOUT,
    SEARCH_LINE_ROAD_BARN_OUT,
    SEARCH_LINE_ROAD_BARN_IN,
    SEARCH_LINE_ROAD_CROSS_TRUE,
    SEARCH_LINE_ROAD_ZEBRA
} search_line_road_type_t;

/* 80x60 灰度压缩图。 */
static uint8 SearchLine_Otsu_Gray[SEARCH_LINE_OTSU_H][SEARCH_LINE_OTSU_W] = {0};
/* 80x60 二值图。 */
static uint8 SearchLine_Otsu_Binary[SEARCH_LINE_OTSU_H][SEARCH_LINE_OTSU_W] = {0};
/* OTSU 直方图统计。 */
static uint16 SearchLine_Otsu_Histogram[256] = {0};
/* 原图到压缩图的行映射。 */
static uint8 SearchLine_Otsu_Row_Map[SEARCH_LINE_OTSU_H] = {0};
/* 原图到压缩图的列映射。 */
static uint8 SearchLine_Otsu_Col_Map[SEARCH_LINE_OTSU_W] = {0};
/* 底部初始化后的左边界。 */
static uint8 SearchLine_Otsu_Left_Border[SEARCH_LINE_OTSU_H] = {0};
/* 底部初始化后的右边界。 */
static uint8 SearchLine_Otsu_Right_Border[SEARCH_LINE_OTSU_H] = {0};
/* 底部初始化后的中线。 */
static uint8 SearchLine_Otsu_Center_Line[SEARCH_LINE_OTSU_H] = {0};
/* 当前行是否已经完成底边初始化。 */
static uint8 SearchLine_Otsu_Row_Valid[SEARCH_LINE_OTSU_H] = {0};
/* 左边界状态。 */
static uint8 SearchLine_Otsu_Left_State[SEARCH_LINE_OTSU_H] = {0};
/* 右边界状态。 */
static uint8 SearchLine_Otsu_Right_State[SEARCH_LINE_OTSU_H] = {0};
/* 对齐参考 Search_Border_OTSU 的首次边界缓存。 */
static uint8 SearchLine_Otsu_Left_Boundary_First[SEARCH_LINE_OTSU_H] = {0};
static uint8 SearchLine_Otsu_Right_Boundary_First[SEARCH_LINE_OTSU_H] = {0};
/* 对齐参考 Search_Border_OTSU 的方向跟踪边界缓存。 */
static uint8 SearchLine_Otsu_Left_Boundary[SEARCH_LINE_OTSU_H] = {0};
static uint8 SearchLine_Otsu_Right_Boundary[SEARCH_LINE_OTSU_H] = {0};
static uint8 SearchLine_Otsu_Left_Extend_Allowed = 1;
static uint8 SearchLine_Otsu_Right_Extend_Allowed = 1;
static uint8 SearchLine_Otsu_Map_Ready = 0;
static uint8 SearchLine_Otsu_Offline_Row = SEARCH_LINE_OTSU_OFFLINE_MIN;
/* 对齐参考代码的单边丢线计数。 */
static uint8 SearchLine_Otsu_Left_Line = 0;
static uint8 SearchLine_Otsu_Right_Line = 0;
static uint8 SearchLine_Otsu_White_Line = 0;
/* 对齐参考代码的前瞻行和加权中线。 */
static uint8 SearchLine_Otsu_TowPoint_True = SEARCH_LINE_OTSU_DET_TOW_POINT;
static uint8 SearchLine_Otsu_Det_True = SEARCH_LINE_OTSU_MIDDLE_LINE;
/* 对齐参考代码的直道判定结果。 */
static uint8 SearchLine_Otsu_Straight_Acc = 0;
static uint16 SearchLine_Otsu_Variance_Acc = 0;
/* 对齐参考 Search_Border_OTSU 的十字/元素观测量。 */
static uint8 SearchLine_Otsu_WhiteLine_L = 0;
static uint8 SearchLine_Otsu_WhiteLine_R = 0;
static uint8 SearchLine_Otsu_OffLine_Boundary = 5;
static uint8 SearchLine_Otsu_Road_Type = SEARCH_LINE_ROAD_NORMAL;
static uint8 SearchLine_Otsu_CirquePass = SEARCH_LINE_STATE_INIT;
static uint8 SearchLine_Otsu_IsCinqueOutIn = SEARCH_LINE_STATE_INIT;
static uint8 SearchLine_Otsu_CirqueOut = SEARCH_LINE_STATE_INIT;
static uint8 SearchLine_Otsu_CirqueOff = SEARCH_LINE_STATE_INIT;
static uint8 SearchLine_Otsu_Fork_Down = 0;
/* 对齐参考代码的舵机位置式 PD 预览量。 */
static int16 SearchLine_Otsu_Steer_Offset = 0;
static uint8 SearchLine_Otsu_Steer_Command = CAR_SERVO_CENTER_ANGLE;
static uint16 SearchLine_Otsu_Steer_P_Tenth = FLASH_STEER_P_DEFAULT_TENTH;
static uint16 SearchLine_Otsu_Steer_D_Tenth = FLASH_STEER_D_DEFAULT_TENTH;
static float SearchLine_Otsu_Steer_Last_Error = 0.0f;
static uint8 SearchLine_Otsu_Threshold_Raw_Cache = SEARCH_LINE_OTSU_THRESHOLD_MIN;
static uint8 SearchLine_Otsu_Threshold_Cache = SEARCH_LINE_OTSU_THRESHOLD_MIN;
static uint8 SearchLine_Otsu_Threshold_Frame_Count = 0;
static float SearchLine_Otsu_Det_Weight[SEARCH_LINE_OTSU_DET_WEIGHT_COUNT] =
{
    0.96f, 0.92f, 0.88f, 0.83f, 0.77f,
    0.71f, 0.65f, 0.59f, 0.53f, 0.47f
};

static int32 SearchLine_Limit_Int32(int32 value, int32 limit1, int32 limit2)
{
    int32 temp = 0;

    if(limit1 > limit2)
    {
        temp = limit1;
        limit1 = limit2;
        limit2 = temp;
    }

    if(value < limit1)
    {
        return limit1;
    }

    if(value > limit2)
    {
        return limit2;
    }

    return value;
}

/* 行列映射表初始化。 */
static void SearchLine_Init_Otsu_Map(void)
{
    uint16 row = 0;
    uint16 col = 0;
    uint32 scaled_value = 0;
    float row_scale = 0.0f;
    float col_scale = 0.0f;

    if(SearchLine_Otsu_Map_Ready)
    {
        return;
    }

    row_scale = (float)CAMERA_RAW_H / (float)SEARCH_LINE_OTSU_H;
    col_scale = (float)CAMERA_RAW_W / (float)SEARCH_LINE_OTSU_W;

    for(row = 0; row < SEARCH_LINE_OTSU_H; row++)
    {
        scaled_value = (uint32)((float)row * row_scale + 0.5f);
        SearchLine_Otsu_Row_Map[row] =
            (uint8)SearchLine_Limit_Int32((int32)scaled_value, 0, CAMERA_RAW_H - 1);
    }

    for(col = 0; col < SEARCH_LINE_OTSU_W; col++)
    {
        scaled_value = (uint32)((float)col * col_scale + 0.5f);
        SearchLine_Otsu_Col_Map[col] =
            (uint8)SearchLine_Limit_Int32((int32)scaled_value, 0, CAMERA_LAST_VALID_COL);
    }

    SearchLine_Otsu_Map_Ready = 1;
}

/* 图像压缩。 */
static void SearchLine_Compress_Otsu_Image(void)
{
    uint16 row = 0;
    uint16 col = 0;

    for(row = 0; row < SEARCH_LINE_OTSU_H; row++)
    {
        for(col = 0; col < SEARCH_LINE_OTSU_W; col++)
        {
            SearchLine_Otsu_Gray[row][col] =
                mt9v03x_image[SearchLine_Otsu_Row_Map[row]][SearchLine_Otsu_Col_Map[col]];
        }
    }
}

static uint8 SearchLine_Get_Otsu_Gray(uint8 row, uint8 col)
{
    return SearchLine_Otsu_Gray[row][col];
}

/* 底边状态初始化。 */
static void SearchLine_Clear_Otsu_State(void)
{
    uint16 row = 0;

    SearchLine_Otsu_Offline_Row = SEARCH_LINE_OTSU_OFFLINE_MIN;
    SearchLine_Otsu_Left_Extend_Allowed = 1;
    SearchLine_Otsu_Right_Extend_Allowed = 1;
    SearchLine_Otsu_Left_Line = 0;
    SearchLine_Otsu_Right_Line = 0;
    SearchLine_Otsu_White_Line = 0;
    SearchLine_Otsu_WhiteLine_L = 0;
    SearchLine_Otsu_WhiteLine_R = 0;
    SearchLine_Otsu_OffLine_Boundary = 5;
    SearchLine_Otsu_Road_Type = SEARCH_LINE_ROAD_NORMAL;
    SearchLine_Otsu_CirquePass = SEARCH_LINE_STATE_INIT;
    SearchLine_Otsu_IsCinqueOutIn = SEARCH_LINE_STATE_INIT;
    SearchLine_Otsu_CirqueOut = SEARCH_LINE_STATE_INIT;
    SearchLine_Otsu_CirqueOff = SEARCH_LINE_STATE_INIT;
    SearchLine_Otsu_Fork_Down = 0;

    for(row = 0; row < SEARCH_LINE_OTSU_H; row++)
    {
        SearchLine_Otsu_Left_Border[row] = 0;
        SearchLine_Otsu_Right_Border[row] = SEARCH_LINE_OTSU_W - 1;
        SearchLine_Otsu_Center_Line[row] = SEARCH_LINE_OTSU_MIDDLE_LINE;
        SearchLine_Otsu_Row_Valid[row] = 0;
        SearchLine_Otsu_Left_State[row] = SEARCH_LINE_STATE_INIT;
        SearchLine_Otsu_Right_State[row] = SEARCH_LINE_STATE_INIT;
        SearchLine_Otsu_Left_Boundary_First[row] = 0;
        SearchLine_Otsu_Right_Boundary_First[row] = SEARCH_LINE_OTSU_W - 1;
        SearchLine_Otsu_Left_Boundary[row] = 0;
        SearchLine_Otsu_Right_Boundary[row] = SEARCH_LINE_OTSU_W - 1;
    }
}

static uint8 SearchLine_Clamp_Otsu_Search_Col(int16 value)
{
    return (uint8)SearchLine_Limit_Int32(value, 1, SEARCH_LINE_OTSU_W - 2);
}

static uint8 SearchLine_Get_Otsu_Search_Border_Pixel(int16 row, int16 col, uint8 bottom_row)
{
    if((row <= 0) || (row > bottom_row))
    {
        return 0;
    }

    if((col <= 0) || (col >= (SEARCH_LINE_OTSU_W - 1)))
    {
        return 0;
    }

    return SearchLine_Otsu_Binary[row][col];
}

/* 一行边界结果写入缓存。 */
static void SearchLine_Set_Otsu_Row(uint8 row, int16 left_border, int16 right_border)
{
    left_border = SearchLine_Limit_Int32(left_border, 0, SEARCH_LINE_OTSU_W - 1);
    right_border = SearchLine_Limit_Int32(right_border, 0, SEARCH_LINE_OTSU_W - 1);

    if(right_border < left_border)
    {
        left_border = 0;
        right_border = SEARCH_LINE_OTSU_W - 1;
    }

    SearchLine_Otsu_Left_Border[row] = (uint8)left_border;
    SearchLine_Otsu_Right_Border[row] = (uint8)right_border;
    SearchLine_Otsu_Center_Line[row] = (uint8)(((uint16)left_border + (uint16)right_border) / 2);
    SearchLine_Otsu_Row_Valid[row] = 1;
    SearchLine_Otsu_Left_State[row] = SEARCH_LINE_STATE_FOUND;
    SearchLine_Otsu_Right_State[row] = SEARCH_LINE_STATE_FOUND;
}

/* 参考 Pixle_Filter 的十字内噪点补白，当前默认不上主链。 */
static void SearchLine_Pixle_Filter_Otsu(void)
{
    int16 row = 0;
    int16 col = 0;

    for(row = 10; row < 40; row++)
    {
        for(col = 10; col < 70; col++)
        {
            if((0 == SearchLine_Otsu_Binary[row][col]) &&
               ((SearchLine_Otsu_Binary[row - 1][col] +
                 SearchLine_Otsu_Binary[row + 1][col] +
                 SearchLine_Otsu_Binary[row][col - 1] +
                 SearchLine_Otsu_Binary[row][col + 1]) >= 3))
            {
                SearchLine_Otsu_Binary[row][col] = 1;
            }
        }
    }
}

/* 参考 Search_Border_OTSU 的底边扫描。 */
static void SearchLine_Search_Bottom_Line_Otsu(uint8 bottom_row)
{
    int16 col = 0;

    SearchLine_Otsu_Left_Boundary[bottom_row] = 0;
    SearchLine_Otsu_Right_Boundary[bottom_row] = SEARCH_LINE_OTSU_W - 1;

    for(col = SEARCH_LINE_OTSU_W / 2 - 2; col > 1; col--)
    {
        if((1 == SearchLine_Get_Otsu_Search_Border_Pixel(bottom_row, col, bottom_row)) &&
           (0 == SearchLine_Get_Otsu_Search_Border_Pixel(bottom_row, col - 1, bottom_row)))
        {
            SearchLine_Otsu_Left_Boundary[bottom_row] = (uint8)col;
            break;
        }
    }

    for(col = SEARCH_LINE_OTSU_W / 2 + 2; col < SEARCH_LINE_OTSU_W - 1; col++)
    {
        if((1 == SearchLine_Get_Otsu_Search_Border_Pixel(bottom_row, col, bottom_row)) &&
           (0 == SearchLine_Get_Otsu_Search_Border_Pixel(bottom_row, col + 1, bottom_row)))
        {
            SearchLine_Otsu_Right_Boundary[bottom_row] = (uint8)col;
            break;
        }
    }
}

/* 参考 Search_Border_OTSU 的方向跟踪边界扫描。 */
static void SearchLine_Search_Left_And_Right_Lines_Otsu(uint8 bottom_row)
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
    uint8 left_y = 0;
    uint8 left_x = 0;
    uint8 left_direction = 0;
    uint8 right_y = 0;
    uint8 right_x = 0;
    uint8 right_direction = 0;
    uint8 pixel_left_y = 0;
    uint8 pixel_left_x = 0;
    uint8 pixel_right_y = 0;
    uint8 pixel_right_x = 0;
    uint8 row = 0;
    uint16 guard_count = 0;
    int16 next_row = 0;
    int16 next_col = 0;

    left_y = bottom_row;
    left_x = SearchLine_Otsu_Left_Boundary[bottom_row];
    right_y = bottom_row;
    right_x = SearchLine_Otsu_Right_Boundary[bottom_row];
    pixel_left_y = bottom_row;
    pixel_right_y = bottom_row;
    row = bottom_row;
    SearchLine_Otsu_OffLine_Boundary = 5;

    while(1)
    {
        guard_count++;
        if(guard_count > 400)
        {
            SearchLine_Otsu_OffLine_Boundary = row;
            break;
        }

        if((row >= pixel_left_y) && (row >= pixel_right_y))
        {
            if(row < SearchLine_Otsu_OffLine_Boundary)
            {
                SearchLine_Otsu_OffLine_Boundary = row;
                break;
            }

            row--;
        }

        if((pixel_left_y > row) || (row == SearchLine_Otsu_OffLine_Boundary))
        {
            next_row = (int16)left_y + left_rule[0][2 * left_direction + 1];
            next_col = (int16)left_x + left_rule[0][2 * left_direction];
            pixel_left_y = (uint8)SearchLine_Limit_Int32(next_row, 0, SEARCH_LINE_OTSU_H - 1);
            pixel_left_x = (uint8)SearchLine_Limit_Int32(next_col, 0, SEARCH_LINE_OTSU_W - 1);

            if(0 == SearchLine_Get_Otsu_Search_Border_Pixel(pixel_left_y, pixel_left_x, bottom_row))
            {
                left_direction = (uint8)((left_direction + 1U) & 0x03U);
            }
            else
            {
                next_row = (int16)left_y + left_rule[1][2 * left_direction + 1];
                next_col = (int16)left_x + left_rule[1][2 * left_direction];
                pixel_left_y = (uint8)SearchLine_Limit_Int32(next_row, 0, SEARCH_LINE_OTSU_H - 1);
                pixel_left_x = (uint8)SearchLine_Limit_Int32(next_col, 0, SEARCH_LINE_OTSU_W - 1);

                if(0 == SearchLine_Get_Otsu_Search_Border_Pixel(pixel_left_y, pixel_left_x, bottom_row))
                {
                    left_y = (uint8)SearchLine_Limit_Int32((int16)left_y + left_rule[0][2 * left_direction + 1],
                                                           0,
                                                           SEARCH_LINE_OTSU_H - 1);
                    left_x = (uint8)SearchLine_Limit_Int32((int16)left_x + left_rule[0][2 * left_direction],
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
                    left_y = (uint8)SearchLine_Limit_Int32((int16)left_y + left_rule[1][2 * left_direction + 1],
                                                           0,
                                                           SEARCH_LINE_OTSU_H - 1);
                    left_x = (uint8)SearchLine_Limit_Int32((int16)left_x + left_rule[1][2 * left_direction],
                                                           0,
                                                           SEARCH_LINE_OTSU_W - 1);
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

        if((pixel_right_y > row) || (row == SearchLine_Otsu_OffLine_Boundary))
        {
            next_row = (int16)right_y + right_rule[0][2 * right_direction + 1];
            next_col = (int16)right_x + right_rule[0][2 * right_direction];
            pixel_right_y = (uint8)SearchLine_Limit_Int32(next_row, 0, SEARCH_LINE_OTSU_H - 1);
            pixel_right_x = (uint8)SearchLine_Limit_Int32(next_col, 0, SEARCH_LINE_OTSU_W - 1);

            if(0 == SearchLine_Get_Otsu_Search_Border_Pixel(pixel_right_y, pixel_right_x, bottom_row))
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
                next_row = (int16)right_y + right_rule[1][2 * right_direction + 1];
                next_col = (int16)right_x + right_rule[1][2 * right_direction];
                pixel_right_y = (uint8)SearchLine_Limit_Int32(next_row, 0, SEARCH_LINE_OTSU_H - 1);
                pixel_right_x = (uint8)SearchLine_Limit_Int32(next_col, 0, SEARCH_LINE_OTSU_W - 1);

                if(0 == SearchLine_Get_Otsu_Search_Border_Pixel(pixel_right_y, pixel_right_x, bottom_row))
                {
                    right_y = (uint8)SearchLine_Limit_Int32((int16)right_y + right_rule[0][2 * right_direction + 1],
                                                            0,
                                                            SEARCH_LINE_OTSU_H - 1);
                    right_x = (uint8)SearchLine_Limit_Int32((int16)right_x + right_rule[0][2 * right_direction],
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
                    right_y = (uint8)SearchLine_Limit_Int32((int16)right_y + right_rule[1][2 * right_direction + 1],
                                                            0,
                                                            SEARCH_LINE_OTSU_H - 1);
                    right_x = (uint8)SearchLine_Limit_Int32((int16)right_x + right_rule[1][2 * right_direction],
                                                            0,
                                                            SEARCH_LINE_OTSU_W - 1);
                    if((SEARCH_LINE_OTSU_W - 1) == SearchLine_Otsu_Right_Boundary_First[right_y])
                    {
                        SearchLine_Otsu_Right_Boundary_First[right_y] = right_x;
                    }
                    SearchLine_Otsu_Right_Boundary[right_y] = right_x;
                    right_direction = (uint8)((right_direction + 1U) & 0x03U);
                }
            }
        }

        if((pixel_right_x >= pixel_left_x) ? ((pixel_right_x - pixel_left_x) < 3U) :
                                             ((pixel_left_x - pixel_right_x) < 3U))
        {
            SearchLine_Otsu_OffLine_Boundary = row;
            break;
        }
    }
}

/* 参考 Search_Border_OTSU 的十字边界跟踪支线。 */
static void SearchLine_Search_Border_Otsu(void)
{
    int16 row = 0;
    uint8 bottom_row = SEARCH_LINE_OTSU_SEARCH_BORDER_BOTTOM_ROW;

    SearchLine_Otsu_WhiteLine_L = 0;
    SearchLine_Otsu_WhiteLine_R = 0;

    for(row = 0; row < SEARCH_LINE_OTSU_H; row++)
    {
        SearchLine_Otsu_Left_Boundary_First[row] = 0;
        SearchLine_Otsu_Right_Boundary_First[row] = SEARCH_LINE_OTSU_W - 1;
        SearchLine_Otsu_Left_Boundary[row] = 0;
        SearchLine_Otsu_Right_Boundary[row] = SEARCH_LINE_OTSU_W - 1;
    }

    SearchLine_Search_Bottom_Line_Otsu(bottom_row);
    SearchLine_Search_Left_And_Right_Lines_Otsu(bottom_row);

    for(row = bottom_row; row > (int16)SearchLine_Otsu_OffLine_Boundary + 1; row--)
    {
        if(SearchLine_Otsu_Left_Boundary[row] < 3)
        {
            SearchLine_Otsu_WhiteLine_L++;
        }
        if(SearchLine_Otsu_Right_Boundary[row] > (SEARCH_LINE_OTSU_W - 3))
        {
            SearchLine_Otsu_WhiteLine_R++;
        }
    }
}

/* 底边初始化。 */
static void SearchLine_Init_Otsu_BottomRows(void)
{
    const uint8 *row_data = 0;
    int16 row = 0;
    int16 col = 0;
    int16 center_col = SEARCH_LINE_OTSU_W / 2 - 1;
    int16 left_border = 0;
    int16 right_border = SEARCH_LINE_OTSU_W - 1;
    int16 offset = 0;
    uint8 current_row = 0;

    row_data = SearchLine_Otsu_Binary[SEARCH_LINE_OTSU_BOTTOM_ROW];
    if(0 == row_data[center_col])
    {
        for(offset = 0; offset < center_col; offset++)
        {
            if(0 != row_data[center_col - offset])
            {
                break;
            }

            if(0 != row_data[center_col + offset])
            {
                break;
            }
        }

        if(0 != row_data[center_col - offset])
        {
            right_border = center_col - offset + 1;
            for(col = right_border; col > 0; col--)
            {
                if((0 == row_data[col]) && (0 == row_data[col - 1]))
                {
                    left_border = col;
                    break;
                }
                else if(1 == col)
                {
                    left_border = 0;
                    break;
                }
            }
        }
        else if(0 != row_data[center_col + offset])
        {
            left_border = center_col + offset - 1;
            for(col = left_border; col < SEARCH_LINE_OTSU_W - 1; col++)
            {
                if((0 == row_data[col]) && (0 == row_data[col + 1]))
                {
                    right_border = col;
                    break;
                }
                else if(col == SEARCH_LINE_OTSU_W - 2)
                {
                    right_border = SEARCH_LINE_OTSU_W - 1;
                    break;
                }
            }
        }
    }
    else
    {
        for(col = SEARCH_LINE_OTSU_W - 1; col > center_col; col--)
        {
            if((1 == row_data[col]) && (1 == row_data[col - 1]))
            {
                right_border = col;
                break;
            }
            else if(col == center_col + 1)
            {
                right_border = center_col;
                break;
            }
        }

        for(col = 0; col < center_col; col++)
        {
            if((1 == row_data[col]) && (1 == row_data[col + 1]))
            {
                left_border = col;
                break;
            }
            else if(col == center_col - 1)
            {
                left_border = center_col;
                break;
            }
        }
    }

    SearchLine_Set_Otsu_Row(SEARCH_LINE_OTSU_BOTTOM_ROW, left_border, right_border);

    for(row = SEARCH_LINE_OTSU_BOTTOM_ROW - 1; row >= SEARCH_LINE_OTSU_BOTTOM_INIT_ROW; row--)
    {
        current_row = (uint8)row;
        row_data = SearchLine_Otsu_Binary[current_row];

        for(col = SEARCH_LINE_OTSU_W - 1; col > SearchLine_Otsu_Center_Line[current_row + 1]; col--)
        {
            if((1 == row_data[col]) && (1 == row_data[col - 1]))
            {
                right_border = col;
                break;
            }
            else if(col == (int16)SearchLine_Otsu_Center_Line[current_row + 1] + 1)
            {
                right_border = SearchLine_Otsu_Center_Line[current_row + 1];
                break;
            }
        }

        for(col = 0; col < SearchLine_Otsu_Center_Line[current_row + 1]; col++)
        {
            if((1 == row_data[col]) && (1 == row_data[col + 1]))
            {
                left_border = col;
                break;
            }
            else if(col == (int16)SearchLine_Otsu_Center_Line[current_row + 1] - 1)
            {
                left_border = SearchLine_Otsu_Center_Line[current_row + 1];
                break;
            }
        }

        SearchLine_Set_Otsu_Row(current_row, left_border, right_border);
    }
}

static uint8 SearchLine_Find_Otsu_JumpPoint(const uint8 *row_data, uint8 search_left, int16 low, int16 high, uint8 *point)
{
    int16 col = 0;
    int16 mid = 0;

    if(0 == row_data || 0 == point)
    {
        return SEARCH_LINE_STATE_INIT;
    }

    low = SearchLine_Limit_Int32(low, 1, SEARCH_LINE_OTSU_W - 2);
    high = SearchLine_Limit_Int32(high, 1, SEARCH_LINE_OTSU_W - 2);
    if(low > high)
    {
        col = low;
        low = high;
        high = col;
    }

    mid = (low + high) / 2;
    if(search_left)
    {
        for(col = high; col >= low; col--)
        {
            if((1 == row_data[col]) && (0 == row_data[col - 1]))
            {
                *point = (uint8)col;
                return SEARCH_LINE_STATE_FOUND;
            }
            else if(col == (low + 1))
            {
                if(0 != row_data[mid])
                {
                    *point = (uint8)mid;
                    return SEARCH_LINE_STATE_WHITE;
                }

                *point = (uint8)high;
                return SEARCH_LINE_STATE_BLACK;
            }
        }
    }
    else
    {
        for(col = low; col <= high; col++)
        {
            if((1 == row_data[col]) && (0 == row_data[col + 1]))
            {
                *point = (uint8)col;
                return SEARCH_LINE_STATE_FOUND;
            }
            else if(col == (high - 1))
            {
                if(0 != row_data[mid])
                {
                    *point = (uint8)mid;
                    return SEARCH_LINE_STATE_WHITE;
                }

                *point = (uint8)low;
                return SEARCH_LINE_STATE_BLACK;
            }
        }
    }

    *point = (uint8)mid;
    return SEARCH_LINE_STATE_INIT;
}

static uint8 SearchLine_Rescue_Otsu_LeftBorder(const uint8 *row_data, uint8 left_border, uint8 right_border, uint8 *point)
{
    int16 col = 0;

    if(0 == row_data || 0 == point)
    {
        return SEARCH_LINE_STATE_INIT;
    }

    for(col = (int16)left_border + 1; col <= (int16)right_border - 1; col++)
    {
        if((0 == row_data[col]) && (0 != row_data[col + 1]))
        {
            *point = (uint8)col;
            return SEARCH_LINE_STATE_FOUND;
        }
        else if(0 != row_data[col])
        {
            break;
        }
        else if(col == (int16)right_border - 1)
        {
            *point = left_border;
            return SEARCH_LINE_STATE_FOUND;
        }
    }

    return SEARCH_LINE_STATE_WHITE;
}

static uint8 SearchLine_Rescue_Otsu_RightBorder(const uint8 *row_data, uint8 left_border, uint8 right_border, uint8 *point)
{
    int16 col = 0;

    if(0 == row_data || 0 == point)
    {
        return SEARCH_LINE_STATE_INIT;
    }

    for(col = (int16)right_border - 1; col >= (int16)left_border + 1; col--)
    {
        if((0 == row_data[col]) && (0 != row_data[col - 1]))
        {
            *point = (uint8)col;
            return SEARCH_LINE_STATE_FOUND;
        }
        else if(0 != row_data[col])
        {
            break;
        }
        else if(col == (int16)left_border + 1)
        {
            *point = (uint8)col;
            return SEARCH_LINE_STATE_FOUND;
        }
    }

    return SEARCH_LINE_STATE_WHITE;
}

/* 逐行向上搜边。 */
static void SearchLine_DrawLinesProcess_Otsu(void)
{
    const uint8 *row_data = 0;
    int16 row = 0;
    int16 low = 0;
    int16 high = 0;
    int16 scan_window = 0;
    int16 search_row = 0;
    uint8 left_point = 0;
    uint8 right_point = 0;
    uint8 left_state = 0;
    uint8 right_state = 0;
    int16 left_border = 0;
    int16 right_border = 0;
    uint8 left_slope_checked = 0;
    uint8 right_slope_checked = 0;
    uint8 left_slope_ready = 0;
    uint8 right_slope_ready = 0;
    uint8 left_found_count = 0;
    uint8 right_found_count = 0;
    uint8 left_anchor_row = 0;
    uint8 right_anchor_row = 0;
    float left_slope = 0.0f;
    float right_slope = 0.0f;

    SearchLine_Otsu_Left_Line = 0;
    SearchLine_Otsu_Right_Line = 0;
    SearchLine_Otsu_White_Line = 0;

    for(row = SEARCH_LINE_OTSU_BOTTOM_INIT_ROW - 1; row > SearchLine_Otsu_Offline_Row; row--)
    {
        row_data = SearchLine_Otsu_Binary[row];
        scan_window = SEARCH_LINE_OTSU_SCAN_WINDOW;
        if(SEARCH_LINE_ROAD_CROSS_TRUE == SearchLine_Otsu_Road_Type)
        {
            scan_window = SEARCH_LINE_OTSU_SCAN_WINDOW_CROSS;
        }

        low = (int16)SearchLine_Otsu_Right_Border[row + 1] - scan_window;
        high = (int16)SearchLine_Otsu_Right_Border[row + 1] + scan_window;
        low = SearchLine_Clamp_Otsu_Search_Col(low);
        high = SearchLine_Clamp_Otsu_Search_Col(high);
        right_state = SearchLine_Find_Otsu_JumpPoint(row_data, 0, low, high, &right_point);

        low = (int16)SearchLine_Otsu_Left_Border[row + 1] - scan_window;
        high = (int16)SearchLine_Otsu_Left_Border[row + 1] + scan_window;
        low = SearchLine_Clamp_Otsu_Search_Col(low);
        high = SearchLine_Clamp_Otsu_Search_Col(high);
        left_state = SearchLine_Find_Otsu_JumpPoint(row_data, 1, low, high, &left_point);

        if(SEARCH_LINE_STATE_WHITE == left_state)
        {
            left_border = SearchLine_Otsu_Left_Border[row + 1];
        }
        else
        {
            left_border = left_point;
        }

        if(SEARCH_LINE_STATE_WHITE == right_state)
        {
            right_border = SearchLine_Otsu_Right_Border[row + 1];
        }
        else
        {
            right_border = right_point;
        }

        SearchLine_Otsu_Left_State[row] = left_state;
        SearchLine_Otsu_Right_State[row] = right_state;
        SearchLine_Otsu_Left_Border[row] = (uint8)SearchLine_Limit_Int32(left_border, 0, SEARCH_LINE_OTSU_W - 1);
        SearchLine_Otsu_Right_Border[row] = (uint8)SearchLine_Limit_Int32(right_border, 0, SEARCH_LINE_OTSU_W - 1);

        if((SEARCH_LINE_STATE_BLACK == left_state) || (SEARCH_LINE_STATE_BLACK == right_state))
        {
            if(SEARCH_LINE_STATE_BLACK == left_state)
            {
                left_state = SearchLine_Rescue_Otsu_LeftBorder(row_data,
                                                               SearchLine_Otsu_Left_Border[row],
                                                               SearchLine_Otsu_Right_Border[row],
                                                               &left_point);
                SearchLine_Otsu_Left_State[row] = left_state;
                if(SEARCH_LINE_STATE_FOUND == left_state)
                {
                    SearchLine_Otsu_Left_Border[row] = left_point;
                }
            }

            if(((int16)SearchLine_Otsu_Right_Border[row] - (int16)SearchLine_Otsu_Left_Border[row]) <= SEARCH_LINE_OTSU_MIN_WIDTH)
            {
                SearchLine_Otsu_Offline_Row = (uint8)(row + 1);
                break;
            }

            if(SEARCH_LINE_STATE_BLACK == right_state)
            {
                right_state = SearchLine_Rescue_Otsu_RightBorder(row_data,
                                                                 SearchLine_Otsu_Left_Border[row],
                                                                 SearchLine_Otsu_Right_Border[row],
                                                                 &right_point);
                SearchLine_Otsu_Right_State[row] = right_state;
                if(SEARCH_LINE_STATE_FOUND == right_state)
                {
                    SearchLine_Otsu_Right_Border[row] = right_point;
                }
            }
        }

        /* 单边丢线时，先按参考斜率补当前行边界。 */
        if((SEARCH_LINE_STATE_WHITE == SearchLine_Otsu_Right_State[row]) &&
           (row > 10) &&
           (row < 50))
        {
            if(!right_slope_checked)
            {
                right_slope_checked = 1;
                right_anchor_row = (uint8)(row + 2);
                right_found_count = 0;
                for(search_row = row + 1;
                    (search_row < SEARCH_LINE_OTSU_H) && (search_row < row + 15);
                    search_row++)
                {
                    if(SEARCH_LINE_STATE_FOUND == SearchLine_Otsu_Right_State[search_row])
                    {
                        right_found_count++;
                    }
                }

                if(right_found_count > 8)
                {
                    right_slope =
                        ((float)SearchLine_Otsu_Right_Border[row + right_found_count] -
                         (float)SearchLine_Otsu_Right_Border[row + 3]) /
                        (float)(right_found_count - 3);
                    if(right_slope > 0.0f)
                    {
                        right_slope_ready = 1;
                    }
                    else
                    {
                        right_slope_ready = 0;
                        if(right_slope < 0.0f)
                        {
                            SearchLine_Otsu_Right_Extend_Allowed = 0;
                        }
                    }
                }
            }

            if(right_slope_ready)
            {
                SearchLine_Otsu_Right_Border[row] =
                    SearchLine_Clamp_Otsu_Search_Col((int16)((float)SearchLine_Otsu_Right_Border[right_anchor_row] -
                                                             right_slope * (float)(right_anchor_row - row)));
            }
        }

        if((SEARCH_LINE_STATE_WHITE == SearchLine_Otsu_Left_State[row]) &&
           (row > 10) &&
           (row < 50))
        {
            if(!left_slope_checked)
            {
                left_slope_checked = 1;
                left_anchor_row = (uint8)(row + 2);
                left_found_count = 0;
                for(search_row = row + 1;
                    (search_row < SEARCH_LINE_OTSU_H) && (search_row < row + 15);
                    search_row++)
                {
                    if(SEARCH_LINE_STATE_FOUND == SearchLine_Otsu_Left_State[search_row])
                    {
                        left_found_count++;
                    }
                }

                if(left_found_count > 8)
                {
                    left_slope =
                        ((float)SearchLine_Otsu_Left_Border[row + 3] -
                         (float)SearchLine_Otsu_Left_Border[row + left_found_count]) /
                        (float)(left_found_count - 3);
                    if(left_slope > 0.0f)
                    {
                        left_slope_ready = 1;
                    }
                    else
                    {
                        left_slope_ready = 0;
                        if(left_slope < 0.0f)
                        {
                            SearchLine_Otsu_Left_Extend_Allowed = 0;
                        }
                    }
                }
            }

            if(left_slope_ready)
            {
                SearchLine_Otsu_Left_Border[row] =
                    SearchLine_Clamp_Otsu_Search_Col((int16)((float)SearchLine_Otsu_Left_Border[left_anchor_row] +
                                                             left_slope * (float)(left_anchor_row - row)));
            }
        }

        SearchLine_Otsu_Center_Line[row] =
            (uint8)(((uint16)SearchLine_Otsu_Left_Border[row] + (uint16)SearchLine_Otsu_Right_Border[row]) / 2);
        SearchLine_Otsu_Row_Valid[row] = 1;
        if(SEARCH_LINE_STATE_WHITE == SearchLine_Otsu_Left_State[row])
        {
            SearchLine_Otsu_Left_Line++;
        }
        if(SEARCH_LINE_STATE_WHITE == SearchLine_Otsu_Right_State[row])
        {
            SearchLine_Otsu_Right_Line++;
        }
        if((SEARCH_LINE_STATE_WHITE == SearchLine_Otsu_Left_State[row]) &&
           (SEARCH_LINE_STATE_WHITE == SearchLine_Otsu_Right_State[row]))
        {
            SearchLine_Otsu_White_Line++;
        }

        if(((int16)SearchLine_Otsu_Right_Border[row] - (int16)SearchLine_Otsu_Left_Border[row]) <= SEARCH_LINE_OTSU_MIN_WIDTH)
        {
            SearchLine_Otsu_Offline_Row = (uint8)(row + 1);
            break;
        }

        if((SearchLine_Otsu_Right_Border[row] <= SEARCH_LINE_OTSU_EDGE_LIMIT) ||
           (SearchLine_Otsu_Left_Border[row] >= (SEARCH_LINE_OTSU_W - 1 - SEARCH_LINE_OTSU_EDGE_LIMIT)))
        {
            SearchLine_Otsu_Offline_Row = (uint8)(row + 1);
            break;
        }
    }
}

/* 延长线补边。 */
static void SearchLine_DrawExtensionLine_Otsu(void)
{
    int16 row = 0;
    int16 scan_row = 0;
    int16 fill_row = 0;
    int16 tfsite = SEARCH_LINE_OTSU_BOTTOM_INIT_ROW;
    int16 ftsite = 0;
    float slope = 0.0f;

    if((((0 == SearchLine_Otsu_Fork_Down) &&
         (SEARCH_LINE_STATE_INIT == SearchLine_Otsu_CirquePass) &&
         (SEARCH_LINE_STATE_INIT == SearchLine_Otsu_IsCinqueOutIn) &&
         (SEARCH_LINE_STATE_INIT == SearchLine_Otsu_CirqueOut) &&
         (SEARCH_LINE_ROAD_BARN_IN != SearchLine_Otsu_Road_Type) &&
         (SEARCH_LINE_ROAD_RAMP != SearchLine_Otsu_Road_Type) &&
         (SEARCH_LINE_ROAD_CROSS_TRUE != SearchLine_Otsu_Road_Type)) ||
        (SEARCH_LINE_STATE_FOUND == SearchLine_Otsu_CirqueOff)))
    {
        if(SearchLine_Otsu_White_Line >= (uint8)(SearchLine_Otsu_TowPoint_True - 15))
        {
            tfsite = SEARCH_LINE_OTSU_BOTTOM_INIT_ROW;
        }
        if((SEARCH_LINE_STATE_FOUND == SearchLine_Otsu_CirqueOff) &&
           (SEARCH_LINE_ROAD_LEFT_CIRQUE == SearchLine_Otsu_Road_Type))
        {
            tfsite = SEARCH_LINE_OTSU_BOTTOM_INIT_ROW;
        }

        if(SearchLine_Otsu_Left_Extend_Allowed)
        {
            for(row = SEARCH_LINE_OTSU_BOTTOM_INIT_ROW - 1; row >= (int16)SearchLine_Otsu_Offline_Row + 4; row--)
            {
                if(SEARCH_LINE_STATE_WHITE == SearchLine_Otsu_Left_State[row])
                {
                    if(SearchLine_Otsu_Left_Border[row + 1] >= 70)
                    {
                        SearchLine_Otsu_Offline_Row = (uint8)(row + 1);
                        break;
                    }

                    scan_row = row;
                    ftsite = 0;
                    while(scan_row >= (int16)SearchLine_Otsu_Offline_Row + 4)
                    {
                        scan_row--;
                        if((SEARCH_LINE_STATE_FOUND == SearchLine_Otsu_Left_State[scan_row]) &&
                           (SEARCH_LINE_STATE_FOUND == SearchLine_Otsu_Left_State[scan_row - 1]) &&
                           (SEARCH_LINE_STATE_FOUND == SearchLine_Otsu_Left_State[scan_row - 2]) &&
                           (SearchLine_Otsu_Left_Border[scan_row - 2] > 0) &&
                           (SearchLine_Otsu_Left_Border[scan_row - 2] < 70))
                        {
                            ftsite = scan_row - 2;
                            break;
                        }
                    }

                    if((0 != ftsite) && (ftsite > SearchLine_Otsu_Offline_Row))
                    {
                        slope = ((float)SearchLine_Otsu_Left_Border[ftsite] -
                                 (float)SearchLine_Otsu_Left_Border[tfsite]) /
                                (float)(ftsite - tfsite);
                        for(fill_row = tfsite; fill_row >= ftsite; fill_row--)
                        {
                            SearchLine_Otsu_Left_Border[fill_row] =
                                SearchLine_Clamp_Otsu_Search_Col((int16)(slope * (float)(fill_row - tfsite) +
                                                                          (float)SearchLine_Otsu_Left_Border[tfsite]));
                            SearchLine_Otsu_Left_State[fill_row] = SEARCH_LINE_STATE_FOUND;
                            SearchLine_Otsu_Row_Valid[fill_row] = 1;
                        }
                        row = ftsite;
                    }
                }
                else
                {
                    tfsite = row + 2;
                }
            }
        }

        if(SearchLine_Otsu_White_Line >= (uint8)(SearchLine_Otsu_TowPoint_True - 15))
        {
            tfsite = SEARCH_LINE_OTSU_BOTTOM_INIT_ROW;
        }
        if((SEARCH_LINE_STATE_FOUND == SearchLine_Otsu_CirqueOff) &&
           (SEARCH_LINE_ROAD_RIGHT_CIRQUE == SearchLine_Otsu_Road_Type))
        {
            tfsite = SEARCH_LINE_OTSU_BOTTOM_INIT_ROW;
        }

        if(SearchLine_Otsu_Right_Extend_Allowed)
        {
            for(row = SEARCH_LINE_OTSU_BOTTOM_INIT_ROW - 1; row >= (int16)SearchLine_Otsu_Offline_Row + 4; row--)
            {
                if(SEARCH_LINE_STATE_WHITE == SearchLine_Otsu_Right_State[row])
                {
                    if(SearchLine_Otsu_Right_Border[row + 1] <= 10)
                    {
                        SearchLine_Otsu_Offline_Row = (uint8)(row + 1);
                        break;
                    }

                    scan_row = row;
                    ftsite = 0;
                    while(scan_row >= (int16)SearchLine_Otsu_Offline_Row + 4)
                    {
                        scan_row--;
                        if((SEARCH_LINE_STATE_FOUND == SearchLine_Otsu_Right_State[scan_row]) &&
                           (SEARCH_LINE_STATE_FOUND == SearchLine_Otsu_Right_State[scan_row - 1]) &&
                           (SEARCH_LINE_STATE_FOUND == SearchLine_Otsu_Right_State[scan_row - 2]) &&
                           (SearchLine_Otsu_Right_Border[scan_row - 2] < 70) &&
                           (SearchLine_Otsu_Right_Border[scan_row - 2] > 10))
                        {
                            ftsite = scan_row - 2;
                            break;
                        }
                    }

                    if((0 != ftsite) && (ftsite > SearchLine_Otsu_Offline_Row))
                    {
                        slope = ((float)SearchLine_Otsu_Right_Border[ftsite] -
                                 (float)SearchLine_Otsu_Right_Border[tfsite]) /
                                (float)(ftsite - tfsite);
                        for(fill_row = tfsite; fill_row >= ftsite; fill_row--)
                        {
                            SearchLine_Otsu_Right_Border[fill_row] =
                                SearchLine_Clamp_Otsu_Search_Col((int16)(slope * (float)(fill_row - tfsite) +
                                                                          (float)SearchLine_Otsu_Right_Border[tfsite]));
                            SearchLine_Otsu_Right_State[fill_row] = SEARCH_LINE_STATE_FOUND;
                            SearchLine_Otsu_Row_Valid[fill_row] = 1;
                        }
                        row = ftsite;
                    }
                }
                else
                {
                    tfsite = row + 2;
                }
            }
        }

        for(row = SEARCH_LINE_OTSU_BOTTOM_ROW; row >= SearchLine_Otsu_Offline_Row; row--)
        {
            SearchLine_Otsu_Center_Line[row] =
                (uint8)(((uint16)SearchLine_Otsu_Left_Border[row] + (uint16)SearchLine_Otsu_Right_Border[row]) / 2);
            SearchLine_Otsu_Row_Valid[row] = 1;
        }
    }
}

/* 中线滤波平滑。 */
static void SearchLine_RouteFilter_Otsu(void)
{
    int16 row = 0;
    int16 search_row = 0;
    int16 fill_row = 0;
    int16 center_temp = 0;
    int16 line_temp = 0;
    float center_slope = 0.0f;

    for(row = SEARCH_LINE_OTSU_BOTTOM_ROW - 1; row >= (int16)SearchLine_Otsu_Offline_Row + 5; row--)
    {
        if((SEARCH_LINE_STATE_WHITE == SearchLine_Otsu_Left_State[row]) &&
           (SEARCH_LINE_STATE_WHITE == SearchLine_Otsu_Right_State[row]) &&
           (row <= 45) &&
           (SEARCH_LINE_STATE_WHITE == SearchLine_Otsu_Left_State[row - 1]) &&
           (SEARCH_LINE_STATE_WHITE == SearchLine_Otsu_Right_State[row - 1]))
        {
            search_row = row;
            while(search_row >= (int16)SearchLine_Otsu_Offline_Row + 5)
            {
                search_row--;
                if((SEARCH_LINE_STATE_FOUND == SearchLine_Otsu_Left_State[search_row]) &&
                   (SEARCH_LINE_STATE_FOUND == SearchLine_Otsu_Right_State[search_row]))
                {
                    center_slope = ((float)SearchLine_Otsu_Center_Line[search_row - 1] -
                                    (float)SearchLine_Otsu_Center_Line[row + 2]) /
                                   (float)((search_row - 1) - (row + 2));
                    center_temp = SearchLine_Otsu_Center_Line[row + 2];
                    line_temp = row + 2;
                    for(fill_row = row; fill_row >= search_row; fill_row--)
                    {
                        SearchLine_Otsu_Center_Line[fill_row] =
                            (uint8)SearchLine_Limit_Int32((int16)((float)center_temp +
                                                                   center_slope * (float)(fill_row - line_temp)),
                                                          0,
                                                          SEARCH_LINE_OTSU_W - 1);
                        SearchLine_Otsu_Row_Valid[fill_row] = 1;
                    }
                    row = search_row;
                    break;
                }
            }
        }

        SearchLine_Otsu_Center_Line[row] =
            (uint8)((SearchLine_Otsu_Center_Line[row - 1] +
                     2 * SearchLine_Otsu_Center_Line[row]) / 3);
        SearchLine_Otsu_Row_Valid[row] = 1;
    }
}

/* 固定前瞻加权中线。 */
static void SearchLine_Update_Otsu_Det(void)
{
    int16 tow_point = 0;
    int16 row = 0;
    int16 weight_index = 0;
    float det_temp = 0.0f;
    float unit_all = 0.0f;
    float speed_gain = 0.0f;
    int16 det_value = 0;

    /* 当前工程没有参考那套 `nowspeed/MinSpeed` 反馈值，这里直接复用本工程已生效的 `speed_goal_eff`
     * 做前瞻动态偏移，口径按参考 `GetDet()` 的增益和限幅收回。
     */
    speed_gain = (speed_goal_eff - SEARCH_LINE_OTSU_DET_SPEED_REF) *
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

    tow_point = (int16)((float)SEARCH_LINE_OTSU_DET_TOW_POINT - speed_gain);
    if((SEARCH_LINE_ROAD_RIGHT_CIRQUE == SearchLine_Otsu_Road_Type) ||
       (SEARCH_LINE_ROAD_LEFT_CIRQUE == SearchLine_Otsu_Road_Type))
    {
        tow_point = 15;
    }
    else if(SEARCH_LINE_ROAD_CROSS_TRUE == SearchLine_Otsu_Road_Type)
    {
        tow_point = 22;
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
            det_temp += SearchLine_Otsu_Det_Weight[weight_index] * (float)SearchLine_Otsu_Center_Line[row];
            unit_all += SearchLine_Otsu_Det_Weight[weight_index];
        }

        for(row = tow_point + SEARCH_LINE_OTSU_DET_WINDOW; row > tow_point; row--)
        {
            weight_index = row - tow_point - 1;
            det_temp += SearchLine_Otsu_Det_Weight[weight_index] * (float)SearchLine_Otsu_Center_Line[row];
            unit_all += SearchLine_Otsu_Det_Weight[weight_index];
        }

        det_temp = ((float)SearchLine_Otsu_Center_Line[tow_point] + det_temp) / (unit_all + 1.0f);
    }
    else if(tow_point > (int16)SearchLine_Otsu_Offline_Row)
    {
        for(row = (int16)SearchLine_Otsu_Offline_Row; row < tow_point; row++)
        {
            weight_index = tow_point - row - 1;
            det_temp += SearchLine_Otsu_Det_Weight[weight_index] * (float)SearchLine_Otsu_Center_Line[row];
            unit_all += SearchLine_Otsu_Det_Weight[weight_index];
        }

        for(row = tow_point + tow_point - (int16)SearchLine_Otsu_Offline_Row; row > tow_point; row--)
        {
            weight_index = row - tow_point - 1;
            det_temp += SearchLine_Otsu_Det_Weight[weight_index] * (float)SearchLine_Otsu_Center_Line[row];
            unit_all += SearchLine_Otsu_Det_Weight[weight_index];
        }

        det_temp = ((float)SearchLine_Otsu_Center_Line[tow_point] + det_temp) / (unit_all + 1.0f);
    }
    else if(SearchLine_Otsu_Offline_Row < SEARCH_LINE_OTSU_DET_TOW_POINT_MAX)
    {
        for(row = (int16)SearchLine_Otsu_Offline_Row + 3; row > (int16)SearchLine_Otsu_Offline_Row; row--)
        {
            weight_index = row - tow_point - 1;
            if((weight_index >= 0) && (weight_index < SEARCH_LINE_OTSU_DET_WEIGHT_COUNT))
            {
                det_temp += SearchLine_Otsu_Det_Weight[weight_index] * (float)SearchLine_Otsu_Center_Line[row];
                unit_all += SearchLine_Otsu_Det_Weight[weight_index];
            }
        }

        det_temp = ((float)SearchLine_Otsu_Center_Line[SearchLine_Otsu_Offline_Row] + det_temp) /
                   (unit_all + 1.0f);
    }
    else
    {
        det_temp = (float)SearchLine_Otsu_Det_True;
    }

    det_value = (int16)(det_temp + 0.5f);
    SearchLine_Otsu_Det_True = (uint8)SearchLine_Limit_Int32(det_value, 0, SEARCH_LINE_OTSU_W - 1);
}

/* 直道方差判定。 */
static void SearchLine_Update_Otsu_StraightAcc(void)
{
    int16 row = 0;
    int16 delta = 0;
    uint16 valid_count = 0;
    uint32 sum = 0;
    float variance_acc = 0.0f;

    SearchLine_Otsu_Straight_Acc = 0;
    SearchLine_Otsu_Variance_Acc = 0;

    if(SearchLine_Otsu_Offline_Row >= 54)
    {
        return;
    }

    for(row = 55; row > ((int16)SearchLine_Otsu_Offline_Row + 1); row--)
    {
        delta = (int16)SearchLine_Otsu_Center_Line[row] - SEARCH_LINE_OTSU_MIDDLE_LINE;
        sum += (uint32)(delta * delta);
        valid_count++;
    }

    if(0 == valid_count)
    {
        return;
    }

    variance_acc = (float)sum / (float)valid_count;
    SearchLine_Otsu_Variance_Acc = (uint16)(variance_acc + 0.5f);
    if((variance_acc < (float)SEARCH_LINE_OTSU_VARIANCE_ACC_LIMIT) &&
       (SearchLine_Otsu_Offline_Row <= SEARCH_LINE_OTSU_STRAIGHT_OFFLINE_MAX) &&
       (SearchLine_Otsu_Left_Line <= SEARCH_LINE_OTSU_STRAIGHT_LOST_LINE_MAX) &&
       (SearchLine_Otsu_Right_Line <= SEARCH_LINE_OTSU_STRAIGHT_LOST_LINE_MAX))
    {
        SearchLine_Otsu_Straight_Acc = 1;
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

    SearchLine_Otsu_Steer_Offset = (int16)SearchLine_Otsu_Det_True - SEARCH_LINE_OTSU_MIDDLE_LINE;
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
    SearchLine_Otsu_Steer_Command = (uint8)SearchLine_Limit_Int32(command_angle,
                                                                   (int16)min_angle,
                                                                   (int16)max_angle);
}

static uint8 SearchLine_Calc_Otsu_Threshold(void)
{
    uint16 row = 0;
    uint16 col = 0;
    uint16 gray = 0;
    uint16 total_pixels = SEARCH_LINE_OTSU_W * SEARCH_LINE_OTSU_H;
    uint16 bg_weight = 0;
    uint16 fg_weight = 0;
    uint32 gray_sum = 0;
    uint32 bg_sum = 0;
    uint8 raw_threshold = SEARCH_LINE_OTSU_THRESHOLD_MIN;
    uint8 threshold = SEARCH_LINE_OTSU_THRESHOLD_MIN;
    float best_score = -1.0f;
    float mean_bg = 0.0f;
    float mean_fg = 0.0f;
    float score = 0.0f;
    float delta = 0.0f;

    if(SEARCH_LINE_OTSU_THRESHOLD_INTERVAL > 1)
    {
        if((0 != SearchLine_Otsu_Threshold_Frame_Count) &&
           (SearchLine_Otsu_Threshold_Frame_Count < SEARCH_LINE_OTSU_THRESHOLD_INTERVAL))
        {
            SearchLine_Otsu_Threshold_Frame_Count++;
            return SearchLine_Otsu_Threshold_Cache;
        }
    }

    for(gray = 0; gray < 256; gray++)
    {
        SearchLine_Otsu_Histogram[gray] = 0;
    }

    for(row = 0; row < SEARCH_LINE_OTSU_H; row++)
    {
        for(col = 0; col < SEARCH_LINE_OTSU_W; col++)
        {
            gray = SearchLine_Get_Otsu_Gray((uint8)row, (uint8)col);
            SearchLine_Otsu_Histogram[gray]++;
            gray_sum += gray;
        }
    }

    for(gray = 0; gray < SEARCH_LINE_OTSU_THRESHOLD_CAP; gray++)
    {
        bg_weight = (uint16)(bg_weight + SearchLine_Otsu_Histogram[gray]);
        if(0 == bg_weight)
        {
            continue;
        }

        fg_weight = (uint16)(total_pixels - bg_weight);
        if(0 == fg_weight)
        {
            break;
        }

        bg_sum += (uint32)gray * (uint32)SearchLine_Otsu_Histogram[gray];
        mean_bg = (float)bg_sum / (float)bg_weight;
        mean_fg = (float)(gray_sum - bg_sum) / (float)fg_weight;
        delta = mean_bg - mean_fg;
        score = (float)bg_weight * (float)fg_weight * delta * delta;
        if(score > best_score)
        {
            best_score = score;
            raw_threshold = (uint8)gray;
        }
    }

    if(raw_threshold < SEARCH_LINE_OTSU_THRESHOLD_MIN)
    {
        raw_threshold = SEARCH_LINE_OTSU_THRESHOLD_MIN;
    }

    threshold = raw_threshold;
    if(threshold < SEARCH_LINE_OTSU_THRESHOLD_STATIC)
    {
        threshold = SEARCH_LINE_OTSU_THRESHOLD_STATIC;
    }

    SearchLine_Otsu_Threshold_Raw_Cache = raw_threshold;
    SearchLine_Otsu_Threshold_Cache = threshold;
    SearchLine_Otsu_Threshold_Frame_Count = 1;
    return threshold;
}

/* 图像二值化。 */
static void SearchLine_Binarize_Otsu_Image(void)
{
    uint16 row = 0;
    uint16 col = 0;
    uint8 threshold = 0;
    uint8 gray = 0;
    int16 row_threshold = 0;

    threshold = SearchLine_Calc_Otsu_Threshold();

    for(row = 0; row < SEARCH_LINE_OTSU_H; row++)
    {
        for(col = 0; col < SEARCH_LINE_OTSU_W; col++)
        {
            row_threshold = threshold;
            if((col <= SEARCH_LINE_OTSU_LEFT_COMP_END) ||
               (col >= SEARCH_LINE_OTSU_RIGHT_COMP_START))
            {
                row_threshold -= 10;
            }

            row_threshold = (int16)SearchLine_Limit_Int32(row_threshold, 0, 255);
            gray = SearchLine_Get_Otsu_Gray((uint8)row, (uint8)col);
            SearchLine_Otsu_Binary[row][col] = (gray > (uint8)row_threshold) ? 1 : 0;
        }
    }
}

/* 图像压缩、图像二值化、底边初始化、逐行搜边、延长线补边、中线滤波。 */
static void SearchLine_Process_Otsu(void)
{
    /* 行列映射表初始化。 */
    SearchLine_Init_Otsu_Map();
    /* 图像压缩。 */
    SearchLine_Compress_Otsu_Image();
    /* 底边状态初始化。 */
    SearchLine_Clear_Otsu_State();
    /* 图像二值化。 */
    SearchLine_Binarize_Otsu_Image();
    /* 参考里保留了 Pixle_Filter 接口，这里默认先不上主链。 */
    if(SEARCH_LINE_OTSU_PIXEL_FILTER_ENABLE)
    {
        SearchLine_Pixle_Filter_Otsu();
    }
    /* 底边初始化。 */
    SearchLine_Init_Otsu_BottomRows();
    /* 逐行向上搜边。 */
    SearchLine_DrawLinesProcess_Otsu();
    /* 十字/元素边界观测支线。 */
    SearchLine_Search_Border_Otsu();
    /* 延长线补边。 */
    SearchLine_DrawExtensionLine_Otsu();
    /* 中线滤波平滑。 */
    SearchLine_RouteFilter_Otsu();
    /* 直道方差判定。 */
    SearchLine_Update_Otsu_StraightAcc();
    /* 固定前瞻加权中线。 */
    SearchLine_Update_Otsu_Det();
    /* 舵机位置式 PD 预览。 */
    SearchLine_Update_Otsu_SteerPreview();
}

void SearchLine_Process(void)
{
    gpio_set_level(IO_P52, 0);
    SearchLine_Process_Otsu();
    gpio_set_level(IO_P52, 1);
}

uint8 SearchLine_GetOtsuThreshold(void)
{
    return SearchLine_Otsu_Threshold_Cache;
}

uint8 SearchLine_GetRawOtsuThreshold(void)
{
    return SearchLine_Otsu_Threshold_Raw_Cache;
}

uint8 SearchLine_GetSteerCommand(void)
{
    return SearchLine_Otsu_Steer_Command;
}

uint8 SearchLine_GetStraightAcc(void)
{
    return SearchLine_Otsu_Straight_Acc;
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

/* 显示压缩二值图。 */
void SearchLine_DrawBinaryPreview(void)
{
    char threshold_text[6];
    char offset_text[4];
    char command_text[4];
    uint16 x = 0;
    uint16 y = 0;
    uint8 row = 0;
    uint8 left_col = 0;
    uint8 right_col = 0;
    uint8 center_col = 0;
    uint16 offset_abs = 0;
    uint16 wheel_text_y = 0;
    uint16 wheel_enc_y = 0;
    uint16 wheel_ref_y = 0;
    uint8 command_value = 0;
    int16 speed_goal_display = 0;
    int16 ref_left_display = 0;
    int16 ref_right_display = 0;

    ips200_show_gray_image(0,
                           0,
                           SearchLine_Otsu_Binary[0],
                           SEARCH_LINE_OTSU_W,
                           SEARCH_LINE_OTSU_H,
                           CAMERA_VALID_W,
                           CAMERA_RAW_H,
                           1);

    /* 相机页底部显示当前 OTSU 阈值，方便边看图边确认二值门限。 */
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_show_string(0, (uint16)(CAMERA_RAW_H + 4), "thr      ");
    if(SearchLine_Otsu_Threshold_Cache >= 100)
    {
        threshold_text[0] = (char)('0' + SearchLine_Otsu_Threshold_Cache / 100U);
        threshold_text[1] = (char)('0' + (SearchLine_Otsu_Threshold_Cache / 10U) % 10U);
        threshold_text[2] = (char)('0' + SearchLine_Otsu_Threshold_Cache % 10U);
        threshold_text[3] = '\0';
    }
    else if(SearchLine_Otsu_Threshold_Cache >= 10)
    {
        threshold_text[0] = (char)('0' + SearchLine_Otsu_Threshold_Cache / 10U);
        threshold_text[1] = (char)('0' + SearchLine_Otsu_Threshold_Cache % 10U);
        threshold_text[2] = '\0';
    }
    else
    {
        threshold_text[0] = (char)('0' + SearchLine_Otsu_Threshold_Cache);
        threshold_text[1] = '\0';
    }
    ips200_show_string(32, (uint16)(CAMERA_RAW_H + 4), threshold_text);

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

    ips200_show_string(0, (uint16)(CAMERA_RAW_H + 20), "ofs");
    ips200_show_string(72, (uint16)(CAMERA_RAW_H + 20), "cmd");
    ips200_show_string(24, (uint16)(CAMERA_RAW_H + 20), offset_text);
    ips200_show_string(104, (uint16)(CAMERA_RAW_H + 20), command_text);

    wheel_text_y = (uint16)(CAMERA_RAW_H + 36);
    wheel_enc_y = (uint16)(CAMERA_RAW_H + 52);
    wheel_ref_y = (uint16)(CAMERA_RAW_H + 68);
    speed_goal_display = (int16)(speed_goal_eff + 0.5f);
    if(ref_left_target >= 0.0f)
    {
        ref_left_display = (int16)(ref_left_target + 0.5f);
    }
    else
    {
        ref_left_display = (int16)(ref_left_target - 0.5f);
    }
    if(ref_right_target >= 0.0f)
    {
        ref_right_display = (int16)(ref_right_target + 0.5f);
    }
    else
    {
        ref_right_display = (int16)(ref_right_target - 0.5f);
    }

    ips200_show_string(0, wheel_text_y, "sa");
    ips200_show_int32(16, wheel_text_y, (int32)SearchLine_Otsu_Straight_Acc, 1);
    ips200_show_string(48, wheel_text_y, "sg");
    ips200_show_int32(64, wheel_text_y, (int32)speed_goal_display, 3);

    ips200_show_string(0, wheel_enc_y, "el");
    ips200_show_int32(16, wheel_enc_y, (int32)enc_l_f, 4);
    ips200_show_string(96, wheel_enc_y, "er");
    ips200_show_int32(112, wheel_enc_y, (int32)enc_r_f, 4);

    ips200_show_string(0, wheel_ref_y, "rl");
    ips200_show_int32(16, wheel_ref_y, (int32)ref_left_display, 4);
    ips200_show_string(96, wheel_ref_y, "rr");
    ips200_show_int32(112, wheel_ref_y, (int32)ref_right_display, 4);

    for(row = SearchLine_Otsu_Offline_Row; row <= SEARCH_LINE_OTSU_BOTTOM_ROW; row++)
    {
        if(!SearchLine_Otsu_Row_Valid[row])
        {
            continue;
        }

        left_col = SearchLine_Otsu_Left_Border[row];
        right_col = SearchLine_Otsu_Right_Border[row];
        center_col = SearchLine_Otsu_Center_Line[row];

        y = (uint16)(((uint32)row * (uint32)CAMERA_RAW_H + (uint32)(SEARCH_LINE_OTSU_H / 2)) /
                     (uint32)SEARCH_LINE_OTSU_H);
        x = (uint16)(((uint32)left_col * (uint32)CAMERA_VALID_W + (uint32)(SEARCH_LINE_OTSU_W / 2)) /
                     (uint32)SEARCH_LINE_OTSU_W);
        ips200_draw_point(x, y, RGB565_GREEN);
        ips200_draw_point(x, (uint16)SearchLine_Limit_Int32((int32)y + 1, 0, CAMERA_RAW_H - 1), RGB565_GREEN);

        x = (uint16)(((uint32)right_col * (uint32)CAMERA_VALID_W + (uint32)(SEARCH_LINE_OTSU_W / 2)) /
                     (uint32)SEARCH_LINE_OTSU_W);
        ips200_draw_point(x, y, RGB565_GREEN);
        ips200_draw_point(x, (uint16)SearchLine_Limit_Int32((int32)y + 1, 0, CAMERA_RAW_H - 1), RGB565_GREEN);

        x = (uint16)(((uint32)center_col * (uint32)CAMERA_VALID_W + (uint32)(SEARCH_LINE_OTSU_W / 2)) /
                     (uint32)SEARCH_LINE_OTSU_W);
        ips200_draw_point(x, y, RGB565_RED);
        ips200_draw_point(x, (uint16)SearchLine_Limit_Int32((int32)y + 1, 0, CAMERA_RAW_H - 1), RGB565_RED);
    }
}
