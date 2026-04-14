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
#define SEARCH_LINE_OTSU_MIDDLE_LINE        (SEARCH_LINE_OTSU_W / 2 - 1)
#define SEARCH_LINE_OTSU_BOUNDARY_BOTTOM_ROW (SEARCH_LINE_OTSU_H - 2)
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
#define SEARCH_LINE_STATE_INIT              ('F') /* 初始态，当前行未完成搜边。 */
#define SEARCH_LINE_STATE_FOUND             ('T') /* 找到跳变边界。 */
#define SEARCH_LINE_STATE_WHITE             ('W') /* 扫描窗内全白，当前侧无明确边界。 */
#define SEARCH_LINE_STATE_BLACK             ('H') /* 扫描窗内全黑，当前侧搜索失败。 */
#define SEARCH_LINE_ROAD_NORMAL             (0)
#define SEARCH_LINE_ROAD_STRAIGHT           (1)
#define SEARCH_LINE_ROAD_CROSS              (2)
#define SEARCH_LINE_ROAD_RAMP               (3)
#define SEARCH_LINE_ROAD_LEFT_CIRQUE        (4)
#define SEARCH_LINE_ROAD_RIGHT_CIRQUE       (5)
#define SEARCH_LINE_ROAD_FORK_IN            (6)
#define SEARCH_LINE_ROAD_FORK_OUT           (7)
#define SEARCH_LINE_ROAD_BARN_OUT           (8)
#define SEARCH_LINE_ROAD_BARN_IN            (9)
#define SEARCH_LINE_ROAD_CROSS_TRUE         (10)
#define SEARCH_LINE_ROAD_ZEBRA              (11)

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
/* 19 国一圆环链依赖的第一组边界跟踪结果。 */
static uint8 SearchLine_Otsu_Left_Boundary_First[SEARCH_LINE_OTSU_H] = {0};
/* 19 国一圆环链依赖的第一组边界跟踪结果。 */
static uint8 SearchLine_Otsu_Right_Boundary_First[SEARCH_LINE_OTSU_H] = {0};
/* 19 国一圆环链依赖的第二组边界跟踪结果。 */
static uint8 SearchLine_Otsu_Left_Boundary[SEARCH_LINE_OTSU_H] = {0};
/* 19 国一圆环链依赖的第二组边界跟踪结果。 */
static uint8 SearchLine_Otsu_Right_Boundary[SEARCH_LINE_OTSU_H] = {0};
static uint8 SearchLine_Otsu_Left_Extend_Allowed = 1;
static uint8 SearchLine_Otsu_Right_Extend_Allowed = 1;
static uint8 SearchLine_Otsu_Map_Ready = 0;
static uint8 SearchLine_Otsu_Offline_Row = SEARCH_LINE_OTSU_OFFLINE_MIN;
static uint8 SearchLine_Otsu_Offline_Boundary_Row = 5;
/* 对齐参考代码的单边丢线计数。 */
static uint8 SearchLine_Otsu_Left_Line = 0;
static uint8 SearchLine_Otsu_Right_Line = 0;
static uint8 SearchLine_Otsu_White_Line = 0;
static uint8 SearchLine_Otsu_White_Line_Left = 0;
static uint8 SearchLine_Otsu_White_Line_Right = 0;
/* 对齐参考代码的前瞻行和加权中线。 */
static uint8 SearchLine_Otsu_TowPoint_True = SEARCH_LINE_OTSU_DET_TOW_POINT;
static uint8 SearchLine_Otsu_Det_True = SEARCH_LINE_OTSU_MIDDLE_LINE;
/* 对齐参考代码的直道判定结果。 */
static uint8 SearchLine_Otsu_Straight_Acc = 0;
static uint16 SearchLine_Otsu_Variance_Acc = 0;
/* 对齐参考代码的舵机位置式 PD 预览量。 */
static int16 SearchLine_Otsu_Steer_Offset = 0;
static uint8 SearchLine_Otsu_Steer_Command = CAR_SERVO_CENTER_ANGLE;
static uint16 SearchLine_Otsu_Steer_P_Tenth = FLASH_STEER_P_DEFAULT_TENTH;
static uint16 SearchLine_Otsu_Steer_D_Tenth = FLASH_STEER_D_DEFAULT_TENTH;
static float SearchLine_Otsu_Steer_Last_Error = 0.0f;
static uint8 SearchLine_Otsu_Threshold_Raw_Cache = SEARCH_LINE_OTSU_THRESHOLD_MIN;
static uint8 SearchLine_Otsu_Threshold_Cache = SEARCH_LINE_OTSU_THRESHOLD_MIN;
static uint8 SearchLine_Otsu_Threshold_Frame_Count = 0;
static uint8 SearchLine_Otsu_Road_Type = SEARCH_LINE_ROAD_NORMAL;
static uint8 SearchLine_Otsu_Cirque_Out_In = 'F';
static uint8 SearchLine_Otsu_Cirque_Pass = 'F';
static uint8 SearchLine_Otsu_Cirque_Out = 'F';
static uint8 SearchLine_Otsu_Cirque_Off = 'F';
static uint8 SearchLine_Otsu_Ring_Element = 0;
static uint8 SearchLine_Otsu_Ring_Size = 0;
static uint8 SearchLine_Otsu_Ring_Flag = 0;
static uint8 SearchLine_Otsu_Ring_Bottom_Ok = 0;
static uint16 SearchLine_Otsu_Cirque_Left_Count = 0;
static uint16 SearchLine_Otsu_Cirque_Right_Count = 0;
static uint16 SearchLine_Otsu_Ring_Stage_Num = 0;
static uint16 SearchLine_Otsu_Ring_Point_Y = 0;
static int16 SearchLine_Otsu_Ring_Straight_Judge_Tenth = -1;
static uint8 SearchLine_Preview_Label_Ready = 0;
static uint8 SearchLine_Preview_Last_Threshold = 0xFF;
static int16 SearchLine_Preview_Last_Offset = 32767;
static uint8 SearchLine_Preview_Last_Command = 0xFF;
static uint8 SearchLine_Preview_Last_Ring_Element = 0xFF;
static uint8 SearchLine_Preview_Last_Ring_Flag = 0xFF;
static uint8 SearchLine_Preview_Last_Ring_Size = 0xFF;
static uint8 SearchLine_Preview_Last_Offline_Row = 0xFF;
static uint8 SearchLine_Preview_Last_White_Line = 0xFF;
static uint8 SearchLine_Preview_Last_Ring_Bottom_Ok = 0xFF;
static uint16 SearchLine_Preview_Last_Cirque_Left_Count = 0xFFFF;
static uint16 SearchLine_Preview_Last_Cirque_Right_Count = 0xFFFF;
static uint8 SearchLine_Preview_Last_Ring_Left_Line = 0xFF;
static uint8 SearchLine_Preview_Last_Ring_Right_Line = 0xFF;
static uint8 SearchLine_Preview_Last_Ring_Left_Line_RightPanel = 0xFF;
static uint8 SearchLine_Preview_Last_Ring_Right_Line_RightPanel = 0xFF;
static uint16 SearchLine_Preview_Last_Ring_Stage_Num = 0xFFFF;
static uint16 SearchLine_Preview_Last_Ring_Point_Y = 0xFFFF;
static int16 SearchLine_Preview_Last_Ring_Straight_Judge_Tenth = 32767;
static float SearchLine_Otsu_Det_Weight[SEARCH_LINE_OTSU_DET_WEIGHT_COUNT] =
{
    0.96f, 0.92f, 0.88f, 0.83f, 0.77f,
    0.71f, 0.65f, 0.59f, 0.53f, 0.47f
};
static const uint8 SearchLine_Otsu_Half_Road_Wide[SEARCH_LINE_OTSU_H] =
{
    6, 7, 7, 8, 8, 9, 9, 9, 10, 10,
    11, 11, 11, 11, 11, 12, 12, 13, 13, 14,
    14, 14, 14, 15, 15, 16, 16, 16, 17, 17,
    17, 18, 18, 19, 19, 20, 20, 20, 21, 21,
    21, 22, 22, 23, 23, 23, 24, 24, 25, 25,
    26, 26, 26, 26, 27, 27, 27, 28, 28, 30
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

static uint8 SearchLine_Get_Otsu_Binary_Pixel(int16 row, int16 col)
{
    if((row <= 0) || (row >= (SEARCH_LINE_OTSU_BOUNDARY_BOTTOM_ROW + 1)) ||
       (col <= 0) || (col >= (SEARCH_LINE_OTSU_W - 1)))
    {
        return 0;
    }

    return SearchLine_Otsu_Binary[row][col];
}

/* 圆环支线缓存初始化。 */
static void SearchLine_Clear_Otsu_BorderTraceState(void)
{
    uint16 row = 0;

    SearchLine_Otsu_Offline_Boundary_Row = 5;
    SearchLine_Otsu_White_Line_Left = 0;
    SearchLine_Otsu_White_Line_Right = 0;

    for(row = 0; row < SEARCH_LINE_OTSU_H; row++)
    {
        SearchLine_Otsu_Left_Boundary_First[row] = 0;
        SearchLine_Otsu_Right_Boundary_First[row] = SEARCH_LINE_OTSU_W - 1;
        SearchLine_Otsu_Left_Boundary[row] = 0;
        SearchLine_Otsu_Right_Boundary[row] = SEARCH_LINE_OTSU_W - 1;
    }
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
    SearchLine_Otsu_Ring_Bottom_Ok = 0;
    SearchLine_Otsu_Cirque_Left_Count = 0;
    SearchLine_Otsu_Cirque_Right_Count = 0;
    SearchLine_Otsu_Ring_Stage_Num = 0;
    SearchLine_Otsu_Ring_Point_Y = 0;
    SearchLine_Otsu_Ring_Straight_Judge_Tenth = -1;
    SearchLine_Clear_Otsu_BorderTraceState();

    for(row = 0; row < SEARCH_LINE_OTSU_H; row++)
    {
        SearchLine_Otsu_Left_Border[row] = 0;
        SearchLine_Otsu_Right_Border[row] = SEARCH_LINE_OTSU_W - 1;
        SearchLine_Otsu_Center_Line[row] = SEARCH_LINE_OTSU_MIDDLE_LINE;
        SearchLine_Otsu_Row_Valid[row] = 0;
        SearchLine_Otsu_Left_State[row] = SEARCH_LINE_STATE_INIT;
        SearchLine_Otsu_Right_State[row] = SEARCH_LINE_STATE_INIT;
    }
}

/* 19 国一 Search_Border_OTSU 的边界跟踪支线。
 * 这一支先只做观测缓存，不直接改普通赛道主链的左右边界。
 */
static void SearchLine_Search_Border_Otsu(void)
{
    uint8 bottom_row = SEARCH_LINE_OTSU_BOUNDARY_BOTTOM_ROW;
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

    if((SEARCH_LINE_OTSU_BOUNDARY_BOTTOM_ROW >= SEARCH_LINE_OTSU_H) ||
       !SearchLine_Otsu_Row_Valid[bottom_row])
    {
        return;
    }

    left_y = bottom_row;
    left_x = SearchLine_Otsu_Left_Border[bottom_row];
    right_y = bottom_row;
    right_x = SearchLine_Otsu_Right_Border[bottom_row];
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
            SearchLine_Otsu_Offline_Boundary_Row = trace_row;
            break;
        }

        if((trace_row >= left_probe_y) && (trace_row >= right_probe_y))
        {
            if(trace_row < SearchLine_Otsu_Offline_Boundary_Row)
            {
                SearchLine_Otsu_Offline_Boundary_Row = trace_row;
                break;
            }
            else
            {
                trace_row--;
            }
        }

        if((left_probe_y > trace_row) || (trace_row == SearchLine_Otsu_Offline_Boundary_Row))
        {
            next_row = (int16)left_y + SearchLine_Otsu_Left_Rule[0][2 * left_direction + 1];
            next_col = (int16)left_x + SearchLine_Otsu_Left_Rule[0][2 * left_direction];
            left_probe_y = (uint8)SearchLine_Limit_Int32(next_row, 0, SEARCH_LINE_OTSU_H - 1);
            left_probe_x = (uint8)SearchLine_Limit_Int32(next_col, 0, SEARCH_LINE_OTSU_W - 1);

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
                left_probe_y = (uint8)SearchLine_Limit_Int32(next_row, 0, SEARCH_LINE_OTSU_H - 1);
                left_probe_x = (uint8)SearchLine_Limit_Int32(next_col, 0, SEARCH_LINE_OTSU_W - 1);

                if(0 == SearchLine_Get_Otsu_Binary_Pixel(next_row, next_col))
                {
                    left_y = (uint8)SearchLine_Limit_Int32((int16)left_y +
                                                           SearchLine_Otsu_Left_Rule[0][2 * left_direction + 1],
                                                           0,
                                                           SEARCH_LINE_OTSU_H - 1);
                    left_x = (uint8)SearchLine_Limit_Int32((int16)left_x +
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
                    left_y = (uint8)SearchLine_Limit_Int32(next_row, 0, SEARCH_LINE_OTSU_H - 1);
                    left_x = (uint8)SearchLine_Limit_Int32(next_col, 0, SEARCH_LINE_OTSU_W - 1);
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

        if((right_probe_y > trace_row) || (trace_row == SearchLine_Otsu_Offline_Boundary_Row))
        {
            next_row = (int16)right_y + SearchLine_Otsu_Right_Rule[0][2 * right_direction + 1];
            next_col = (int16)right_x + SearchLine_Otsu_Right_Rule[0][2 * right_direction];
            right_probe_y = (uint8)SearchLine_Limit_Int32(next_row, 0, SEARCH_LINE_OTSU_H - 1);
            right_probe_x = (uint8)SearchLine_Limit_Int32(next_col, 0, SEARCH_LINE_OTSU_W - 1);

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
                right_probe_y = (uint8)SearchLine_Limit_Int32(next_row, 0, SEARCH_LINE_OTSU_H - 1);
                right_probe_x = (uint8)SearchLine_Limit_Int32(next_col, 0, SEARCH_LINE_OTSU_W - 1);

                if(0 == SearchLine_Get_Otsu_Binary_Pixel(next_row, next_col))
                {
                    right_y = (uint8)SearchLine_Limit_Int32((int16)right_y +
                                                            SearchLine_Otsu_Right_Rule[0][2 * right_direction + 1],
                                                            0,
                                                            SEARCH_LINE_OTSU_H - 1);
                    right_x = (uint8)SearchLine_Limit_Int32((int16)right_x +
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
                    right_y = (uint8)SearchLine_Limit_Int32(next_row, 0, SEARCH_LINE_OTSU_H - 1);
                    right_x = (uint8)SearchLine_Limit_Int32(next_col, 0, SEARCH_LINE_OTSU_W - 1);
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
            SearchLine_Otsu_Offline_Boundary_Row = trace_row;
            break;
        }
    }

    for(row = bottom_row; row > (uint8)(SearchLine_Otsu_Offline_Boundary_Row + 1); row--)
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
    return (uint8)SearchLine_Limit_Int32(value, 1, SEARCH_LINE_OTSU_W - 2);
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

/* 参考 Pixle_Filter 的噪点补白，当前默认不上主链。 */
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
        low = (int16)SearchLine_Otsu_Right_Border[row + 1] - SEARCH_LINE_OTSU_SCAN_WINDOW;
        high = (int16)SearchLine_Otsu_Right_Border[row + 1] + SEARCH_LINE_OTSU_SCAN_WINDOW;
        low = SearchLine_Clamp_Otsu_Search_Col(low);
        high = SearchLine_Clamp_Otsu_Search_Col(high);
        right_state = SearchLine_Find_Otsu_JumpPoint(row_data, 0, low, high, &right_point);

        low = (int16)SearchLine_Otsu_Left_Border[row + 1] - SEARCH_LINE_OTSU_SCAN_WINDOW;
        high = (int16)SearchLine_Otsu_Left_Border[row + 1] + SEARCH_LINE_OTSU_SCAN_WINDOW;
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

    if(SearchLine_Otsu_White_Line >= (uint8)(SearchLine_Otsu_TowPoint_True - 15))
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
                    /* 圆环阶段统计依赖原始丢边标志，这里只补边界，不改丢边状态。 */
                    for(fill_row = tfsite; fill_row >= ftsite; fill_row--)
                    {
                        SearchLine_Otsu_Left_Border[fill_row] =
                            SearchLine_Clamp_Otsu_Search_Col((int16)(slope * (float)(fill_row - tfsite) +
                                                                      (float)SearchLine_Otsu_Left_Border[tfsite]));
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
                    /* 圆环阶段统计依赖原始丢边标志，这里只补边界，不改丢边状态。 */
                    for(fill_row = tfsite; fill_row >= ftsite; fill_row--)
                    {
                        SearchLine_Otsu_Right_Border[fill_row] =
                            SearchLine_Clamp_Otsu_Search_Col((int16)(slope * (float)(fill_row - tfsite) +
                                                                      (float)SearchLine_Otsu_Right_Border[tfsite]));
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
                    row = search_row - 1;
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

    if(((SEARCH_LINE_ROAD_RIGHT_CIRQUE == SearchLine_Otsu_Road_Type) ||
        (SEARCH_LINE_ROAD_LEFT_CIRQUE == SearchLine_Otsu_Road_Type)) &&
       ('F' == SearchLine_Otsu_Cirque_Off))
    {
        tow_point = 15;
    }
    else
    {
        tow_point = (int16)((float)SEARCH_LINE_OTSU_DET_TOW_POINT - speed_gain);
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

static float SearchLine_Straight_Judge_Otsu(uint8 dir, uint8 start_row, uint8 end_row)
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

static uint16 SearchLine_Cirque_Or_Cross_Otsu(uint8 type, uint8 start_row)
{
    uint16 num = 0;
    uint8 row = 0;
    uint8 end_row = 0;
    int16 col = 0;

    if(start_row >= SEARCH_LINE_OTSU_H)
    {
        return 0;
    }

    end_row = (uint8)SearchLine_Limit_Int32((int16)start_row + 10, 0, SEARCH_LINE_OTSU_H);
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

static void SearchLine_Element_Judgment_Left_Rings_Otsu(void)
{
    uint8 ring_ysite = 3;
    uint8 point1_y = 0;
    uint8 point2_y = 0;
    uint8 judge_start_y = 0;
    uint8 row = 0;
    uint8 ring_help_flag = 0;

    for(row = SEARCH_LINE_OTSU_BOUNDARY_BOTTOM_ROW; row > ring_ysite; row--)
    {
        if((int16)SearchLine_Otsu_Left_Boundary_First[row] -
           (int16)SearchLine_Otsu_Left_Boundary_First[row - 1] > 4)
        {
            point1_y = row;
            break;
        }
    }
    for(row = SEARCH_LINE_OTSU_BOUNDARY_BOTTOM_ROW; row > ring_ysite; row--)
    {
        if((int16)SearchLine_Otsu_Left_Boundary[row + 1] -
           (int16)SearchLine_Otsu_Left_Boundary[row] > 4)
        {
            point2_y = row;
            break;
        }
    }

    judge_start_y = point1_y;
    if(judge_start_y > (SEARCH_LINE_OTSU_H - 7))
    {
        judge_start_y = SEARCH_LINE_OTSU_H - 7;
    }
    for(row = judge_start_y; row > 10; row--)
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
        SearchLine_Otsu_Ring_Size = 0;
        SearchLine_Otsu_Road_Type = SEARCH_LINE_ROAD_LEFT_CIRQUE;
    }
}

static void SearchLine_Element_Judgment_Right_Rings_Otsu(void)
{
    uint8 ring_ysite = 3;
    uint8 point1_y = 0;
    uint8 point2_y = 0;
    uint8 judge_start_y = 0;
    uint8 row = 0;
    uint8 ring_help_flag = 0;

    for(row = SEARCH_LINE_OTSU_BOUNDARY_BOTTOM_ROW; row > ring_ysite; row--)
    {
        if((int16)SearchLine_Otsu_Right_Boundary_First[row - 1] -
           (int16)SearchLine_Otsu_Right_Boundary_First[row] > 4)
        {
            point1_y = row;
            break;
        }
    }
    for(row = SEARCH_LINE_OTSU_BOUNDARY_BOTTOM_ROW; row > ring_ysite; row--)
    {
        if((int16)SearchLine_Otsu_Right_Boundary[row] -
           (int16)SearchLine_Otsu_Right_Boundary[row + 1] > 4)
        {
            point2_y = row;
            break;
        }
    }

    judge_start_y = point1_y;
    if(judge_start_y > (SEARCH_LINE_OTSU_H - 7))
    {
        judge_start_y = SEARCH_LINE_OTSU_H - 7;
    }
    for(row = judge_start_y; row > 10; row--)
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
        SearchLine_Otsu_Ring_Size = 0;
        SearchLine_Otsu_Road_Type = SEARCH_LINE_ROAD_RIGHT_CIRQUE;
    }
}

static void SearchLine_Element_Handle_Left_Rings_Otsu(void)
{
    int16 num = 0;
    int16 black = 0;
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

    /* 参考旧分支用左侧黑列区分大小圆环。 */
    if(0 == SearchLine_Otsu_Ring_Size)
    {
        black = 0;
        for(row = 30; row > 0; row--)
        {
            if(0 == SearchLine_Get_Otsu_Binary_Pixel(row, 5))
            {
                black++;
            }
        }
        if(black > 10)
        {
            SearchLine_Otsu_Ring_Size = 1;
        }
        else
        {
            SearchLine_Otsu_Ring_Size = 2;
        }
    }

    for(row = 55; row > 30; row--)
    {
        if(SEARCH_LINE_STATE_WHITE == SearchLine_Otsu_Left_State[row])
        {
            num++;
        }
        if((SEARCH_LINE_STATE_WHITE == SearchLine_Otsu_Left_State[row + 3]) &&
           (SEARCH_LINE_STATE_WHITE == SearchLine_Otsu_Left_State[row + 2]) &&
           (SEARCH_LINE_STATE_WHITE == SearchLine_Otsu_Left_State[row + 1]) &&
           (SEARCH_LINE_STATE_FOUND == SearchLine_Otsu_Left_State[row]))
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
            if(SEARCH_LINE_STATE_WHITE == SearchLine_Otsu_Left_State[row])
            {
                num++;
            }
        }
        if(num < 5)
        {
            SearchLine_Otsu_Road_Type = SEARCH_LINE_ROAD_NORMAL;
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
        for(row = SEARCH_LINE_OTSU_BOTTOM_ROW; row > (int16)SearchLine_Otsu_Offline_Row; row--)
        {
            SearchLine_Otsu_Center_Line[row] =
                (uint8)SearchLine_Limit_Int32((int16)SearchLine_Otsu_Right_Border[row] -
                                              (int16)SearchLine_Otsu_Half_Road_Wide[row],
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
                if((SEARCH_LINE_STATE_FOUND == SearchLine_Otsu_Left_State[row]) &&
                   (SEARCH_LINE_STATE_FOUND == SearchLine_Otsu_Left_State[row + 1]) &&
                   (SEARCH_LINE_STATE_WHITE == SearchLine_Otsu_Left_State[row + 2]) &&
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
                    (uint8)SearchLine_Limit_Int32((int16)((float)flag_x_1 +
                                                          slope_rings * (float)(row - flag_y_1)),
                                                  0,
                                                  SEARCH_LINE_OTSU_W - 1);
                SearchLine_Otsu_Center_Line[row] =
                    (uint8)SearchLine_Limit_Int32(((int16)SearchLine_Otsu_Right_Border[row] +
                                                   (int16)SearchLine_Otsu_Left_Border[row]) / 2,
                                                  4,
                                                  SEARCH_LINE_OTSU_W - 1);
            }

            SearchLine_Otsu_Right_Border[flag_y_1] =
                (uint8)SearchLine_Limit_Int32(flag_x_1, 0, SEARCH_LINE_OTSU_W - 1);

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
                            (uint8)SearchLine_Limit_Int32(col, 0, SEARCH_LINE_OTSU_W - 1);
                        SearchLine_Otsu_Center_Line[row] =
                            (uint8)SearchLine_Limit_Int32(((int16)SearchLine_Otsu_Right_Border[row] +
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
            SearchLine_Otsu_Center_Line[row] = 15;
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
                (uint8)SearchLine_Limit_Int32((((int16)SearchLine_Otsu_Right_Border[58] - repair_x) *
                                               (row - 58)) /
                                              (58 - repair_y) +
                                              (int16)SearchLine_Otsu_Right_Border[58],
                                              0,
                                              SEARCH_LINE_OTSU_W - 1);
            SearchLine_Otsu_Center_Line[row] =
                (uint8)(((int16)SearchLine_Otsu_Right_Border[row] +
                         (int16)SearchLine_Otsu_Left_Border[row]) / 2);
        }
    }

    if((9 == SearchLine_Otsu_Ring_Flag) || (10 == SearchLine_Otsu_Ring_Flag))
    {
        for(row = SEARCH_LINE_OTSU_BOTTOM_ROW; row > (int16)SearchLine_Otsu_Offline_Row; row--)
        {
            SearchLine_Otsu_Center_Line[row] =
                (uint8)SearchLine_Limit_Int32((int16)SearchLine_Otsu_Right_Border[row] -
                                              (int16)SearchLine_Otsu_Half_Road_Wide[row],
                                              0,
                                              SEARCH_LINE_OTSU_W - 1);
        }
    }
}

static void SearchLine_Element_Handle_Right_Rings_Otsu(void)
{
    int16 num = 0;
    int16 black = 0;
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

    /* 参考旧分支用右侧黑列区分大小圆环。 */
    if(0 == SearchLine_Otsu_Ring_Size)
    {
        black = 0;
        for(row = 30; row > 0; row--)
        {
            if(0 == SearchLine_Get_Otsu_Binary_Pixel(row, 75))
            {
                black++;
            }
        }
        if(black > 10)
        {
            SearchLine_Otsu_Ring_Size = 1;
        }
        else
        {
            SearchLine_Otsu_Ring_Size = 2;
        }
    }

    for(row = 55; row > 30; row--)
    {
        if(SEARCH_LINE_STATE_WHITE == SearchLine_Otsu_Right_State[row])
        {
            num++;
        }
        if((SEARCH_LINE_STATE_WHITE == SearchLine_Otsu_Right_State[row + 3]) &&
           (SEARCH_LINE_STATE_WHITE == SearchLine_Otsu_Right_State[row + 2]) &&
           (SEARCH_LINE_STATE_WHITE == SearchLine_Otsu_Right_State[row + 1]) &&
           (SEARCH_LINE_STATE_FOUND == SearchLine_Otsu_Right_State[row]))
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
    if((5 == SearchLine_Otsu_Ring_Flag) && (SearchLine_Otsu_Left_Line > 15))
    {
        SearchLine_Otsu_Ring_Flag = 6;
    }
    if((6 == SearchLine_Otsu_Ring_Flag) && (SearchLine_Otsu_Left_Line < 4))
    {
        SearchLine_Otsu_Ring_Flag = 7;
    }

    if(7 == SearchLine_Otsu_Ring_Flag)
    {
        point_y = 0;
        for(row = 45; row > ((int16)SearchLine_Otsu_Offline_Row + 3); row--)
        {
            if((SearchLine_Otsu_Left_Border[row] >= SearchLine_Otsu_Left_Border[row + 1]) &&
               (SearchLine_Otsu_Left_Border[row] >= SearchLine_Otsu_Left_Border[row - 1]))
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
        straight_judge = SearchLine_Straight_Judge_Otsu(1,
                                                        (uint8)SearchLine_Limit_Int32((int16)SearchLine_Otsu_Offline_Row + 10,
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

    if(9 == SearchLine_Otsu_Ring_Flag)
    {
        num = 0;
        for(row = 40; row > 10; row--)
        {
            if(SEARCH_LINE_STATE_WHITE == SearchLine_Otsu_Right_State[row])
            {
                num++;
            }
        }
        if(num < 5)
        {
            SearchLine_Otsu_Road_Type = SEARCH_LINE_ROAD_NORMAL;
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
        for(row = SEARCH_LINE_OTSU_BOTTOM_ROW; row > (int16)SearchLine_Otsu_Offline_Row; row--)
        {
            SearchLine_Otsu_Center_Line[row] =
                (uint8)SearchLine_Limit_Int32((int16)SearchLine_Otsu_Left_Border[row] +
                                              (int16)SearchLine_Otsu_Half_Road_Wide[row],
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
                if((SEARCH_LINE_STATE_FOUND == SearchLine_Otsu_Right_State[row]) &&
                   (SEARCH_LINE_STATE_FOUND == SearchLine_Otsu_Right_State[row + 1]) &&
                   (SEARCH_LINE_STATE_WHITE == SearchLine_Otsu_Right_State[row + 2]) &&
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
                    (uint8)SearchLine_Limit_Int32((int16)((float)flag_x_1 +
                                                          slope_right_rings * (float)(row - flag_y_1)),
                                                  0,
                                                  SEARCH_LINE_OTSU_W - 1);
                SearchLine_Otsu_Center_Line[row] =
                    (uint8)SearchLine_Limit_Int32(((int16)SearchLine_Otsu_Left_Border[row] +
                                                   (int16)SearchLine_Otsu_Right_Border[row]) / 2,
                                                  0,
                                                  SEARCH_LINE_OTSU_W - 1);
            }

            SearchLine_Otsu_Left_Border[flag_y_1] =
                (uint8)SearchLine_Limit_Int32(flag_x_1, 0, SEARCH_LINE_OTSU_W - 1);

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
                            (uint8)SearchLine_Limit_Int32(col, 0, SEARCH_LINE_OTSU_W - 1);
                        SearchLine_Otsu_Center_Line[row] =
                            (uint8)SearchLine_Limit_Int32(((int16)SearchLine_Otsu_Left_Border[row] +
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

    if(6 == SearchLine_Otsu_Ring_Flag)
    {
        for(row = 57; row > (int16)SearchLine_Otsu_Offline_Row; row--)
        {
            SearchLine_Otsu_Center_Line[row] = 63;
        }
    }

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
                (uint8)SearchLine_Limit_Int32((((int16)SearchLine_Otsu_Left_Border[58] - repair_x) *
                                               (row - 58)) /
                                              (58 - repair_y) +
                                              (int16)SearchLine_Otsu_Left_Border[58],
                                              0,
                                              SEARCH_LINE_OTSU_W - 1);
            SearchLine_Otsu_Center_Line[row] =
                (uint8)(((int16)SearchLine_Otsu_Left_Border[row] +
                         (int16)SearchLine_Otsu_Right_Border[row]) / 2);
        }
    }

    if(9 == SearchLine_Otsu_Ring_Flag)
    {
        for(row = SEARCH_LINE_OTSU_BOTTOM_ROW; row > (int16)SearchLine_Otsu_Offline_Row; row--)
        {
            SearchLine_Otsu_Center_Line[row] =
                (uint8)SearchLine_Limit_Int32((int16)SearchLine_Otsu_Left_Border[row] +
                                              (int16)SearchLine_Otsu_Half_Road_Wide[row],
                                              0,
                                              SEARCH_LINE_OTSU_W - 1);
        }
    }
}

static void SearchLine_Element_Handle(void)
{
    if(1 == SearchLine_Otsu_Ring_Element)
    {
        SearchLine_Element_Handle_Left_Rings_Otsu();
    }
    else if(2 == SearchLine_Otsu_Ring_Element)
    {
        SearchLine_Element_Handle_Right_Rings_Otsu();
    }
}

/* 元素判断。 */
static void SearchLine_Element_Test(void)
{
    uint8 bottom_ok = 0;
    uint8 row = 0;

    SearchLine_Otsu_Cirque_Left_Count =
        SearchLine_Cirque_Or_Cross_Otsu(1, SearchLine_Otsu_Left_Line);
    SearchLine_Otsu_Cirque_Right_Count =
        SearchLine_Cirque_Or_Cross_Otsu(2, SearchLine_Otsu_Right_Line);

    for(row = 52; row <= 58; row++)
    {
        if(SEARCH_LINE_STATE_WHITE != SearchLine_Otsu_Left_State[row])
        {
            bottom_ok++;
        }
    }
    SearchLine_Otsu_Ring_Bottom_Ok = bottom_ok;

    /* 非十字、非圆环时更新直道标志。 */
    if((SearchLine_Otsu_Road_Type != SEARCH_LINE_ROAD_CROSS) &&
       (SearchLine_Otsu_Road_Type != SEARCH_LINE_ROAD_LEFT_CIRQUE) &&
       (SearchLine_Otsu_Road_Type != SEARCH_LINE_ROAD_RIGHT_CIRQUE))
    {
        SearchLine_Update_Otsu_StraightAcc();
    }
    else
    {
        SearchLine_Otsu_Straight_Acc = 0;
        SearchLine_Otsu_Variance_Acc = 0;
    }

    /* 圆环判断。 */
    if((SearchLine_Otsu_Offline_Row < 5) &&
       (SearchLine_Otsu_White_Line < 3) &&
       (SEARCH_LINE_STATE_WHITE != SearchLine_Otsu_Left_State[52]) &&
       (SEARCH_LINE_STATE_WHITE != SearchLine_Otsu_Left_State[53]) &&
       (SEARCH_LINE_STATE_WHITE != SearchLine_Otsu_Left_State[54]) &&
       (SEARCH_LINE_STATE_WHITE != SearchLine_Otsu_Left_State[55]) &&
       (SEARCH_LINE_STATE_WHITE != SearchLine_Otsu_Left_State[56]) &&
       (SEARCH_LINE_STATE_WHITE != SearchLine_Otsu_Left_State[57]) &&
       (SEARCH_LINE_STATE_WHITE != SearchLine_Otsu_Left_State[58]))
    {
        /* 左圆环判断。 */
        if((SearchLine_Otsu_Right_Line < 2) &&
           (SearchLine_Otsu_Left_Line > 13) &&
           (SearchLine_Otsu_Cirque_Left_Count > 70))
        {
            SearchLine_Element_Judgment_Left_Rings_Otsu();
        }

        /* 右圆环判断。 */
        if((SearchLine_Otsu_Left_Line < 2) &&
           (SearchLine_Otsu_Right_Line > 17) &&
           (SearchLine_Otsu_Cirque_Right_Count > 120))
        {
            SearchLine_Element_Judgment_Right_Rings_Otsu();
        }
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
        else if(score < best_score)
        {
            break;
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

/* 图像压缩、图像二值化、底边初始化、逐行搜边、边界支线、延长线补边、中线滤波。 */
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
    /* 圆环支线先补边界跟踪缓存，后续判据和补线都依赖这一组观测量。 */
    SearchLine_Search_Border_Otsu();
    /* 元素判断。 */
    SearchLine_Element_Test();
    /* 延长线补边。 */
    SearchLine_DrawExtensionLine_Otsu();
    /* 中线滤波平滑。 */
    SearchLine_RouteFilter_Otsu();
    /* 元素处理。 */
    SearchLine_Element_Handle();
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

static void SearchLine_DrawPreviewLabels(void)
{
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_show_string(0, (uint16)(CAMERA_RAW_H + 4), "yu zhi");
    ips200_show_string(0, (uint16)(CAMERA_RAW_H + 20), "qian zhan pian cha");
    ips200_show_string(0, (uint16)(CAMERA_RAW_H + 36), "duo ji jiao du");

    /* 圆环条件调试量。 */
    ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
    ips200_show_string(0, (uint16)(CAMERA_RAW_H + 52), "yuan huan");
    ips200_set_color(RGB565_CYAN, RGB565_BLACK);
    ips200_show_string(0, (uint16)(CAMERA_RAW_H + 68), "ru huan");
    ips200_set_color(RGB565_GREEN, RGB565_BLACK);
    ips200_show_string(0, (uint16)(CAMERA_RAW_H + 84), "zuo pan");
    ips200_set_color(RGB565_MAGENTA, RGB565_BLACK);
    ips200_show_string(0, (uint16)(CAMERA_RAW_H + 100), "you pan");
    ips200_set_color(RGB565_BLUE, RGB565_BLACK);
    ips200_show_string(0, (uint16)(CAMERA_RAW_H + 116), "jie duan");
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

void SearchLine_ResetPreviewOverlay(void)
{
    /* 相机页切黑底后重新补标签与数值，运行时尽量只刷变化项。 */
    SearchLine_Preview_Label_Ready = 0;
    SearchLine_Preview_Last_Threshold = 0xFF;
    SearchLine_Preview_Last_Offset = 32767;
    SearchLine_Preview_Last_Command = 0xFF;
    SearchLine_Preview_Last_Ring_Element = 0xFF;
    SearchLine_Preview_Last_Ring_Flag = 0xFF;
    SearchLine_Preview_Last_Ring_Size = 0xFF;
    SearchLine_Preview_Last_Offline_Row = 0xFF;
    SearchLine_Preview_Last_White_Line = 0xFF;
    SearchLine_Preview_Last_Ring_Bottom_Ok = 0xFF;
    SearchLine_Preview_Last_Cirque_Left_Count = 0xFFFF;
    SearchLine_Preview_Last_Cirque_Right_Count = 0xFFFF;
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
    /* int16 speed_goal_display = 0; */
    /* int16 ref_left_display = 0; */
    /* int16 ref_right_display = 0; */

    if(show_raw)
    {
        ips200_show_gray_image(0,
                               0,
                               mt9v03x_image[0],
                               CAMERA_RAW_W,
                               CAMERA_RAW_H,
                               CAMERA_VALID_W,
                               CAMERA_RAW_H,
                               0);
    }
    else
    {
        ips200_show_gray_image(0,
                               0,
                               SearchLine_Otsu_Binary[0],
                               SEARCH_LINE_OTSU_W,
                               SEARCH_LINE_OTSU_H,
                               CAMERA_VALID_W,
                               CAMERA_RAW_H,
                               1);
    }

    /* 相机页底部显示当前 OTSU 阈值，方便边看图边确认二值门限。 */
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    if(!SearchLine_Preview_Label_Ready)
    {
        SearchLine_DrawPreviewLabels();
    }
    if(SearchLine_Preview_Last_Threshold != SearchLine_Otsu_Threshold_Cache)
    {
        SearchLine_FormatThresholdText(threshold_text, SearchLine_Otsu_Threshold_Cache);
        ips200_show_string(160, (uint16)(CAMERA_RAW_H + 4), threshold_text);
        SearchLine_Preview_Last_Threshold = SearchLine_Otsu_Threshold_Cache;
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
        ips200_show_string(160, (uint16)(CAMERA_RAW_H + 20), offset_text);
        SearchLine_Preview_Last_Offset = SearchLine_Otsu_Steer_Offset;
    }
    if(SearchLine_Preview_Last_Command != command_value)
    {
        ips200_show_string(160, (uint16)(CAMERA_RAW_H + 36), command_text);
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
    gate_text[7] = ' ';
    gate_text[8] = 'k';
    gate_text[9] = (char)('0' + SearchLine_Otsu_Ring_Bottom_Ok % 10U);
    gate_text[10] = '\0';

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
        ips200_show_string(104, (uint16)(CAMERA_RAW_H + 52), "            ");
        ips200_show_string(104, (uint16)(CAMERA_RAW_H + 52), ring_text);
        SearchLine_Preview_Last_Ring_Element = SearchLine_Otsu_Ring_Element;
        SearchLine_Preview_Last_Ring_Flag = SearchLine_Otsu_Ring_Flag;
        SearchLine_Preview_Last_Ring_Size = SearchLine_Otsu_Ring_Size;
    }

    ips200_set_color(RGB565_CYAN, RGB565_BLACK);
    if((SearchLine_Preview_Last_Offline_Row != SearchLine_Otsu_Offline_Row) ||
       (SearchLine_Preview_Last_White_Line != SearchLine_Otsu_White_Line) ||
       (SearchLine_Preview_Last_Ring_Bottom_Ok != SearchLine_Otsu_Ring_Bottom_Ok))
    {
        ips200_show_string(104, (uint16)(CAMERA_RAW_H + 68), "            ");
        ips200_show_string(104, (uint16)(CAMERA_RAW_H + 68), gate_text);
        SearchLine_Preview_Last_Offline_Row = SearchLine_Otsu_Offline_Row;
        SearchLine_Preview_Last_White_Line = SearchLine_Otsu_White_Line;
        SearchLine_Preview_Last_Ring_Bottom_Ok = SearchLine_Otsu_Ring_Bottom_Ok;
    }

    ips200_set_color(RGB565_GREEN, RGB565_BLACK);
    if((SearchLine_Preview_Last_Ring_Left_Line != SearchLine_Otsu_Left_Line) ||
       (SearchLine_Preview_Last_Ring_Right_Line != SearchLine_Otsu_Right_Line) ||
       (SearchLine_Preview_Last_Cirque_Left_Count != SearchLine_Otsu_Cirque_Left_Count))
    {
        ips200_show_string(104, (uint16)(CAMERA_RAW_H + 84), "            ");
        ips200_show_string(104, (uint16)(CAMERA_RAW_H + 84), left_text);
        SearchLine_Preview_Last_Cirque_Left_Count = SearchLine_Otsu_Cirque_Left_Count;
        SearchLine_Preview_Last_Ring_Left_Line = SearchLine_Otsu_Left_Line;
        SearchLine_Preview_Last_Ring_Right_Line = SearchLine_Otsu_Right_Line;
    }

    ips200_set_color(RGB565_MAGENTA, RGB565_BLACK);
    if((SearchLine_Preview_Last_Ring_Left_Line_RightPanel != SearchLine_Otsu_Left_Line) ||
       (SearchLine_Preview_Last_Ring_Right_Line_RightPanel != SearchLine_Otsu_Right_Line) ||
       (SearchLine_Preview_Last_Cirque_Right_Count != SearchLine_Otsu_Cirque_Right_Count))
    {
        ips200_show_string(104, (uint16)(CAMERA_RAW_H + 100), "            ");
        ips200_show_string(104, (uint16)(CAMERA_RAW_H + 100), right_text);
        SearchLine_Preview_Last_Cirque_Right_Count = SearchLine_Otsu_Cirque_Right_Count;
        SearchLine_Preview_Last_Ring_Left_Line_RightPanel = SearchLine_Otsu_Left_Line;
        SearchLine_Preview_Last_Ring_Right_Line_RightPanel = SearchLine_Otsu_Right_Line;
    }

    ips200_set_color(RGB565_BLUE, RGB565_BLACK);
    if((SearchLine_Preview_Last_Ring_Stage_Num != SearchLine_Otsu_Ring_Stage_Num) ||
       (SearchLine_Preview_Last_Ring_Point_Y != SearchLine_Otsu_Ring_Point_Y) ||
       (SearchLine_Preview_Last_Ring_Straight_Judge_Tenth != SearchLine_Otsu_Ring_Straight_Judge_Tenth))
    {
        ips200_show_string(104, (uint16)(CAMERA_RAW_H + 116), "            ");
        ips200_show_string(104, (uint16)(CAMERA_RAW_H + 116), stage_text);
        SearchLine_Preview_Last_Ring_Stage_Num = SearchLine_Otsu_Ring_Stage_Num;
        SearchLine_Preview_Last_Ring_Point_Y = SearchLine_Otsu_Ring_Point_Y;
        SearchLine_Preview_Last_Ring_Straight_Judge_Tenth = SearchLine_Otsu_Ring_Straight_Judge_Tenth;
    }

    /* 暂时关闭底部速度与目标值文字显示，先只保留图像预览。 */
    /*
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

    ips200_show_string(0, (uint16)(CAMERA_RAW_H + 52), line_clear_text);
    ips200_show_string(0, (uint16)(CAMERA_RAW_H + 52), "mu biao su du");
    ips200_show_int32(160, (uint16)(CAMERA_RAW_H + 52), (int32)speed_goal_display, 3);

    ips200_show_string(0, (uint16)(CAMERA_RAW_H + 68), line_clear_text);
    ips200_show_string(0, (uint16)(CAMERA_RAW_H + 68), "zuo bian ma mu biao");
    ips200_show_int32(160, (uint16)(CAMERA_RAW_H + 68), (int32)ref_left_display, 4);

    ips200_show_string(0, (uint16)(CAMERA_RAW_H + 84), line_clear_text);
    ips200_show_string(0, (uint16)(CAMERA_RAW_H + 84), "you bian ma mu biao");
    ips200_show_int32(160, (uint16)(CAMERA_RAW_H + 84), (int32)ref_right_display, 4);
    */

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

    for(row = SearchLine_Otsu_Offline_Boundary_Row; row <= SEARCH_LINE_OTSU_BOUNDARY_BOTTOM_ROW; row++)
    {
        y = (uint16)(((uint32)row * (uint32)CAMERA_RAW_H + (uint32)(SEARCH_LINE_OTSU_H / 2)) /
                     (uint32)SEARCH_LINE_OTSU_H);

        boundary_col = SearchLine_Otsu_Left_Boundary_First[row];
        x = (uint16)(((uint32)boundary_col * (uint32)CAMERA_VALID_W + (uint32)(SEARCH_LINE_OTSU_W / 2)) /
                     (uint32)SEARCH_LINE_OTSU_W);
        ips200_draw_point(x, y, RGB565_YELLOW);

        boundary_col = SearchLine_Otsu_Right_Boundary_First[row];
        x = (uint16)(((uint32)boundary_col * (uint32)CAMERA_VALID_W + (uint32)(SEARCH_LINE_OTSU_W / 2)) /
                     (uint32)SEARCH_LINE_OTSU_W);
        ips200_draw_point(x, y, RGB565_YELLOW);

        boundary_col = SearchLine_Otsu_Left_Boundary[row];
        x = (uint16)(((uint32)boundary_col * (uint32)CAMERA_VALID_W + (uint32)(SEARCH_LINE_OTSU_W / 2)) /
                     (uint32)SEARCH_LINE_OTSU_W);
        ips200_draw_point(x, y, RGB565_CYAN);

        boundary_col = SearchLine_Otsu_Right_Boundary[row];
        x = (uint16)(((uint32)boundary_col * (uint32)CAMERA_VALID_W + (uint32)(SEARCH_LINE_OTSU_W / 2)) /
                     (uint32)SEARCH_LINE_OTSU_W);
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
