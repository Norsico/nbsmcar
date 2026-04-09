#include "SearchLine.h"

#define SEARCH_LINE_OTSU_W                  (80)
#define SEARCH_LINE_OTSU_H                  (60)
#define SEARCH_LINE_OTSU_BOTTOM_ROW         (SEARCH_LINE_OTSU_H - 1)
#define SEARCH_LINE_OTSU_BOTTOM_INIT_ROW    (55)
#define SEARCH_LINE_OTSU_OFFLINE_MIN        (2)
#define SEARCH_LINE_OTSU_THRESHOLD_MIN      (0)
#define SEARCH_LINE_OTSU_THRESHOLD_CAP      (180)
/* OTSU 阈值重算周期。
 * 1 表示每帧都重算。
 * 2-20 表示隔 N 帧重算一次，中间帧直接复用上一次阈值。
 * 当前先用 10，后续可按赛道光照稳定性手调，步进 1。
 */
#define SEARCH_LINE_OTSU_THRESHOLD_INTERVAL (10)
#define SEARCH_LINE_OTSU_LEFT_COMP_END      (15)
#define SEARCH_LINE_OTSU_RIGHT_COMP_START   (65)
#define SEARCH_LINE_OTSU_MIN_WIDTH          (7)
#define SEARCH_LINE_OTSU_EDGE_LIMIT         (10)
#define SEARCH_LINE_OTSU_SCAN_WINDOW        (2)

#define SEARCH_LINE_STATE_INIT              ('F')
#define SEARCH_LINE_STATE_FOUND             ('T')
#define SEARCH_LINE_STATE_WHITE             ('W')
#define SEARCH_LINE_STATE_BLACK             ('H')

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
static uint8 SearchLine_Otsu_Left_Extend_Allowed = 1;
static uint8 SearchLine_Otsu_Right_Extend_Allowed = 1;
static uint8 SearchLine_Otsu_Map_Ready = 0;
static uint8 SearchLine_Otsu_Offline_Row = SEARCH_LINE_OTSU_OFFLINE_MIN;
static uint8 SearchLine_Otsu_Threshold_Cache = SEARCH_LINE_OTSU_THRESHOLD_MIN;
static uint8 SearchLine_Otsu_Threshold_Frame_Count = 0;

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

    for(row = 0; row < SEARCH_LINE_OTSU_H; row++)
    {
        SearchLine_Otsu_Left_Border[row] = 0;
        SearchLine_Otsu_Right_Border[row] = SEARCH_LINE_OTSU_W - 1;
        SearchLine_Otsu_Center_Line[row] = SEARCH_LINE_OTSU_W / 2;
        SearchLine_Otsu_Row_Valid[row] = 0;
        SearchLine_Otsu_Left_State[row] = SEARCH_LINE_STATE_INIT;
        SearchLine_Otsu_Right_State[row] = SEARCH_LINE_STATE_INIT;
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

    if(SearchLine_Otsu_Left_Extend_Allowed)
    {
        tfsite = SEARCH_LINE_OTSU_BOTTOM_INIT_ROW;
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

    if(SearchLine_Otsu_Right_Extend_Allowed)
    {
        tfsite = SEARCH_LINE_OTSU_BOTTOM_INIT_ROW;
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
            threshold = (uint8)gray;
        }
    }

    if(threshold < SEARCH_LINE_OTSU_THRESHOLD_MIN)
    {
        threshold = SEARCH_LINE_OTSU_THRESHOLD_MIN;
    }

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
    /* 底边初始化。 */
    SearchLine_Init_Otsu_BottomRows();
    /* 逐行向上搜边。 */
    SearchLine_DrawLinesProcess_Otsu();
    /* 延长线补边。 */
    SearchLine_DrawExtensionLine_Otsu();
    /* 中线滤波平滑。 */
    SearchLine_RouteFilter_Otsu();
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

/* 显示压缩二值图。 */
void SearchLine_DrawBinaryPreview(void)
{
    char threshold_text[6];
    uint16 x = 0;
    uint16 y = 0;
    uint8 row = 0;
    uint8 left_col = 0;
    uint8 right_col = 0;
    uint8 center_col = 0;

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
