#include "SearchLine.h"

uint8 Left_Edge_Line[Search_Image_H] = {0};
uint8 Right_Edge_Line[Search_Image_H] = {0};
uint8 Center_Line[Search_Image_H] = {0};
/* 单独记录显示有效性，避免沿用默认边线把旧结果画成假轨迹。 */
static uint8 Left_Edge_Found[Search_Image_H] = {0};
static uint8 Right_Edge_Found[Search_Image_H] = {0};
static uint8 Center_Line_Found[Search_Image_H] = {0};

#define SEARCH_LINE_STATE_INIT              ('F')
#define SEARCH_LINE_STATE_FOUND             ('T')
#define SEARCH_LINE_STATE_WHITE             ('W')
#define SEARCH_LINE_STATE_BLACK             ('H')

#define SEARCH_LINE_OTSU_W                  (80)
#define SEARCH_LINE_OTSU_H                  (60)
#define SEARCH_LINE_OTSU_BOTTOM_ROW         (SEARCH_LINE_OTSU_H - 1)
#define SEARCH_LINE_OTSU_PREFILL_ROWS       (5)
#define SEARCH_LINE_OTSU_START_ROW          (SEARCH_LINE_OTSU_BOTTOM_ROW - SEARCH_LINE_OTSU_PREFILL_ROWS)
#define SEARCH_LINE_OTSU_OFFLINE_MIN        (2)
#define SEARCH_LINE_OTSU_THRESHOLD_MIN      (70)
#define SEARCH_LINE_OTSU_THRESHOLD_CAP      (180)
#define SEARCH_LINE_OTSU_LEFT_COMP_END      (15)
#define SEARCH_LINE_OTSU_RIGHT_COMP_START   (65)
#define SEARCH_LINE_OTSU_MIN_WIDTH          (7)
#define SEARCH_LINE_OTSU_EDGE_LIMIT         (10)
#define SEARCH_LINE_OTSU_SCAN_WINDOW        (2)

/* 先沿用国一方案的普通赛道半宽模型，单边丢失时按这一组宽度补另一边。 */
static const uint8 SearchLine_Otsu_Half_Road_Wide[SEARCH_LINE_OTSU_H] =
{
    6, 7, 7, 8, 8, 9, 9, 9, 10, 10,
    11, 11, 11, 11, 11, 12, 12, 13, 13, 14,
    14, 14, 14, 15, 15, 16, 16, 16, 17, 17,
    17, 18, 18, 19, 19, 20, 20, 20, 21, 21,
    21, 22, 22, 23, 23, 23, 24, 24, 25, 25,
    26, 26, 26, 26, 27, 27, 27, 28, 28, 30
};

static uint8 SearchLine_Otsu_Binary[SEARCH_LINE_OTSU_H][SEARCH_LINE_OTSU_W] = {0};
static uint8 SearchLine_Otsu_Left_Border[SEARCH_LINE_OTSU_H] = {0};
static uint8 SearchLine_Otsu_Right_Border[SEARCH_LINE_OTSU_H] = {0};
static uint8 SearchLine_Otsu_Center_Line[SEARCH_LINE_OTSU_H] = {0};
static uint8 SearchLine_Otsu_Left_State[SEARCH_LINE_OTSU_H] = {0};
static uint8 SearchLine_Otsu_Right_State[SEARCH_LINE_OTSU_H] = {0};
static uint8 SearchLine_Otsu_Center_Found[SEARCH_LINE_OTSU_H] = {0};
static uint16 SearchLine_Otsu_Histogram[256] = {0};
static uint8 SearchLine_Otsu_Row_Map[SEARCH_LINE_OTSU_H] = {0};
static uint8 SearchLine_Otsu_Col_Map[SEARCH_LINE_OTSU_W] = {0};
static uint8 SearchLine_Otsu_To_Raw_Col_Map[SEARCH_LINE_OTSU_W] = {0};
static uint8 SearchLine_Otsu_Row_Start_Map[SEARCH_LINE_OTSU_H] = {0};
static uint8 SearchLine_Otsu_Row_End_Map[SEARCH_LINE_OTSU_H] = {0};
static uint8 SearchLine_Otsu_Map_Ready = 0;
static uint8 SearchLine_Otsu_Threshold = 0;
static uint8 SearchLine_Otsu_Offline_Row = SEARCH_LINE_OTSU_OFFLINE_MIN;
static uint8 SearchLine_Otsu_WhiteLine_Count = 0;
static uint8 SearchLine_Otsu_Left_Lost_Count = 0;
static uint8 SearchLine_Otsu_Right_Lost_Count = 0;
static uint8 SearchLine_Otsu_Left_Extend_Enable = 1;
static uint8 SearchLine_Otsu_Right_Extend_Enable = 1;

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

static uint8 SearchLine_Clamp_Otsu_Col(int16 value)
{
    return (uint8)SearchLine_Limit_Int32(value, 0, SEARCH_LINE_OTSU_W - 1);
}

static uint8 SearchLine_Clamp_Otsu_Search_Col(int16 value)
{
    return (uint8)SearchLine_Limit_Int32(value, 1, SEARCH_LINE_OTSU_W - 2);
}

/* 压缩域和原始图尺寸不一致，先把行列映射固定下来，后面每帧直接复用。 */
static void SearchLine_Init_Otsu_Map(void)
{
    uint16 row = 0;
    uint16 col = 0;
    uint32 scaled_value = 0;

    if(SearchLine_Otsu_Map_Ready)
    {
        return;
    }

    for(row = 0; row < SEARCH_LINE_OTSU_H; row++)
    {
        scaled_value = ((uint32)row * (uint32)(CAMERA_RAW_H - 1) + (uint32)((SEARCH_LINE_OTSU_H - 1) / 2)) / (uint32)(SEARCH_LINE_OTSU_H - 1);
        SearchLine_Otsu_Row_Map[row] = (uint8)SearchLine_Limit_Int32((int32)scaled_value, 0, CAMERA_RAW_H - 1);
        SearchLine_Otsu_Row_Start_Map[row] = (uint8)(((uint32)row * (uint32)Search_Image_H) / (uint32)SEARCH_LINE_OTSU_H);
        scaled_value = (((uint32)(row + 1) * (uint32)Search_Image_H) / (uint32)SEARCH_LINE_OTSU_H);
        if(0 == scaled_value)
        {
            scaled_value = 1;
        }
        SearchLine_Otsu_Row_End_Map[row] = (uint8)SearchLine_Limit_Int32((int32)scaled_value - 1, 0, Search_Image_H - 1);
    }

    for(col = 0; col < SEARCH_LINE_OTSU_W; col++)
    {
        scaled_value = ((uint32)col * (uint32)CAMERA_LAST_VALID_COL + (uint32)((SEARCH_LINE_OTSU_W - 1) / 2)) / (uint32)(SEARCH_LINE_OTSU_W - 1);
        SearchLine_Otsu_Col_Map[col] = (uint8)SearchLine_Limit_Int32((int32)scaled_value, 0, CAMERA_LAST_VALID_COL);
        SearchLine_Otsu_To_Raw_Col_Map[col] = (uint8)SearchLine_Limit_Int32((int32)scaled_value, 0, CAMERA_LAST_VALID_COL);
    }

    SearchLine_Otsu_Map_Ready = 1;
}

static uint8 SearchLine_Get_Otsu_Gray(uint8 row, uint8 col)
{
    return mt9v03x_image[SearchLine_Otsu_Row_Map[row]][SearchLine_Otsu_Col_Map[col]];
}

static void SearchLine_Reset_Output_Buffer(void)
{
    uint16 row = 0;

    SearchLine_Otsu_Threshold = 0;

    for(row = 0; row < Search_Image_H; row++)
    {
        Left_Edge_Line[row] = 0;
        Right_Edge_Line[row] = CAMERA_LAST_VALID_COL;
        Left_Edge_Found[row] = 0;
        Right_Edge_Found[row] = 0;
        Center_Line[row] = Search_Image_W / 2;
        Center_Line_Found[row] = 0;
    }
}

static void SearchLine_Clear_Otsu_State(void)
{
    uint16 row = 0;

    SearchLine_Otsu_Offline_Row = SEARCH_LINE_OTSU_OFFLINE_MIN;
    SearchLine_Otsu_WhiteLine_Count = 0;
    SearchLine_Otsu_Left_Lost_Count = 0;
    SearchLine_Otsu_Right_Lost_Count = 0;
    SearchLine_Otsu_Left_Extend_Enable = 1;
    SearchLine_Otsu_Right_Extend_Enable = 1;

    for(row = 0; row < SEARCH_LINE_OTSU_H; row++)
    {
        SearchLine_Otsu_Left_Border[row] = 0;
        SearchLine_Otsu_Right_Border[row] = SEARCH_LINE_OTSU_W - 1;
        SearchLine_Otsu_Center_Line[row] = SEARCH_LINE_OTSU_W / 2;
        SearchLine_Otsu_Left_State[row] = SEARCH_LINE_STATE_INIT;
        SearchLine_Otsu_Right_State[row] = SEARCH_LINE_STATE_INIT;
        SearchLine_Otsu_Center_Found[row] = 0;
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

    return threshold;
}

/* 参考国一做法：先用 OTSU 求整图阈值，再给边缘列留一点补偿余量。 */
static void SearchLine_Binarize_Otsu_Image(void)
{
    uint16 row = 0;
    uint16 col = 0;
    uint8 threshold = 0;
    uint8 gray = 0;
    int16 row_threshold = 0;

    threshold = SearchLine_Calc_Otsu_Threshold();
    SearchLine_Otsu_Threshold = threshold;

    for(row = 0; row < SEARCH_LINE_OTSU_H; row++)
    {
        for(col = 0; col < SEARCH_LINE_OTSU_W; col++)
        {
            row_threshold = threshold;
            if(col <= SEARCH_LINE_OTSU_LEFT_COMP_END)
            {
                row_threshold -= 10;
            }
            else if(col >= SEARCH_LINE_OTSU_RIGHT_COMP_START)
            {
                row_threshold -= 15;
            }

            row_threshold = (int16)SearchLine_Limit_Int32(row_threshold, 0, 255);
            gray = SearchLine_Get_Otsu_Gray((uint8)row, (uint8)col);
            SearchLine_Otsu_Binary[row][col] = (gray > (uint8)row_threshold) ? 1 : 0;
        }
    }
}

/* 只先保留最轻的一层补点，观察 STC 上的实时性。 */
static void SearchLine_Filter_Otsu_Binary(void)
{
    uint16 row = 0;
    uint16 col = 0;

    for(row = 10; row < 40; row++)
    {
        for(col = 10; col < 70; col++)
        {
            if((0 == SearchLine_Otsu_Binary[row][col]) &&
               (SearchLine_Otsu_Binary[row - 1][col] +
                SearchLine_Otsu_Binary[row + 1][col] +
                SearchLine_Otsu_Binary[row][col - 1] +
                SearchLine_Otsu_Binary[row][col + 1] >= 3))
            {
                SearchLine_Otsu_Binary[row][col] = 1;
            }
        }
    }
}

static void SearchLine_Otsu_Set_Row(uint8 row, uint8 left, uint8 right, uint8 left_state, uint8 right_state)
{
    SearchLine_Otsu_Left_Border[row] = left;
    SearchLine_Otsu_Right_Border[row] = right;
    SearchLine_Otsu_Center_Line[row] = (uint8)(((uint16)left + (uint16)right) / 2);
    SearchLine_Otsu_Left_State[row] = left_state;
    SearchLine_Otsu_Right_State[row] = right_state;
    SearchLine_Otsu_Center_Found[row] = 1;
}

/* 先把底部几行定稳，后面逐行窗口搜边才不会一上来就漂。 */
static void SearchLine_Init_Otsu_BottomRows(void)
{
    const uint8 *row_data = 0;
    int16 row = 0;
    int16 col = 0;
    int16 center_col = SEARCH_LINE_OTSU_W / 2 - 1;
    int16 left_border = 0;
    int16 right_border = SEARCH_LINE_OTSU_W - 1;
    int16 hit_left = -1;
    int16 hit_right = -1;
    uint8 current_row = 0;

    row_data = SearchLine_Otsu_Binary[SEARCH_LINE_OTSU_BOTTOM_ROW];
    if(0 == row_data[center_col])
    {
        for(col = 0; col < center_col; col++)
        {
            if(0 != row_data[center_col - col])
            {
                hit_left = center_col - col;
                break;
            }

            if(0 != row_data[center_col + col])
            {
                hit_right = center_col + col;
                break;
            }
        }

        if(hit_left >= 0)
        {
            right_border = hit_left + 1;
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
        else if(hit_right >= 0)
        {
            left_border = hit_right - 1;
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

    left_border = SearchLine_Limit_Int32(left_border, 0, SEARCH_LINE_OTSU_W - 1);
    right_border = SearchLine_Limit_Int32(right_border, 0, SEARCH_LINE_OTSU_W - 1);
    if(right_border <= left_border)
    {
        left_border = 0;
        right_border = SEARCH_LINE_OTSU_W - 1;
    }

    SearchLine_Otsu_Set_Row(SEARCH_LINE_OTSU_BOTTOM_ROW, (uint8)left_border, (uint8)right_border, SEARCH_LINE_STATE_FOUND, SEARCH_LINE_STATE_FOUND);

    for(row = SEARCH_LINE_OTSU_BOTTOM_ROW - 1; row > SEARCH_LINE_OTSU_START_ROW; row--)
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

        left_border = SearchLine_Limit_Int32(left_border, 0, SEARCH_LINE_OTSU_W - 1);
        right_border = SearchLine_Limit_Int32(right_border, 0, SEARCH_LINE_OTSU_W - 1);
        if(right_border <= left_border)
        {
            left_border = SearchLine_Otsu_Left_Border[current_row + 1];
            right_border = SearchLine_Otsu_Right_Border[current_row + 1];
        }

        SearchLine_Otsu_Set_Row(current_row, (uint8)left_border, (uint8)right_border, SEARCH_LINE_STATE_FOUND, SEARCH_LINE_STATE_FOUND);
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
            *point = (uint8)col;
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

static void SearchLine_DrawLinesProcess_Otsu(void)
{
    const uint8 *row_data = 0;
    uint8 get_left_line = 0;
    uint8 get_right_line = 0;
    uint8 left_extend_ready = 0;
    uint8 right_extend_ready = 0;
    int16 row = 0;
    int16 low = 0;
    int16 high = 0;
    int16 col = 0;
    int16 search_row = 0;
    int16 left_border = 0;
    int16 right_border = 0;
    int16 extend_anchor_left = 0;
    int16 extend_anchor_right = 0;
    uint8 left_point = 0;
    uint8 right_point = 0;
    uint8 left_state = 0;
    uint8 right_state = 0;
    uint8 left_found_count = 0;
    uint8 right_found_count = 0;
    float left_slope = 0.0f;
    float right_slope = 0.0f;

    SearchLine_Otsu_Left_Extend_Enable = 1;
    SearchLine_Otsu_Right_Extend_Enable = 1;
    SearchLine_Otsu_WhiteLine_Count = 0;
    SearchLine_Otsu_Left_Lost_Count = 0;
    SearchLine_Otsu_Right_Lost_Count = 0;

    for(row = SEARCH_LINE_OTSU_START_ROW; row > SearchLine_Otsu_Offline_Row; row--)
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
        SearchLine_Otsu_Left_Border[row] = SearchLine_Clamp_Otsu_Col(left_border);
        SearchLine_Otsu_Right_Border[row] = SearchLine_Clamp_Otsu_Col(right_border);

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

            if((int16)SearchLine_Otsu_Right_Border[row] - (int16)SearchLine_Otsu_Left_Border[row] <= SEARCH_LINE_OTSU_MIN_WIDTH)
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

        left_found_count = 0;
        right_found_count = 0;

        if((SEARCH_LINE_STATE_WHITE == SearchLine_Otsu_Right_State[row]) && (row > 10) && (row < 50))
        {
            if(!get_right_line)
            {
                get_right_line = 1;
                extend_anchor_right = (int16)row + 2;
                for(search_row = (int16)row + 1; (search_row < (int16)row + 15) && (search_row < SEARCH_LINE_OTSU_H); search_row++)
                {
                    if(SEARCH_LINE_STATE_FOUND == SearchLine_Otsu_Right_State[search_row])
                    {
                        right_found_count++;
                    }
                }

                if(right_found_count > 8)
                {
                    search_row = (int16)row + (int16)right_found_count;
                    if(search_row > SEARCH_LINE_OTSU_BOTTOM_ROW)
                    {
                        search_row = SEARCH_LINE_OTSU_BOTTOM_ROW;
                    }
                    right_slope = ((float)SearchLine_Otsu_Right_Border[search_row] -
                                   (float)SearchLine_Otsu_Right_Border[row + 3]) /
                                  (float)(right_found_count - 3);
                    if(right_slope > 0.0f)
                    {
                        right_extend_ready = 1;
                    }
                    else
                    {
                        right_extend_ready = 0;
                        if(right_slope < 0.0f)
                        {
                            SearchLine_Otsu_Right_Extend_Enable = 0;
                        }
                    }
                }
            }

            if(right_extend_ready)
            {
                right_border = (int16)((float)SearchLine_Otsu_Right_Border[extend_anchor_right] -
                                       right_slope * (float)(extend_anchor_right - row));
                SearchLine_Otsu_Right_Border[row] = SearchLine_Clamp_Otsu_Search_Col(right_border);
            }
        }

        if((SEARCH_LINE_STATE_WHITE == SearchLine_Otsu_Left_State[row]) && (row > 10) && (row < 50))
        {
            if(!get_left_line)
            {
                get_left_line = 1;
                extend_anchor_left = (int16)row + 2;
                for(search_row = (int16)row + 1; (search_row < (int16)row + 15) && (search_row < SEARCH_LINE_OTSU_H); search_row++)
                {
                    if(SEARCH_LINE_STATE_FOUND == SearchLine_Otsu_Left_State[search_row])
                    {
                        left_found_count++;
                    }
                }

                if(left_found_count > 8)
                {
                    search_row = (int16)row + (int16)left_found_count;
                    if(search_row > SEARCH_LINE_OTSU_BOTTOM_ROW)
                    {
                        search_row = SEARCH_LINE_OTSU_BOTTOM_ROW;
                    }
                    left_slope = ((float)SearchLine_Otsu_Left_Border[row + 3] -
                                  (float)SearchLine_Otsu_Left_Border[search_row]) /
                                 (float)(left_found_count - 3);
                    if(left_slope > 0.0f)
                    {
                        left_extend_ready = 1;
                    }
                    else
                    {
                        left_extend_ready = 0;
                        if(left_slope < 0.0f)
                        {
                            SearchLine_Otsu_Left_Extend_Enable = 0;
                        }
                    }
                }
            }

            if(left_extend_ready)
            {
                left_border = (int16)((float)SearchLine_Otsu_Left_Border[extend_anchor_left] +
                                      left_slope * (float)(extend_anchor_left - row));
                SearchLine_Otsu_Left_Border[row] = SearchLine_Clamp_Otsu_Search_Col(left_border);
            }
        }

        if((SEARCH_LINE_STATE_WHITE == SearchLine_Otsu_Left_State[row]) &&
           (SEARCH_LINE_STATE_WHITE == SearchLine_Otsu_Right_State[row]))
        {
            SearchLine_Otsu_WhiteLine_Count++;
        }
        if((SEARCH_LINE_STATE_WHITE == SearchLine_Otsu_Left_State[row]) && (row < 55))
        {
            SearchLine_Otsu_Left_Lost_Count++;
        }
        if((SEARCH_LINE_STATE_WHITE == SearchLine_Otsu_Right_State[row]) && (row < 55))
        {
            SearchLine_Otsu_Right_Lost_Count++;
        }

        SearchLine_Otsu_Left_Border[row] = SearchLine_Clamp_Otsu_Search_Col(SearchLine_Otsu_Left_Border[row]);
        SearchLine_Otsu_Right_Border[row] = SearchLine_Clamp_Otsu_Search_Col(SearchLine_Otsu_Right_Border[row]);
        SearchLine_Otsu_Center_Line[row] = (uint8)(((uint16)SearchLine_Otsu_Left_Border[row] + (uint16)SearchLine_Otsu_Right_Border[row]) / 2);
        SearchLine_Otsu_Center_Found[row] = 1;

        if(((int16)SearchLine_Otsu_Right_Border[row] - (int16)SearchLine_Otsu_Left_Border[row]) <= SEARCH_LINE_OTSU_MIN_WIDTH)
        {
            SearchLine_Otsu_Offline_Row = (uint8)(row + 1);
            break;
        }
        else if((SearchLine_Otsu_Right_Border[row] <= SEARCH_LINE_OTSU_EDGE_LIMIT) ||
                (SearchLine_Otsu_Left_Border[row] >= (SEARCH_LINE_OTSU_W - 1 - SEARCH_LINE_OTSU_EDGE_LIMIT)))
        {
            SearchLine_Otsu_Offline_Row = (uint8)(row + 1);
            break;
        }
    }
}

static void SearchLine_DrawExtensionLine_Otsu(void)
{
    int16 row = 0;
    int16 scan_row = 0;
    int16 fill_row = 0;
    int16 tfsite = 55;
    int16 ftsite = 0;
    float slope = 0.0f;

    tfsite = 55;
    if(SearchLine_Otsu_Left_Extend_Enable)
    {
        for(row = SEARCH_LINE_OTSU_START_ROW; row >= (int16)SearchLine_Otsu_Offline_Row + 4; row--)
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
                        SearchLine_Otsu_Left_Border[fill_row] = SearchLine_Clamp_Otsu_Search_Col((int16)(slope * (float)(fill_row - tfsite) +
                                                                                                         (float)SearchLine_Otsu_Left_Border[tfsite]));
                        SearchLine_Otsu_Left_State[fill_row] = SEARCH_LINE_STATE_FOUND;
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

    tfsite = 55;
    if(SearchLine_Otsu_Right_Extend_Enable)
    {
        for(row = SEARCH_LINE_OTSU_START_ROW; row >= (int16)SearchLine_Otsu_Offline_Row + 4; row--)
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
                        SearchLine_Otsu_Right_Border[fill_row] = SearchLine_Clamp_Otsu_Search_Col((int16)(slope * (float)(fill_row - tfsite) +
                                                                                                          (float)SearchLine_Otsu_Right_Border[tfsite]));
                        SearchLine_Otsu_Right_State[fill_row] = SEARCH_LINE_STATE_FOUND;
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
        SearchLine_Otsu_Center_Line[row] = (uint8)(((uint16)SearchLine_Otsu_Left_Border[row] + (uint16)SearchLine_Otsu_Right_Border[row]) / 2);
        SearchLine_Otsu_Center_Found[row] = 1;
    }
}

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
                        SearchLine_Otsu_Center_Line[fill_row] = SearchLine_Clamp_Otsu_Col((int16)((float)center_temp +
                                                                                                  center_slope * (float)(fill_row - line_temp)));
                        SearchLine_Otsu_Center_Found[fill_row] = 1;
                    }
                    row = search_row;
                    break;
                }
            }
        }

        SearchLine_Otsu_Center_Line[row] = SearchLine_Clamp_Otsu_Col((int16)(((uint16)SearchLine_Otsu_Center_Line[row - 1] +
                                                                               (uint16)(2 * SearchLine_Otsu_Center_Line[row])) / 3));
        SearchLine_Otsu_Center_Found[row] = 1;
    }
}

/*
 * 普通赛道先保留最小主链：
 * 上一行约束窗口搜边；
 * 单边丢失时按半宽模型补一侧；
 * 两边都失真或宽度异常时提前停搜。
 */
static void SearchLine_Update_Center(void)
{
    uint16 row = 0;

    for(row = 0; row < Search_Image_H; row++)
    {
        if(row < SEARCH_VALID_TOP_ROW || row > SEARCH_VALID_BOTTOM_ROW)
        {
            Center_Line[row] = Search_Image_W / 2;
            Center_Line_Found[row] = 0;
        }
        else
        {
            Center_Line[row] = (uint8)SearchLine_Limit_Int32(Center_Line[row], CONTRASTOFFSET, Search_Image_W - 1 - CONTRASTOFFSET);
        }
    }
}

/*
 * OTSU 链路内部跑 80x60，但上层控制和显示都还是按 188x120 的逐行数组取值。
 * 这里把压缩域结果映射回原始数组，尽量少动 app 层代码。
 */
static void SearchLine_Copy_Otsu_To_Raw(void)
{
    uint16 row = 0;
    uint16 raw_row = 0;
    uint8 raw_left = 0;
    uint8 raw_right = 0;
    uint8 raw_center = 0;

    SearchLine_Reset_Output_Buffer();

    for(row = SearchLine_Otsu_Offline_Row; row < SEARCH_LINE_OTSU_H; row++)
    {
        raw_left = SearchLine_Otsu_To_Raw_Col_Map[SearchLine_Otsu_Left_Border[row]];
        raw_right = SearchLine_Otsu_To_Raw_Col_Map[SearchLine_Otsu_Right_Border[row]];
        raw_center = SearchLine_Otsu_To_Raw_Col_Map[SearchLine_Otsu_Center_Line[row]];

        for(raw_row = SearchLine_Otsu_Row_Start_Map[row]; raw_row <= SearchLine_Otsu_Row_End_Map[row]; raw_row++)
        {
            Left_Edge_Line[raw_row] = raw_left;
            Right_Edge_Line[raw_row] = raw_right;
            Left_Edge_Found[raw_row] = (SEARCH_LINE_STATE_FOUND == SearchLine_Otsu_Left_State[row]) ? 1 : 0;
            Right_Edge_Found[raw_row] = (SEARCH_LINE_STATE_FOUND == SearchLine_Otsu_Right_State[row]) ? 1 : 0;
            Center_Line[raw_row] = raw_center;
            Center_Line_Found[raw_row] = SearchLine_Otsu_Center_Found[row];
        }
    }

    SearchLine_Update_Center();
}

static void SearchLine_Process_Otsu(void)
{
    SearchLine_Init_Otsu_Map();
    SearchLine_Clear_Otsu_State();
    SearchLine_Binarize_Otsu_Image();
    SearchLine_Filter_Otsu_Binary();
    SearchLine_Init_Otsu_BottomRows();
    SearchLine_DrawLinesProcess_Otsu();
    SearchLine_DrawExtensionLine_Otsu();
    SearchLine_RouteFilter_Otsu();
    SearchLine_Copy_Otsu_To_Raw();
}

void SearchLine_Process(void)
{
    gpio_set_level(IO_P52, 0);
    SearchLine_Process_Otsu();
    gpio_set_level(IO_P52, 1);
}

void SearchLine_DrawOverlay(void)
{
    uint16 row = 0;

    for(row = SEARCH_VALID_TOP_ROW; row <= SEARCH_VALID_BOTTOM_ROW; row++)
    {
        if(Left_Edge_Found[row] && Left_Edge_Line[row] < CAMERA_VALID_W)
        {
            ips200_draw_point(Left_Edge_Line[row], row, RGB565_YELLOW);
        }

        if(Right_Edge_Found[row] && Right_Edge_Line[row] < CAMERA_VALID_W)
        {
            ips200_draw_point(Right_Edge_Line[row], row, RGB565_CYAN);
        }

        if(Center_Line_Found[row] && Center_Line[row] < CAMERA_VALID_W)
        {
            ips200_draw_point(Center_Line[row], row, RGB565_RED);
        }
    }
}

uint8 SearchLine_IsCenterFound(uint8 row)
{
    if(row >= Search_Image_H)
    {
        return 0;
    }

    return Center_Line_Found[row];
}

uint8 SearchLine_GetCenterFoundRowCount(void)
{
    uint16 row = 0;
    uint8 found_count = 0;

    for(row = SEARCH_VALID_TOP_ROW; row <= SEARCH_VALID_BOTTOM_ROW; row++)
    {
        if(Center_Line_Found[row])
        {
            if(found_count < 255)
            {
                found_count++;
            }
        }
    }

    return found_count;
}
