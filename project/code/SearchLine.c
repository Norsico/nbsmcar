#include "SearchLine.h"

uint8 Reference_Point = 0;
uint8 Reference_Col = Search_Image_W / 2;
uint8 White_Max_Point = 0;
uint8 White_Min_Point = 0;
uint8 Reference_Contrast_Ratio = 20;

uint8 Remote_Distance[Search_Image_W] = {0};
uint8 Left_Edge_Line[Search_Image_H] = {0};
uint8 Right_Edge_Line[Search_Image_H] = {0};
uint8 Center_Line[Search_Image_H] = {0};

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

static uint8 SearchLine_Find_Extreme_Value(uint8 *value_array, uint8 start_index, uint8 end_index, uint8 find_max)
{
    uint16 i = 0;
    uint8 best_index = 0;
    uint8 best_value = 0;
    uint8 temp_index = 0;

    start_index = (uint8)SearchLine_Limit_Int32(start_index, 0, Search_Image_W - 1);
    end_index = (uint8)SearchLine_Limit_Int32(end_index, 0, Search_Image_W - 1);

    if(start_index > end_index)
    {
        temp_index = start_index;
        start_index = end_index;
        end_index = temp_index;
    }

    best_index = start_index;
    best_value = value_array[start_index];

    for(i = (uint16)start_index + 1; i <= end_index; i++)
    {
        if(find_max)
        {
            if(value_array[i] > best_value)
            {
                best_value = value_array[i];
                best_index = (uint8)i;
            }
        }
        else
        {
            if(value_array[i] < best_value)
            {
                best_value = value_array[i];
                best_index = (uint8)i;
            }
        }
    }

    return best_index;
}

void Get_Reference_Point(void)
{
    uint16 end_row = SEARCH_VALID_BOTTOM_ROW + 1;
    uint16 start_row = 0;
    uint16 sample_rows = 0;
    uint16 i = 0;
    uint16 row = 0;
    uint32 gray_sum = 0;

    start_row = (uint16)SearchLine_Limit_Int32((int32)end_row - REFRENCEROW, SEARCH_VALID_TOP_ROW, SEARCH_VALID_BOTTOM_ROW);
    sample_rows = end_row - start_row;

    for(row = start_row; row < end_row; row++)
    {
        for(i = 0; i < Search_Image_W; i++)
        {
            gray_sum += mt9v03x_image[row][i];
        }
    }

    Reference_Point = (uint8)(gray_sum / (sample_rows * Search_Image_W));
    White_Max_Point = (uint8)SearchLine_Limit_Int32((int32)Reference_Point * WHITEMAXMUL / 10, BLACKPOINT, 255);
    White_Min_Point = (uint8)SearchLine_Limit_Int32((int32)Reference_Point * WHITEMINMUL / 10, BLACKPOINT, 255);
}

void Search_Reference_Col(void)
{
    int16 col = 0;
    int16 row = 0;
    int16 row_bottom = SEARCH_VALID_BOTTOM_ROW;
    int16 row_top = SEARCH_VALID_TOP_ROW + STOPROW;
    uint8 current_gray = 0;
    uint8 compare_gray = 0;
    uint16 gray_sum = 0;
    int16 contrast = 0;

    for(col = 0; col < Search_Image_W; col++)
    {
        Remote_Distance[col] = SEARCH_VALID_TOP_ROW;

        for(row = row_bottom; row >= row_top; row--)
        {
            current_gray = mt9v03x_image[row][col];
            compare_gray = mt9v03x_image[row - STOPROW][col];

            if(current_gray < White_Min_Point)
            {
                Remote_Distance[col] = (uint8)SearchLine_Limit_Int32(row - 1, SEARCH_VALID_TOP_ROW, SEARCH_VALID_BOTTOM_ROW);
                break;
            }

            if(compare_gray > White_Max_Point)
            {
                continue;
            }

            gray_sum = current_gray + compare_gray;
            if(!gray_sum)
            {
                continue;
            }

            contrast = ((int16)current_gray - (int16)compare_gray) * 200 / (int16)gray_sum;
            if(contrast > Reference_Contrast_Ratio)
            {
                Remote_Distance[col] = (uint8)SearchLine_Limit_Int32(row - 1, SEARCH_VALID_TOP_ROW, SEARCH_VALID_BOTTOM_ROW);
                break;
            }
        }
    }

    Reference_Col = SearchLine_Find_Extreme_Value(Remote_Distance, 10, Search_Image_W - 11, 0);
    Reference_Col = (uint8)SearchLine_Limit_Int32(Reference_Col, CONTRASTOFFSET, Search_Image_W - 1 - CONTRASTOFFSET);
}

void Search_line(void)
{
    int16 row_max = SEARCH_VALID_BOTTOM_ROW;
    int16 row_min = SEARCH_VALID_TOP_ROW;
    uint8 col_max = Search_Image_W - 1 - CONTRASTOFFSET;
    uint8 col_min = 3;
    int16 leftstartcol = Reference_Col;
    int16 leftendcol = col_min;
    int16 rightstartcol = Reference_Col;
    int16 rightendcol = col_max;
    uint8 temp1 = 0;
    uint8 temp2 = 0;
    int16 temp3 = 0;
    int16 col = 0;
    int16 row = 0;
    uint8 left_found_this_row = 0;
    uint8 right_found_this_row = 0;

    // 初始化边界线为默认值
    for(row = row_max; row >= row_min; row--)
    {
        Left_Edge_Line[row] = col_min - CONTRASTOFFSET;
        Right_Edge_Line[row] = col_max + CONTRASTOFFSET;
    }

    // 从底部向上逐行搜索
    for(row = row_max; row >= row_min; row--)
    {
        // 左边界搜索
        left_found_this_row = 0;

        if(row < row_max)
        {
            leftstartcol = SearchLine_Limit_Int32((int32)Left_Edge_Line[row + 1] + SEARCHRANGE, col_min, Search_Image_W - 1 - CONTRASTOFFSET);
            leftendcol = SearchLine_Limit_Int32((int32)Left_Edge_Line[row + 1] - SEARCHRANGE, col_min, leftstartcol);
            Left_Edge_Line[row] = Left_Edge_Line[row + 1];
        }
        else
        {
            leftstartcol = Reference_Col;
            leftendcol = col_min;
            Left_Edge_Line[row] = col_min;
        }

        for(col = leftstartcol; col >= leftendcol; col--)
        {
            temp1 = mt9v03x_image[row][col];

            // 当前点已经是黑区，说明这一行搜索起点不在白区内，直接保留上一行结果
            if(temp1 < White_Min_Point)
            {
                break;
            }

            temp2 = mt9v03x_image[row][col - CONTRASTOFFSET];

            // 左侧对比点仍然很白，说明还在白区内部，继续往左搜
            if(temp2 > White_Max_Point)
            {
                continue;
            }

            // 计算白区到左侧暗区的对比度，超过阈值就认为命中左边界
            if((temp1 + temp2) != 0)
            {
                temp3 = (temp1 - temp2) * 200 / (temp1 + temp2);
                if(temp3 > Reference_Contrast_Ratio)
                {
                    Left_Edge_Line[row] = (uint8)SearchLine_Limit_Int32(col - CONTRASTOFFSET + 1, col_min, Search_Image_W - 1 - CONTRASTOFFSET);
                    left_found_this_row = 1;
                    break;
                }
            }
        }

        if(!left_found_this_row && row == row_max)
        {
            Left_Edge_Line[row] = col_min;
        }

        // 右边界搜索
        right_found_this_row = 0;

        if(row < row_max)
        {
            rightstartcol = SearchLine_Limit_Int32((int32)Right_Edge_Line[row + 1] - SEARCHRANGE, Reference_Col, col_max);
            rightendcol = SearchLine_Limit_Int32((int32)Right_Edge_Line[row + 1] + SEARCHRANGE, rightstartcol, col_max);
            Right_Edge_Line[row] = Right_Edge_Line[row + 1];
        }
        else
        {
            rightstartcol = Reference_Col;
            rightendcol = col_max;
            Right_Edge_Line[row] = Search_Image_W - 1;
        }

        for(col = rightstartcol; col <= rightendcol; col++)
        {
            temp1 = mt9v03x_image[row][col];

            // 当前点已经是黑区，说明这一行搜索起点不在白区内，直接保留上一行结果
            if(temp1 < White_Min_Point)
            {
                break;
            }

            temp2 = mt9v03x_image[row][col + CONTRASTOFFSET];

            // 右侧对比点仍然很白，说明还在白区内部，继续往右搜
            if(temp2 > White_Max_Point)
            {
                continue;
            }

            // 计算白区到右侧暗区的对比度，超过阈值就认为命中右边界
            if((temp1 + temp2) != 0)
            {
                temp3 = (temp1 - temp2) * 200 / (temp1 + temp2);
                if(temp3 > Reference_Contrast_Ratio)
                {
                    Right_Edge_Line[row] = (uint8)SearchLine_Limit_Int32(col + CONTRASTOFFSET - 1, Reference_Col, Search_Image_W - 1);
                    right_found_this_row = 1;
                    break;
                }
            }
        }

        if(!right_found_this_row && row == row_max)
        {
            Right_Edge_Line[row] = Search_Image_W - 1;
        }

        // 更新下一行的搜索起始点
    }
}

void SearchLine_Update_Center(void)
{
    uint16 row = 0;
    uint16 center_value = 0;

    for(row = 0; row < Search_Image_H; row++)
    {
        if(row < SEARCH_VALID_TOP_ROW || row > SEARCH_VALID_BOTTOM_ROW)
        {
            Center_Line[row] = Search_Image_W / 2;
        }
        else
        {
            center_value = ((uint16)Left_Edge_Line[row] + (uint16)Right_Edge_Line[row]) / 2;
            Center_Line[row] = (uint8)SearchLine_Limit_Int32(center_value, CONTRASTOFFSET, Search_Image_W - 1 - CONTRASTOFFSET);
        }
    }
}

void SearchLine_Process(void)
{
    Get_Reference_Point();
    Search_Reference_Col();
    Search_line();
    SearchLine_Update_Center();
}

void SearchLine_DrawOverlay(void)
{
    uint16 row = 0;

    for(row = SEARCH_VALID_TOP_ROW; row <= SEARCH_VALID_BOTTOM_ROW; row++)
    {
        ips200_draw_point(Left_Edge_Line[row], row, RGB565_RED);
        ips200_draw_point(Right_Edge_Line[row], row, RGB565_GREEN);
        ips200_draw_point(Center_Line[row], row, RGB565_YELLOW);
    }

    ips200_show_string(0, 128, "REF:");
    ips200_show_uint8(32, 128, Reference_Col);
    ips200_show_string(64, 128, "CTR:");
    ips200_show_uint8(96, 128, Center_Line[SEARCH_VALID_BOTTOM_ROW]);
    ips200_show_string(0, 144, "WMIN:");
    ips200_show_uint8(40, 144, White_Min_Point);
    ips200_show_string(72, 144, "WMAX:");
    ips200_show_uint8(112, 144, White_Max_Point);
}