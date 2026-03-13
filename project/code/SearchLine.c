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
    uint16 start_row = Search_Image_H - REFRENCEROW;
    uint16 end_row = Search_Image_H;
    uint16 i = 0;
    uint16 row = 0;
    uint32 gray_sum = 0;

    for(row = start_row; row < end_row; row++)
    {
        for(i = 0; i < Search_Image_W; i++)
        {
            gray_sum += mt9v03x_image[row][i];
        }
    }

    Reference_Point = (uint8)(gray_sum / (REFRENCEROW * Search_Image_W));
    White_Max_Point = (uint8)SearchLine_Limit_Int32((int32)Reference_Point * WHITEMAXMUL / 10, BLACKPOINT, 255);
    White_Min_Point = (uint8)SearchLine_Limit_Int32((int32)Reference_Point * WHITEMINMUL / 10, BLACKPOINT, 255);
}

void Search_Reference_Col(void)
{
    int16 col = 0;
    int16 row = 0;
    uint8 current_gray = 0;
    uint8 compare_gray = 0;
    uint16 gray_sum = 0;
    int16 contrast = 0;

    for(col = 0; col < Search_Image_W; col++)
    {
        Remote_Distance[col] = STOPROW;

        for(row = Search_Image_H - 1; row > STOPROW; row--)
        {
            current_gray = mt9v03x_image[row][col];
            compare_gray = mt9v03x_image[row - STOPROW][col];

            if(current_gray < White_Min_Point)
            {
                Remote_Distance[col] = (uint8)(row - 1);
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
            if((contrast > Reference_Contrast_Ratio) || (row == (STOPROW + 1)))
            {
                Remote_Distance[col] = (uint8)(row - 1);
                break;
            }
        }
    }

    Reference_Col = SearchLine_Find_Extreme_Value(Remote_Distance, 10, Search_Image_W - 11, 0);
    Reference_Col = (uint8)SearchLine_Limit_Int32(Reference_Col, CONTRASTOFFSET, Search_Image_W - 1 - CONTRASTOFFSET);
}

void Search_line(void)
{
    uint8 row_max = Search_Image_H - STOPROW;
    uint8 row_min = STOPROW;
    uint8 col_max = Search_Image_W - 3;
    uint8 col_min = 3;
    int16 leftstartcol = Reference_Col;
    int16 rightstartcol = Reference_Col;
    int16 leftendcol = 0;
    int16 rightendcol = Search_Image_W;
    uint8 temp1 = 0;
    uint8 temp2 = 0;
    int16 temp3 = 0;
    int16 leftstop = 0;
    int16 rightstop = 0;
    int16 stoppoint = 0;
    int16 col = 0;
    int16 row = 0;

    // 初始化边界线为默认值
    for(row = row_max; row > row_min; row--)
    {
        Left_Edge_Line[row] = col_min - CONTRASTOFFSET;
        Right_Edge_Line[row] = col_max + CONTRASTOFFSET;
    }

    // 从底部向上逐行搜索
    for(row = row_max; row > row_min; row--)
    {
        // 左边界搜索
        if(!leftstop)
        {
            for(col = leftstartcol; col >= leftendcol; col--)
            {
                temp1 = mt9v03x_image[row][col];
                temp2 = mt9v03x_image[row][col - CONTRASTOFFSET];

                // 判断参考列是否为黑点，如为黑点则停止搜索
                if(temp1 < White_Min_Point && col == leftstartcol && leftstartcol == Reference_Col)
                {
                    leftstop = 1;
                    for(stoppoint = row; stoppoint >= 0; stoppoint--)
                    {
                        Left_Edge_Line[stoppoint] = col_min;
                    }
                    break;
                }

                // 判断搜索起始列是否为黑点，如为黑点则修正搜索起始范围
                if((temp1 < White_Min_Point && col == leftstartcol) || (leftstartcol != Reference_Col && col == col_min + 1))
                {
                    if(leftstartcol < Reference_Col)
                    {
                        col = Reference_Col;
                        leftstartcol = Reference_Col;
                    }
                }

                // 判断搜索结束列是否为白点，如为白点则修正搜索结束范围
                if(temp1 > White_Min_Point && col == leftendcol)
                {
                    if(leftendcol > col_min)
                    {
                        leftendcol = col_min;
                    }
                }

                // 判断当前点是否为黑点，如为黑点则不进行对比直接赋值
                if(temp1 < White_Min_Point && leftstartcol == Reference_Col)
                {
                    Left_Edge_Line[row] = (uint8)col;
                    break;
                }

                // 判断对比点是否为白点，如为白点则直接跳过
                if(temp2 > White_Max_Point)
                {
                    continue;
                }

                // 计算对比度
                if((temp1 + temp2) != 0)
                {
                    temp3 = (temp1 - temp2) * 200 / (temp1 + temp2);
                    // 判断对比度是否超过阈值，如超过阈值则认为是左边界，或者已经搜索到搜索边界，则直接赋值边界
                    if(temp3 > Reference_Contrast_Ratio || col == col_min + 1)
                    {
                        Left_Edge_Line[row] = col - CONTRASTOFFSET + 1;
                        // 刷新搜索范围
                        leftendcol = SearchLine_Limit_Int32(col - SEARCHRANGE, col_min, col);
                        break;
                    }
                }
            }
        }

        // 右边界搜索
        if(!rightstop)
        {
            for(col = rightstartcol; col <= rightendcol; col++)
            {
                temp1 = mt9v03x_image[row][col];
                temp2 = mt9v03x_image[row][col + CONTRASTOFFSET];

                // 判断参考列是否为黑点，如为黑点则停止搜索
                if(temp1 < White_Min_Point && col == rightstartcol && rightstartcol == Reference_Col)
                {
                    rightstop = 1;
                    for(stoppoint = row; stoppoint >= 0; stoppoint--)
                    {
                        Right_Edge_Line[stoppoint] = col_max;
                    }
                    break;
                }

                // 判断搜索起始列是否为黑点，如为黑点则修正搜索起始范围
                if((temp1 < White_Min_Point && col == rightstartcol) || (rightstartcol != Reference_Col && col == col_max - 1))
                {
                    if(rightstartcol > Reference_Col)
                    {
                        col = Reference_Col;
                        rightstartcol = Reference_Col;
                    }
                }

                // 判断搜索结束列是否为白点，如为白点则修正搜索结束范围
                if(temp1 > White_Min_Point && col == rightendcol)
                {
                    if(rightendcol < col_max)
                    {
                        rightendcol = col_max;
                    }
                }

                // 判断当前点是否为黑点，如为黑点则不进行对比直接赋值
                if(temp1 < White_Min_Point && rightstartcol == Reference_Col)
                {
                    Right_Edge_Line[row] = (uint8)col;
                    break;
                }

                // 判断对比点是否为白点，如为白点则直接跳过
                if(temp2 > White_Max_Point)
                {
                    continue;
                }

                // 计算对比度
                if((temp1 + temp2) != 0)
                {
                    temp3 = (temp1 - temp2) * 200 / (temp1 + temp2);
                    // 判断对比度是否超过阈值，如超过阈值则认为是右边界，或者已经搜索到搜索边界，则直接赋值边界
                    if(temp3 > Reference_Contrast_Ratio || col == col_max - 1)
                    {
                        Right_Edge_Line[row] = col + CONTRASTOFFSET - 1;
                        // 刷新搜索范围
                        rightendcol = SearchLine_Limit_Int32(col + SEARCHRANGE, col, col_max);
                        break;
                    }
                }
            }
        }

        // 更新下一行的搜索起始点
        leftstartcol = Left_Edge_Line[row];
        rightstartcol = Right_Edge_Line[row];
    }
}

void SearchLine_Update_Center(void)
{
    uint16 row = 0;
    uint16 center_value = 0;

    for(row = 0; row < Search_Image_H; row++)
    {
        center_value = ((uint16)Left_Edge_Line[row] + (uint16)Right_Edge_Line[row]) / 2;
        Center_Line[row] = (uint8)SearchLine_Limit_Int32(center_value, CONTRASTOFFSET, Search_Image_W - 1 - CONTRASTOFFSET);
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

    for(row = STOPROW + 1; row < Search_Image_H; row++)
    {
        ips200_draw_point(Left_Edge_Line[row], row, RGB565_RED);
        ips200_draw_point(Right_Edge_Line[row], row, RGB565_GREEN);
        ips200_draw_point(Center_Line[row], row, RGB565_YELLOW);
    }

    ips200_show_string(0, 128, "REF:");
    ips200_show_uint8(32, 128, Reference_Col);
    ips200_show_string(64, 128, "CTR:");
    ips200_show_uint8(96, 128, Center_Line[Search_Image_H - 1]);
    ips200_show_string(0, 144, "WMIN:");
    ips200_show_uint8(40, 144, White_Min_Point);
    ips200_show_string(72, 144, "WMAX:");
    ips200_show_uint8(112, 144, White_Max_Point);
}
