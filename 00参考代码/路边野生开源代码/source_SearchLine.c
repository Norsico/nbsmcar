#include "SearchLine.h"

uint8 Reference_Point = 0;                // 动态参考点
uint8 Reference_Col   = 0;                // 动态参考列
uint8 White_Max_Point = 0;                // 动态白点最大值
uint8 White_Min_Point = 0;                // 动态白点最小值
uint8 Reference_Contrast_Ratio = 20;      // 参考对比度


uint8 Remote_Distance[Search_Image_W] ={ 0 };      // 白点远端距离
uint8 Left_Edge_Line[Search_Image_H]  ={ 0 };      // 左边界
uint8 Right_Edge_Line[Search_Image_H] ={ 0 };      // 右边界


/*******************************************************************************
* 函 数 名 * : LimitDouMid
* 函数功能 * :  将value限幅到limit1与limit2之间
*******************************************************************************/
int32 LimitDouMid(int32 value,int32 limit1,int32 limit2)
{
    int32 buf = 0;

    if(limit1 > limit2)     // 要求limit1 < limit2
    {
        buf    = limit1;
        limit1 = limit2;
        limit2 = buf;
    }

    if(value < limit1)      return limit1;
    else if(value > limit2) return limit2;
    else                    return value;
}


/*******************************************************************************
* 函 数 名 * : Find_extreme_value
* 函数功能 * :  返回一维数组最大值或最小值
*  *va     :数组指针
*  num0    :起始点
*  num1    :结束点
*  model   : 1 返回最大值    0  返回最小值
*******************************************************************************/
int Find_extreme_value(uint8 *va , uint8 num0 ,uint8 num1 ,uint8 model)
{
    uint8 i = 0, temp = 0, temp1 = 0, temp2 = 0, value = 0;
    if(num0 > num1){
        temp1 = num0 - num1;
        temp2 = num1;
        va += num0;
        value = *va;
        if(model)
        {
            for(i = 0; i <= temp1; i ++){
                temp = *(va - i);
                if(temp > value) { temp2 = num0 - i; value = temp; }
            }
        }
        else {
            for(i = 0; i <= temp1; i ++){
                temp = *(va - i);
                if(temp < value) { temp2 = num0 - i; value = temp; }
            }
        }
    }
    else{
        temp1 = num1 - num0;
        temp2 = num0;
        va += num0;
        value = *va;
        if(model)
        {
            for(i = 0; i <= temp1; i ++){
                temp = *(va + i);
                if(temp > value) { temp2 = i + num0; value = temp; }
            }
        }
        else {
            for(i = 0; i <= temp1; i ++){
                temp = *(va + i);
                if(temp < value) { temp2 = i + num0; value = temp; }
            }
        }
    }
    return temp2;
}

// 获取图像参考点，白点最大值，白点最小值
void Get_Reference_Point(void)
{
  uint8 *p = mt9v03x_image[Search_Image_H-REFRENCEROW];     // 图像数组指针指向待统计的起始点
  uint16 temp  = 0;                                         // 保存统计点总数量
  uint32 temp1 = 0;                                         // 保存所有统计点加起来的和
  temp = REFRENCEROW * Search_Image_W;                      // 计算待统计点总数量
  for(int i = 0; i < temp; i ++) temp1 += *(p + i);         // 统计点求和
  Reference_Point = (uint8)(temp1 / temp);                  // 计算点平均值，作为本幅图像的参考点
  White_Max_Point = (uint8)LimitDouMid((int32)Reference_Point * WHITEMAXMUL / 10, BLACKPOINT, 255);    // 根据参考点计算白点最大值
  White_Min_Point = (uint8)LimitDouMid((int32)Reference_Point * WHITEMINMUL / 10, BLACKPOINT, 255);    // 根据参考点计算白点最小值
}

// 搜索每列白点距离及图像参考列，便于搜索边线
void Search_Reference_Col(void)
{
    int col, row;
    uint8 temp1 = 0, temp2 = 0;
    int temp3 = 0;
    uint8 *p = mt9v03x_image[0];          // 图像数组指针
    for(col = 0; col < Search_Image_W; col ++)
    {
        for(row = Search_Image_H - 1; row > STOPROW; row --){

            temp1 = *(p + row*Search_Image_W + col);                // 获取当前点灰度值
            temp2 = *(p + (row - STOPROW) * Search_Image_W + col);  // 获取对比点灰度值

            if(temp2 > White_Max_Point) continue;                   // 判断对比点是否为白点 若为白点则直接跳过

            if(temp1 < White_Min_Point) { Remote_Distance[col] = row - 1; break; }  // 判断当前点是否为黑点 若为黑点则不进行对比直接赋值

            temp3 = (temp1 - temp2) * 200 / (temp1 + temp2);        // 计算对比度
            // 判断对比度是否大于阈值 如果大于阈值则认为搜索到边界 或者已经搜索到该行最边界，则直接赋值边界
            if(temp3 > Reference_Contrast_Ratio || row == STOPROW + 1){Remote_Distance[col] = row - 1; break;}
        }
    }
    Reference_Col = (uint8)Find_extreme_value(Remote_Distance, 10, Search_Image_W - 10, 0);     // 求出最远的白点列作为参考列
    Reference_Col = (uint8)LimitDouMid(Reference_Col, 1, Search_Image_W - 2);                   // 参考列限幅 防止数据溢出
}

//搜索赛道边界
void Search_line(void)
{
    uint8 *p = mt9v03x_image[0];              // 图像数组指针
    uint8 row_max = Search_Image_H-STOPROW;   // 行最大值
    uint8 row_min = STOPROW;                  // 行最小值
    uint8 col_max = Search_Image_W-3;         // 列最大值
    uint8 col_min = 3;                        // 列最小值
    int16 leftstartcol  = Reference_Col;      // 搜线左起始列
    int16 rightstartcol = Reference_Col;      // 搜线右起始列
    int16 leftendcol    = 0;                  // 搜线左终止列
    int16 rightendcol   = Search_Image_W;     // 搜线右终止列
    uint8 temp1 = 0, temp2 = 0;               // 临时变量  用于存储图像数据
    int temp3 = 0;                            // 临时变量  用于存储对比度
    int leftstop = 0, rightstop = 0, stoppoint = 0; // 搜线自锁变量

    int col, row;

    for(row = row_max; row > row_min; row --){
        Left_Edge_Line[row]  = col_min - CONTRASTOFFSET;
        Right_Edge_Line[row] = col_max + CONTRASTOFFSET;
    }

    for(row = row_max; row > row_min; row --){
        p = &mt9v03x_image[row][0];           // 获取本行起点位置指针
        if(!leftstop){
            for(col = leftstartcol; col >= leftendcol; col --){
                temp1 = *(p + col);                 //获取当前点灰度值
                temp2 = *(p + col - CONTRASTOFFSET);  //获取对比点灰度值
                //判断参考列是否为黑点 若为黑点则放弃搜线
                if(temp1 < White_Min_Point && col == leftstartcol && leftstartcol == Reference_Col){
                    leftstop = 1;                 //线搜索自锁 不在进行左边线搜索
                    for(stoppoint = row; stoppoint >= 0; stoppoint --) Left_Edge_Line[stoppoint] = col_min;    //清除剩余的无效左边线
                    break;
                }
                //判断行搜索起始点是否为黑点 若为黑点则重新确立起始范围
                if((temp1 < White_Min_Point && col == leftstartcol) || (leftstartcol != Reference_Col && col == col_min + 1)){
                    if(leftstartcol < Reference_Col){ col = Reference_Col; leftstartcol = Reference_Col; }
                }
                //判断行搜索终止点是否为白点 若为白点则重新确立结束范围
                if(temp1 > White_Min_Point && col == leftendcol){
                    if(leftendcol > col_min){ leftendcol = col_min; }
                }
                //判断当前点是否为黑点 若为黑点则不进行对比直接赋值
                if(temp1 < White_Min_Point && leftstartcol == Reference_Col){ Left_Edge_Line[row] = (uint8)col; break;}
                //判断对比点是否为白点 若为白点则直接跳过
                if(temp2 > White_Max_Point) continue;
                //计算对比度
                temp3 = (temp1 - temp2) * 200 / (temp1 + temp2);
                //判断对比度是否大于阈值 如果大于阈值则认为是行边界  或者已经搜索到该行最边界，则直接赋值行边界
                if(temp3 > Reference_Contrast_Ratio || col == col_min + 1){
                    Left_Edge_Line[row] = col - CONTRASTOFFSET+1;     //保存当前行左边界
                    //刷新搜线范围
                    //leftstartcol = (uint8)LimitDouMid(col + SEARCHRANGE, col, Reference_Col);
                    leftendcol   = (uint8)LimitDouMid(col - SEARCHRANGE, col_min, col);
                    break;
                }
            }
        }
        if(!rightstop){
            for(col = rightstartcol; col <= rightendcol; col ++){
                temp1 = *(p + col);                 //获取当前点灰度值
                temp2 = *(p + col + CONTRASTOFFSET);  //获取对比点灰度值
                //判断参考列是否为黑点 若为黑点则放弃搜线
                if(temp1 < White_Min_Point && col == rightstartcol && rightstartcol == Reference_Col){
                    rightstop = 1;                 //线搜索自锁 不再进行右边线搜索
                    for(stoppoint = row; stoppoint >= 0; stoppoint --) Right_Edge_Line[stoppoint] = col_max;    //清除剩余的无效右边线
                    break;
                }
                //判断行搜索起始点是否为黑点 若为黑点则重新确立搜线范围
                if((temp1 < White_Min_Point && col == rightstartcol) || (rightstartcol != Reference_Col && col == col_max - 1)){
                    if(rightstartcol < Reference_Col){col = Reference_Col; rightstartcol = Reference_Col;}
                }
                //判断行搜索终止点是否为白点 若为白点则重新确立结束范围
                if(temp1 < White_Max_Point && col == rightendcol){
                    if(rightendcol > col_max) rightendcol = col_max;
                }
                //判断当前点是否为黑点 若为黑点则不进行对比直接赋值
                if(temp1 < White_Min_Point && rightstartcol == Reference_Col){ Right_Edge_Line[row] = (uint8)col; break;}
                //判断待对比点是否为白点 若为白点则直接跳过
                if(temp2 > White_Max_Point) continue;
                //计算对比度
                temp3 = (temp1 - temp2) * 200 / (temp1 + temp2);
                //判断对比度是否大于阈值 如果大于阈值则认为是行边界  或者已经搜索到该行最边界，则直接赋值行边界
                if(temp3 > Reference_Contrast_Ratio || col == col_max - 1){
                    Right_Edge_Line[row] = col + CONTRASTOFFSET - 1;     //保存当前行右边界
                    //刷新搜线范围
                    //rightstartcol = (uint8)LimitDouMid(col - SEARCHRANGE, Reference_Col, col);
                    rightendcol   = (uint8)LimitDouMid(col + SEARCHRANGE, col, col_max);
                    break;
                }
            }
        }
    }
}


