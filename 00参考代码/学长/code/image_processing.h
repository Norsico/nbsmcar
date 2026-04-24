/*
 * image_processing.h
 *
 *  Created on: 2025年2月26日
 *      Author: 15958
 */

#ifndef CODE_IMAGE_PROCESSING_H_
#define CODE_IMAGE_PROCESSING_H_

#include "Platform_Types.h"                  //不加这个头文件没法使用外部声明extern 会导致识别不到
#include "zf_common_headfile.h"
#define TRUE               1
#define FALSE               0
//数据
#define Height                  1
#define ROW                  60  //图像行数
#define COL                  80  //图像列数

//*图像相关********************************************************************/
#define SIZE_HRE  60   //图像行数
extern uint16 MID_POINT;    //中值
#define Lstart                  1
#define Rstart                  2
#define Mstart                  3
#define White                   1
#define Black                   0
typedef struct {
  //图像X坐标
  uint8 ui8_ImageX;
  //图像Y坐标
  uint8  ui8_ImageY;
  //图像单边最远点
  uint8 ui8_MaxY;
  //图像最远点
  uint8 ui8_AllMaxY;
} LadderMovePoint;


typedef struct {
    //状态判断计数
  uint16        ui16_counter[5];
  //状态标志
  int8          i8_StatusFlag[9];
  //状态处理变量
  int8          i8_StatusHandle[9];
  //图像处理范围
  uint8         ui8_DisposeScopeUp;
  uint8         ui8_DisposeScopeDown;
  uint8         ui8_DisposeScopeLeft;
  uint8         ui8_DisposeScopeRight;
    //标准直道行位置
  float        f_BaseY[10];
    //标准数组
  uint8*        ui8_LineWidth;
    //标准权重
  float        f_BaseLineWeight[10];
      //行权重
  float        f_LineWeight[10];
  //控制量数组
  int16*        i16p_dataImage;
  //图像数组
  uint8         ui8_ImageArray[ROW][COL];
  //左边界
  int8         ui8_LPoint[ROW];
  //右边界
  int8         ui8_RPoint[ROW];
  //扫描行距离
  uint8         ui8_ScanLineY[10];  //十条动态横线的位置
  //扫描行左边界(补线)
  uint8         ui8_ScanLineL[10];
  //扫描行右边界(补线)
  uint8         ui8_ScanLineR[10];
  //扫描行左边界(最边界)
  uint8         ui8_ScanLineToL[10];
  //扫描行右边界(最边界)
  uint8         ui8_ScanLineToR[10];
  //扫描赛道宽度
  uint8         ui8_ScanLineWidth[10];
  //中值求取起点
  uint8         ui8_ScanDirection;
  //初始中值
  int16         i16_Mid[10];
  //最终中值
  int16         i16_FinallyMid[10];
  //最小可视距离
  uint8         ui8_MinH;
  int8          i8_MinHX;
  //反向可视距离
  uint8         VisitableScope;
  //爬梯最远点
  uint8        MaxPoint;
} Dispose_Image;
extern uint8 street_len_40; //中40线的直道长度
extern uint8 street_len_5; //中10线的直道长度
extern uint8 street_len_75; //中70线的直道长度;
extern uint8 street_len_47; //中47线的直道长度
extern uint8 street_len_33; //中33线的直道长度
extern uint8 street_len[80]; //每一列的直道长度
extern uint8 L_point;
extern uint8 R_point;
extern uint8 Vistable_scale;
extern uint8 Small;
extern uint8 Change_time;
extern uint16 hangkuan60[60];
extern void ConstructImage (void);
extern void Ladder (void) ;
extern uint8 LeftPointLadder (uint8* ui8p_LF);
extern uint8 image_threshold;
extern uint8 RightPointLadder (uint8* ui8p_RF);
extern void FitRoad (void);
extern void Fit (int8 i8_X1, int8 i8_X2, uint8 ui8_Y1, uint8 ui8_Y2);
extern void Fits (int8 i8_X1, int8 i8_X2, uint8 ui8_Y1, uint8 ui8_Y2);
extern void GetReverseVisualRange(void);
extern void Soft_Binarization(void);
extern uint8 otsuThreshold(uint8 *image, uint16 col, uint16 row);
extern void dispose_D_to_B(uint8 *p,uint8 value1);
extern void dispose_D_to_B1(uint8 *p,uint8 value1,uint8 value2,uint8 value3);
extern void Get_len(void);
extern void InitDisposeImageData (void);
extern void Display_ImageData(void);
void Seek_point(uint8 Y);
uint8 Seek_Write_point(uint8 x1,uint8 Y);
uint8 Seek_Black_point(uint8 x1,uint8 Y);



#endif /* CODE_IMAGE_PROCESSING_H_ */
