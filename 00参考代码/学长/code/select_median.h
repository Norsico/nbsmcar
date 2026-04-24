/*
 * select_median.h
 *
 *  Created on: 2025年2月26日
 *      Author: 15958
 */

#ifndef CODE_SELECT_MEDIAN_H_
#define CODE_SELECT_MEDIAN_H_


#include "zf_common_headfile.h"
extern int poserror ;
extern int poserror_array[4];
typedef struct
{
  float        f_KpL;
  float        f_KdL;
  float        f_KiL;
  float        f_KpR;
  float        f_KdR;
  float        f_KiR;

  uint32        ui16_StuckTime; //卡死计数
  uint8         ui8_Hto[3];     //反向可视距离偏差02.22
  int32         i16_PulseBase;  //基础速度
  int32         i16_PulseCtrl;
  int32         i16_PulseL;
  int32         i16_PulseR;
  int32         i16_PulseAvg;
  int32         i16_PulseErL[10];
  int32         i16_PulseErR[10];
  int16         i32_GoalPulseL;
  int32         i32_GoalPulseR;
  int32         i16_DutyL;
  int32         i16_DutyR;
  float         f_KDif;     //由舵机占空比算差速角系数（陀螺仪）
} DianjiData;
/*起始控制********************************************************************/
/*图像处理函数*/
void DisposeImage (void);
/*起始控制********************************************************************/

/*共用处理********************************************************************/
/*正常提取中值*/
void NormalControl (void);

/*确定最后中值*/
void DetermineMid (void);

void LostWayControl (void);
/*共用处理********************************************************************/

/*行处理**********************************************************************/
/*确定权重*/
void DetermineWeight (void);

/*确定扫描行*/
void DetermineScanLine (void);
/*行处理**********************************************************************/

/*入环处理*/
void  InRingControl(void);

/*出环处理*/
void  OutRingControl(void);

/*环岛中*/
void  RingControl(void);
/*ruku*/
void InChekuControl(void);
//元素定义声明
#define   OUT_PARK  0  //出车库
#define   IN_PARK   1 //进入车库
#define   L_Little  2  //左小环
#define   R_Little  3  //右小环
#define   L_Big  4  //左大环
#define   R_Big  5  //右大环
#define   RAMP  6  //坡道
#define   L_SANCHA  7 //左入三叉
#define   R_SANCHA  8 //右入三叉
#define   Crosses   9//十字路口
extern uint16 Goal_Speed;
extern uint8 Special_Count;
extern uint8 Special_type[12];//元素数组





#endif /* CODE_SELECT_MEDIAN_H_ */
