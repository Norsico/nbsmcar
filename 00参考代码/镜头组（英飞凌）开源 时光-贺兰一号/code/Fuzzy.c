/*
 * Fuzzy.c
 *
 *  Created on: 2024年2月27日
 *      Author: xiaoming
 */

#include "Fuzzy.h"
#include "C_H.h"
#include "motor.h"
extern PID_Datatypedef SteerPIDdata;    //舵机的PID参数
#define H_Min   2
uint16 adcsum;
float Left_Speed_Co = 0.007, Right_Speed_Co = 0.007;   //0.6
///extern int BlueTooth_Flag;
void Data_Settings(void)           //参数赋值
{
  //图像参数
  //adcsum = 0;
  ImageStatus.MiddleLine = 39;//中线
  ImageStatus.TowPoint_Gain = 0.2;
  ImageStatus.TowPoint_Offset_Max = 5;
  ImageStatus.TowPoint_Offset_Min = -2;
  ImageStatus.TowPointAdjust_v = 160;
  ImageStatus.Det_all_k = 0.7;  //待定自动补线斜率
  ImageStatus.CirquePass = 'F';
  ImageStatus.IsCinqueOutIn = 'F';
  ImageStatus.CirqueOut = 'F';
  ImageStatus.CirqueOff = 'F';
  ImageStatus.Barn_Flag = 0;
  ImageStatus.straight_acc = 0;

  ImageStatus.TowPoint =24;           //前瞻  ///
  ImageStatus.Threshold_static = 70;   //静态阈值  40-80
  ImageStatus.Threshold_detach = 180;  //阳光算法  亮斑分离140-220
  ImageScanInterval = 2;               //扫边范围    上一行的边界+-ImageScanInterval
  ImageScanInterval_Cross = 2;         //十字扫线范围
  ImageStatus.variance_acc = 25;       //直道检测
  SystemData.clrcle_num=0;
  ImageStatus.newblue_flag=0;
  SystemData.Stop = 1;                 //启动标志位

  /**位置式pid参数**/
  SteerPIDdata.P = 11.8;
  SteerPIDdata.I = 0.0;
  SteerPIDdata.D = 30.0;

}
