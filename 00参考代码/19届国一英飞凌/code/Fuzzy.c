/*
 * Fuzzy.c
 *
 *  Created on: 2024年3月1日
 *      Author: Kurumi
 */

#include <image.h>
#include "Fuzzy.h"
#include "steer.h"
#include "motor.h"
extern PID_Datatypedef SteerPIDdata;    //舵机的PID参数
#define H_Min   2
extern uint16 adcsum;
extern SystemDatatypdef SystemData;
float Left_Speed_Co = 0.007, Right_Speed_Co = 0.007;
float kz= 1,err_s= 0; //主动偏差系数与被动偏差系数
extern float   fan_pwm_coff;         //非直道负压风扇系数
extern uint16 fan_pwm_Straight;      //负压风扇基础pwm
extern uint16 fan_pwm_out1_max;       //负压风扇1最大输出
extern uint16 fan_pwm_out2_max;       //负压风扇2最大输出
extern int fan_pin;                   //负压风扇频率

extern uint8 die;
void Data_Settings()           //参数赋值
{
  //如果不用按键和flash赋值就往这里面赋值
                       //调整好的图像数据就别动了

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
  ImageStatus.Zebra_read = 0;
  SystemData.Stop = 1; //停车标志位

//  ImageStatus.TowPoint =27;           //前瞻
  ImageStatus.Threshold_static = 70;   //静态阈值  40-80
  ImageStatus.Threshold_detach = 180;  //阳光算法  亮斑分离140-220
  ImageScanInterval = 2;               //扫边范围    上一行的边界+-ImageScanInterval
  ImageScanInterval_Cross = 2;         //十字扫线范围
 // ImageStatus.variance = 26;           //直道方差阈值
  ImageStatus.variance_acc = 25;       //直道检测
//  SystemData.outbent_acc  =  5;
  SystemData.clrcle_num=0;
  ImageStatus.newblue_flag=0;
  rings_choose = 0;
//  fan_pwm_Straight = 750;     //负压风扇基础pwm
  fan_pwm_coff = 4;        //负压风扇系数        已经没用了
  fan_pwm_out1_max = 975;      //最大值限幅  也没用了
  fan_pwm_out2_max = 975;
  Speed_decision();     //驱动二套代码赋值     这个优先级高于flash赋值
  /**位置式pid参数**/
//  SteerPIDdata.P = 24;  //16  2.2   25  48
//  SteerPIDdata.I = 0;
//  SteerPIDdata.D = 56;  //64

//    steer_middle = 513;  //舵机中值440
//    steer_left = 435;    //左转   372
//    steer_right = 594;   //右转   505

}
