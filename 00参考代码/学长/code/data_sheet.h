/*
 * data_sheet.h
 *
 *  Created on: 2025年2月26日
 *      Author: 15958
 */

#ifndef CODE_DATA_SHEET_H_
#define CODE_DATA_SHEET_H_

#include "zf_common_headfile.h"
//*动态中值****************************/

#define HtoE            0.65   //可视距离对偏差影响 0.25  0.58  0.295
#define VtoLine         0.0460  //速度对权重影响 0.003    0.045 0.0340
//*动态中值****************************/

extern int jump_linelength;
/*状态****************************/
//直道
#define Straight                2
//弯道
#define Curve                   3
//圆环
#define Ring                    4
//十字路
//#define Cross                   5
//坡道
#define Ramp                    6
//断路
#define Breaker                 7
//横断路障
#define Roadblock               8
/*状态*****************************/

/*状态标志*****************************/
//*长直道**************************/
#define LSF                     1
//长直道判定终止行
#define LSEndLine               40
//长直道判定左右终止行
#define LSEndLineLR             40
//波动最大值
#define LSMaxVariance           2
//直道宽度偏差
#define LSWidthEr               5
//*长直道**************************/

//*环岛**************************/
#define RingF                   2
#define RightRing               1
#define LeftRing                2

/**************************************/
#define Ring_flag1              1





//*坡道**************************/
#define RampF                   3
#define RAMP_In_Flag            1
#define RAMP_On_Flag            2
#define RAMP_ToOut_Flag         3
#define RAMP_Out_Flag           4
#define RAMP_Speed              230
#define Ramp_MinLine            16
//*坡道**************************/

//*斑马线**************************/
#define ZebraF                  4
#define ZebraTimes              3//最小跳变次数
#define FlagZebra(x)            x//斑马线所在行
#define ZebraWidth              1//斑马线最小宽度
#define END_SpeedPluse          100
#define END_COUNT               11
//*坡道**************************/


//安全等级划分距离(mm)
#define Blue_Distance          1000
#define Yellow_Distance        900
#define Orange_Distance        600
#define Red_Distance           400
#define Unable_Distance        270
#define Max_Distance           2000
#define HD_SpeedPluse          70
#define HD_MINLINE              13//路障最小距离
//*坡道**************************/

//电机卡死计数
#define STUCK_TIMES             500


//*当前中值************************/
#define Mid_point               0
/*当前中值*************************/

//*可视距离***********************/
#define Height                  1
/*可视距离************************/

//*状态****************************/
#define State                   2
/*状态*****************************/
#define BeeOn                gpio_set_level (P33_10, 1);
#define BeeOff               gpio_set_level (P33_10, 0);
//按键
#define OK                     !gpio_get_level(B0)
#define Up                     !gpio_get_level(A8)
#define Down                   !gpio_get_level(D8)
#define Right                  !gpio_get_level(B12)
extern int Ring_type[6];//当前要过的圆环类型，设定了最多5个圆环
extern int Ring_times;//当前已通过的圆环次数
extern int Ring_mode;//圆环循迹方式,1为旧版本,2为新版本
extern int Ringtypes;//小圆环为1，中圆环为2，大圆环为3
extern int speed_extra1;
extern int DB_times;//当前已过单边桥次数
extern int DB_type[5];//当前要过的单边桥类型,设定了最多5个单边桥
extern int jumptypes;//跳过为1，绕过为2
extern int luzhang_types[5];//路障为1，斑马线为2
extern int luzhang_times;//路障通过次数
extern int ramp_length[5];//坡道长度
extern int ramp_times;//坡道次数
extern int ramp_speed;//过坡道的速度
extern int ring_speed;//过圆环的速度
extern int jump_speed;//过横断的速度
extern int DBBridge_speed;//过单边桥的速度
extern int DBBridge_stop_speed;//过单边桥减速的速度
extern int jump_ready;//是否开启横断
extern int ramp_ready;//是否开启坡道
extern int lz_no_dis;//路障不处理长度
/////////////////////过完各元素的速度///////////////////
extern int ramp_after_speed;//过完坡道的速度
extern int ring_after_speed;//过完圆环的速度
extern int jump_after_speed;//过完跳跃的速度
extern int DBBridge_after_speed;//过完单边桥的速度
extern int luzhang_after_speed;//过完小路障的速度
/////////////////////过完各元素的速度///////////////////
extern int dbbridge_stop_time;//过单边桥减速时间
extern int dbbridge_pass_time;
extern int dbbridge_stop_time3;//过3cm单边桥减速时间
extern int dbbridge_pass_time3;//最后通过3cm单边桥的时间
////////////////////左圆环参数/////////////////////////
extern int Lring_dis;//积分入环距离
extern int Lring_ang1;//积分入环角度
extern int Lring_ang2;//积分出环角度1
extern int Lring_ang3;//积分出环角度2
extern int Lring_dis2;//积分出环距离

////////////////////右圆环参数/////////////////////////
extern int Rring_dis;//积分入环距离
extern int Rring_ang1;//积分入环角度
extern int Rring_ang2;//积分出环角度1
extern int Rring_ang3;//积分出环角度2
extern int Rring_dis2;//积分出环距离

////////////////////横断参数//////////////////////////
extern int jump_linelength;
extern int vs_jump;//跳跃点

////////////////////单边桥参数////////////////////////


///////////////////左大圆环参数///////////////////////
extern int Lbig_ring_dis11;//积分入环距离
extern int Lbig_ring_ang11;//积分入环角度
extern int Lbig_ring_ang12;//积分出环角度1
extern int Lbig_ring_ang13;//积分出环角度2
extern int Lbig_ring_dis12;//积分出环距离
///////////////////左小圆环参数///////////////////////
extern int Lsmall_ring_dis11;//积分入环距离    1200
extern int Lsmall_ring_ang11;//积分入环角度   44
extern int Lsmall_ring_ang12;//积分出环角度1  100
extern int Lsmall_ring_ang13;//积分出环角度2  45
extern int Lsmall_ring_dis12;//积分出环距离   2000
extern int Lring_errmax ;//左圆环误差最大值
///////////////////右大圆环参数///////////////////////
extern int Rbig_ring_dis11;//积分入环距离
extern int Rbig_ring_ang11;//积分入环角度
extern int Rbig_ring_ang12;//积分出环角度1
extern int Rbig_ring_ang13;//积分出环角度2
extern int Rbig_ring_dis12;//积分出环距离
///////////////////右小圆环参数///////////////////////
extern int Rsmall_ring_dis11;//积分入环距离    1100
extern int Rsmall_ring_ang11;//积分入环角度   -30
extern int Rsmall_ring_ang12;//积分出环角度1  -75
extern int Rsmall_ring_ang13;//积分出环角度2  -40
extern int Rsmall_ring_dis12;//积分出环距离   2000
extern int Rring_errmax ;//右圆环误差最大值
///////////////////左大圆环参数///////////////////////

extern int Lbig_ring_dis21 ;//积分入环距离    1850
extern int Lbig_ring_ang21 ;//积分入环角度   44
extern int Lbig_ring_ang22 ;//积分出环角度1  130
extern int Lbig_ring_ang23 ;//积分出环角度2  45
extern int Lbig_ring_dis22 ;//积分出环距离   2000

///////////////////左小圆环参数///////////////////////

extern int Lsmall_ring_dis21 ;//积分入环距离    1200
extern int Lsmall_ring_ang21 ;//积分入环角度   44
extern int Lsmall_ring_ang22 ;//积分出环角度1  100
extern int Lsmall_ring_ang23 ;//积分出环角度2  45
extern int Lsmall_ring_dis22 ;//积分出环距离   2000

///////////////////右大圆环参数///////////////////////

extern int Rbig_ring_dis21 ;//积分入环距离
extern int Rbig_ring_ang21 ;//积分入环角度
extern int Rbig_ring_ang22 ;//积分出环角度1
extern int Rbig_ring_ang23 ;//积分出环角度2
extern int Rbig_ring_dis22 ;//积分出环距离

///////////////////右小圆环参数///////////////////////

extern int Rsmall_ring_dis21 ;//积分入环距离    1100
extern int Rsmall_ring_ang21 ;//积分入环角度   -30
extern int Rsmall_ring_ang22 ;//积分出环角度1  -75
extern int Rsmall_ring_ang23 ;//积分出环角度2  -40
extern int Rsmall_ring_dis22 ;//积分出环距离   2000
void Ring_data(void);
void speed_base_init(void);
#endif /* CODE_DATA_SHEET_H_ */
