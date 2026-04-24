/*
 * situation.h
 *
 *  Created on: 2025年2月26日
 *      Author: 15958
 */

#ifndef CODE_SITUATION_H_
#define CODE_SITUATION_H_


#include "zf_common_headfile.h"

/*提取初始中值*/
void SelectMid(void);

/*读取AD*/
void Get_AD();
/*获取最大值**/
void Get_MaxAD();
/*电磁标定值*/
void Get_TrueAD();
/*元素判定*/
void Judgu_Typeof_Road(void);
/*冲出赛道停车判定*/
void OutStop_Judge(void);
/*坡道判定*/
uint8 Ramp_Judge(void);
/*十字判定*/
void Cross_Judge(void);

/*斑马线判定*/
uint8 Zebra_Judge(void);
void Garage_Identification(void);
/*岔路判定*/
uint8 ChaLu_Judge(void);
void jump_judge(void);
/*横断路障判定*/
uint8 Ring_Judge();
/*状态处理*/
void Treatment_Status(void);
/*直到**/
void Straight_Judge(void);
/*十字处理*/
void CrossTreatment(void);
/*环岛处理*/
void LRingTreatment(void);
void RRingTreatment(void);
/*岔路处理*/
void ChaLuTreatment(void);
/*坡道处理*/
void RampsTreatment(void);
/*车库处理*/
void ZebraTreatment(void);

/*正常处理*/
void NormalTreatment(void);

/*计白点数*/
void Show_Wirte_Point(void);
/*状态处理********************************************************************/

//车库
#define L_go 0
#define R_go 1

extern uint16 Max_LAD;
extern uint16 Max_MAD;
extern uint16 Max_RAD;
extern int S_Left;
extern int S_Right;
extern float k_r;          //左线斜率
extern float k_l;          //右线斜率
extern float SumAngle;     //角度
extern uint8 Record_Angle; // 角度记录标志位
extern uint8 CHALU_TIME;
extern int jump_wait;


extern uint8 L_Ring_flag4 ;
extern uint8 L_Ring_delay_flag4 ;
extern uint8 R_Ring_flag4 ;
extern uint8 R_Ring_delay_flag4 ;
extern uint8 L_Ring_flag2 ;
extern uint8 L_Ring_delay_flag2 ;
extern uint8 R_Ring_flag2;
extern uint8 R_Ring_delay_flag2;
//----------------匹配滤波-三岔路口---------------
void Line_sum(uint8 x1, uint8 x2, uint8 y1, uint8 y2, float line_sum[]);
float conv(float model[], float test[]);
float ChaLu_Slope_Judge(void);
void cross(void);
uint8 ChaLu_Judge_New(void);
uint8 ChaLu_OutJudge(void);
extern uint8 line_length;
extern float left;
extern float right;
extern uint16 match_threshold;
extern uint8 Run;
extern uint16 Black_Num ;
extern uint8 ui8_ZebraTimes ;
extern int jump_treatment2_flag2;
extern int jump_treatment2_flag3;
void CrossTreatment(void);
void danbianqiao(void);
void cross_treat(void);
void DBBridge_try(void);
void DBBridgetreatment(void);
void DBBridgetreatment3(void);
void ramp(void);
void jump_treatment2(void);
void xunxiandanbian(void);
void LRingTreatment2(void);
void RRingTreament2(void);
void ringnotreatment(void);
void DBnotreatment(void);
void zebrs_treatment(void);
#endif /* CODE_SITUATION_H_ */
