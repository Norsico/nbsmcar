/*
 * speed_prossess.c
 *
 *  Created on: 2025年4月11日
 *      Author: 15958
 */
#include "speed_prossess.h"


float f_PulseCtrl = 0;
uint8 stop_flag = 0;
int speed_base;
int speed_extra;

int speed_goal;
void GetGoalPulse(void)
{
    speed_base = SPEED_BASE; //(int32)go;//135;//145 ;//150     90//基础速度72
    speed_extra = speed_extra1;  // 40; 34 28 26                      7
  uint8 Hit = 59 - Vistable_scale; //中40线的直道长度

  f_PulseCtrl = speed_extra *(Hit-52)/10; // 减数越小 越提前关闭加速算法
/*
  f_PulseCtrl = speed_extra *
                (0.00001466 * street_len_40 * street_len_40 * street_len_40 - 0.001047 * street_len_40 * street_len_40 + 0.10184 * street_len_40 - 4.4192);
*/

  // Motor.i32_GoalPulseL = Motor.i16_PulseBase;
   //Motor.i32_GoalPulseL = Motor.i16_PulseBase - f_PulseCtrl;
  if(f_PulseCtrl<-5)
      f_PulseCtrl=-5;
  speed_goal = speed_base + f_PulseCtrl;
/*
  if (Statu == Zebra && Zebra_Count == 2)
  {
    Motor.i32_GoalPulseL = 74;
  }
  if (Statu == Zebra && Zebra_Count == 1)
  {
    Motor.i32_GoalPulseL = 74;
  }

  if (Statu == LRing)
  {
    Motor.i32_GoalPulseL = 62; // 130;//135;//140 60
    //Motor.f_KDif = 0.45;
  }
  if (Statu == Lukou && CHALU_TIME == 1)
  {                            // && CHALU_TIME==2
    Motor.i32_GoalPulseL = 65; // 120;//130; 50 68
  }
  if (Statu == Lukou && CHALU_TIME == 2)
  {
    Motor.i32_GoalPulseL = 62; // 120;//130; 50
  }
*/

}


/*void Speed_Operation()
{
  Get_Speed();
  GetGoalPulse();

  if (Statu == Stops)
  {
    if (V_Avg > 8)
      Dianji_Out(-3000, -3000);
    else if (V_Avg < -8)
      Dianji_Out(3000, 3000);
    else
      Dianji_Out(0, 0);
  }
  else
  {
    Dianji_Out(Out_PWM_L, Out_PWM_R);
  }
}*/
