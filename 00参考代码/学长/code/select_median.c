/*
 * select_median.c
 *
 *  Created on: 2025Äê2ÔÂ26ÈÕ
 *      Author: 15958
 */


#include "select_median.h"
#include "stdlib.h"
#include "zf_common_headfile.h"
//Í¼Ïñ´¦Àí½á¹¹ÌåÉùÃ÷
extern Dispose_Image DI;
//µç´Å±êÖ¾ÉùÃ÷
int poserror = 0;
uint16 COUNTTS = 0; //¼ÆÊýÆ÷
uint16 Goal_Speed = 60;
// uint16 Base_Speed=240;
uint16 Add_speed = 130;
extern float Distance;
extern uint8 Record_Dis;
extern uint8 have_ring;
extern uint8 street_len_45;
extern uint8 street_len_5;  //ÖÐ10ÏßµÄÖ±µÀ³¤¶È
extern uint8 street_len_75; //ÖÐ70ÏßµÄÖ±µÀ³¤¶È
/*extern uint8  L_Ring_delay_flag1;
extern uint8  L_Ring_delay_flag2;
extern uint8  L_Ring_delay_flag3;
extern uint8  L_Ring_delay_flag4;
extern uint8  L_Ring_delay_flag5;
extern uint8  L_Ring_delay_flag6;
extern uint8  R_Ring_delay_flag1;
extern uint8  R_Ring_delay_flag2;
extern uint8  R_Ring_delay_flag3;
extern uint8  R_Ring_delay_flag4;
extern uint8  R_Ring_delay_flag6;
extern uint8 ChaLu_delay_flag1;
extern uint8 ChaLu_delay_flag2;
extern uint8 ChaLu_delay_flag3;
extern uint8 Zebra_delay_flag2;*/
extern uint8 CHALU_TIME;

extern uint8 Statu;
extern uint8 ChaLu_delay_flag1;
extern float Rate_error;
extern LadderMovePoint L_Move; //×óÓÒÁ½µãÅÀÌÝËã·¨×óµã
extern LadderMovePoint R_Move; //×óÓÒÁ½µãÅÀÌÝËã·¨ÓÒµã
extern uint8 CHALU_count;
extern uint8 have_podao;

int poserror_array[4] = {0};
//ÔªËØ´¦Àíº¯Êý
uint8 SC = 0; //µ±Ç°ÔªËØ¸öÊý
uint8 Special_Count = 4;
// uint8 Special_type[12]={L_Big,R_Little,L_SANCHA,L_Little,RAMP,IN_PARK};//,IN_PARK};//ÔªËØÊý×é
uint16 t = 0;
// uint8 Special_type[12]={L_Big,R_Little,L_SANCHA,L_Little,RAMP,IN_PARK};//ÔªËØÊý×é
extern uint8 Ring_count;
extern uint8 Chalu_num;
extern uint8 Cross_num;
extern uint8 RuKu_Flag;
  uint8 GuoKu_Flag;
  uint8 RuKu_Finsh_Flag;
extern uint8 RuKu_Prepare_Flag;
extern uint8_t wireless_ch573_data[];
/************************************************************************
º¯ÊýÃû£ºÍ¼Ïñ´¦Àíº¯Êý
¹¦ÄÜ£ºÍ¼Ïñ´¦ÀíÁ÷³Ì
************************************************************************/
extern uint8 LRing_Out_Flag;
void DisposeImage(void)
{
  ConstructImage();    //Í¼Ïñ¹¹Ôì
  DetermineScanLine(); //È·¶¨É¨ÃèÐÐ
  //OutStop_Judge();
  Ring_data();
  Straight_Judge();
  if(ramp_ready==1)
  {
      ramp();
  }
   danbianqiao();
      DBBridge_try();
      luzhang_judge();
  Ring_Judge();
  if(jump_ready==1)
  {
      jump_judge();
  }

//    Zebra_Judge();
 //   cross( );
/*
  if ((LRing_Out_Flag == 1 && Chalu_num == 0) || (LRing_Out_Flag == 2 && Chalu_num == 1))
  {
    ChaLu_Judge();
    // ChaLu_Slope_Judge();
  }
*/

  if (Statu == LRing)
  {
      if(Ring_type[Ring_times]==0)
      {
        ringnotreatment();
      }
      else if(Ring_mode==1)
      {
        LRingTreatment();
      }
      else if(Ring_mode==2)
      {
        LRingTreatment2();
      }


  }
   if(Statu == RRing)
   {
       if(Ring_type[Ring_times]==0)
       {
         ringnotreatment();
       }
       else if(Ring_mode==1)
       {
          RRingTreatment();
       }
       else if(Ring_mode==2)
       {
          RRingTreament2();
       }

   }
//    RRingTreatment();



   if(Statu == JUMP)
   {
    jump_treatment();
    //jump_treatment2( );
   }
   if(Statu == Cross)
   {
     cross_treat( );
   }
   if(Statu == DBBrige)
   {
       if(DB_type[DB_times]==1)
       {
           DBBridgetreatment();
       }
       if(DB_type[DB_times]==2)
       {
           DBBridgetreatment3();
       }
       if(DB_type[DB_times]==0)
       {
             DBnotreatment();
       }
   }
/*
   if(Statu == Zebra)
   {
         zebrs_treatment( );
   }
*/
/*
  if (Statu == Lukou)
  {
    ChaLuTreatment();
  }
*/
/*
  if(Chalu_num == 2)
  {
      Record_Dis = 1;
      if(Distance >= 53)
      {
          Statu = Stops;
          RuKu_Prepare_Flag = 1;
      }

  }
*/
/*
  if ((Chalu_num == 1 && GuoKu_Flag == 0) )//|| (Chalu_num == 2)
  {
    Garage_Identification();
  }
  if (Chalu_num == 2 && RuKu_Prepare_Flag == 1)
  {
 //   A_Enter_Sign();
  }
  //B_Start_Sign();
//  ChuKu_Sign();
  if (RuKu_Finsh_Flag == 1)
  {
//    C_Stop_Sign();
  }
// A_GuoKu_Sign();
  if (GuoKu_Flag == 1)
  {
    Statu = Normals;
    GuoKu_Flag++;
  }
*/

  NormalControl();     //Õý³£¿ØÖÆ
}

//¿¨¶ûÂüÂË²¨
float kalmanFilter_A(float inData)
{
  static float prevData = 0;
  static float p = 10, q = 0.0001, r = 0.005, kGain = 0;
  // static float p=10, q=0.0001, r=0.005, kGain=0;
  p = p + q;
  kGain = p / (p + r);

  inData = prevData + kGain * (inData - prevData);
  p = (1 - kGain) * p;

  prevData = inData;
  return inData;
}

/*
 *³£¹æÆ«²î´¦Àí
 *1¡¢È·¶¨¶¯Ì¬ºáÏß×îÔ¶µã
 *2¡¢È·¶¨¶¯Ì¬ºáÏß
 *3¡¢È·¶¨ºáÏßÈ¨ÖØ
 *4¡¢Çó½âÆ«²î
 */

/*ÆðÊ¼¿ØÖÆ-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*/

/*¹²ÓÃ´¦Àí-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*/
/************************************************************************
º¯ÊýÃû£ºÕý³£¿ØÖÆ
************************************************************************/
void NormalControl(void)
{
//  DetermineWeight();
  SelectMid();    //ÌáÈ¡³õÊ¼ÖÐÖµ
  DetermineMid(); //È·¶¨×îºóÖÐÖµ
  xunxiandanbian();
  XUNXIAN_DBBridge();

}

/************************************************************************
º¯ÊýÃû£ºÈ·¶¨×îºóÖÐÖµ
¹¦ÄÜ£º¸ù¾ÝµÀÂ·×´¿öºÍÖÐÖµ±ä»¯È·¶¨ÓÐÐ§ÐÔ         //ÅÐ¶ÏÑô¹â½øÐÐ×ªµç´ÅÔÚÕâÀïÐ´     oo
²ÎÊý£ºÐÐÎ»ÖÃ
************************************************************************/
void DetermineMid(void)
{
  double i32_Mid = 0;
  double d_SumWeight = 0;
  for (uint8 ui8_I = 0; ui8_I < 10; ui8_I++)
  {
    DI.i16_FinallyMid[ui8_I] = (int)(DI.i16_Mid[ui8_I] + HtoE * (DI.i16_Mid[ui8_I] - MID_POINT)              //±ÈÖµ£¬ÓÃÒÔÅÐ¶ÏÍäµÀµÄ»º¼± HtoEÔÚÍ¬ÑùµÄÍäµÀ·´Ó¦²»Í¬µÄÆ«²î
                                                             * DI.ui8_ScanLineY[ui8_I] / DI.f_BaseY[ui8_I]); //µ±Ç°¶¯Ì¬ºáÏß/±ê×¼ºáÏß

    i32_Mid = i32_Mid + DI.i16_FinallyMid[ui8_I] * DI.f_BaseLineWeight[ui8_I];

    d_SumWeight = d_SumWeight + DI.f_BaseLineWeight[ui8_I];
  }

  i32_Mid = i32_Mid / d_SumWeight;

  //  if(IN_L==2){
  //      if (i32_Mid < DI.ui8_DisposeScopeLeft ) {
  //    i32_Mid = DI.ui8_DisposeScopeLeft ;
  //  }
  // }
  //  else if(IN_R==2){
  //    if(i32_Mid > DI.ui8_DisposeScopeRight){
  //    i32_Mid = DI.ui8_DisposeScopeRight;
  //    }
  //  }

  if (i32_Mid < DI.ui8_DisposeScopeLeft - 10)
  {
    i32_Mid = DI.ui8_DisposeScopeLeft - 10;
  }
  else if (i32_Mid > DI.ui8_DisposeScopeRight + 10)
  {
    i32_Mid = DI.ui8_DisposeScopeRight + 10;
  }

  else
  poserror = (int)(i32_Mid - MID_POINT);
    if(Ring_mode==1)
  {
/*
        if(L_Ring_flag2||L_Ring_delay_flag2){
            poserror = -20;
        }
*/
         if(L_Ring_flag4 || L_Ring_delay_flag4){
            poserror = -22;
        }

/*
        else if(R_Ring_flag2||R_Ring_delay_flag2){
            poserror = 20;
        }
*/

        else if(R_Ring_flag4 || R_Ring_delay_flag4){
            poserror = 22;
        }
  }
  if(Statu == Normals)
  {
      if(poserror>23)//21
          poserror=23;
      if(poserror<-23)
          poserror=-23;
  }
  if(Statu == DBBrige)
  {
      poserror =poserror;
  }
  if(jump_treatment2_flag2==1)
  {
      poserror = 10;
  }
  if(jump_treatment2_flag3==1)
  {
      poserror = -10;
  }
  for (int i = 0; i < 3; i++)
  {
    poserror_array[i + 1] = poserror_array[i];
  }
  poserror_array[0] = poserror;
}
/*¹²ÓÃ´¦Àí-*-*-*-*-*-*-*-*-*-*-*-*-
 * *-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*/

/*ÐÐ´¦Àí-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*/
/************************************************************************
º¯ÊýÃû£ºÈ·¶¨È¨ÖØ
¹¦ÄÜ£º¸ù¾ÝËÙ¶È¸Ä±äÈ¨ÖØ
************************************************************************/
  float V_Avg;

extern DianjiData Motor;
void DetermineWeight(void)
{

  double d_k_v = VtoLine * (motor_value.receive_left_speed_data-motor_value.receive_right_speed_data - 200) + 1; //±¾À´¼õÊýÓ¦¸ÃÊÇ³µ¼È²»ÇÐÄÚÈ¦Ò²²»ÇÐÍâÈ¦µÄÒ»¸ö»ù´¡³µËÙ
  if (d_k_v > 1)
  {
    double d_AddWeight = 1;
    for (uint8 ui8_I = 5; ui8_I < 10; ui8_I++)
    {
      d_AddWeight = d_AddWeight * d_k_v;
      DI.f_LineWeight[ui8_I] = DI.f_BaseLineWeight[ui8_I] + d_AddWeight - 1; //È·¶¨×ÜÈ¨ÖØ
    }
  }
  else
  {
    for (uint8 ui8_I = 0; ui8_I < 10; ui8_I++)
    {
      DI.f_LineWeight[ui8_I] = DI.f_BaseLineWeight[ui8_I];
    }
  }

  //   DI.f_LineWeight[4]
}

/************************************************************************
º¯ÊýÃû£ºÈ·¶¨É¨ÃèÐÐ
¹¦ÄÜ£º¸ù¾Ý¿ÉÊÓ¾àÀëÈ·¶¨É¨ÃèÐÐÒÔÈ·±£É¨ÃèÇøÓòÕýÈ·
************************************************************************/
double f_E_H = 0;
double d_Y = 0;
void DetermineScanLine(void)
{
  //ÏÈÈ·¶¨µÚ¾ÅÐÐ²ÎÊý
  if (Statu != LRing && Statu != RRing && Statu != Lukou)
  {
    if (59 - street_len_5 < Vistable_scale && street_len_5 > street_len_75)
      DI.ui8_ScanLineY[9] = 60 - street_len_5;
    else if (59 - street_len_75 < Vistable_scale && street_len_75 > street_len_5)
      DI.ui8_ScanLineY[9] = 60 - street_len_75;
    else
      DI.ui8_ScanLineY[9] = Vistable_scale + 1;
  }
  else
    DI.ui8_ScanLineY[9] = Vistable_scale + 1;
/*  if(DI.ui8_RPoint[DI.ui8_ScanLineY[9]]-DI.ui8_LPoint[DI.ui8_ScanLineY[9]]<35)
  {
      DI.ui8_ScanLineY[9]+=10;
  }*/
  DI.ui8_ScanLineL[9] = DI.ui8_LPoint[DI.ui8_ScanLineY[9]]; //ÓÉÔ¶¼°½ü
  DI.ui8_ScanLineR[9] = DI.ui8_RPoint[DI.ui8_ScanLineY[9]];

  DI.ui8_ScanLineToL[9] =
      (DI.ui8_ScanLineL[9] + DI.ui8_ScanLineR[9]) / 2;

  //Â·¿í¼ÆËã
  // DI.ui8_ScanLineWidth[9]= DI.ui8_ScanLineR[9]-DI.ui8_ScanLineL[9];

  DI.ui8_ScanLineToR[9] = DI.ui8_ScanLineToL[9];

  while (DI.ui8_ScanLineToL[9] > DI.ui8_DisposeScopeLeft //ÕÒ×î±ß½ç
         && (DI.ui8_ImageArray[DI.ui8_ScanLineY[9]]
                              [DI.ui8_ScanLineToL[9]--] ||
             DI.ui8_ImageArray[DI.ui8_ScanLineY[9]]
                              [DI.ui8_ScanLineToL[9]]))
  {
  }

  if (!DI.ui8_ImageArray[DI.ui8_ScanLineY[9]][DI.ui8_ScanLineToL[9]] && DI.ui8_ScanLineToL[9] < DI.ui8_DisposeScopeRight)
  {
    DI.ui8_ScanLineToL[9]++; //ÕÒµ½±ß½ç±ãÕÒÏÂÒ»ÐÐ
  }

  while (DI.ui8_ScanLineToR[9] < DI.ui8_DisposeScopeRight && (DI.ui8_ImageArray[DI.ui8_ScanLineY[9]]
                                                                               [DI.ui8_ScanLineToR[9]++] ||
                                                              DI.ui8_ImageArray[DI.ui8_ScanLineY[9]]
                                                                               [DI.ui8_ScanLineToR[9]]))
  {
  }

  if (!DI.ui8_ImageArray[DI.ui8_ScanLineY[9]][DI.ui8_ScanLineToR[9]] && DI.ui8_ScanLineToR[9] > DI.ui8_DisposeScopeLeft)
  {
    DI.ui8_ScanLineToR[9]--;
  }
  //  //ÔÙ´ÎÇóÈ¡Â·¿í
  //  //µÚÒ»ÖÖ2±ß¶¼µ½°×±ß
  //  if(DI.ui8_ScanLineToR[9]<DI.ui8_DisposeScopeRight -3 && DI.ui8_ScanLineToL[9]>DI.ui8_DisposeScopeLeft+3
  //     && (DI.ui8_ScanLineToR[9]-DI.ui8_RPoint[DI.ui8_ScanLineY[9]]>=3 ||  DI.ui8_LPoint[DI.ui8_ScanLineY[9]]-DI.ui8_ScanLineToL[9]>=3){
  //        DI.ui8_ScanLineWidth[9]=DI.ui8_ScanLineToR[9]-DI.ui8_ScanLineToL[9];
  //     }
  //  else if(DI.ui8_ScanLineToR[9]<DI.ui8_DisposeScopeRight-3 && DI.ui8_ScanLineToL[9]>DI.ui8_DisposeScopeLeft+3
  //          && (DI.ui8_ScanLineToR[9]-DI.ui8_RPoint[DI.ui8_ScanLineY[9]]<3 ||  DI.ui8_LPoint[DI.ui8_ScanLineY[9]]-DI.ui8_ScanLineToL[9]<3){
  //
  //
  //  if(DI.ui8_ScanLineToR[9]>=DI.ui8_DisposeScopeRight -3 && DI.ui8_ScanLineToL[9]<= DI.ui8_DisposeScopeLeft+3){
  // DI.[9]=(uint8)(DI.ui8_LineWidth[DI.ui8_ScanLineY[9]]*Rate_error);
  //}

  uint8 ui8_LineWidth = DI.ui8_DisposeScopeDown - DI.ui8_ScanLineY[9];
  f_E_H = ui8_LineWidth / 45.0;
  d_Y = DI.ui8_ScanLineY[9];

  for (int8 i8_I = 8; i8_I >= 0; i8_I--)
  {
    d_Y = d_Y + (9 - i8_I) * f_E_H; //Ê®µÈ·Ö Ã¿·Ö¿í¶ÈÎª1,2,3,4,5,6,7,8,9¹Ê·ÖÎª45Ð¡·Ö
    //²É¼¯É¨ÃèÐÐÎ»ÖÃ
    DI.ui8_ScanLineY[i8_I] = (int)(d_Y + 0.5);

    //²É¼¯É¨ÃèÐÐ×ó±ß½ç£¨²¹Ïß£©
    DI.ui8_ScanLineL[i8_I] = DI.ui8_LPoint[DI.ui8_ScanLineY[i8_I]];
    //²É¼¯É¨ÃèÐÐÓÒ±ß½ç£¨²¹Ïß£©
    DI.ui8_ScanLineR[i8_I] = DI.ui8_RPoint[DI.ui8_ScanLineY[i8_I]];
    //²É¼¯É¨ÃèÐÐ×î±ß½ç£¨ºÚ°×½»½ç´¦£©
    DI.ui8_ScanLineToL[i8_I] =
        DI.ui8_ScanLineL[i8_I] + DI.ui8_ScanLineR[i8_I] >> 1;
    DI.ui8_ScanLineToR[i8_I] = DI.ui8_ScanLineToL[i8_I];

    while (DI.ui8_ScanLineToL[i8_I] > DI.ui8_DisposeScopeLeft && (DI.ui8_ImageArray[DI.ui8_ScanLineY[i8_I]]
                                                                                   [DI.ui8_ScanLineToL[i8_I]--] ||
                                                                  DI.ui8_ImageArray[DI.ui8_ScanLineY[i8_I]]
                                                                                   [DI.ui8_ScanLineToL[i8_I]]))
    {
    }

    if (!DI.ui8_ImageArray[DI.ui8_ScanLineY[i8_I]][DI.ui8_ScanLineToL[i8_I]] && DI.ui8_ScanLineToL[i8_I] < DI.ui8_DisposeScopeRight)
    {
      DI.ui8_ScanLineToL[i8_I]++;
    }

    while (DI.ui8_ScanLineToR[i8_I] < DI.ui8_DisposeScopeRight && (DI.ui8_ImageArray[DI.ui8_ScanLineY[i8_I]]
                                                                                    [DI.ui8_ScanLineToR[i8_I]++] ||
                                                                   DI.ui8_ImageArray[DI.ui8_ScanLineY[i8_I]]
                                                                                    [DI.ui8_ScanLineToR[i8_I]]))
    {
    }

    if (!DI.ui8_ImageArray[DI.ui8_ScanLineY[i8_I]][DI.ui8_ScanLineToR[i8_I]] && DI.ui8_ScanLineToR[i8_I] > DI.ui8_DisposeScopeLeft)
    {
      DI.ui8_ScanLineToR[i8_I]--;
    }
  }
}


