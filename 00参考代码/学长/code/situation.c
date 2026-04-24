/*
 * situation.c
 *
 *  Created on: 2025Äê2ÔÂ26ÈÕ
 *      Author: 15958
 */

#include "situation.h"
//±äÁ¿³õÊ¼»¯-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*/
//左右两点爬梯算法左点
extern LadderMovePoint L_Move;
//左右两点爬梯算法右点
extern LadderMovePoint R_Move;

//图像处理结构体声明
extern Dispose_Image DI;

uint8 Run;

extern   int poserror;
//变量初始化-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
//×Ü×´Ì¬±äÁ¿
uint8 Statu = 1;
//»·µº±äÁ¿
#define Big 0
#define Little 1
uint8 Open_Ring = 1;
uint16 Times = 0;
uint8 Time = 0;
uint8 L_Ring_flag0 = 0;
uint8 L_Ring_flag1 = 0;
uint8 L_Ring_delay_flag1 = 0;
uint8 L_Ring_flag2 = 0;
uint8 L_Ring_delay_flag2 = 0;
uint8 L_Ring_flag3 = 0;
uint8 L_Ring_delay_flag3 = 0;
uint8 L_Ring_flag4 = 0;
uint8 L_Ring_delay_flag4 = 0;
uint8 L_Ring_flag5 = 0;
uint8 L_Ring_delay_flag5 = 0;
uint8 L_Ring_flag6 = 0;
uint8 L_Ring_delay_flag6 = 0;
uint8 IN_LRing = 0;

uint8 R_Ring_flag0 = 0;
uint8 R_Ring_flag1 = 0;
uint8 R_Ring_delay_flag1 = 0;
uint8 R_Ring_flag2 = 0;
uint8 R_Ring_delay_flag2 = 0;
uint8 R_Ring_flag3 = 0;
uint8 R_Ring_delay_flag3 = 0;
uint8 R_Ring_flag4 = 0;
uint8 R_Ring_delay_flag4 = 0;
uint8 R_Ring_flag5 = 0;
uint8 R_Ring_delay_flag5 = 0;
uint8 R_Ring_flag6 = 0;
uint8 R_Ring_delay_flag6 = 0;
uint8 IN_RRing = 0;
uint8 Ring_flag = 0;
uint8 have_ring = 0;
//Ö±µÀ±äÁ¿
uint8 ui8_CacuLineToL[10] = {0};
uint8 ui8_CacuLineToR[10] = {0};
int S_Left = 0;  //×ó±ê×¼²î
int S_Right = 0; //ÓÒ±ê×¼²î
float k_r = 0;   //×óÏßÐ±ÂÊ
float k_l = 0;   //ÓÒÏßÐ±ÂÊ
//²íÂ·±äÁ¿
uint8 CHALU_count = 0; //  ²íÂ·´ÎÊý
uint8 ChaLu_flag0 = 0;
uint8 ChaLu_delay_flag0 = 0;
uint8 ChaLu_flag1 = 0;
uint8 ChaLu_delay_flag1 = 0;
uint8 ChaLu_flag2 = 0;
uint8 ChaLu_delay_flag2 = 0;
uint8 ChaLu_flag3 = 0;
uint8 ChaLu_delay_flag3 = 0;
//³µ¿â´¦Àí
uint8 Zebra_delay_flag1 = 0;
uint8 Zebra_flag1 = 0;
uint8 Zebra_delay_flag2 = 0;
uint8 Zebra_flag2 = 0;
uint8 Zerbra_Dir = 1; // Ä¬ÈÏ×ó
//ÆÂµÀ´¦Àí
uint8 Podao_flag1 = 0;
uint8 Podao_delay_flag1 = 0;
uint8 Podao_flag2 = 0;
uint8 Podao_delay_flag2 = 0;
uint8 Podao_flag3 = 0;
uint8 Podao_delay_flag3 = 0;
uint8 Podao_flag4 = 0;
uint8 Podao_delay_flag4 = 0;
uint8 have_podao = 0;
//Ê®×Ö±äÁ¿
uint8 Cross_flag0 = 0;
uint8 Cross_delay_flag0 = 0;
uint8 Cross_flag1 = 0;
uint8 Cross_delay_flag1 = 0;
uint8 Cross_flag2 = 0;
uint8 Cross_delay_flag2 = 0;

//°ßÂíÏß
uint8 Zebra_Count = 0;

//ÇøÓò¿í¶È
uint8 line_length = 32;

//Ð±ÈëÊ®×Ö
uint8 Xie_Cross_flag0 = 0;
/*
***********************************************************************
º¯ÊýÃû£ºÌáÈ¡³õÊ¼ÖÐÖµ
¹¦ÄÜ£ºÅÐ¶ÏµÀÂ·ÀàÐÍ·Ö±ð´¦ÀíµÃµ½³õÊ¼ÖÐÖµ
²ÎÊý£ºÐÐÎ»ÖÃ
***********************************************************************
***********************************************************************
************************************************************************/
void SelectMid(void)
{
  NormalTreatment();


}
/*
//uint8  MAX(int a, int b){
//  if(a>b) return a;
//  else return b;
//}
***µç´Å¶ÁÈ¡Êý¾Ý**
*/
uint16 Black_Num = 0;
uint16 Black_Num_Show = 0;


uint16 danbianqiao_flag_1;
uint16 danbianqiao_flag_2;
uint16 danbianqiao_flag_3;
uint16 danbianqiao_flag_4;
uint16 danbianqiao_flag_5;
uint16 dbq_count;

void ringnotreatment(void)
{
    if((L_Ring_flag1 || L_Ring_delay_flag1))
    {
        L_Ring_flag1=0;
        L_Ring_delay_flag1=1;
         BeeOn;
         Record_Dis=1;
         if(Distance>1500)
         {
             Distance=0;
             Record_Dis=0;
             Statu = Normals;
             L_Ring_flag1=0;
             L_Ring_delay_flag1=0;
             BeeOff;
             Ring_flag=0;
             have_ring=1;
             Ring_times++;
             SPEED_BASE=ring_after_speed;
         }
    }
    if((R_Ring_flag1 || R_Ring_delay_flag1))
    {
        R_Ring_flag1=0;
        R_Ring_delay_flag1=1;
         BeeOn;
         Record_Dis=1;
         if(Distance>1500)
         {
             Distance=0;
             Record_Dis=0;
             Statu = Normals;
             R_Ring_flag1=0;
             R_Ring_delay_flag1=0;
             BeeOff;
             Ring_flag=0;
             have_ring=1;
             Ring_times++;
             SPEED_BASE=ring_after_speed;
         }
    }

}
void DBnotreatment(void)
{
    if(danbianqiao_flag_1==1)
{
    BeeOn;
    Record_Dis=1;
    if(Distance>1500)
    {
        Distance=0;
        Record_Dis=0;
        danbianqiao_flag_1=0;
        BeeOff;
        Statu=Normals;
    }
}
}
uint16 ramp_flag_1;
void ramp(void)
{
    if(    // (DI.ui8_ScanLineToR[2]-DI.ui8_ScanLineToL[2])>37
           // &&(DI.ui8_ScanLineToR[2]-DI.ui8_ScanLineToL[2])<50
            (DI.ui8_ScanLineToR[6]-DI.ui8_ScanLineToL[6])>26
            &&(DI.ui8_ScanLineToR[6]-DI.ui8_ScanLineToL[6])<60
            &&(DI.ui8_ScanLineToR[7]-DI.ui8_ScanLineToL[7])>24
            &&(DI.ui8_ScanLineToR[7]-DI.ui8_ScanLineToL[7])<60
/*            &&(DI.ui8_ScanLineToR[3]-DI.ui8_ScanLineToL[3])>33
            &&(DI.ui8_ScanLineToR[3]-DI.ui8_ScanLineToL[3])<50*/
/*
            &&(DI.ui8_ScanLineToR[4]-DI.ui8_ScanLineToL[4])>32
            &&(DI.ui8_ScanLineToR[4]-DI.ui8_ScanLineToL[4])<50
*/
            &&(DI.ui8_ScanLineToR[5]-DI.ui8_ScanLineToL[8])>22
            &&(DI.ui8_ScanLineToR[5]-DI.ui8_ScanLineToL[8])<60
/*            &&(DI.ui8_ScanLineToR[5]-DI.ui8_ScanLineToL[5])>29
            &&(DI.ui8_ScanLineToR[5]-DI.ui8_ScanLineToL[5])<60*/
            &&DI.ui8_ScanLineToR[4]<75&&DI.ui8_ScanLineToL[4]>5
            &&DI.ui8_ScanLineToR[3]<75&&DI.ui8_ScanLineToL[3]>5
            &&DI.ui8_ScanLineToR[6]<70&&DI.ui8_ScanLineToL[6]>10
            &&DI.ui8_ScanLineToR[7]<70&&DI.ui8_ScanLineToL[7]>10
/*
            &&(DI.ui8_RPoint[52]-DI.ui8_RPoint[35])<7
            &&(DI.ui8_LPoint[30]-DI.ui8_LPoint[52])<7
*/
            &&(DI.ui8_ScanLineToR[0]-DI.ui8_ScanLineToR[2])<6
            &&(DI.ui8_ScanLineToR[2]-DI.ui8_ScanLineToR[4])<6
            &&(DI.ui8_ScanLineToR[4]-DI.ui8_ScanLineToR[7])<6
            &&(DI.ui8_ScanLineToR[7]-DI.ui8_ScanLineToR[9])<6
            &&(DI.ui8_ScanLineToL[9]-DI.ui8_ScanLineToL[7])<6
            &&(DI.ui8_ScanLineToL[7]-DI.ui8_ScanLineToL[4])<6
            &&(DI.ui8_ScanLineToL[4]-DI.ui8_ScanLineToL[2])<6
            &&(DI.ui8_ScanLineToL[2]-DI.ui8_ScanLineToL[0])<6
            &&street_len_40>50
            &&Vistable_scale<15
            &&poserror>=-5&&poserror<=5
            && Statu == Normals
            )
    {
        BeeOn;
        SPEED_BASE=ramp_speed;
        Statu == Ramps;
        Distance=0;
        Record_Dis=1;
        ramp_flag_1=1;

    }
    if(ramp_flag_1==1)
    {
       if( Distance>ramp_length[ramp_times])
       {
           Distance=0;
           Record_Dis=0;
           SPEED_BASE=ramp_after_speed;
           BeeOff;
           Statu = Normals;
           ramp_times++;
       }
    }
}
void cross(void)
{
    if(DI.ui8_ImageArray[20][60] && DI.ui8_ImageArray[20][20] && DI.ui8_ImageArray[20][65] && DI.ui8_ImageArray[20][15]&&
            DI.ui8_ImageArray[25][60] && DI.ui8_ImageArray[25][20] && DI.ui8_ImageArray[25][65] && DI.ui8_ImageArray[25][15]&&
            DI.ui8_ImageArray[35][70] && DI.ui8_ImageArray[35][10] && DI.ui8_ImageArray[35][10] && DI.ui8_ImageArray[35][10]&&
            DI.ui8_ImageArray[25][75] && DI.ui8_ImageArray[25][5] && DI.ui8_ImageArray[25][75] && DI.ui8_ImageArray[25][5]
          &&DI.ui8_ImageArray[30][75] && DI.ui8_ImageArray[30][75] && DI.ui8_ImageArray[30][5] && DI.ui8_ImageArray[15][5]
//           && !DI.ui8_ImageArray[50][70] &&  !DI.ui8_ImageArray[50][10] && ! DI.ui8_ImageArray[50][10] &&  !DI.ui8_ImageArray[50][10]&&
           &&! DI.ui8_ImageArray[55][79] &&  !DI.ui8_ImageArray[55][1] &&  !DI.ui8_ImageArray[55][79] &&  !DI.ui8_ImageArray[55][1]
//            && !DI.ui8_ImageArray[60][75] &&  !DI.ui8_ImageArray[60][75] &&  !DI.ui8_ImageArray[60][5] &&  !DI.ui8_ImageArray[60][5]
                               &&   Statu == Normals
            //poserror>=-5&&poserror<=5
      )
    {
        Cross_flag0=1;
        Statu = Cross;
    }
}
void cross_treat(void)
{
    int a;
    if(Cross_flag0==1)
    {
        BeeOn;
        a=turn_limit;
        turn_limit=25;
        Record_Dis=1;
        if(Distance>2000||Distance<-2000)
        {
/*
            if(DI.ui8_ImageArray[20][60] && DI.ui8_ImageArray[20][20] && DI.ui8_ImageArray[20][65] && DI.ui8_ImageArray[20][15]&&
                        DI.ui8_ImageArray[25][60] && DI.ui8_ImageArray[25][20] && DI.ui8_ImageArray[25][65] && DI.ui8_ImageArray[25][15]&&
                        DI.ui8_ImageArray[20][70] && DI.ui8_ImageArray[20][10] && DI.ui8_ImageArray[20][10] && DI.ui8_ImageArray[20][10]
                        //poserror>=-5&&poserror<=5
                  )
            {
*/
                turn_limit=a;
                Distance=0;
                Record_Dis=0;
                Statu = Normals;
                Cross_flag0=0;
                BeeOff;
            //}
        }
    }
}

void danbianqiao(void)
{
    if(danbianqiao_flag_1==0&&danbianqiao_flag_2==0&&danbianqiao_flag_3==0
/*
            !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 15][DI.ui8_DisposeScopeRight - 1] && !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 10][DI.ui8_DisposeScopeRight - 1]&&
            !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 40][DI.ui8_DisposeScopeRight - 1] && !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 45][DI.ui8_DisposeScopeRight - 1]&&
            !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 30][DI.ui8_DisposeScopeRight - 1] && !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 35][DI.ui8_DisposeScopeRight - 1]&&
            !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 25][DI.ui8_DisposeScopeRight - 1] && !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 19][DI.ui8_DisposeScopeRight - 1]&&
            !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 15][DI.ui8_DisposeScopeLeft + 1] && !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 10][DI.ui8_DisposeScopeLeft + 1]&&
            !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 40][DI.ui8_DisposeScopeLeft + 1] && !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 45][DI.ui8_DisposeScopeLeft + 1]&&
            !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 30][DI.ui8_DisposeScopeLeft + 1] && !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 35][DI.ui8_DisposeScopeLeft + 1]&&
            !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 25][DI.ui8_DisposeScopeLeft + 1] && !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 19][DI.ui8_DisposeScopeLeft + 1]
*/                &&DI.ui8_ScanLineToL[0]<70
                        &&DI.ui8_ScanLineToL[2]>10
                        &&DI.ui8_ScanLineToL[4]>10
                        &&DI.ui8_ScanLineToL[3]>10
                        &&DI.ui8_ScanLineToL[6]>10
                        &&DI.ui8_ScanLineToL[8]>10
                        &&DI.ui8_ScanLineToR[0]<70
                        &&DI.ui8_ScanLineToR[2]<70
                        &&DI.ui8_ScanLineToR[4]<70
                        &&DI.ui8_ScanLineToR[3]<70
                        &&DI.ui8_ScanLineToR[6]<70
                        &&DI.ui8_ScanLineToR[8]<70//               &&(DI.ui8_ScanLineToL[5] - DI.ui8_ScanLineToL[2])<3

                      && poserror>=-12&&poserror<=12

    )
    {
        if(k_r<6&&k_l<6&&k_r>0&&k_l>0&&
             street_len[46]<25
             &street_len[34]>30
             && Statu == Normals)
            {
                if(    // DI.ui8_ImageArray[60- street_len[49]-5][33]&&DI.ui8_ImageArray[60- street_len[49]-5][33]&&DI.ui8_ImageArray[60- street_len[49]-5][33]
                        !DI.ui8_ImageArray[60- street_len[46]-5][46]&&!DI.ui8_ImageArray[60- street_len[46]-5][45]&&!DI.ui8_ImageArray[60- street_len[46]-5][47]
                                 &&DI.ui8_ImageArray[60- street_len[46]-17][46]&&DI.ui8_ImageArray[60- street_len[46]-16][45]&&DI.ui8_ImageArray[60- street_len[46]-16][47]
                              // &&DI.ui8_ImageArray[60- street_len[49]-15][44]&&DI.ui8_ImageArray[60- street_len[49]-15][44]&&DI.ui8_ImageArray[60- street_len[49]-15][45]
                    )
                {
                    danbianqiao_flag_1=1;
                     Statu = DBBrige;
                }
            }
        if(k_r<6&&k_l<6&&k_r>0&&k_l>0&&
             street_len[46]<30
             &street_len[34]>35
             && Statu == Normals)
            {
                if(    // DI.ui8_ImageArray[60- street_len[49]-5][33]&&DI.ui8_ImageArray[60- street_len[49]-5][33]&&DI.ui8_ImageArray[60- street_len[49]-5][33]
                        !DI.ui8_ImageArray[60- street_len[46]-5][46]&&!DI.ui8_ImageArray[60- street_len[46]-5][47]&&!DI.ui8_ImageArray[60- street_len[46]-5][45]
                                 &&DI.ui8_ImageArray[60- street_len[46]-15][47]&&DI.ui8_ImageArray[60- street_len[46]-15][45]&&DI.ui8_ImageArray[60- street_len[46]-15][46]
                              // &&DI.ui8_ImageArray[60- street_len[49]-15][44]&&DI.ui8_ImageArray[60- street_len[49]-15][44]&&DI.ui8_ImageArray[60- street_len[49]-15][45]
                    )
                {
                    danbianqiao_flag_1=1;
                     Statu = DBBrige;
                }
            }
        if(k_r<6&&k_l<6&&k_r>0&&k_l>0&&
             street_len[48]<45
             &street_len[32]>50
             && Statu == Normals)
            {
                if(    // DI.ui8_ImageArray[60- street_len[49]-5][33]&&DI.ui8_ImageArray[60- street_len[49]-5][33]&&DI.ui8_ImageArray[60- street_len[49]-5][33]
                        !DI.ui8_ImageArray[60- street_len[48]-3][48]&&!DI.ui8_ImageArray[60- street_len[48]-3][47]&&!DI.ui8_ImageArray[60- street_len[48]-3][46]
                                 &&DI.ui8_ImageArray[60- street_len[48]-8][46]&&DI.ui8_ImageArray[60- street_len[48]-8][47]&&DI.ui8_ImageArray[60- street_len[48]-8][48]
                              // &&DI.ui8_ImageArray[60- street_len[49]-15][44]&&DI.ui8_ImageArray[60- street_len[49]-15][44]&&DI.ui8_ImageArray[60- street_len[49]-15][45]
                    )
                {
                    danbianqiao_flag_1=1;
                     Statu = DBBrige;
                }
            }
        if(
                k_r<6&&k_l<6&&k_r>0&&k_l>0&&
               //&& street_len[44]<47
               street_len[48]<40
               &&street_len[32]>45
               && Statu == Normals
           )
        {
            if(    // DI.ui8_ImageArray[60- street_len[49]-5][33]&&DI.ui8_ImageArray[60- street_len[49]-5][33]&&DI.ui8_ImageArray[60- street_len[49]-5][33]
                    !DI.ui8_ImageArray[60- street_len[48]-4][48]&&!DI.ui8_ImageArray[60- street_len[48]-5][47]&&!DI.ui8_ImageArray[60- street_len[48]-5][46]
                             &&DI.ui8_ImageArray[60- street_len[48]-12][48]&&DI.ui8_ImageArray[60- street_len[48]-12][47]&&DI.ui8_ImageArray[60- street_len[48]-12][46]
                          // &&DI.ui8_ImageArray[60- street_len[49]-15][44]&&DI.ui8_ImageArray[60- street_len[49]-15][44]&&DI.ui8_ImageArray[60- street_len[49]-15][45]
                )
            {
                danbianqiao_flag_1=1;
                 Statu = DBBrige;
            }

        }
        if(k_r<6&&k_l<6&&k_r>0&&k_l>0&&
           street_len[34]<25
           &&street_len[46]>30
           && Statu == Normals)
            {
                if(  // DI.ui8_ImageArray[60- street_len[49]-5][46]&&DI.ui8_ImageArray[60- street_len[49]-5][46]&&DI.ui8_ImageArray[60- street_len[49]-5][46]
                        !DI.ui8_ImageArray[60- street_len[34]-5][35]&&!DI.ui8_ImageArray[60- street_len[34]-5][33]&&!DI.ui8_ImageArray[60- street_len[34]-5][34]
                             &&DI.ui8_ImageArray[60- street_len[34]-16][35]&&DI.ui8_ImageArray[60- street_len[34]-16][34]&&DI.ui8_ImageArray[60- street_len[34]-16][33]
                          //  &&DI.ui8_ImageArray[60- street_len[31]-15][36]&&DI.ui8_ImageArray[60- street_len[31]-15][35]&&DI.ui8_ImageArray[60- street_len[31]-15][34]
                    )
                {
                        danbianqiao_flag_1=1;
                        Statu = DBBrige;
                }
            }
        if(k_r<6&&k_l<6&&k_r>0&&k_l>0&&
           street_len[34]<30
           &&street_len[46]>35
           && Statu == Normals)
            {
                if(  // DI.ui8_ImageArray[60- street_len[49]-5][46]&&DI.ui8_ImageArray[60- street_len[49]-5][46]&&DI.ui8_ImageArray[60- street_len[49]-5][46]
                        !DI.ui8_ImageArray[60- street_len[34]-5][33]&&!DI.ui8_ImageArray[60- street_len[34]-5][34]&&!DI.ui8_ImageArray[60- street_len[34]-5][35]
                             &&DI.ui8_ImageArray[60- street_len[34]-13][35]&&DI.ui8_ImageArray[60- street_len[34]-13][33]&&DI.ui8_ImageArray[60- street_len[34]-13][34]
                          //  &&DI.ui8_ImageArray[60- street_len[31]-15][36]&&DI.ui8_ImageArray[60- street_len[31]-15][35]&&DI.ui8_ImageArray[60- street_len[31]-15][34]
                    )
                {
                        danbianqiao_flag_1=1;
                        Statu = DBBrige;
                }
            }
        if(k_r<6&&k_l<6&&k_r>0&&k_l>0&&
           street_len[32]<40
           &&street_len[48]>45
           && Statu == Normals)
            {
                if(  // DI.ui8_ImageArray[60- street_len[49]-5][46]&&DI.ui8_ImageArray[60- street_len[49]-5][46]&&DI.ui8_ImageArray[60- street_len[49]-5][46]
                        !DI.ui8_ImageArray[60- street_len[32]-4][32]&&!DI.ui8_ImageArray[60- street_len[32]-4][33]&&!DI.ui8_ImageArray[60- street_len[32]-4][34]
                             &&DI.ui8_ImageArray[60- street_len[32]-12][32]&&DI.ui8_ImageArray[60- street_len[32]-12][33]&&DI.ui8_ImageArray[60- street_len[32]-12][34]
                          //  &&DI.ui8_ImageArray[60- street_len[31]-15][36]&&DI.ui8_ImageArray[60- street_len[31]-15][35]&&DI.ui8_ImageArray[60- street_len[31]-15][34]
                    )
                {
                        danbianqiao_flag_1=1;
                        Statu = DBBrige;
                }
            }
        if(
                k_r<6&&k_l<6&&k_r>0&&k_l>0&&
                //&& hangkuan60[14]<20&&hangkuan60[12]<20&&hangkuan60[13]<20
////                &&hangkuan60[14]<23&&hangkuan60[12]<23&&hangkuan60[13]<23
////                &&hangkuan60[15]<23&&hangkuan60[16]<23&&hangkuan60[17]<23
////                &&hangkuan60[9]<20&&hangkuan60[11]<20&&hangkuan60[10]<20
               // &&DI.ui8_RPoint[DI.ui8_ScanLineY[9]]-DI.ui8_LPoint[DI.ui8_ScanLineY[9]]>32
 //               &&hangkuan60[11]>3&&hangkuan60[10]>2&&hangkuan60[9]>2
 //          &&hangkuan60[14]>5&&hangkuan60[13]>5&&hangkuan60[12]>5
//           &&hangkuan60[17]>8&&hangkuan60[16]>8&&hangkuan60[15]>8
////           &&hangkuan60[45]>30&&hangkuan60[40]>30&&hangkuan60[50]>30
              // && street_len[36]<45
               street_len[32]<45
               &&street_len[48]>50
               //&&street_len[29]<45
 //              &&DI.ui8_ImageArray[8][43]&&DI.ui8_ImageArray[8][44]&&DI.ui8_ImageArray[8][45]
 //              &&DI.ui8_ImageArray[60- street_len[36]-15][36]&&DI.ui8_ImageArray[60- street_len[36]-15][35]&&DI.ui8_ImageArray[60- street_len[36]-15][34]
//                &&DI.ui8_ImageArray[11][38]&&DI.ui8_ImageArray[11][39]&&DI.ui8_ImageArray[11][34]
               //&&street_len[37]>50&&street_len[35]>50&&street_len[36]>50
               && Statu == Normals
           )
        {
            if(  // DI.ui8_ImageArray[60- street_len[49]-5][46]&&DI.ui8_ImageArray[60- street_len[49]-5][46]&&DI.ui8_ImageArray[60- street_len[49]-5][46]
                    !DI.ui8_ImageArray[60- street_len[32]-3][32]&&!DI.ui8_ImageArray[60- street_len[32]-3][33]&&!DI.ui8_ImageArray[60- street_len[32]-3][34]
                         &&DI.ui8_ImageArray[60- street_len[32]-8][32]&&DI.ui8_ImageArray[60- street_len[32]-8][33]&&DI.ui8_ImageArray[60- street_len[32]-8][34]
                      //  &&DI.ui8_ImageArray[60- street_len[31]-15][36]&&DI.ui8_ImageArray[60- street_len[31]-15][35]&&DI.ui8_ImageArray[60- street_len[31]-15][34]
                )
            {
                    danbianqiao_flag_1=1;
                    Statu = DBBrige;
            }

        }
    }




}
void DBBridge_try(void)
{
/*
    if(street_len[46]<50&&street_len[46]>40)
    {
         if
             (
                 poserror>=-11&&poserror<=11&&
                 Vistable_scale<15&&
                 (DI.ui8_RPoint[60-street_len[46]-2]-DI.ui8_RPoint[60-street_len[46]+2])<-10
                 &&(DI.ui8_RPoint[60-street_len[46]-15]-DI.ui8_RPoint[60-street_len[46]-5])>-4
                 &&(DI.ui8_RPoint[58]-DI.ui8_RPoint[47])<3
                 &&(DI.ui8_RPoint[52]-DI.ui8_RPoint[48])<3
                 &&(DI.ui8_LPoint[30]-DI.ui8_LPoint[58])<9 && Statu == Normals
             )
         {
             BeeOn;
             danbianqiao_flag_1=1;
              Statu = DBBrige;
         }
    }
*/
   if(street_len[46]<38&&street_len[46]>25)
   {
        if
            (
                poserror>=-4&&poserror<=4&&
                Vistable_scale<15&&
                (DI.ui8_RPoint[60-street_len[46]-2]-DI.ui8_RPoint[60-street_len[46]+2])<-12
                &&(DI.ui8_RPoint[60-street_len[46]-15]-DI.ui8_RPoint[60-street_len[46]-5])>-4
                &&(DI.ui8_RPoint[45]-DI.ui8_RPoint[36])<3
                &&(DI.ui8_RPoint[58]-DI.ui8_RPoint[47])<3
                &&(DI.ui8_RPoint[52]-DI.ui8_RPoint[48])<3
                &&(DI.ui8_LPoint[30]-DI.ui8_LPoint[58])<9
                &&(DI.ui8_LPoint[40]-DI.ui8_LPoint[50])<3
                &&(DI.ui8_LPoint[50]-DI.ui8_LPoint[58])<3
                &&DI.ui8_ScanLineToL[2]>10
                &&DI.ui8_ScanLineToL[4]>10
                &&DI.ui8_ScanLineToL[3]>10
                &&DI.ui8_ScanLineToL[6]>10
                &&DI.ui8_ScanLineToL[8]>10
                &&DI.ui8_ScanLineToR[0]<70
                &&DI.ui8_ScanLineToR[2]<70
                &&DI.ui8_ScanLineToR[4]<70
                &&DI.ui8_ScanLineToR[3]<70
                &&DI.ui8_ScanLineToR[6]<70
                &&DI.ui8_ScanLineToR[8]<70//               &&(DI.ui8_ScanLineToL[5] - DI.ui8_ScanLineToL[2])<3
                &&(DI.ui8_ScanLineToL[2] - DI.ui8_ScanLineToL[0])<5
                &&(DI.ui8_ScanLineToL[5] - DI.ui8_ScanLineToL[2])<5
                &&DI.ui8_ScanLineToR[1] - DI.ui8_ScanLineToR[5]<9
                && Statu == Normals
            )
        {
            BeeOn;
            danbianqiao_flag_1=1;
             Statu = DBBrige;
        }
   }
   if(street_len[46]<25&&street_len[46]>10)
   {
        if
            (
                poserror>=-8&&poserror<=8&&
                Vistable_scale<15&&
                (DI.ui8_RPoint[60-street_len[46]-2]-DI.ui8_RPoint[60-street_len[46]+2])<-14
                &&(DI.ui8_RPoint[60-street_len[46]-15]-DI.ui8_RPoint[60-street_len[46]-5])>-4
                &&(DI.ui8_RPoint[58]-DI.ui8_RPoint[47])<3
                &&(DI.ui8_RPoint[52]-DI.ui8_RPoint[48])<3
                &&(DI.ui8_LPoint[30]-DI.ui8_LPoint[58])<9
                &&(DI.ui8_LPoint[40]-DI.ui8_LPoint[50])<3
                &&(DI.ui8_LPoint[50]-DI.ui8_LPoint[58])<3
                &&DI.ui8_ScanLineToL[2]>10
                &&DI.ui8_ScanLineToL[4]>10
                &&DI.ui8_ScanLineToL[3]>10
                &&DI.ui8_ScanLineToL[6]>10
                &&DI.ui8_ScanLineToL[8]>10
                &&DI.ui8_ScanLineToR[0]<70
                &&DI.ui8_ScanLineToR[2]<70
                &&DI.ui8_ScanLineToR[4]<70
                &&DI.ui8_ScanLineToR[3]<70
                &&DI.ui8_ScanLineToR[6]<70
                &&DI.ui8_ScanLineToR[8]<70//               &&(DI.ui8_ScanLineToL[5] - DI.ui8_ScanLineToL[2])<3
                &&(DI.ui8_ScanLineToL[5] - DI.ui8_ScanLineToL[2])<5
                &&DI.ui8_ScanLineToR[1] - DI.ui8_ScanLineToR[5]<9
                && Statu == Normals
            )
        {
            BeeOn;
            danbianqiao_flag_1=1;
             Statu = DBBrige;
        }
   }
/*
   if(street_len[48]<42&&street_len[48]>38)
   {
        if
            (
                poserror>=-13&&poserror<=13&&
                (DI.ui8_RPoint[60-street_len[48]-5]-DI.ui8_RPoint[60-street_len[48]+5])<-11
                &&(DI.ui8_RPoint[60-street_len[48]-10]-DI.ui8_RPoint[60-street_len[48]-5])>-3
                &&(DI.ui8_RPoint[58]-DI.ui8_RPoint[47])<3
                &&(DI.ui8_RPoint[52]-DI.ui8_RPoint[48])<3
                &&(DI.ui8_LPoint[35]-DI.ui8_LPoint[58])<7 && Statu == Normals
            )
        {
            BeeOn;
            danbianqiao_flag_1=1;
             Statu = DBBrige;
        }
   }
*/

/*
   if(street_len[34]<50&&street_len[34]>40)
   {
        if
            (
                poserror>=-11&&poserror<=11&&
                Vistable_scale<15&&
                (DI.ui8_LPoint[60-street_len[34]+2]-DI.ui8_LPoint[60-street_len[34]-2])<-10
                &&(DI.ui8_LPoint[60-street_len[34]-5]-DI.ui8_LPoint[60-street_len[34]-10])>-3
                &&(DI.ui8_LPoint[47]-DI.ui8_LPoint[58])<3
                &&(DI.ui8_LPoint[48]-DI.ui8_LPoint[52])<3
                &&(DI.ui8_RPoint[56]-DI.ui8_RPoint[30])<9 && Statu == Normals
            )
        {
            BeeOn;
            danbianqiao_flag_1=1;
             Statu = DBBrige;
        }
   }
*/
   if(street_len[34]<38&&street_len[34]>25)
   {
        if
            (
                poserror>=-8&&poserror<=8&&
                Vistable_scale<15&&
                (DI.ui8_LPoint[60-street_len[34]+2]-DI.ui8_LPoint[60-street_len[34]-2])<-12
                &&(DI.ui8_LPoint[60-street_len[34]-5]-DI.ui8_LPoint[60-street_len[34]-10])>-3
                &&(DI.ui8_LPoint[36]-DI.ui8_LPoint[45])<3
                &&(DI.ui8_LPoint[47]-DI.ui8_LPoint[58])<3
                &&(DI.ui8_LPoint[48]-DI.ui8_LPoint[52])<3
                &&DI.ui8_ScanLineToL[2]>10
                &&DI.ui8_ScanLineToL[4]>10
                &&DI.ui8_ScanLineToL[3]>10
                &&DI.ui8_ScanLineToL[6]>10
                &&DI.ui8_ScanLineToL[8]>10
                &&DI.ui8_ScanLineToR[0]<70
                &&DI.ui8_ScanLineToR[2]<70
                &&DI.ui8_ScanLineToR[4]<70
                &&DI.ui8_ScanLineToR[3]<70
                &&DI.ui8_ScanLineToR[6]<70
                &&DI.ui8_ScanLineToR[8]<70//               &&(DI.ui8_ScanLineToL[5] - DI.ui8_ScanLineToL[2])<3
 //               &&(DI.ui8_ScanLineToL[5] - DI.ui8_ScanLineToL[2])<3
                &&DI.ui8_ScanLineToR[0] - DI.ui8_ScanLineToR[2]<5
                &&DI.ui8_ScanLineToR[5] - DI.ui8_ScanLineToR[2]<5
                &&(DI.ui8_RPoint[56]-DI.ui8_RPoint[30])<9 && Statu == Normals
            )
        {
            BeeOn;
            danbianqiao_flag_1=1;
             Statu = DBBrige;
        }
   }
   if(street_len[34]<25&&street_len[34]>10)
   {
        if
            (
                poserror>=-8&&poserror<=8&&
                Vistable_scale<15&&
                (DI.ui8_LPoint[60-street_len[34]+2]-DI.ui8_LPoint[60-street_len[34]-2])<-14
                &&(DI.ui8_LPoint[60-street_len[34]-5]-DI.ui8_LPoint[60-street_len[34]-10])>-3
                &&(DI.ui8_LPoint[47]-DI.ui8_LPoint[58])<3
                &&(DI.ui8_LPoint[48]-DI.ui8_LPoint[52])<3
                &&(DI.ui8_ScanLineToL[5] - DI.ui8_ScanLineToL[1])<9
                &&DI.ui8_ScanLineToL[2]>10
                &&DI.ui8_ScanLineToL[4]>10
                &&DI.ui8_ScanLineToL[3]>10
                &&DI.ui8_ScanLineToL[6]>10
                &&DI.ui8_ScanLineToL[8]>10
                &&DI.ui8_ScanLineToR[0]<70
                &&DI.ui8_ScanLineToR[2]<70
                &&DI.ui8_ScanLineToR[4]<70
                &&DI.ui8_ScanLineToR[3]<70
                &&DI.ui8_ScanLineToR[6]<70
                &&DI.ui8_ScanLineToR[8]<70//               &&(DI.ui8_ScanLineToL[5] - DI.ui8_ScanLineToL[2])<3
                &&DI.ui8_ScanLineToR[0] - DI.ui8_ScanLineToR[2]<5
                &&DI.ui8_ScanLineToR[5] - DI.ui8_ScanLineToR[2]<5
                &&(DI.ui8_RPoint[56]-DI.ui8_RPoint[30])<9 && Statu == Normals
            )
        {
            BeeOn;
            danbianqiao_flag_1=1;
             Statu = DBBrige;
        }
   }
/*
   if(street_len[34]<42&&street_len[34]>38)
   {
        if
            (
                poserror>=-13&&poserror<=13&&
                (DI.ui8_LPoint[60-street_len[34]+5]-DI.ui8_LPoint[60-street_len[34]-5])<-11
                &&(DI.ui8_LPoint[60-street_len[34]-5]-DI.ui8_LPoint[60-street_len[34]-10])>-3
                &&(DI.ui8_LPoint[47]-DI.ui8_LPoint[58])<3
                &&(DI.ui8_LPoint[48]-DI.ui8_LPoint[52])<3
                &&(DI.ui8_RPoint[56]-DI.ui8_RPoint[35])<7 && Statu == Normals
            )
        {
            BeeOn;
            danbianqiao_flag_1=1;
             Statu = DBBrige;
        }
   }
*/


}
void DBBridgetreatment(void)
  {
    if(danbianqiao_flag_1==1)
    {
        SPEED_BASE=DBBridge_stop_speed;
        fz_p_high=0.13;
        tracking_way=1;
        high_mode=1;
        high_out_max=53;
        s_p=0.08;
/*
         if(DI.ui8_ScanLineR[2]-DI.ui8_ScanLineL[2]<22&&DI.ui8_ScanLineR[3]-DI.ui8_ScanLineL[3]<22)
        {
             MID_POINT=32;
        }

         else
         {
             MID_POINT=40;
         }
*/
         BeeOn;
         dbq_count++;
         if(dbq_count>dbbridge_stop_time)
         {
             BeeOff;

             dbq_count=0;
             danbianqiao_flag_2=1;
             danbianqiao_flag_1=0;
//             s_p=0.0;
         }




    }

    if(danbianqiao_flag_2==1)
    {

          dbq_count++;
          SPEED_BASE=DBBridge_stop_speed;
          if(dbq_count>400)
          {
              tracking_way=0;
          }
          if(dbq_count>800)
          {

//              high_mode=1;
              high_out_max=70;
              fz_p_high=0.03;
              s_p=0.03;
              SPEED_BASE=DBBridge_speed;
              BeeOn;
              dbq_count=0;
                danbianqiao_flag_2=0;
                danbianqiao_flag_3=1;
          }


    }
    if(danbianqiao_flag_3==1)
    {
        dbq_count++;
        if(dbq_count>3000)
        {
/*            danbianqiao_flag_3=0;
            BeeOff;
            SPEED=270;*/

            SPEED_BASE=400;
                    BeeOff;
                    danbianqiao_flag_3=0;
                    danbianqiao_flag_4=1;
                    dbq_count=0;
/*            tracking_way=0;
            dbq_count=0;
            high_mode=0;*/
        }
    }
    if(danbianqiao_flag_4==1)
    {
            if(DI.ui8_ScanLineR[0]-DI.ui8_ScanLineL[0]>20&&DI.ui8_ScanLineR[1]-DI.ui8_ScanLineL[1]>20
                    &&DI.ui8_ScanLineR[6]-DI.ui8_ScanLineL[6]>13&&DI.ui8_ScanLineR[7]-DI.ui8_ScanLineL[7]>13)
                 {
                       dbq_count++;
                       SPEED_BASE=350;
                 }

            BeeOn;
            if(dbq_count>dbbridge_pass_time)
            {
                danbianqiao_flag_5=1;
                danbianqiao_flag_4=0;
                high_out_max=49;
                SPEED_BASE=DBBridge_after_speed;
                tracking_way=0;
                dbq_count=0;
            }
    }
    if(danbianqiao_flag_5==1)
    {
            BeeOn;
            dbq_count++;
            if(dbq_count>500)
            {
                danbianqiao_flag_5=0;
                BeeOff;
//                SPEED_BASE=510;
                dbq_count=0;
                high_mode=0;
                Statu=Normals;
                DB_times++;
            }
    }


  }
void DBBridgetreatment3(void)
  {
    if(danbianqiao_flag_1==1)
    {
        SPEED_BASE=DBBridge_stop_speed;
        fz_p_high=0.13;
        tracking_way=1;
        high_mode=1;
        high_out_max=60;
        s_p=0.08;
/*
         if(DI.ui8_ScanLineR[2]-DI.ui8_ScanLineL[2]<22&&DI.ui8_ScanLineR[3]-DI.ui8_ScanLineL[3]<22)
        {
             MID_POINT=32;
        }

         else
         {
             MID_POINT=40;
         }
*/
         BeeOn;
         dbq_count++;
         if(dbq_count>dbbridge_stop_time3)
         {
             BeeOff;

             dbq_count=0;
             danbianqiao_flag_2=1;
             danbianqiao_flag_1=0;
//             s_p=0.0;
         }




    }

    if(danbianqiao_flag_2==1)
    {
//        fz_p_high=0.0;
          dbq_count++;
          SPEED_BASE=DBBridge_stop_speed;
          if(dbq_count>100)
          {

          }
          if(dbq_count>400)
          {
              tracking_way=0;
//              high_mode=1;
              high_out_max=60;
              fz_p_high=0.05;
              s_p=0.0;
              SPEED_BASE=DBBridge_speed;
              BeeOn;
              dbq_count=0;
                danbianqiao_flag_2=0;
                danbianqiao_flag_3=1;
          }


    }
    if(danbianqiao_flag_3==1)
    {
        dbq_count++;
        if(dbq_count>500)
        {
/*            danbianqiao_flag_3=0;
            BeeOff;
            SPEED=270;*/
            SPEED_BASE=DBBridge_speed;
                    BeeOff;
                    danbianqiao_flag_3=0;
                    danbianqiao_flag_4=1;
                    dbq_count=0;
/*            tracking_way=0;
            dbq_count=0;
            high_mode=0;*/
        }
    }
    if(danbianqiao_flag_4==1)
    {
            if(DI.ui8_ScanLineR[0]-DI.ui8_ScanLineL[0]>20&&DI.ui8_ScanLineR[1]-DI.ui8_ScanLineL[1]>20
                    &&DI.ui8_ScanLineR[6]-DI.ui8_ScanLineL[6]>13&&DI.ui8_ScanLineR[7]-DI.ui8_ScanLineL[7]>13)
                 {
                       dbq_count++;
                       SPEED_BASE=350;
                 }

            BeeOn;
            if(dbq_count>dbbridge_pass_time3)
            {
                danbianqiao_flag_5=1;
                danbianqiao_flag_4=0;
                high_out_max=49;
                SPEED_BASE=DBBridge_after_speed;
                tracking_way=0;
                dbq_count=0;
            }
    }
    if(danbianqiao_flag_5==1)
    {
            BeeOn;
            dbq_count++;
            if(dbq_count>500)
            {
                danbianqiao_flag_5=0;
                BeeOff;
//                SPEED_BASE=510;
                dbq_count=0;
                high_mode=0;
                Statu=Normals;
                DB_times++;
            }
    }


  }


//ºá¶ÏÂ·ÕÏÅÐ¶¨
int jump_judge_flag1=0;
int jump_judge_flag2=0;
int jump_judge_flag3=0;
int jump_judge_flag4=0;
int jump_count;
int jump_wait=900;
int jump_linelength2=35;
void jump_judge(void)
    {

           if(street_len_40>5)
           {
                 if(street_len_40<jump_linelength&&S_Right<7&&S_Right>-7&&S_Left<7&&S_Left>-7
                         && street_len[51]<jump_linelength&&street_len[48]<jump_linelength&&street_len[44]<jump_linelength
                         &&street_len[36]<jump_linelength&&street_len[31]<jump_linelength&&street_len[29]<jump_linelength
                         //&&(street_len_47-street_len_33)<4&&(street_len_47-street_len_33)>-4
                         &&DI.ui8_ImageArray[12][40]&&DI.ui8_ImageArray[10][40]&&DI.ui8_ImageArray[8][40]
                         && Statu==Normals&&poserror<=3&&poserror>=-3
                        && (DI.ui8_RPoint[56]-DI.ui8_RPoint[40])<7&& (DI.ui8_LPoint[40]-DI.ui8_LPoint[56])<7
                                                                                                       )
                {
                     Statu=JUMP;
                    jump_judge_flag1=1;
                    BeeOn;
                    tracking_way=1;


/*
                    if(Vistable_scale>40)
                    {
                        jump_flag=1;
                    }
*/

                }

           }



    }

void jump_treatment(void)
{
    if(jump_judge_flag1==1)
    {
        SPEED_BASE=jump_speed;
/*
        jump_count++;
        if(jump_count>jump_wait)
        {
            Record_Dis=1;      //¼ÇÂ¼¾àÀë
            jump_flag=1;
            jump_count=0;
            jump_judge_flag2=1;
            jump_judge_flag1=0;
        }
*/
        if(Vistable_scale>vs_jump)
        {
            Record_Dis=1;      //¼ÇÂ¼¾àÀë
            jump_flag=1;
            jump_count=0;
            jump_judge_flag2=1;
            jump_judge_flag1=0;
        }

    }
    if(jump_judge_flag2==1)
    {
        if(Distance>200)
        {
            Distance=0;
            Record_Dis=0;      //¼ÇÂ¼¾àÀë
            jump_judge_flag2=0;
            jump_judge_flag3=1;

        }
    }
    if(jump_judge_flag3==1)
    {
        jump_count++;
        if(jump_count>900)
        {
            jump_judge_flag3=0;
            jump_judge_flag4=1;
            tracking_way=0;
            jump_count=0;
        }
    }
    if(jump_judge_flag4==1)
    {
        jump_count++;
        if(jump_count>1500)
        {
            Statu=Normals;
            jump_judge_flag4=0;
            jump_judge_flag1=0;
            BeeOff;
            SPEED_BASE=jump_after_speed;
        }
    }
}
int jump_treatment2_flag1=0;
int jump_treatment2_flag2=0;
int jump_treatment2_flag3=0;
int jump_treatment2_flag4=0;
void jump_treatment2(void)
{
    if(jump_judge_flag1==1&&jump_treatment2_flag1==0)
    {
        tracking_way=1;
        now_angle+=45;
        jump_treatment2_flag1=1;
    }
    if(jump_treatment2_flag1==1)
    {
        Record_Dis=1;      //¼ÇÂ¼¾àÀë
        if(Distance>1400)
        {
        jump_treatment2_flag2=1;
        jump_treatment2_flag1=0;
        Distance=0;
        }
    }
    if(jump_treatment2_flag2==1)
    {
        tracking_way=0;
        Record_Dis=1;      //¼ÇÂ¼¾àÀë
        if(Distance>900)
        {
            jump_treatment2_flag3=1;
            jump_treatment2_flag2=0;
            Distance=0;
 //           now_angle-=40;
        }
    }
    if(jump_treatment2_flag3==1)
    {
        Record_Dis=1;      //¼ÇÂ¼¾àÀë
        if(Distance>900)
        {
            jump_treatment2_flag4=1;
            jump_treatment2_flag3=0;
            Distance=0;
//            now_angle+=40;
        }
    }
    if(jump_treatment2_flag4==1)
    {
        Record_Dis=1;      //¼ÇÂ¼¾àÀë
        if(Distance>200)
        {
            jump_treatment2_flag4=0;
            jump_treatment2_flag3=0;
            Distance=0;
            Record_Dis=0;
            tracking_way=0;
        }
    }
}
int luzhang_right_flag_1=0;
int luzhang_left_flag_1=0;
int luzhang_stop_flag_1=0;
int luzhang_run_flag_1=0;
int luzhang_left_count;
int luzhang_right_count;
void luzhang_judge(void)
{
/*    if(DI.ui8_ScanLineToR[4]-DI.ui8_ScanLineToL[4]>25
       &&DI.ui8_ScanLineToR[5]-DI.ui8_ScanLineToL[5]>20
       &&(DI.ui8_ScanLineToR[0]-DI.ui8_ScanLineToL[0]<40||
       DI.ui8_ScanLineToR[1]-DI.ui8_ScanLineToL[1]<35))
    {
        if(((40-DI.ui8_ScanLineToL[0])>(DI.ui8_ScanLineToR[0]-40))||
             ((40-DI.ui8_ScanLineToL[1])>(DI.ui8_ScanLineToR[1]-40)))
             {
                    BeeOn;
             }

    }*/
/*    if(DI.ui8_ScanLineToR[4]-DI.ui8_ScanLineToL[4]>25
       &&DI.ui8_ScanLineToR[5]-DI.ui8_ScanLineToL[5]>20
       &&DI.ui8_ScanLineToR[2]-DI.ui8_ScanLineToL[2]<32
       &&Statu==Normals
       &&poserror<=20&&poserror>=-20)
    {
        if((40-DI.ui8_ScanLineToL[2])>(DI.ui8_ScanLineToR[2]-40))
             {
//                    BeeOn;
             }

    }*/
    if(DI.ui8_ScanLineToR[7]-DI.ui8_ScanLineToL[7]>20
       &&DI.ui8_ScanLineToR[6]-DI.ui8_ScanLineToL[6]>22
       &&DI.ui8_ScanLineToR[3]-DI.ui8_ScanLineToL[3]<25
//       &&DI.ui8_ScanLineToR[4]-DI.ui8_ScanLineToL[4]<27
       &&Statu==Normals
       && ((DI.ui8_RPoint[56]-DI.ui8_RPoint[35])<6||(DI.ui8_LPoint[35]-DI.ui8_LPoint[58])<6)
       &&poserror<=5&&poserror>=-5)
    {
/*        if((40-DI.ui8_ScanLineToL[3])<(DI.ui8_ScanLineToR[3]-40))
             {
                    BeeOn;
                    luzhang_left_flag_1=1;
                    Statu=Luzhang;
             }
        if((40-DI.ui8_ScanLineToL[3])>(DI.ui8_ScanLineToR[3]-40))
             {
                    BeeOn;
                    luzhang_right_flag_1=1;
                    Statu=Luzhang;
             }*/
        if(luzhang_types[luzhang_times]==1)
        {
            BeeOn;
            luzhang_left_flag_1=1;
            Statu=Luzhang;
        }
        if(luzhang_types[luzhang_times]==2)
        {
            BeeOn;
            luzhang_right_flag_1=1;
            Statu=Luzhang;
        }
        if(luzhang_types[luzhang_times]==3)
        {
            BeeOn;
            luzhang_stop_flag_1=1;
            Statu=Luzhang;
        }
        if(luzhang_types[luzhang_times]==0)
        {
            BeeOn;
            luzhang_run_flag_1=1;
            Statu=Luzhang;
        }
    }
    if(luzhang_left_flag_1==1)
    {
          MID_POINT=30;
          Record_Dis=1;      //¼ÇÂ¼¾àÀë
          if(Distance>300)
          {
              MID_POINT=35;
          }
          if(Distance>700)
          {
              MID_POINT=40;
          }
          if(Distance>750)
          {
              Distance=0;
              Record_Dis=0;      //¼ÇÂ¼¾àÀë
              luzhang_left_flag_1=0;
              BeeOff;
              Statu=Normals;
              SPEED_BASE=luzhang_after_speed;
              luzhang_times++;
          }
    }
        if(luzhang_right_flag_1==1)
        {
            MID_POINT=50;
            Record_Dis=1;      //¼ÇÂ¼¾àÀë
            if(Distance>300)
            {
                MID_POINT=45;
            }
            if(Distance>700)
            {
                MID_POINT=40;
            }
            if(Distance>750)
            {
                Distance=0;
                Record_Dis=0;      //¼ÇÂ¼¾àÀë
                luzhang_right_flag_1=0;
                BeeOff;
                Statu=Normals;
                SPEED_BASE=luzhang_after_speed;
                luzhang_times++;
            }
        }
        if(luzhang_stop_flag_1==1)
        {
            Record_Dis=1;      //¼ÇÂ¼¾àÀë
            if(Distance>200)
            {
                Distance=0;
                Record_Dis=0;      //¼ÇÂ¼¾àÀë
                luzhang_stop_flag_1=0;
                BeeOff;
                s_p=0.10;
//                Statu=Normals;
                SPEED_BASE=0;
//                luzhang_times++;
            }

        }
        if(luzhang_run_flag_1==1)
        {
            Record_Dis=1;      //¼ÇÂ¼¾àÀë
            if(Distance>lz_no_dis)
            {
                Distance=0;
                Record_Dis=0;      //¼ÇÂ¼¾àÀë
                luzhang_run_flag_1=0;
                BeeOff;
                Statu=Normals;
                luzhang_times++;
            }

        }

}
//³å³öÈüµÀÍ£³µÅÐ¶¨
void OutStop_Judge()
{
  uint8 White_Flag = 0;
  for (uint8 y = 59; y >= 55; y--)
  {
    for (uint8 x = 19; x <= 59; x++)
    {
      if (DI.ui8_ImageArray[y][x] == 0)
        Black_Num++;
      if (y == 59 || y == 55)
      {
        if (DI.ui8_ImageArray[58][x] == 1 && White_Flag == 0)
          White_Flag = 1;
      }
    }
  }
  for (uint8 y = 0; y <= 4; y++)
  {
    for (uint8 x = 19; x <= 59; x++)
    {
      if (DI.ui8_ImageArray[y][x] == 0)
        Black_Num++;
    }
  }
  if (Black_Num >= 408 && White_Flag == 0)
    Statu = Stops;
  Black_Num_Show = Black_Num;
  Black_Num = 0;
}
//Ö±µÀÅÐ¶¨
void Straight_Judge(void)
{
  S_Left = 0;
  S_Right = 0;
  k_l = (DI.ui8_ScanLineY[1] - DI.ui8_ScanLineY[5]) * 1.0 / (DI.ui8_ScanLineToL[5] - DI.ui8_ScanLineToL[1]);
  k_r = (DI.ui8_ScanLineY[1] - DI.ui8_ScanLineY[5]) * 1.0 / (DI.ui8_ScanLineToR[1] - DI.ui8_ScanLineToR[5]);
  for (uint8 ui8_I = 2; ui8_I <= 7; ui8_I++)
  {
    ui8_CacuLineToL[ui8_I] = (uint8)(DI.ui8_ScanLineToL[1] + (DI.ui8_ScanLineY[1] - DI.ui8_ScanLineY[ui8_I]) / k_l); //ÀíÂÛ¼ÆËãÖ±Ïß×ó±ß½ç
    ui8_CacuLineToR[ui8_I] = (uint8)(DI.ui8_ScanLineToR[1] - (DI.ui8_ScanLineY[1] - DI.ui8_ScanLineY[ui8_I]) / k_r);
    S_Left += (DI.ui8_ScanLineToL[ui8_I] - ui8_CacuLineToL[ui8_I]); //ÀíÂÛÓëÊµ¼ÊµÄÎó²îÀÛ¼Æ£¬ºÜ´óÔòÊÇÍäµÄ
    S_Right += (DI.ui8_ScanLineToR[ui8_I] - ui8_CacuLineToR[ui8_I]);
  }
}

/*
*Ô²»·ÅÐ¶Ï*
//  uint8 Ring_Judge (void)
//{
//    //×ó±ßÔ²»·ÅÐ¶¨
//  if(!DI.ui8_ImageArray[DI.ui8_DisposeScopeDown-5][DI.ui8_DisposeScopeLeft+1] && !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown-10][DI.ui8_DisposeScopeLeft+1]  //Ç°10ÐÐºÚÉ«
//      &&DI.ui8_ImageArray[DI.ui8_DisposeScopeDown-30][DI.ui8_DisposeScopeLeft+3] && DI.ui8_ImageArray[DI.ui8_DisposeScopeDown-32][DI.ui8_DisposeScopeLeft+3]  //ºóÃæÈ«ÊÇ°×É«
//       && DI.ui8_ImageArray[DI.ui8_DisposeScopeDown-34][DI.ui8_DisposeScopeLeft+3] && DI.ui8_ImageArray[DI.ui8_DisposeScopeDown-36][DI.ui8_DisposeScopeLeft+3]
//        && DI.ui8_ImageArray[DI.ui8_DisposeScopeDown-38][DI.ui8_DisposeScopeLeft+3] && DI.ui8_ImageArray[DI.ui8_DisposeScopeDown-40][DI.ui8_DisposeScopeLeft+3]
//         && !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown-35][DI.ui8_DisposeScopeRight-3] && !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown-40][DI.ui8_DisposeScopeRight-3]
//             && S_Right<=20 && S_Right>=0 &&street_len_40>=55 && Open_Ring)
//    {
//      Statu=LRing;
//      L_Ring_flag1=1;
//    }
//  //ÓÒÔ²»·ÅÐ¶¨
////    if(!DI.ui8_ImageArray[DI.ui8_DisposeScopeDown-5][DI.ui8_DisposeScopeRight-1] && !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown-10][DI.ui8_DisposeScopeRight-1]
////      &&DI.ui8_ImageArray[DI.ui8_DisposeScopeDown-30][DI.ui8_DisposeScopeRight-3] && DI.ui8_ImageArray[DI.ui8_DisposeScopeDown-32][DI.ui8_DisposeScopeRight-3]
////       && DI.ui8_ImageArray[DI.ui8_DisposeScopeDown-34][DI.ui8_DisposeScopeRight-3] && DI.ui8_ImageArray[DI.ui8_DisposeScopeDown-36][DI.ui8_DisposeScopeRight-3]
////        && DI.ui8_ImageArray[DI.ui8_DisposeScopeDown-38][DI.ui8_DisposeScopeRight-3] && DI.ui8_ImageArray[DI.ui8_DisposeScopeDown-40][DI.ui8_DisposeScopeRight-3]
////         && !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown-35][DI.ui8_DisposeScopeLeft+3] && !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown-40][DI.ui8_DisposeScopeLeft+3]
////             && S_Left>=-20 && S_Left<=5 &&street_len_40>=55 &&  Open_Ring)
////    {
////      Statu=RRing;
////      R_Ring_flag1=1;
////     // BeeOn;
////    }
//}
 * */
//Ô²»·ÅÐ¶¨
uint8 Ring_count = 0;
uint8 Ring_Judge(void)
{
/*
  if (
          DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 2][DI.ui8_DisposeScopeLeft] &&
          DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 4][DI.ui8_DisposeScopeLeft ]&&
          DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 6][DI.ui8_DisposeScopeLeft + 5] //&& DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 8][DI.ui8_DisposeScopeLeft ]//²»ÄÜÔÙ±È6¸ßÁË
                //&& DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 10][DI.ui8_DisposeScopeLeft ] && DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 12][DI.ui8_DisposeScopeLeft ]
             // &&DI.ui8_ImageArray[DI.ui8_DisposeScopeDown-20][DI.ui8_DisposeScopeLeft+5] && DI.ui8_ImageArray[DI.ui8_DisposeScopeDown-15][DI.ui8_DisposeScopeLeft+5]&&
             && DI.ui8_ImageArray[DI.ui8_DisposeScopeDown-48][DI.ui8_DisposeScopeLeft+5]
&&!DI.ui8_ImageArray[DI.ui8_DisposeScopeDown-30][DI.ui8_DisposeScopeLeft+10] //&& !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown-25][DI.ui8_DisposeScopeLeft+5]
             &&!DI.ui8_ImageArray[DI.ui8_DisposeScopeDown-35][DI.ui8_DisposeScopeLeft+10] && !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown-40][DI.ui8_DisposeScopeLeft+10]&&
       //&&!DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 44][DI.ui8_DisposeScopeRight - 1] && !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 42][DI.ui8_DisposeScopeRight - 1]&&
             !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 50][DI.ui8_DisposeScopeRight - 1]&&
             !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 15][DI.ui8_DisposeScopeRight - 1] && !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 10][DI.ui8_DisposeScopeRight - 1]&&
             !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 40][DI.ui8_DisposeScopeRight - 1] && !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 45][DI.ui8_DisposeScopeRight - 1]&&
             !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 30][DI.ui8_DisposeScopeRight - 1] && !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 35][DI.ui8_DisposeScopeRight - 1]&&
             !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 25][DI.ui8_DisposeScopeRight - 1] && !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 19][DI.ui8_DisposeScopeRight - 1]&&
       DI.ui8_ScanLineToR[4] < 78 && DI.ui8_ScanLineToR[5] < 78 && DI.ui8_ScanLineToR[6] < 78 && DI.ui8_ScanLineToR[3] < 78
            &&S_Right <= 6 && S_Right >= -6 && S_Left <= -20 //Õâ¼¸¸öÐ±ÂÊÅÐ¶Ï±È½ÏÖØÒª£¬×óÔ²»·µÄS_LeftµÄÈ·¶¨±È½ÏÖØÒª Ô­À´×ó<=-10
      && street_len[49] >= 45
      //&& Open_Ring
      && Statu == Normals
      )                                                                                                                                                                                                                                                                                                                                                                                                                                                          //Õâ¸öZebra_Count²»Ì«¶Ô°É  && Ring_count==Zebra_Count && street_len_5 >=30
  {
   BeeOn;
   Statu = LRing; // LRing Stops
    L_Ring_flag1 = 1;
    Ring_count++;
  }
  else if(
          //DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 2][DI.ui8_DisposeScopeRight] &&
            //       DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 4][DI.ui8_DisposeScopeRight ]&&
           //        DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 6][DI.ui8_DisposeScopeRight - 5]
        //   && DI.ui8_ImageArray[DI.ui8_DisposeScopeDown-48][DI.ui8_DisposeScopeRight-5]
          !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown-25][DI.ui8_DisposeScopeRight-5]
          &&!DI.ui8_ImageArray[DI.ui8_DisposeScopeDown-30][DI.ui8_DisposeScopeRight-5] //&& !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown-25][DI.ui8_DisposeScopeLeft+5]
          &&!DI.ui8_ImageArray[DI.ui8_DisposeScopeDown-35][DI.ui8_DisposeScopeRight-5] && !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown-40][DI.ui8_DisposeScopeRight-5]
         && DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 45][DI.ui8_DisposeScopeRight]
          &&!DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 50][DI.ui8_DisposeScopeLeft + 1]&&
          !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 15][DI.ui8_DisposeScopeLeft + 1] && !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 10][DI.ui8_DisposeScopeLeft + 1]&&
          !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 40][DI.ui8_DisposeScopeLeft + 1] && !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 45][DI.ui8_DisposeScopeLeft + 1]&&
          !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 30][DI.ui8_DisposeScopeLeft + 1] && !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 35][DI.ui8_DisposeScopeLeft + 1]&&
          !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 25][DI.ui8_DisposeScopeLeft + 1] && !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 19][DI.ui8_DisposeScopeLeft + 1]&&
    DI.ui8_ScanLineToL[4] < 78 && DI.ui8_ScanLineToL[5] < 78 && DI.ui8_ScanLineToL[6] < 78 && DI.ui8_ScanLineToL[3] < 78
    && S_Left<=6 && S_Left>=-6   && S_Right <= -10   //Õâ¼¸¸öÐ±ÂÊÅÐ¶Ï±È½ÏÖØÒª£¬ÓÒÔ²»·µÄS_RightµÄÈ·¶¨±È½ÏÖØÒª Ô­À´×ó<=-10 && S_Right>= -28
           DI.ui8_ImageArray[DI.ui8_DisposeScopeDown-2][DI.ui8_DisposeScopeRight-5] && DI.ui8_ImageArray[DI.ui8_DisposeScopeDown-4][DI.ui8_DisposeScopeRight-5]
         &&DI.ui8_ImageArray[DI.ui8_DisposeScopeDown-6][DI.ui8_DisposeScopeRight-5] && DI.ui8_ImageArray[DI.ui8_DisposeScopeDown-8][DI.ui8_DisposeScopeRight-5]
          && DI.ui8_ImageArray[DI.ui8_DisposeScopeDown-10][DI.ui8_DisposeScopeRight-5] && DI.ui8_ImageArray[DI.ui8_DisposeScopeDown-12][DI.ui8_DisposeScopeRight-5]
          && !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown-30][DI.ui8_DisposeScopeLeft+1] && !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown-25][DI.ui8_DisposeScopeLeft+1]
          && !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown-20][DI.ui8_DisposeScopeLeft+1] && !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown-15][DI.ui8_DisposeScopeLeft+1]
          && DI.ui8_ScanLineToL[4]<78  && DI.ui8_ScanLineToL[5]<78 && DI.ui8_ScanLineToL[6]<78 && DI.ui8_ScanLineToL[7]<78
           && S_Left<=20 && S_Left>=0   && S_Right <= -10   //Õâ¼¸¸öÐ±ÂÊÅÐ¶Ï±È½ÏÖØÒª£¬ÓÒÔ²»·µÄS_RightµÄÈ·¶¨±È½ÏÖØÒª Ô­À´×ó<=-10 && S_Right>= -28
           &&street_len_40>=55 && Open_Ring && Statu==Normals
            && Statu == Normals
           )
  {
      BeeOn;
      Statu = RRing; //RRing Stops
      R_Ring_flag1 = 1;
      Ring_count++;
  }
*/
/*    if (
            DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 40][DI.ui8_DisposeScopeLeft+5] &&
            DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 42][DI.ui8_DisposeScopeLeft+5 ]&&
            DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 45][DI.ui8_DisposeScopeLeft+5]&&
            !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 20][DI.ui8_DisposeScopeLeft+5] &&
            !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 23][DI.ui8_DisposeScopeLeft+5 ]&&
            !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 25][DI.ui8_DisposeScopeLeft+5] //&& +DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 8][DI.ui8_DisposeScopeLeft ]//²»ÄÜÔÙ±È6¸ßÁË
                  //&& DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 10][DI.ui8_DisposeScopeLeft ] && DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 12][DI.ui8_DisposeScopeLeft ]
               // &&DI.ui8_ImageArray[DI.ui8_DisposeScopeDown-20][DI.ui8_DisposeScopeLeft+5] && DI.ui8_ImageArray[DI.ui8_DisposeScopeDown-15][DI.ui8_DisposeScopeLeft+5]&&
               && !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown-32][DI.ui8_DisposeScopeLeft+5]
                &&!DI.ui8_ImageArray[DI.ui8_DisposeScopeDown-35][DI.ui8_DisposeScopeLeft+5] //&& !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown-25][DI.ui8_DisposeScopeLeft+5]
//               &&!DI.ui8_ImageArray[DI.ui8_DisposeScopeDown-35][DI.ui8_DisposeScopeLeft+10] && !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown-40][DI.ui8_DisposeScopeLeft+10]&&
         //&&!DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 44][DI.ui8_DisposeScopeRight - 1] && !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 42][DI.ui8_DisposeScopeRight - 1]&&
              &&DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 2][DI.ui8_DisposeScopeLeft+5] &&
              DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 4][DI.ui8_DisposeScopeLeft+5 ]
//             DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 8][DI.ui8_DisposeScopeLeft+19]
           &&!DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 50][DI.ui8_DisposeScopeRight - 1]&&
               !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 15][DI.ui8_DisposeScopeRight - 1] && !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 10][DI.ui8_DisposeScopeRight - 1]&&
               !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 40][DI.ui8_DisposeScopeRight - 1] && !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 45][DI.ui8_DisposeScopeRight - 1]&&
               !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 30][DI.ui8_DisposeScopeRight - 1] && !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 35][DI.ui8_DisposeScopeRight - 1]&&
               !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 25][DI.ui8_DisposeScopeRight - 1] && !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 19][DI.ui8_DisposeScopeRight - 1]
//         DI.ui8_ScanLineToR[4] < 78 && DI.ui8_ScanLineToR[5] < 78 && DI.ui8_ScanLineToR[6] < 78 && DI.ui8_ScanLineToR[3] < 78
//              &&S_Right <= 6 && S_Right >= -6 && S_Left <= -20 //Õâ¼¸¸öÐ±ÂÊÅÐ¶Ï±È½ÏÖØÒª£¬×óÔ²»·µÄS_LeftµÄÈ·¶¨±È½ÏÖØÒª Ô­À´×ó<=-10
        && street_len[44] >= 45&&poserror>=-5&&poserror<=5
        //&& Open_Ring
        && Statu == Normals
        )                                                                                                                                                                                                                                                                                                                                                                                                                                                          //Õâ¸öZebra_Count²»Ì«¶Ô°É  && Ring_count==Zebra_Count && street_len_5 >=30
    {
     BeeOn;
     Statu = LRing; // LRing Stops
      L_Ring_flag1 = 1;
      Ring_count++;
    }*/
    if (
            DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 17][DI.ui8_DisposeScopeLeft-3] &&
            DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 15][DI.ui8_DisposeScopeLeft-3]&&
            DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 8][DI.ui8_DisposeScopeLeft-3]&&
            DI.ui8_ImageArray[DI.ui8_DisposeScopeDown ][DI.ui8_DisposeScopeLeft-3]&&
            DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 20][DI.ui8_DisposeScopeLeft+15] &&
            DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 10][DI.ui8_DisposeScopeLeft+15 ]&&
            DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 15][DI.ui8_DisposeScopeLeft+15]&&
            DI.ui8_ImageArray[DI.ui8_DisposeScopeDown ][DI.ui8_DisposeScopeLeft+15]&&
              DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 20][DI.ui8_DisposeScopeLeft+10] &&
                        DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 15][DI.ui8_DisposeScopeLeft+10 ]&&
             DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 10][DI.ui8_DisposeScopeLeft+10]&&
              DI.ui8_ImageArray[DI.ui8_DisposeScopeDown ][DI.ui8_DisposeScopeLeft+10]&&
              (DI.ui8_RPoint[56]-DI.ui8_RPoint[35])<10&&
//              (DI.ui8_ScanLineToR[0]-DI.ui8_ScanLineToR[5])<8&&
//              !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 45][DI.ui8_DisposeScopeLeft+15]
//             !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 32][DI.ui8_DisposeScopeLeft+15 ]&&
//             !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 30][DI.ui8_DisposeScopeLeft+15]
 //           !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 15][DI.ui8_DisposeScopeLeft+10] &&
//            !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 5][DI.ui8_DisposeScopeLeft+10 ]&&
//            !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 3][DI.ui8_DisposeScopeLeft+5]&& //&& +DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 8][DI.ui8_DisposeScopeLeft ]//²»ÄÜÔÙ±È6¸ßÁË
              DI.ui8_ScanLineToR[4] < 70 && DI.ui8_ScanLineToR[5] < 70 && DI.ui8_ScanLineToR[0] < 75 && DI.ui8_ScanLineToR[3] < 70&& DI.ui8_ScanLineToR[1] < 75&& DI.ui8_ScanLineToR[2] < 75
              && DI.ui8_ScanLineToR[3] < 70
              &&!DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 3][DI.ui8_DisposeScopeRight - 5] && !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 5][DI.ui8_DisposeScopeRight - 8]
              &&!DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 10][DI.ui8_DisposeScopeRight - 10]
              /*
           &&!DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 35][DI.ui8_DisposeScopeRight - 1]&&
               !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 2][DI.ui8_DisposeScopeRight - 1] && !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 10][DI.ui8_DisposeScopeRight - 1]&&
               !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 25][DI.ui8_DisposeScopeRight - 1] && !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 45][DI.ui8_DisposeScopeRight - 1]&&
               !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 15][DI.ui8_DisposeScopeRight - 1] && !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 35][DI.ui8_DisposeScopeRight - 1]&&
               !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 5][DI.ui8_DisposeScopeRight - 1] && !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 19][DI.ui8_DisposeScopeRight - 1]
*/

//         DI.ui8_ScanLineToR[4] < 78 && DI.ui8_ScanLineToR[5] < 78 && DI.ui8_ScanLineToR[6] < 78 && DI.ui8_ScanLineToR[3] < 78
//              &&S_Right <= 6 && S_Right >= -6 && S_Left <= -20 //Õâ¼¸¸öÐ±ÂÊÅÐ¶Ï±È½ÏÖØÒª£¬×óÔ²»·µÄS_LeftµÄÈ·¶¨±È½ÏÖØÒª Ô­À´×ó<=-10
        &&street_len_47 >= 40
        &&poserror>=-10&&poserror<=10
        //&& Open_Ring
        && Statu == Normals
        )                                                                                                                                                                                                                                                                                                                                                                                                                                                          //Õâ¸öZebra_Count²»Ì«¶Ô°É  && Ring_count==Zebra_Count && street_len_5 >=30
    {
     BeeOn;
     SPEED_BASE=ring_speed;
     Statu = LRing; // LRing Stops
      L_Ring_flag1 = 1;
      Ring_count++;
    }

    if (
            DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 17][DI.ui8_DisposeScopeRight+3] &&
            DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 15][DI.ui8_DisposeScopeRight+3 ]&&
            DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 10][DI.ui8_DisposeScopeRight+3 ]&&
            DI.ui8_ImageArray[DI.ui8_DisposeScopeDown ][DI.ui8_DisposeScopeRight+3]&&
            DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 20][DI.ui8_DisposeScopeRight-15] &&
            DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 15][DI.ui8_DisposeScopeRight-15 ]&&
            DI.ui8_ImageArray[DI.ui8_DisposeScopeDown ][DI.ui8_DisposeScopeRight-15]&&
//           ! DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 45][DI.ui8_DisposeScopeRight-15]&&
            DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 20][DI.ui8_DisposeScopeRight-10] &&
            DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 15][DI.ui8_DisposeScopeRight-10 ]&&
            DI.ui8_ImageArray[DI.ui8_DisposeScopeDown ][DI.ui8_DisposeScopeRight-10]&&
            DI.ui8_ScanLineToL[4]>10  && DI.ui8_ScanLineToL[5]>10 && DI.ui8_ScanLineToL[3]>8 && DI.ui8_ScanLineToL[0]>5&& DI.ui8_ScanLineToL[1]>5
           && DI.ui8_ScanLineToL[2]>5&&
           !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 3][DI.ui8_DisposeScopeLeft + 5] && !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 8][DI.ui8_DisposeScopeLeft + 7]&&
           !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 10][DI.ui8_DisposeScopeLeft + 10]&&
           /*
               !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 15][DI.ui8_DisposeScopeLeft + 1] && !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 10][DI.ui8_DisposeScopeLeft + 1]&&
               !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 5][DI.ui8_DisposeScopeLeft + 1] && !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 45][DI.ui8_DisposeScopeLeft + 1]&&
               !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 10][DI.ui8_DisposeScopeLeft + 1] && !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 35][DI.ui8_DisposeScopeLeft + 1]&&
               !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 25][DI.ui8_DisposeScopeLeft + 1] && !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 19][DI.ui8_DisposeScopeLeft + 1]&&
*/
               (DI.ui8_LPoint[35]-DI.ui8_LPoint[58])<10
//           (DI.ui8_ScanLineToL[5]-DI.ui8_ScanLineToL[0])<9
//         DI.ui8_ScanLineToR[4] < 78 && DI.ui8_ScanLineToR[5] < 78 && DI.ui8_ScanLineToR[6] < 78 && DI.ui8_ScanLineToR[3] < 78
//              &&S_Right <= 6 && S_Right >= -6 && S_Left <= -20 //Õâ¼¸¸öÐ±ÂÊÅÐ¶Ï±È½ÏÖØÒª£¬×óÔ²»·µÄS_LeftµÄÈ·¶¨±È½ÏÖØÒª Ô­À´×ó<=-10
//        && street_len[36] >= 45&&street_len_33>= 40
        &&poserror>=-11&&poserror<=11
        //&& Open_Ring
        && Statu == Normals
        )                                                                                                                                                                                                                                                                                                                                                                                                                                                          //Õâ¸öZebra_Count²»Ì«¶Ô°É  && Ring_count==Zebra_Count && street_len_5 >=30
    {
     BeeOn;
     SPEED_BASE=ring_speed;
     Statu = RRing; // RRing Stops
      R_Ring_flag1 = 1;
      Ring_count++;
    }
}

// int k_chalu = 0;
// uint16 Write_Point = 0;
// uint16 Write_Point_Show;
// uint8 Left_Lost_Flag = 0;
// uint8 Right_Lost_Flag = 0;
// extern uint8 image;
////²íÂ·ÅÐ¶¨
// uint8 ChaLu_Judge(void)
//{
////  uint8 x1, x2, x3, x4;
////  uint8 y1, y2, y3, y4;
////  if (DI.ui8_ImageArray[Start_y1][0])
////  {
////    for (uint8 X = 0; X < DI.ui8_DisposeScopeRight - 2; X++)
////    {
////      if (!DI.ui8_ImageArray[Start_y1][X] && !DI.ui8_ImageArray[Start_y1][X + 1] && !DI.ui8_ImageArray[Start_y1][X + 2]) //ºÚÉ«
////      {
////        x3 = X;
////        y3 = Start_y1;
////        break;
////      }
////    }
////    for (uint8 X = x3; X < DI.ui8_DisposeScopeRight - 2; X++)
////    {
////      if (DI.ui8_ImageArray[Start_y1][X] && DI.ui8_ImageArray[Start_y1][X + 1] && DI.ui8_ImageArray[Start_y1][X + 2]) //°×É«
////      {
////        x4 = X;
////        y4 = Start_y1;
////        break;
////      }
////    }
////  }
////  for (uint8 Y = Start_y1 - 1; Y > DI.ui8_DisposeScopeUp + 2; Y--)
////  {
////    if (x3 - 10 > 0)
////    {
////      if (!DI.ui8_ImageArray[Y][x3 - 10] && !DI.ui8_ImageArray[Y - 1][x3 - 10] && !DI.ui8_ImageArray[Y - 2][x3 - 10])
////      {
////        x2 = x3 - 10;
////        y2 = Y;
////        break;
////      }
////    }
////    else
////    {
////      if (!DI.ui8_ImageArray[Y][0] && !DI.ui8_ImageArray[Y - 1][0] && !DI.ui8_ImageArray[Y - 2][0])
////      {
////        x2 = 0;
////        y2 = Y;
////        break;
////      }
////    }
////  }
////  for (uint8 Y = Start_y1 - 1; Y > DI.ui8_DisposeScopeUp + 2; Y--)
////  {
////    if (x4 + 10 < 79)
////    {
////      if (DI.ui8_ImageArray[Y][x4 + 10] && DI.ui8_ImageArray[Y - 1][x4 + 10] && DI.ui8_ImageArray[Y - 2][x4 + 10])
////      {
////        x1 = x4 + 10;
////        y1 = Y;
////        break;
////      }
////    }
////    else
////    {
////      if (DI.ui8_ImageArray[Y][79] && DI.ui8_ImageArray[Y - 1][79] && DI.ui8_ImageArray[Y - 2][79])
////      {
////        x1 = 79;
////        y1 = Y;
////        break;
////      }
////    }
////  }
////  k_chalu = (x3 - x2) * (x4 - x1) + (y3 - y2) * (y4 - y1);
//  //ÉÏÃæµÄ´úÂë¶¼ÊÇÎªÁËµÃµ½k_chalu
//  uint8 Start_y1 = 59 - street_len_40;
//  uint8 Start_x1 = 40;
//  static uint8 len_Lukou, len_Lukou_pre = 0;
//  uint8 X1, X2 = 0;
//  //Write_Point = 0;
//  if (street_len_40 <= 45 && Statu == Normals) //¿ÉÊÓ¾àÀë+×´Ì¬ÏÞÖÆ
//  {
//    for (uint8 Y = Start_y1 - 2; Y >= Start_y1 - 6; Y--) //ÉÏ5ÐÐ³¤¶ÈÔö¼Ó£¬³¤±ßÒÀ´ÎÔö³¤
//    {
//      for (uint8 X = Start_x1; X >= DI.ui8_DisposeScopeLeft + 10; X--)
//      {
//        if (DI.ui8_ImageArray[Y][X] && DI.ui8_ImageArray[Y][X - 1]) //ºÚÉ«ÇøÓò ºÚ°×Ìø±äµã
//        {
//          X1 = X;
//          break;
//        }
//        else
//        {
//          if (X == DI.ui8_DisposeScopeLeft + 3)
//          {
//              return 0;
//              //Left_Lost_Flag = 1;
//              //X1 = DI.ui8_DisposeScopeLeft + 3;
//          }
//        }
//      }
//      for (uint8 X = Start_x1; X <= DI.ui8_DisposeScopeRight - 10; X++) //°×É«ÇøÓò
//      {
//        if (DI.ui8_ImageArray[Y][X] && DI.ui8_ImageArray[Y][X + 1])
//        {
//          X2 = X;
//          break;
//        }
//        else
//        {
//          if (X == DI.ui8_DisposeScopeRight - 3)
//          {
//              return 0;
//              //Right_Lost_Flag = 1;
//              //X2 = DI.ui8_DisposeScopeRight - 3;
//          }
//        }
//      }
//          len_Lukou = X2 - X1;
//          if (len_Lukou < len_Lukou_pre)   //Ã»ÓÐÂú×ã³¤±ßÔö¼Ó
//          return 0;
//          len_Lukou_pre = len_Lukou;
//    }
//    //¼ì²âµãÏÂ15ÐÐ°×µãÊý
//    for (uint8 Y = Start_y1; Y <= Start_y1 + 15; Y++) //ÏÂ15ÐÐ
//    {
//      for (uint8 X = DI.ui8_DisposeScopeLeft; X <= DI.ui8_DisposeScopeRight; X = X + 2)
//      {
//        if (DI.ui8_ImageArray[Y][X] == 1)
//        {
//          Write_Point++;
//          Write_Point_Show = Write_Point;
//        }
//      }
//    }
//    //if (Write_Point < 100) //50
//    //  return 0;
//    Write_Point = 0;
//    //¼ì²âÏòÉÏÊÇºÚÉ«
//    for (uint8 Y = 57 - street_len_40; Y > DI.ui8_DisposeScopeUp; Y--)
//    {
//      if (DI.ui8_ImageArray[Y][40])
//      {
//        return 0;
//      }
//      else
//      {
//        if (Y == DI.ui8_DisposeScopeUp + 3)
//        {
//          // if(Zebra_Count == CHALU_count)
//          //{
//            if(Left_Lost_Flag == 0 && Right_Lost_Flag == 0)
//            {
//                CHALU_count++;
//                Statu = Lukou; // Lukou Stops
//                //ZW_Send_Image(image);
//                ChaLu_flag0 = 1;
//                return 1;
//                break;
//            }
//            else if(Right_Lost_Flag == 1)
//            {
//                if(DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 5][DI.ui8_DisposeScopeRight - 3]
//                   && DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 10][DI.ui8_DisposeScopeRight - 3]
//                   && DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 15][DI.ui8_DisposeScopeRight - 3])
//                {
//                    CHALU_count++;
//                    Statu = Lukou; // Lukou Stops
//                    //ZW_Send_Image(image);
//                    ChaLu_flag0 = 1;
//                    return 1;
//                    break;
//                }
//                else
//                    return 0;
//            }
//            else if(Left_Lost_Flag == 1)
//            {
//                if(DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 5][DI.ui8_DisposeScopeLeft + 3]
//                  && DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 10][DI.ui8_DisposeScopeLeft + 3]
//                  && DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 15][DI.ui8_DisposeScopeLeft + 3])
//                {
//                    CHALU_count++;
//                    Statu = Lukou; // Lukou Stops
//                    //ZW_Send_Image(image);
//                    ChaLu_flag0 = 1;
//                    return 1;
//                    break;
//                }
//                else
//                    break;
//            }
//          //                      else{
//          //                          Statu=Stops;
//          //                          Xie_Cross_flag0=1;  //Ð±×Å½øÈëÊ®×Ö
//          //                          Distance=0;
//          //                          Record_Dis=1;
//          //                      }
//        }
//      }
//    }
//  }
//}

uint16 Black_Point = 0;
uint16 Black_Point_Show;
//²íÂ·ÅÐ¶¨
//uint8 ChaLu_Judge(void)
//{
//  uint8 Start_y1 = 59 - street_len_40;
//  uint8 Start_x1 = 40;
//  static uint8 len_Lukou, len_Lukou_pre = 0;
//  uint8 X1, X2 = 0;
//  Write_Point = 0;
//  if (street_len_40 <= 45 && Statu == Stops)
//  { //¿ÉÊÓ¾àÀë+×´Ì¬ÏÞÖÆ
//    for (uint8 Y = Start_y1 - 3; Y >= Start_y1 - 7; Y--)
//    { //ÉÏ5ÐÐ³¤¶ÈÔö¼Ó
//      for (uint8 X = 0; X <= DI.ui8_DisposeScopeRight; X++)
//      {
//        if (!DI.ui8_ImageArray[Y][X] && !DI.ui8_ImageArray[Y][X + 1])
//        { //ºÚÉ«ÇøÓò
//          X1 = X;
//          break;
//        }
//        else
//        {
//          if (X == DI.ui8_DisposeScopeRight - 3)
//            return 0;
//        }
//      }
//      for (uint8 X = X1; X <= DI.ui8_DisposeScopeRight; X++)
//      { //°×É«ÇøÓò
//        if (DI.ui8_ImageArray[Y][X] && DI.ui8_ImageArray[Y][X + 1])
//        {
//          X2 = X;
//          break;
//        }
//        else
//        {
//          if (X == DI.ui8_DisposeScopeRight - 3)
//            return 0;
//        }
//      }
//      len_Lukou = X2 - X1;
//      if (len_Lukou < len_Lukou_pre)
//        return 0; //³¤±ßÒÀ´ÎÔö³¤
//      len_Lukou_pre = len_Lukou;
//    }
//    //¼ì²âµãÏÂ15ÐÐ°×µãÊý
//    for (uint8 Y = Start_y1; Y <= Start_y1 + 15; Y++)
//    { //ÏÂ15ÐÐ
//      for (uint8 X = 0; X <= DI.ui8_DisposeScopeRight; X = X + 2)
//      {
//        if (DI.ui8_ImageArray[Y][X])
//        {
//          Write_Point++;
//        }
//      }
//    }
//    Write_Point_Show = Write_Point;
//    //if (Write_Point > 50)
//    //  return 0;
//    //¼ì²âÏòÉÏÊÇºÚÉ«
//    for (uint8 Y = 57 - street_len_40; Y > DI.ui8_DisposeScopeUp; Y--)
//    {
//      if (DI.ui8_ImageArray[Y][40])
//      {
//        return 0;
//      }
//      else
//      {
//        if (Y == DI.ui8_DisposeScopeUp + 3)
//        {
//          CHALU_count++;
//          Statu = Lukou;
//          ChaLu_flag0 = 1;
//          BeeOn;
//          return 1;
//          break;
//        }
//      }
//    }
//  }
//}
uint8 ChaLu_Judge(void)
{
  uint8 Start_y1 = 59 - street_len_40;
  uint8 Start_x1 = 40;
  uint8 len_Lukou, len_Lukou_pre = 0;
  uint8 X1, X2 = 0;
  Black_Point = 0;
  if (street_len_40 <= 45 && Statu == Normals)
  { //¿ÉÊÓ¾àÀë+×´Ì¬ÏÞÖÆ
    for (uint8 Y = Start_y1 - 2; Y >= Start_y1 - 6; Y--)
    { //ÉÏ5ÐÐ³¤¶ÈÔö¼Ó
      for (uint8 X = 15; X >= DI.ui8_DisposeScopeRight; X++)
      {
        if (!DI.ui8_ImageArray[Y][X] && !DI.ui8_ImageArray[Y][X + 1])
        { //ºÚÉ«ÇøÓò
          X1 = X;
          break;
        }
        else
        {
          if (X == Start_x1 + 3)
          {
              return 0;
          }
        }
      }
      for (uint8 X = X1; X <= DI.ui8_DisposeScopeRight; X++)
      { //°×É«ÇøÓò
        if (DI.ui8_ImageArray[Y][X] && DI.ui8_ImageArray[Y][X + 1])
        {
          X2 = X;
          break;
        }
        else
        {
          if (X == DI.ui8_DisposeScopeRight - 3)
          {
              X2 = X;
              break;
              //return 0;
          }
        }
      }
      len_Lukou = X2 - X1;
      if (len_Lukou < len_Lukou_pre)
        return 0; //³¤±ßÒÀ´ÎÔö³¤
      len_Lukou_pre = len_Lukou;
    }
    //¼ì²âµãÏÂ15ÐÐºÚµãÊý
    for (uint8 Y = Start_y1; Y <= Start_y1 + 15; Y++)
    { //ÏÂ15ÐÐ
      for (uint8 X = 0; X <= DI.ui8_DisposeScopeRight; X = X + 2)
      {
        if (!DI.ui8_ImageArray[Y][X])
        {
            Black_Point++;
        }
      }
    }
    if (Black_Point > 80)//<290 || Black_Point > 430
      return 0;
    //¼ì²âÏòÉÏÊÇºÚÉ«
    for (uint8 Y = 56 - street_len_40; Y > DI.ui8_DisposeScopeUp; Y--)
    {
      if (DI.ui8_ImageArray[Y][40])
      {
        return 0;
      }
      else
      {
        if (Y == DI.ui8_DisposeScopeUp + 3)
        {
          CHALU_count++;
          Statu = Lukou;
          ChaLu_flag0 = 1;
          BeeOn;
          return 1;
          break;
        }
      }
    }
  }
}

void Show_Wirte_Point()
{
    uint8 Start_y1 = 59 - street_len_40;
    uint8 Start_x1 = 40;
    if (street_len_40 <= 45)
    { //¿ÉÊÓ¾àÀë+×´Ì¬ÏÞÖÆ
        //¼ì²âµãÏÂ15ÐÐ°×µãÊý
        for (uint8 Y = Start_y1; Y <= Start_y1 + 15; Y++)
        { //ÏÂ15ÐÐ
          for (uint8 X = 0; X <= DI.ui8_DisposeScopeRight; X = X + 2)
          {
            if (DI.ui8_ImageArray[Y][X])
            {
                Black_Point++;
            }
          }
        }
        Black_Point_Show = Black_Point;
//        ips114_show_uint16(170,0,Black_Point_Show);
    }
    Black_Point = 0;
    }
//*ÆÂµÀÅÐ¶¨*
uint8 Ramp_Judge()
{
  if (DI.ui8_ScanDirection == Mstart && street_len_40 >= 58 && (DI.ui8_ScanLineR[9] - DI.ui8_ScanLineL[9]) - DI.ui8_LineWidth[DI.ui8_ScanLineY[9]] >= 20 && (DI.ui8_ScanLineR[9] - DI.ui8_ScanLineL[9]) - DI.ui8_LineWidth[DI.ui8_ScanLineY[8]] >= 14 && (DI.ui8_ScanLineR[8] - DI.ui8_ScanLineL[8]) - DI.ui8_LineWidth[DI.ui8_ScanLineY[7]] >= 10 && (DI.ui8_ScanLineR[7] - DI.ui8_ScanLineL[7]) - DI.ui8_LineWidth[DI.ui8_ScanLineY[6]] >= 8
      // && (DI.ui8_ScanLineR[4]-DI.ui8_ScanLineL[4])-DI.ui8_LineWidth[DI.ui8_ScanLineY[4]]>=6
      && S_Left <= 10 && S_Left >= -10 && S_Right <= 10 && S_Right >= -10)
  {
    for (uint8 Y = DI.ui8_DisposeScopeDown - 10; Y > 1; Y--)
    {
      if (Y == 2 && !DI.ui8_ImageArray[Y][3] && !DI.ui8_ImageArray[Y][76])
      {
        BeeOn;
        Statu = Ramps;
        Podao_flag1 = 1;
        Record_Dis = 1;
        return 1;
      }
      else if (DI.ui8_ImageArray[Y][3] && DI.ui8_ImageArray[Y][76])
      {
        break;
      }
    }
  }
  return 0;
}
/*
*Ê®×ÖÅÐ¶¨*
void Cross_Judge(){//ÖÐ¼ä´óÁ¿¶ªÏß+×óÓÒÏß±ê×¼²îÏÞÖÆ
  if(DI.ui8_ImageArray[25][5] && DI.ui8_ImageArray[25][75] &&
      DI.ui8_ImageArray[27][5] && DI.ui8_ImageArray[27][75] &&
       DI.ui8_ImageArray[29][5] && DI.ui8_ImageArray[29][75] &&
        DI.ui8_ImageArray[31][5] && DI.ui8_ImageArray[31][75] &&
         DI.ui8_ImageArray[33][5] && DI.ui8_ImageArray[33][75] &&
           DI.ui8_ImageArray[35][5] && DI.ui8_ImageArray[35][75] &&
             Statu!=LRing &&  Statu!=RRing && S_Left<=-80 && S_Right>=80 ){
                  Statu=Cross;
                   Small=1;
                   Record_Dis=1;
                     BeeOn;
                    }
    if(  Distance>=50&&
         Statu!=LRing &&  Statu!=RRing){
            Small=0;
         Statu=Normals;
         BeeOff;
       }
}
     */
//°ßÂíÏßÅÐ¶¨
uint8 ui8_ZebraTimes = 0;
uint8 Zebra_Judge(void)
{
    if (
          //DI.ui16_counter[ZebraF] > 60 &&
    Statu == Normals)
  {
    //°ßÂíÏß¾àÀë´óÓÚ×î½üÁ½ÐÐ
//    if (DI.i8_StatusFlag[ZebraF] < DI.ui8_ScanLineY[0])
//    {
      //°ßÂíÏß´ÎÊý


      //²éÕÒ½ü¶Ë°ßÂíÏß
      for (uint8 ui8_I = 0; ui8_I < 4; ui8_I++)
      {
        //×î±ß½çÔÚÅÀ±ß½çÄÚ
//        if (DI.ui8_ScanLineL[ui8_I] < DI.ui8_ScanLineToL[ui8_I] || DI.ui8_ScanLineR[ui8_I] > DI.ui8_ScanLineToR[ui8_I])
//        {
          //ºÚÉ«¿í¶È
          uint8 ui8_BlackWidth = 0;
          //ÉÏÒ»´ÎÑÕÉ«
          uint8 ui8_LastColor =
              DI.ui8_ImageArray[DI.ui8_ScanLineY[ui8_I]][DI.ui8_ScanLineToL[ui8_I]];

          //×î×óÏò×ó
          for (uint8 ui8_X =65;
               ui8_X > 40; ui8_X--)//DI.ui8_ScanLineToL[ui8_I], DI.ui8_ScanLineL[ui8_I]
          {
            //°×É«µã
            if (DI.ui8_ImageArray[DI.ui8_ScanLineY[ui8_I]][ui8_X])
            {
              //ÊÇ·ñºÚ±ä°×
              if (ui8_LastColor == Black)
              {
                //ºÚÉ«¿í¶È´óÓÚ°ßÂíÏß¿í¶È
                if (ui8_BlackWidth >= ZebraWidth)
                {
                  ui8_ZebraTimes++;
                }
                ui8_BlackWidth = 0;
              }
              ui8_LastColor = White;
            }
            else
            {
              ui8_LastColor = Black;
              ui8_BlackWidth++;
            }
          }

          ui8_BlackWidth = 0;
          ui8_LastColor =
              DI.ui8_ImageArray[DI.ui8_ScanLineY[ui8_I]][DI.ui8_ScanLineToR[ui8_I]];

          //×îÓÒÏòÓÒ
          for (uint8 ui8_X = 15;
               ui8_X <40; ui8_X++)//DI.ui8_ScanLineToR[ui8_I] DI.ui8_ScanLineR[ui8_I]
          {
            //°×É«µã
            if (DI.ui8_ImageArray[DI.ui8_ScanLineY[ui8_I]][ui8_X])
            {
              //ÊÇ·ñºÚ±ä°×
              if (ui8_LastColor == Black)
              {
                //ºÚÉ«¿í¶È´óÓÚ°ßÂíÏß¿í¶È
                if (ui8_BlackWidth >= ZebraWidth)
                {
                  ui8_ZebraTimes++;
                }
                ui8_BlackWidth = 0;
              }
              ui8_LastColor = White;
            }
            else
            {
              ui8_LastColor = Black;
              ui8_BlackWidth++;
            }
          }
//        }

        //ÊÇ·ñ×ã¹»°ßÂíÏß
        if (ui8_ZebraTimes > ZebraTimes)
        {
          DI.i8_StatusFlag[ZebraF] = FlagZebra(DI.ui8_ScanLineY[ui8_I]);
          Statu = Zebra;
          Zebra_Count++;
          if (Zebra_Count > 2)
            Zebra_Count = 2;
          if (Zebra_Count == 2)
          {
            Zebra_flag1 = 1;
            BeeOn;
            Statu = Zebra;
          }
          //   BeeOn;
          return TRUE;
        }
        else
        {
          ui8_ZebraTimes = 0;
        }
      }
  }
}
void zebrs_treatment(void)
{
    if(Zebra_flag1 == 1)
    {
        Record_Dis=1;
        if(Distance>1500)
        {
            Statu = Normals;
            BeeOff;
            Zebra_flag1 =0;
            s_p=0.1;
            SPEED_BASE=0;

        }
    }
}

uint8 Zebra_num = 0;
uint8 Zebra_len = 0;
uint8 Zebra_Cnt = 0;
uint8 Car_Barn_num = 0;
static uint8 Ysite = 0, Xsite = 0;                //Ysite¾ÍÊÇÍ¼ÏñµÄÐÐ£¬Xsite¾ÍÊÇÍ¼ÏñµÄÁÐ¡£
uint8 Zebra_Flag = 0;
uint8 Y = 0;
uint8 RuKu_Prepare_Flag = 0;
extern uint8 image[60][80];
extern uint8 Chalu_num;
extern uint8 GuoKu_Flag;
void Garage_Identification()
{
    int Zebra_X1 = 0, Zebra_Y1 = 0, Zebra_X2 = 0, Zebra_Y2 = 0, wid = 0;
    int Ysite_Flag[15] = {0}, temp = 0;
    int dir = 0;
    /*25-15ÁÐ£¬10-70ÐÐËÑ°ßÂíÏß*/
    /*if(Car_Barn_num == 2)
        flag_stop_Car = 1;*/
    if(Zebra_Cnt == 0)
    {
        Zebra_num = 0;
        for(Ysite = 35; Ysite >= 25; Ysite--)
        {
            for(Xsite = 10; Xsite <= 70; Xsite++)
            {
                if(DI.ui8_ImageArray[Ysite][Xsite] == 1 && DI.ui8_ImageArray[Ysite][Xsite + 1] == 0)
                {
                    Zebra_num++;//ºÚ°×Ìø±äµã+1
                }
            }
        }
        if(Zebra_num >= 14)
        {
            Statu = Stops;  //ÔÚ55-59ÐÐÅÐ¶Ï³öÁË°ßÂíÏß

//                    if(Chalu_num == 1)
//                    {
//                        system_delay_ms(1000);
//                        GuoKu_Flag = 1;
//                    }
//            if(Chalu_num == 2)
//                RuKu_Prepare_Flag = 1;
            BeeOn;
//            Zebra_Cnt = 1;
//            Car_Barn_num++;
            //break;
        }
    }
    //°ßÂíÏß´¦ÓÚ×´Ì¬
//    if(Zebra_Cnt==1)
//    {
//        if(Zebra_Flag==0)//ÅÐ¶Ï50-55ÐÐÊÇ·ñ´æÔÚ°ßÂíÏß Èç¹û´æÔÚ¼ÇÂ¼´Ë¿Ì×´Ì¬
//        {
//            Zebra_len = 0;
//            Zebra_num = 0;
//            for(Ysite = 35; Ysite >= 30; Ysite--)
//            {
//                for(Xsite = 10; Xsite <= 70; Xsite++)
//                {
//                    if(DI.ui8_ImageArray[Ysite][Xsite] == 1 && DI.ui8_ImageArray[Ysite][Xsite + 1] == 0)
//                    {
//                        Zebra_num++;//ºÚ°×Ìø±äµã+1
//                    }
//                    if(Zebra_num >= 4)
//                    {
//                        Zebra_len++;//°ßÂíÏß³¤¶È+1
//                        Zebra_num = 0;
//                        break;//½øÐÐÏÂÒ»ÐÐÅÐ¶Ï
//                    }
//                }
//                if(Zebra_len >= 2)
//                {
//                    Zebra_Flag = 1;
//                    Statu = Stops;  //ÔÚ55-59ÐÐÅÐ¶Ï³öÁË°ßÂíÏß
//
////                    if(Chalu_num == 1)
////                    {
////                        system_delay_ms(1000);
////                        GuoKu_Flag = 1;
////                    }
//                    if(Chalu_num == 2)
//                        RuKu_Prepare_Flag = 1;
//                    BeeOn;
//                    Zebra_Cnt = 0;
//                    break;
//                }
//            }

//        }
//        if(Zebra_Flag == 1)//ÅÐ¶Ï55-59ÐÐ°ßÂíÏß²»´æÔÚ ±êÖ¾µ±Ç°ÒÑ¾­¹ýÈ¥³µ¿â
//        {
//            Zebra_len = 0;
//            Zebra_num = 0;
//            for(Ysite = 59; Ysite >= 55; Ysite--)
//            {
//                for(Xsite = 10; Xsite <= 70; Xsite++)
//                {
//                    if(DI.ui8_ImageArray[Ysite][Xsite] == 1 && DI.ui8_ImageArray[Ysite][Xsite + 1] == 0)
//                    {
//                        Zebra_num++;//ºÚ°×Ìø±äµã+1
//                    }
//                    if(Zebra_num >= 4)
//                    {
//                        Zebra_len++;//°ßÂíÏß³¤¶È+1
//                        Zebra_num = 0;
//                        break;//½øÐÐÏÂÒ»ÐÐÅÐ¶Ï
//                    }
//                }
//            }
//            if(Zebra_len == 0)  //´ËÊ±ÎÞºÚ°×Ìø±ä
//            {
//                Zebra_Flag = 2;
//            }
//        }
//        if(Zebra_Flag == 2)//±êÖ¾Î»ÇåÁã
//        {
//            Zebra_Flag=0;
//            Zebra_Cnt=0;
//            //flag_stop_Car = 1;
//        }
    }
    /*È·¶¨°ßÂíÏßyÖáÎ»ÖÃ*/
    /*¸¯Ê´°ßÂíÏß*/
//    if(Zebra_Cnt == 1)
//    {
//        for(Xsite = 35; Xsite <= 45; Xsite++)
//        {
//            for(Ysite = 59; Ysite >= 15; Ysite--)
//            {
//                if(DI.ui8_ImageArray[Ysite][Xsite] == 0 && DI.ui8_ImageArray[Ysite - 1][Xsite] == 0)
//                {
//                    Ysite_Flag[Xsite - 35] = Ysite;
//                    break;
//                }
//            }
//        }
//        temp = Ysite_Flag[0];
//        for(int i = 0; i <= 10; i++)
//        {
//            if(temp < Ysite_Flag[i + 1])
//            {
//                temp = Ysite_Flag[i + 1];//±£´æ×îÏÂ¶Ë°ßÂíÏß
//            }
//        }
//        Y = temp;
//        if(temp + 2 <= 59)
//        {
//            dir = temp + 2;
//        }
//        else
//        {
//            dir = 59;
//        }
//
//        for(Ysite = dir; Ysite >= dir - 20; Ysite--)
//        {
//            Zebra_X1 = 0;
//            Zebra_X2 = 0;
//            for(Xsite = 0; Xsite < 79; Xsite++)
//            {
//                if(DI.ui8_ImageArray[Ysite][Xsite - 1] == 1 && DI.ui8_ImageArray[Ysite][Xsite] == 0)
//                {
//                    Zebra_X1 = Xsite;
//                    while(Xsite++)
//                    {
//                        if(DI.ui8_ImageArray[Ysite][Xsite] == 0 && DI.ui8_ImageArray[Ysite][Xsite + 1] == 1)
//                        {
//                            Zebra_X2 = Xsite;
//                            wid = Zebra_X2 - Zebra_X1 + 1;
//                            if(wid <= 4)
//                            {
//                                //X1 = Zebra_X1;
//                                //X2 = Zebra_X2;
//                                for(int i = Zebra_X1; i <= Zebra_X2; i++)
//                                {
//                                    DI.ui8_ImageArray[Ysite][i] = 1;
//                                    image[Ysite][i] = 255;
//                                }
//                            }
//                            break;
//                        }
//                    }
//                }
//            }
//        }
//    }
    /****************/
//}
/*
*Â·¿öÅÐ¶¨*
void Judgu_Typeof_Road()
{
  Straight_Judge();   //Ö±µÀÅÐ¶¨,ÇóÈüµÀÐ±ÂÊ¡¢ÈüµÀºÍÖ±µÀµÄ±ê×¼²î
  if(Statu==Normals){
   Zebra_Judge();       //°ßÂíÏßÅÐ¶¨
   Cross_Judge();      //Ê®×ÖÅÐ¶¨
   Ring_Judge();       //Ô²»·ÅÐ¶Ï
   Ramp_Judge();       //ÆÂµÀÅÐ¶Ï
   //ChaLu_Judge();       //²íÂ·ÅÐ¶¨
   ChaLuJudge();
  }
}
*/

//---------------------------------Æ¥ÅäÂË²¨-²íÂ·ÅÐ¶¨Óë´¦Àí---------------------------------------------
//°´ÐÐÇóºÍº¯Êý

// x1<x2,y1<y2ÇóºÍÇøÓò 23 56 1 30 // 23 56 50 79
//Õâ¸öº¯ÊýÊÇÓÃÀ´ÕÒÒ»¸öÇøÓòÖÐÃ¿Ò»ÁÐºÚµãÕ¼µÄ±ÈÖØ
void Line_sum(uint8 x1, uint8 x2, uint8 y1, uint8 y2, float line_sum[])
{
  float sum_max = 1;
  float sum0 = 0;
  for (int8 i = x1; i < x2; i++)
  {
    float sum = 0;
    for (int8 j = y1; j < y2; j++)
    {
      sum = sum + (DI.ui8_ImageArray[j][i] + 1) % 2; //Í³¼ÆºÚµãÊý
    }
    if (i == x1)
      sum0 = sum; //µÚx1ÁÐµÄºÚµãÊý
    if (sum > sum_max)
      sum_max = sum;
    line_sum[i - x1] = (sum - sum0) / sum_max; //È¥³ýÖ±Á÷ ,¹éÒ»»¯ 34¸ö
  }
}
//¾í»ý
float conv(float model[], float test[])
{

  float conv_sum = 0;
  for (int i = 0; i < line_length; i++)
  {
    conv_sum = conv_sum + model[i] * test[line_length - i - 1];
  }
  return conv_sum;
}
float Left_Right_Sum = 0.0f;
uint8 T_Flag_With = 0;
uint8 T_Flag_High = 0;
float ChaLu_Slope_Judge()
{
  float model[32] = {0.0, 1.0, 1.0, 2.0, 3.0, 3.0, 4.0, 4.0, 5.0, 5.0, 6.0, 6.0, 7.0, 8.0, 8.0, 9.0, 10.0, 10.0, 11.0, 11.0, 12.0, 12.0, 13.0, 14.0, 14.0, 14.0, 13.0, 10.0, 8.0, 4.0, 1.0, 0.0};
  float left_sum[32] = {0};
  float right_sum[32] = {0};
  float left_A[3] = {0};
  float right_A[3] = {0};
  float left = 0;
  float right = 0;
  uint8 Start_Y = 59 - street_len_40;
  uint8 Start_X = 40;
  uint8 x1, x2, x3, y1, y2, y3;
  uint8 Dir_Flag = 0; // x3,y3µãµÄÕÒµã·½Ïò
  if (Start_Y >= 25)
  {
    x1 = Start_X;
    y1 = Start_Y;
    x2 = Start_X;
    y2 = Start_Y;
    while (1) //ÏÈÕÒx1ºÍy1µÄ×ø±êµã
    {
      if (y1 <= 0 || y1 >= 59)
      {
        break; //×ø±êµã´íÎó
      }
      else if (!DI.ui8_ImageArray[y1][x1 - 1])
      {
        x1--; //Ïò×óÊÇºÚÉ«£¬Íù×óÉÏÕÒ£¬×óÓÅÏÈ
      }
      else if (!DI.ui8_ImageArray[y1 - 1][x1 - 1])
      {
        x1--;
        y1--;
      }
      else if (!DI.ui8_ImageArray[y1 - 1][x1 - 2])
      {
        x1 -= 2;
        y1--;
      }
      else if (!DI.ui8_ImageArray[y1 - 1][x1 - 3])
      {
        x1 -= 3;
        y1--;
      }
      else
      {
        break;
      }
    }
    while (1) //ÕÒx2,y2,ÍùÓÒÉÏÕÒ,ÓÒÓÅÏÈ
    {
      if (y2 <= 0 || y2 >= 59)
      {
        break;
      }
      else if (!DI.ui8_ImageArray[y2][x2 + 1])
      {
        x2++;
      }
      else if (!DI.ui8_ImageArray[y2 - 1][x2 + 1])
      {
        x2++;
        y2--;
      }
      else if (!DI.ui8_ImageArray[y2 - 1][x2 + 2])
      {
        x2 += 2;
        y2--;
      }
      else if (!DI.ui8_ImageArray[y2 - 1][x2 + 3])
      {
        x2 += 3;
        y2--;
      }
      else
      {
        break;
      }
    }
    if ((x1 + x2) / 2 >= 40)
      Dir_Flag = 1; //´Ó×óÍùÓÒÕÒ
    else
      Dir_Flag = 2; //´ÓÓÒÍù×óÕÒ
    if (Dir_Flag == 1)
    {
      x3 = Start_X;
      y3 = Start_Y;
      while (1) //ÕÒx3,y3
      {
        if (y3 <= 0 || y3 >= 59)
        {
          break;
        }
        else if (!DI.ui8_ImageArray[y3 + 1][x3])
        {
          y3++;
        }
        else if (!DI.ui8_ImageArray[y3 + 1][x3 + 1])
        {
          x3++;
          y3++;
        }
        else if (!DI.ui8_ImageArray[y3 + 2][x3 + 1])
        {
          x3++;
          y3 += 2;
        }
        else if (!DI.ui8_ImageArray[y3 + 3][x3 + 1])
        {
          x3++;
          y3 += 3;
        }
        else
        {
          break;
        }
      }
    }
    else if (Dir_Flag == 2)
    {
      x3 = Start_X;
      y3 = Start_Y;
      while (1)
      {
        if (y3 <= 0 || y3 >= 59)
        {
          break;
        }
        else if (!DI.ui8_ImageArray[y3 + 1][x3])
        {
          y3++;
        }
        else if (!DI.ui8_ImageArray[y3 + 1][x3 - 1])
        {
          y3++;
          x3--;
        }
        else if (!DI.ui8_ImageArray[y3 + 2][x3 - 1])
        {
          x3--;
          y3 += 2;
        }
        else if (!DI.ui8_ImageArray[y3 + 3][x3 - 1])
        {
          x3--;
          y3 += 3;
        }
        else
        {
          break;
        }
      }
    }
  }
  //    Line_sum(x1, x2, 1, y2, left_sum);
  //    Line_sum(x1, x2, y1, y3, right_sum);
  //      for (int i = 2; i > 0; i--)
  //      {
  //        left_A[i] = left_A[i - 1];
  //        right_A[i] = right_A[i - 1];
  //      }
  //        left = conv(model, left_sum);
  //        right = conv(model, right_sum);
  //        left_A[0] = left;
  //        right_A[0] = right;
  //        Left_Right_Sum = left + right;
  T_Flag_With = x2 - x1;
  T_Flag_High = 59 - (y1 + y2) / 2;
  return Left_Right_Sum;
}
////²íÂ·ÅÐ¶¨
// extern uint8 SC;     //µ±Ç°ÔªËØ¸öÊý
// uint8 Chalu_num = 0; //Îª1,´ú±í½ø²íÂ·£¬Îª2´ú±í³ö²íÂ·
// uint8 Cross_num = 0;
// float Left_Right_Sum = 0.0f;
// float left = 0;
// float right = 0;
// uint16 match_threshold = 170;
// float left_A[3] = {0};
// float right_A[3] = {0};
// uint16 Write_Point_Show = 0;
// uint8 ChaLu_Judge_New(void)
//{
//   float model[32] = {0.0, 1.0, 1.0, 2.0, 3.0, 3.0, 4.0, 4.0, 5.0, 5.0, 6.0, 6.0, 7.0, 8.0, 8.0, 9.0, 10.0, 10.0, 11.0, 11.0, 12.0, 12.0, 13.0, 14.0, 14.0, 14.0, 13.0, 10.0, 8.0, 4.0, 1.0, 0.0};
//   float left_sum[32] = {0};
//   float right_sum[32] = {0};
//   Line_sum(23, 56, 1, 30, left_sum);
//   Line_sum(23, 56, 50, 79, right_sum);
//   for (int i = 2; i > 0; i--)
//   {
//     left_A[i] = left_A[i - 1];
//     right_A[i] = right_A[i - 1];
//   }
//   left = conv(model, left_sum);
//   right = conv(model, right_sum);
//   left_A[0] = left;
//   right_A[0] = right;
//   Left_Right_Sum = left + right;
//   if (left + right < -130)
//   { //²î²»¶à
//     uint8 Start_y = 59 - street_len_40;
//     for (uint8 Y = Start_y; Y <= Start_y + 15; Y++) //ÏÂ15ÐÐ
//     {
//       for (uint8 X = 0; X <= DI.ui8_DisposeScopeRight; X = X + 2)
//       {
//         if (DI.ui8_ImageArray[Y][X])
//         {
//           Write_Point++;
//           Write_Point_Show = Write_Point;
//         }
//       }
//     }
//     for (uint8 Y = 57 - street_len_40; Y > DI.ui8_DisposeScopeUp; Y--)
//     {
//       if (DI.ui8_ImageArray[Y][40])
//       {
//         break;
//       }
//       else
//       {
//         if (Y == DI.ui8_DisposeScopeUp + 2 && Write_Point >= 580
//                 && DI.ui8_ScanLineToR[5] > 25 && DI.ui8_ScanLineToR[6] > 25 && DI.ui8_ScanLineToR[7] > 25
//                 && Statu == Normals)
//         {
//           Statu = Lukou;
//           ChaLu_flag1 = 1;
//           Chalu_num++;
//           return 1;
//           BeeOn;
//         }
//       }
//     }
//     Write_Point = 0;
//   }
////  else if (left >= 0 && right > 30 && street_len_40 > 40 && street_len_40 < 50)
////  { //×ó±ß
////    uint8 Start_y = 59 - street_len_40;
////    for (uint8 Y = 57 - street_len_40; Y > DI.ui8_DisposeScopeUp; Y--)
////    {
////      if (DI.ui8_ImageArray[Y][40])
////      {
////        break;
////      }
////      else
////      {
////        if (Y == DI.ui8_DisposeScopeUp + 2)
////        {
////          Statu = Lukou;
////          ChaLu_flag1 = 1;
////          Chalu_num++;
////          return 1;
////          BeeOn;
////        }
////      }
////    }
////  }
////  else if (right >= 0 && left > 30 && street_len_40 > 40 && street_len_40 < 50)
////  { //ÓÒ±ß && street_len_40>40
////    uint8 Start_y = 59 - street_len_40;
////    for (uint8 Y = 57 - street_len_40; Y > DI.ui8_DisposeScopeUp; Y--)
////    {
////      if (DI.ui8_ImageArray[Y][40])
////      {
////        break;
////      }
////      else
////      {
////        if (Y == DI.ui8_DisposeScopeUp + 2)
////        {
////          Statu = Lukou;
////          ChaLu_flag1 = 1;
////          Chalu_num++;
////          return 1;
////          BeeOn;
////        }
////      }
////    }
////  }
////  else if (right <= -170 && left > 100 && street_len_40 > 50 && poserror > -9)
////  { //ÓÒ±ß && street_len_40>40
////    uint8 Start_y = 59 - street_len_40;
////    for (uint8 Y = 57 - street_len_40; Y > DI.ui8_DisposeScopeUp; Y--)
////    {
////      if (DI.ui8_ImageArray[Y][40])
////      {
////        break;
////      }
////      else
////      {
////        if (Y == DI.ui8_DisposeScopeUp + 2)
////        {
////          Statu = Lukou;
////          ChaLu_flag1 = 1;
////          Chalu_num++;
////          return 1;
////          BeeOn;
////        }
////      }
////    }
////  }
////  else if (DI.ui8_ImageArray[59][3] && DI.ui8_ImageArray[59][76] &&
////           DI.ui8_ImageArray[54][3] && DI.ui8_ImageArray[54][76] &&
////           DI.ui8_ImageArray[49][3] && DI.ui8_ImageArray[49][76] &&
////           DI.ui8_ImageArray[44][3] && DI.ui8_ImageArray[44][76] && poserror > -8)
////  { //&&  Special_type[SC]==Crosses
////    Statu = Cross;
////    Cross_flag0 = 1;
////    Cross_num++;
////    return 1;
////  }
////
////  else
////  {
////    return 0;
////  }
//  // else Statu = Normals;
//}
//
//
////uint8 ChaLu_Judge_New(void)
////{
////  uint8 Start_Y1 = 59 - street_len_40;
////  uint8 Start_X1 = 40;
////  uint8 x1,y1,x2,y2 = 0;
////
////
////}
//
//
//
////³öÈ¥£¡
//
// uint8 ChaLu_OutJudge(void)
//{
//  float model[32] = {0.0, 1.0, 1.0, 2.0, 3.0, 3.0, 4.0, 4.0, 5.0, 5.0, 6.0, 6.0, 7.0, 8.0, 8.0, 9.0, 10.0, 10.0, 11.0, 11.0, 12.0, 12.0, 13.0, 14.0, 14.0, 14.0, 13.0, 10.0, 8.0, 4.0, 1.0, 0.0};
//  float left_sum[32] = {0};
//  float right_sum[32] = {0};
//  Line_sum(23, 56, 1, 30, left_sum);
//  Line_sum(23, 56, 50, 79, right_sum);
//  for (int i = 2; i > 0; i--)
//  {
//    left_A[i] = left_A[i - 1];
//    right_A[i] = right_A[i - 1];
//  }
//  left = conv(model, left_sum);
//  right = conv(model, right_sum);
//  left_A[0] = left;
//  right_A[0] = right;
//  if (left + right > 200)
//  { //²î²»¶à
//    if (Special_type[SC] == L_SANCHA || Special_type[SC] == R_SANCHA)
//    {
//      return 1;
//    }
//  }
//  else if (left >= 0 && right > 30 && street_len_40 > 40 && street_len_40 < 50)
//  { //×ó±ß
//    if (Special_type[SC] == L_SANCHA || Special_type[SC] == R_SANCHA)
//    {
//      return 1;
//    }
//  }
//  else if (right >= 0 && left > 30 && street_len_40 > 40 && street_len_40 < 50)
//  { //ÓÒ±ß
//    if (Special_type[SC] == L_SANCHA || Special_type[SC] == R_SANCHA)
//    {
//
//      return 1;
//    }
//  }
//  else
//  {
//    return 0;
//  }
//}

void XUNXIAN_DBBridge(void)
{
    if(Statu==DBBrige)
    {
        if(danbianqiao_flag_1==1||danbianqiao_flag_2==1)
        {
            if(!DI.ui8_ImageArray[59][49]||!DI.ui8_ImageArray[59][46]||!DI.ui8_ImageArray[59][50]||!DI.ui8_ImageArray[57][49]
                     ||!DI.ui8_ImageArray[55][49]||!DI.ui8_ImageArray[53][49])
            {
                poserror = ((DI.ui8_ScanLineToL[0] +20-40)+(DI.ui8_ScanLineToL[1] +18-40)+(DI.ui8_ScanLineToL[2] +17-40))/3;
                if(poserror>27)
                    poserror=27;
                if(poserror<-27)
                     poserror=-27;
            }


            else
            {
                poserror = ((DI.ui8_ScanLineToR[0] -20-40)+(DI.ui8_ScanLineToR[2] -17-40)+(DI.ui8_ScanLineToR[1] -18-40))/3;
                if(poserror>27)
                    poserror=27;
                if(poserror<-27)
                     poserror=-27;
            }

        }
    }


}

void xunxiandanbian(void)
{
    if(Ring_mode==2)
    {
        if((L_Ring_flag2 || L_Ring_delay_flag2)
                ||(L_Ring_flag4 || L_Ring_delay_flag4)
         )
        {
    /*
            for (uint8 ui8_I = 2; ui8_I < 10; ui8_I++)
           {
               DI.i16_Mid[ui8_I] =0; //ÓÒ±ß¶ªÏß£¬ÄÇÖÐÏß¾ÍÊÇ×ó±ß¼Ó°ëÂ·¿í£¬Â·¿íÀ´×ÔÓÚ±ê×¼Êý×é

           }
    */
               // poserror = DI.ui8_ScanLineL[1] - (DI.ui8_LineWidth[DI.ui8_ScanLineY[1]] >> 1);
            //poserror = DI.ui8_ScanLineL[4] +10-40;
            poserror = ((DI.ui8_ScanLineToL[2] +10-40)+(DI.ui8_ScanLineToL[3] +10-40)+(DI.ui8_ScanLineToL[1] +10-40))/3;
            if(poserror>Lring_errmax)
                poserror=Lring_errmax;
            if(poserror<-Lring_errmax)
                 poserror=-Lring_errmax;
        }
        if((L_Ring_flag3 || L_Ring_delay_flag3)

         )
        {
    /*
            for (uint8 ui8_I = 2; ui8_I < 10; ui8_I++)
           {
               DI.i16_Mid[ui8_I] =0; //ÓÒ±ß¶ªÏß£¬ÄÇÖÐÏß¾ÍÊÇ×ó±ß¼Ó°ëÂ·¿í£¬Â·¿íÀ´×ÔÓÚ±ê×¼Êý×é

           }
    */
               // poserror = DI.ui8_ScanLineL[1] - (DI.ui8_LineWidth[DI.ui8_ScanLineY[1]] >> 1);
            //poserror = DI.ui8_ScanLineL[4] +10-40;
            poserror = ((DI.ui8_ScanLineToR[2] -30-40)+(DI.ui8_ScanLineToR[3] -30-40)+(DI.ui8_ScanLineToR[1] -30-40))/3;
            if(poserror>26)
                poserror=26;
            if(poserror<-26)
                 poserror=-26;
        }
        if((R_Ring_flag2 || R_Ring_delay_flag2)||(R_Ring_flag4 || R_Ring_delay_flag4))
        {
    /*
             for (uint8 ui8_I = 2; ui8_I < 10; ui8_I++)
            {
                DI.i16_Mid[ui8_I] = 0; //ÓÒ±ß¶ªÏß£¬ÄÇÖÐÏß¾ÍÊÇ×ó±ß¼Ó°ëÂ·¿í£¬Â·¿íÀ´×ÔÓÚ±ê×¼Êý×é

            }
    */
            // poserror = DI.ui8_ScanLineR[1] + (DI.ui8_LineWidth[DI.ui8_ScanLineY[1]] >> 1); //ÓÒ±ß¶ªÏß£¬ÄÇÖÐÏß¾ÍÊÇ×ó±ß¼Ó°ëÂ·¿í£¬Â·¿íÀ´×ÔÓÚ±ê×¼Êý×é
     //        poserror = DI.ui8_ScanLineR[4] -10-40;
             poserror = ((DI.ui8_ScanLineToR[2] -10-40)+(DI.ui8_ScanLineToR[2] -10-40)+(DI.ui8_ScanLineToR[2] -10-40))/3;
             if(poserror>Rring_errmax)
                 poserror=Rring_errmax;
             if(poserror<-Rring_errmax)
                  poserror=-Rring_errmax;
        }
        if((R_Ring_flag3 || R_Ring_delay_flag3))
        {
    /*
             for (uint8 ui8_I = 2; ui8_I < 10; ui8_I++)
            {
                DI.i16_Mid[ui8_I] = 0; //ÓÒ±ß¶ªÏß£¬ÄÇÖÐÏß¾ÍÊÇ×ó±ß¼Ó°ëÂ·¿í£¬Â·¿íÀ´×ÔÓÚ±ê×¼Êý×é

            }
    */
            // poserror = DI.ui8_ScanLineR[1] + (DI.ui8_LineWidth[DI.ui8_ScanLineY[1]] >> 1); //ÓÒ±ß¶ªÏß£¬ÄÇÖÐÏß¾ÍÊÇ×ó±ß¼Ó°ëÂ·¿í£¬Â·¿íÀ´×ÔÓÚ±ê×¼Êý×é
     //        poserror = DI.ui8_ScanLineR[4] -10-40;
             poserror = ((DI.ui8_ScanLineToL[2] +30-40)+(DI.ui8_ScanLineToL[2] +30-40)+(DI.ui8_ScanLineToL[2] +30-40))/3;
             if(poserror>26)
                 poserror=26;
             if(poserror<-26)
                  poserror=-26;
        }
    }
}

/***********************************************************************
º¯ÊýÃû£ºÕý³£´¦Àí
***********************************************************************/
void NormalTreatment(void)
{
  //Ö±µÀ
  for (uint8 ui8_I = 0; ui8_I < 10; ui8_I++)
  {
    //ÆÕÍ¨½×¶Î
    //  else{
    //    ×óÓÒÓÐ±ß½ç
    if (DI.ui8_ScanLineL[ui8_I] > DI.ui8_DisposeScopeLeft+10 && DI.ui8_ScanLineR[ui8_I] < DI.ui8_DisposeScopeRight-10)
    {
      DI.i16_Mid[ui8_I] = (DI.ui8_ScanLineL[ui8_I] + DI.ui8_ScanLineR[ui8_I]) >> 1;
      //×ó±ßÓÐ±ß½ç
    }
    else if (DI.ui8_ScanLineL[ui8_I] > DI.ui8_DisposeScopeLeft+10)
    {
      DI.i16_Mid[ui8_I] = DI.ui8_ScanLineL[ui8_I] + (DI.ui8_LineWidth[DI.ui8_ScanLineY[ui8_I]] >> 1); //ÓÒ±ß¶ªÏß£¬ÄÇÖÐÏß¾ÍÊÇ×ó±ß¼Ó°ëÂ·¿í£¬Â·¿íÀ´×ÔÓÚ±ê×¼Êý×é
      //ÓÒ±ßÓÐ±ß½ç
    }
    else if (DI.ui8_ScanLineR[ui8_I] < DI.ui8_DisposeScopeRight-10)
    {
      DI.i16_Mid[ui8_I] = DI.ui8_ScanLineR[ui8_I] - (DI.ui8_LineWidth[DI.ui8_ScanLineY[ui8_I]] >> 1);
    } //×ó±ß¶ªÏß£¬ÖÐÏßÊÇÓÒ±ß¼õÈ¥°ëÂ·¿í
  }
  //  }
}

//*³µ¿â´¦Àí
void ZebraTreatment()
{
  if (Zebra_Count == 1)
  {
    BeeOn;
    Record_Dis = 1;
    // Distance=0;
    if (Distance > 100)
    {
      Record_Dis = 0;
      Distance = 0;
      Statu = Normals;
      BeeOff;
    }
  }
  else
  {
    //  Run=0;
    if (Zebra_delay_flag1 || Zebra_flag1)
    {
      Zebra_flag1 = 0;
      Zebra_delay_flag1 = 1;

      if ((DI.ui8_ImageArray[35][78] && DI.ui8_ImageArray[40][78]) ||
          (DI.ui8_ImageArray[35][1] && DI.ui8_ImageArray[40][1]))
      {
        Zebra_delay_flag1 = 0;
        Zebra_flag2 = 1;
        Record_Angle = 1;
        Record_Dis = 1;
        //  BeeOff;
      }
      //    else{
      //      if(Zerbra_Dir==R_go){
      //      uint8 R_Y2=0;
      //      uint8 R_X2=0;
      //      for(uint8 Y=DI.ui8_DisposeScopeUp+5;Y<DI.ui8_DisposeScopeDown-10;Y++){
      //        if(DI.ui8_ImageArray[Y][70] && DI.ui8_ImageArray[Y+1][70]
      //           && DI.ui8_ImageArray[Y+2][70] && DI.ui8_ImageArray[Y+3][70]
      //             && DI.ui8_ImageArray[Y+4][70]){
      //               R_Y2=Y-3;
      //               break;
      //             }
      //      }
      //      R_X2=DI.ui8_RPoint[R_Y2];
      //      while(1){
      //                if(R_Y2>=59 || R_X2>79){
      //                  break;
      //                }
      //                else if(!DI.ui8_ImageArray[R_Y2+1][R_X2])
      //
      //               { //ÏòÏÂÊÇºÚÉ«
      //                    R_Y2++;
      //                }
      //                else if(!DI.ui8_ImageArray[R_Y2+1][R_X2+1]){
      //                    R_Y2++;
      //                    R_X2++;
      //                }
      //                else if(!DI.ui8_ImageArray[R_Y2+1][R_X2+2]){
      //                    R_Y2++;
      //                    R_X2+=2;
      //                }
      //               else if(!DI.ui8_ImageArray[R_Y2+1][R_X2+3]){
      //                    R_Y2++;
      //                    R_X2+=3;
      //                }
      //                else{
      //               break;
      //             }
      //           }
      //     for(uint8 Y=R_Y2+7;Y<DI.ui8_DisposeScopeDown-5;Y++){
      //        if(!DI.ui8_ImageArray[Y][70] && !DI.ui8_ImageArray[Y+1][70]){
      //               //R_Y1=Y;
      //               break;
      //             }
      //        else{
      //         //R_Y1=DI.ui8_DisposeScopeDown;
      //        }
      //      }
      //     //R_X1=DI.ui8_LPoint[R_Y1];
      //      Fits(DI.ui8_DisposeScopeLeft+1,40,DI.ui8_DisposeScopeDown,59-street_len_40);
      //      Ladder();
      //      FitRoad();//»¹Ã»ÓÐ¿¼ÂÇ»·µº
      //      Get_len();
      //      DetermineScanLine();      //È·¶¨É¨ÃèÐÐ
      //      NormalControl();
      //    }
      //    else{
      //      uint8 R_Y2=0;
      //      uint8 R_X2=0;
      //      for(uint8 Y=DI.ui8_DisposeScopeUp+5;Y<DI.ui8_DisposeScopeDown-10;Y++){
      //        if(DI.ui8_ImageArray[Y][10] && DI.ui8_ImageArray[Y+1][10]
      //           && DI.ui8_ImageArray[Y+2][10] && DI.ui8_ImageArray[Y+3][10]
      //             && DI.ui8_ImageArray[Y+4][10]){
      //               R_Y2=Y-3;
      //               break;
      //             }
      //      }
      //      R_X2=DI.ui8_LPoint[R_Y2];
      //      while(1){
      //                if(R_Y2>=59 || R_X2<0){
      //                  break;
      //                }
      //                else if(!DI.ui8_ImageArray[R_Y2+1][R_X2])
      //
      //               { //ÏòÏÂÊÇºÚÉ«
      //                    R_Y2++;
      //                }
      //                else if(!DI.ui8_ImageArray[R_Y2+1][R_X2-1]){
      //                    R_Y2++;
      //                    R_X2--;
      //                }
      //                else if(!DI.ui8_ImageArray[R_Y2+1][R_X2-2]){
      //                    R_Y2++;
      //                    R_X2-=2;
      //                }
      //               else if(!DI.ui8_ImageArray[R_Y2+1][R_X2-3]){
      //                    R_Y2++;
      //                    R_X2-=3;
      //                }
      //                else{
      //               break;
      //             }
      //           }
      //     for(uint8 Y=R_Y2+7;Y<DI.ui8_DisposeScopeDown-5;Y++){
      //        if(!DI.ui8_ImageArray[Y][10] && !DI.ui8_ImageArray[Y+1][10]){
      //               //R_Y1=Y;
      //               break;
      //             }
      //        else{
      //         //R_Y1=DI.ui8_DisposeScopeDown;
      //        }
      //      }
      //     //R_X1=DI.ui8_LPoint[R_Y1];
      //      Fits(DI.ui8_DisposeScopeLeft+1,40,DI.ui8_DisposeScopeDown,59-street_len_40);
      //      Ladder();
      //      FitRoad();//»¹Ã»ÓÐ¿¼ÂÇ»·µº
      //      Get_len();
      else
      {
        DetermineScanLine(); //È·¶¨É¨ÃèÐÐ
        NormalControl();
      }
    }
    if (Zebra_delay_flag2 || Zebra_flag2)
    {
      Zebra_flag2 = 0;
      Zebra_delay_flag2 = 1;
      if (Zerbra_Dir == R_go)
      {
        if (SumAngle > 120 || SumAngle <= -120)
        {
          SumAngle = 0;
          Distance = 0;
          Record_Angle = 0;
          BeeOff;
          Zebra_delay_flag2 = 0;
          Statu = Stops; //Í£³µ
                         //   Run=0;
        }
        Fit(DI.ui8_DisposeScopeLeft, 75, DI.ui8_DisposeScopeDown, 59 - street_len_75);
        for (uint8 Y = DI.ui8_DisposeScopeDown; Y > 59 - street_len_75; Y--)
        {
          DI.ui8_RPoint[Y] = 79;
        }
        poserror = 50; //(DI.ui8_LPoint[72-street_len_75]+DI.ui8_RPoint[72-street_len_75]+DI.ui8_LPoint[71-street_len_75]+DI.ui8_RPoint[71-street_len_75]+DI.ui8_LPoint[70-street_len_75]+DI.ui8_RPoint[70-street_len_75])/6-30;
      }
      else
      {
        if (SumAngle <= -120 || SumAngle > 120)
        {
          SumAngle = 0;
          Distance = 0;
          Record_Angle = 0;
          BeeOff;
          Zebra_delay_flag2 = 0;
          Statu = Stops; //Í£³µ
                         //   Run=0;
        }
        Fit(DI.ui8_DisposeScopeRight, 5, DI.ui8_DisposeScopeDown, 59 - street_len_5);
        for (uint8 Y = DI.ui8_DisposeScopeDown; Y > 59 - street_len_5; Y--)
        {
          DI.ui8_LPoint[Y] = 5;
        }
        poserror = -50; //(DI.ui8_LPoint[72-street_len_5]+DI.ui8_RPoint[72-street_len_5]+DI.ui8_LPoint[71-street_len_5]+DI.ui8_RPoint[71-street_len_5]+DI.ui8_LPoint[70-street_len_5]+DI.ui8_RPoint[70-street_len_5])/6-50;
      }
    }
  }
}
/***********************************************************************
º¯ÊýÃû£º»·µº´¦Àí
***********************************************************************/
//uint8 LRing_Out_Flag = 0;
//void LRingTreatment()
//{
//  //³µ»áÅÜµ½Èë»·¿Ú
//  if ((L_Ring_flag1 || L_Ring_delay_flag1))
//  {
//    L_Ring_flag1 = 0;
//    L_Ring_delay_flag1 = 1;
//    Record_Dis=1;      //¼ÇÂ¼¾àÀë
//    Record_Angle=1;    //¼ÇÂ¼½Ç¶È
//    if ((street_len_5 > 20 && street_len_5 <= 55
//            && !DI.ui8_ImageArray[57 - street_len_5][5] && !DI.ui8_ImageArray[55 - street_len_5][5]
//            && !DI.ui8_ImageArray[53 - street_len_5][5] && Distance >= 15) || Distance >= 45)
//    { //×óÉÏ¶¼ÊÇºÚÉ«
//        Record_Dis=0;
//        Distance=0;
//        L_Ring_delay_flag1=0;
//        L_Ring_flag2=1;
//        BeeOff;
//      //Statu = Stops;
//    }
//  }
//
//  //Èë»·,µ½Èë»·½áÊø
//  if ((L_Ring_flag2 || L_Ring_delay_flag2))
//  {
//    L_Ring_flag2 = 0;
//    L_Ring_delay_flag2 = 1;
//    // BeeOn;
//    if (SumAngle>250) //»ý·ÖÈë»·½áÊø//SumAngle >= 155 || SumAngle <= -155
//    {
//      BeeOff;
//      L_Ring_delay_flag2 = 0;
//      L_Ring_flag3 = 1;
//      Statu = Stops;
//    }
//    if (street_len_5 > 20)
//    {
//      uint8 L_Y1 = DI.ui8_DisposeScopeDown - 1;
//      uint8 L_X1 = DI.ui8_DisposeScopeRight-1; // DI.ui8_RPoint[L_Y1] DI.ui8_DisposeScopeRight-1
//      uint8 L_Y3 = 60 - street_len_5;
//      L_Y3 = L_Y3 - 7;
//      uint8 L_X3 = DI.ui8_LPoint[L_Y3];
//      while (1)
//      {
//        if (L_Y3 >= 59 || L_X3 <= 0)
//        {
//          break;
//        }
//        else if (!DI.ui8_ImageArray[L_Y3 + 1][L_X3])
//        { //ÏòÏÂÊÇºÚÉ«
//          L_Y3++;
//        }
//        else if (!DI.ui8_ImageArray[L_Y3 + 1][L_X3 - 1])
//        {
//          L_Y3++;
//          L_X3--;
//        }
//        else if (!DI.ui8_ImageArray[L_Y3 + 1][L_X3 - 2])
//        {
//          L_Y3++;
//          L_X3 -= 2;
//        }
//        else if (!DI.ui8_ImageArray[L_Y3 + 1][L_X3 - 3])
//        {
//          L_Y3++;
//          L_X3 -= 3;
//        }
//        else
//        {
//          break;
//        }
//      }
//      Vistable_scale = L_Y3;
//      for (uint8 ui8_Y = L_Y1 - 1; ui8_Y > L_Y3; ui8_Y--)
//      {
//        DI.ui8_LPoint[ui8_Y] = 0; //È«±äµ½±ß½ç´¦,ÓÐÀûÓÚ²¹Ïß
//      }
//      Fit(L_X1, L_X3, L_Y1, L_Y3);
//      BeeOn;
//    }
//    else
//    { //ºÚÉ«³öÏÖ
//      uint8 L_Y1 = DI.ui8_DisposeScopeDown - 1;
//      uint8 L_X1 = DI.ui8_DisposeScopeRight - 1;
//      uint8 L_Y3 = 59 - street_len_5;
//      uint8 L_X3 = DI.ui8_LPoint[L_Y3];
//      for (uint8 Y = L_Y3 - 1; Y > DI.ui8_DisposeScopeUp; Y--)
//      {
//        if (DI.ui8_ImageArray[Y][5] && DI.ui8_ImageArray[Y - 1][5])
//        {
//          L_Y3 = Y;
//          break;
//        }
//      }
//      for (uint8 Y = L_Y3 - 1; Y > DI.ui8_DisposeScopeUp; Y--)
//      {
//        if (!DI.ui8_ImageArray[Y][5] && !DI.ui8_ImageArray[Y - 1][5])
//        {
//          L_Y3 = Y;
//          break;
//        }
//      }
//      L_Y3 = L_Y3 - 7;
//      L_X3 = DI.ui8_LPoint[L_Y3];
//      while (1)
//      {
//        if (L_Y3 >= 59 || L_X3 <= 0)
//        {
//          break;
//        }
//        else if (!DI.ui8_ImageArray[L_Y3 + 1][L_X3])
//
//        { //ÏòÏÂÊÇºÚÉ«
//          L_Y3++;
//        }
//        else if (!DI.ui8_ImageArray[L_Y3 + 1][L_X3 - 1])
//        {
//          L_Y3++;
//          L_X3--;
//        }
//        else if (!DI.ui8_ImageArray[L_Y3 + 1][L_X3 - 2])
//        {
//          L_Y3++;
//          L_X3 -= 2;
//        }
//        else if (!DI.ui8_ImageArray[L_Y3 + 1][L_X3 - 3])
//        {
//          L_Y3++;
//          L_X3 -= 3;
//        }
//        else
//        {
//          break;
//        }
//      }
//      Vistable_scale = L_Y3;
//      for (uint8 ui8_Y = L_Y1 - 1; ui8_Y > L_Y3; ui8_Y--)
//      {
//        DI.ui8_LPoint[ui8_Y] = 0;
//      }
//      Fit(L_X1, L_X3, L_Y1, L_Y3);
//    }
//
//    for (uint8 Y = Vistable_scale; Y > DI.ui8_DisposeScopeUp; Y--)
//    {
//      if (DI.ui8_ImageArray[Y][3] && DI.ui8_ImageArray[Y][4])
//      {
//        for (uint8 X = 4; X < DI.ui8_DisposeScopeRight; X++)
//        {
//          if (!DI.ui8_ImageArray[Y][X])
//          {
//            DI.ui8_RPoint[Y] = X;
//            DI.ui8_LPoint[Y] = 0;
//            break;
//          }
//        }
//      }
//      else
//      {
//        Vistable_scale = Y + 1;
//        break;
//      }
//    }
//    //Ò»ÖÖÆæ¹ÖµÄÇé¿ö
//    if (DI.ui8_ImageArray[59][5] && DI.ui8_ImageArray[58][5])
//      Vistable_scale = 59 - street_len_5;
//
//    DetermineScanLine(); //È·¶¨É¨ÃèÐÐ
//    NormalControl();
//    //poserror = poserror * 2.0;
//  }
//  //ÏÂÃæÊÇ»·ÖÐ
//  if ((L_Ring_flag3 || L_Ring_delay_flag3))
//  {
//    L_Ring_flag3 = 0;
//    L_Ring_delay_flag3 = 1;
//    //Record_Angle = 1; //¼ÇÂ¼½Ç¶È  ....
//    if ((DI.ui8_ImageArray[50][3] && DI.ui8_ImageArray[53][3] && DI.ui8_ImageArray[55][3] &&
//         DI.ui8_ImageArray[30][78] && DI.ui8_ImageArray[32][78] && DI.ui8_ImageArray[34][78] && Distance >= 60)
//            ||(SumAngle >= 800)) //800 550
//    {
//      //Statu = Stops;
//      L_Ring_delay_flag3 = 0;
//      L_Ring_flag4 = 1;
//      BeeOn;
//      SumAngle=0;
//    }
//    NormalControl();
//    if (poserror <= -40)
//      poserror = -40;
//    if (poserror >= 40)
//      poserror = 40;
//    // poserror = poserror * 1.1;
//  }
//  //³ö»·¿Ú³ö»·
//
//  if ((L_Ring_flag4 || L_Ring_delay_flag4))
//  {
//    L_Ring_flag4 = 0;
//    L_Ring_delay_flag4 = 1;
//    if (SumAngle>=250)//´ýÑéÖ¤
//    {
//      L_Ring_delay_flag4 = 0;
//      L_Ring_flag5 = 1;
//      Record_Dis = 1;
//      Distance = 0;
//      //Statu = Stops;
//      BeeOff;
//    }
//    int L_Y1 = DI.ui8_DisposeScopeDown - 10; //Ô­ -10
//    int L_X1 = DI.ui8_DisposeScopeRight - 1;         // DI.ui8_DisposeScopeRight-1 DI.ui8_RPoint[L_Y1]
//    int L_Y3 = 59 - street_len_5;
//    int L_X3 = 5;
////    while (1)
////    {
////      if (L_Y1 <= 15 || L_X1 <= 15)
////      {
////        break;
////      }
////      else if (!DI.ui8_ImageArray[L_Y1][L_X1 - 1])
////      { //ÏòÉÏÊÇºÚÉ«
////        L_X1--;
////      }
////      else if (!DI.ui8_ImageArray[L_Y1 - 1][L_X1 - 1])
////      {
////        L_Y1--;
////        L_X1--;
////      }
////      else if (!DI.ui8_ImageArray[L_Y1 - 1][L_X1 - 2])
////      {
////        L_Y1--;
////        L_X1 -= 2;
////      }
////      else if (!DI.ui8_ImageArray[L_Y1 - 1][L_X1 - 3])
////      {
////        L_Y1--;
////        L_X1 -= 3;
////      }
////      else
////      {
////        break;
////      }
////    }
////
////    if (L_Y1 <= 15) // whileÕÒ´íµãÁË£¬Ê¹ÓÃÔ­µã
////    {
////      L_Y1 = DI.ui8_DisposeScopeDown - 10;
////      L_X1 = DI.ui8_DisposeScopeRight - 1;
////    }
////    for (uint8 Y = 20; Y < 59; Y++)
////    {
////      if (DI.ui8_ImageArray[5][Y] && DI.ui8_ImageArray[5][Y + 1])
////      {
////        L_Y3 = Y;
////        break;
////      }
////    }
//    for (uint8 Y = 59; Y > 15; Y--)
//    {
//      if (!DI.ui8_ImageArray[Y][5] && !DI.ui8_ImageArray[Y - 1][5])
//      {
//        L_Y3 = Y;
//        break;
//      }
//    }
//    //×ó±ßÈ«²¿Îª0
//    for (uint8 ui8_Y = DI.ui8_DisposeScopeDown - 1; ui8_Y > L_Y3; ui8_Y--)
//    {
//      DI.ui8_LPoint[ui8_Y] = 0;
//    }
//    Fit(L_X1, L_X3, L_Y1, L_Y3);
//
//    // DisposeImage();
//    Vistable_scale = L_Y3;
//    DetermineScanLine(); //È·¶¨É¨ÃèÐÐ
//    NormalControl();
//    //poserror=poserror*1.08;
//  }
//  if (L_Ring_flag5 || L_Ring_delay_flag5)
//  {
//    L_Ring_flag5 = 0;
//    L_Ring_delay_flag5 = 1;
//    if (Distance >= 68) // 58 75ÊÇÎªÁË±ÜÃâÐ±ÈëÈý²í Æ½³£30¾ÍOK 71   60 70 1.15-----Èý²í¿ÉÊ¶±ð
//    {
//      BeeOff;
//      L_Ring_delay_flag5 = 0;
//      Ring_flag = 0;
//      have_ring = 1;
//      Record_Dis = 0;
//      Record_Angle = 0;
//      Distance = 0;
//      SumAngle = 0;
//      LRing_Out_Flag++;
//      Statu = Normals;
//      //Statu = Stops;
//    }
//    // if(Ring_Size==Big){
//    if (street_len_5 >= 7)
//    {
//      int L_Y1 = 54;
//      int L_X1 = 0; // 79-(79-DI.ui8_LineWidth[54]-DI.ui8_LPoint[54]);
//      int L_Y3 = 59 - street_len_5;
//      int L_X3 = 15;
//      for (uint8 X = 5; X < 79; X++)
//      {
//        if (DI.ui8_ImageArray[L_Y3 - 2][X] && DI.ui8_ImageArray[L_Y3 - 2][X + 1])
//        {
//          L_X3 = X;
//          break;
//        }
//      }
//      Fit(L_X1, L_X3, L_Y1, L_Y3);
//      DetermineScanLine(); //È·¶¨É¨ÃèÐÐ
//      NormalControl();
//      poserror = poserror/1.08; //±ÜÃâÐ±ÈëÈý²í 1.22
//    }
//    else
//    {
//      poserror = (DI.ui8_RPoint[20] + DI.ui8_RPoint[21] + DI.ui8_RPoint[22] + DI.ui8_LPoint[20] + DI.ui8_LPoint[21] + DI.ui8_LPoint[22]) / 6 - 40;
//    }
//  }
//}

void LRingTreatment2(void)
{
    if((L_Ring_flag1 || L_Ring_delay_flag1)){
       L_Ring_flag1=0;
       L_Ring_delay_flag1=1;
       Record_Dis=1;      //¼ÇÂ¼¾àÀë
       if(//(street_len_5>20 && street_len_5<=55 && !DI.ui8_ImageArray[57-street_len_5][5] && !DI.ui8_ImageArray[55-street_len_5][5] &&  !DI.ui8_ImageArray[53-street_len_5][5] && Distance>=500)||
               Distance>Lring_dis
               )
       {
         Record_Dis=0;
         Distance=0;
         L_Ring_delay_flag1=0;
         L_Ring_flag2=1;
         BeeOff;
       }
    }
    if((L_Ring_flag2 || L_Ring_delay_flag2))
    {
      L_Ring_flag2=0;
      L_Ring_delay_flag2=1;
      MID_POINT=40;
      Record_Angle=1;    //开启角度积分
      if(SumAngle>Lring_ang1)   //积分入环角度
      {
        BeeOff;
        MID_POINT=40;
        L_Ring_delay_flag2=0;
        L_Ring_flag3=1;
      }
    }
    if((L_Ring_flag3 || L_Ring_delay_flag3))
    {
       L_Ring_flag3=0;
       L_Ring_delay_flag3=1;
       BeeOn;
       MID_POINT=40;
       if(
      //         (DI.ui8_ImageArray[50][3]&& DI.ui8_ImageArray[53][3] && DI.ui8_ImageArray[55][3] &&
      //         DI.ui8_ImageArray[30][78]&& DI.ui8_ImageArray[32][78] && DI.ui8_ImageArray[34][78]
        //     &&(SumAngle>=130)   )||
               (SumAngle>=Lring_ang2))
       {
           MID_POINT=40;
         L_Ring_delay_flag3=0;
         L_Ring_flag4=1;
         BeeOff;
         SumAngle=0;
       }
     //  NormalControl();
       if(poserror<=-24) poserror=-24;
       if(poserror>=24) poserror=24;

  }
    if((L_Ring_flag4 || L_Ring_delay_flag4))
    {
      L_Ring_flag4=0;
      L_Ring_delay_flag4=1;
      MID_POINT=40;
      Record_Angle=1;    //开启角度积分
      if(SumAngle>Lring_ang3)   //积分入环角度
      {
        BeeOff;
        MID_POINT=40;
        L_Ring_delay_flag4=0;
        L_Ring_flag5=1;
      }
    }
    if(L_Ring_flag5 || L_Ring_delay_flag5)
    {
    L_Ring_flag5=0;
    L_Ring_delay_flag5=1;
    Record_Dis = 1;
    BeeOn;
    if(Distance>=Lring_dis2){ //60¿ÉÄÜÓÐµã¶àÁË
      BeeOff;
      L_Ring_delay_flag5=0;
      Ring_flag=0;
      have_ring=1;
      Record_Dis=0;
      Record_Angle=0;
      Distance=0;
      SumAngle=0;
      Statu=Normals;
      Ring_times++;
      SPEED_BASE=ring_after_speed;
    }
   // if(Ring_Size==Big){
      if(street_len_5>=7){
      int L_Y1=54;
      int L_X1=0;//79-(79-DI.ui8_LineWidth[54]-DI.ui8_LPoint[54]);
      int L_Y3=59-street_len_5;
      int L_X3=15;
        for(uint8 X=5;X<79;X++){
          if(DI.ui8_ImageArray[L_Y3-2][X] && DI.ui8_ImageArray[L_Y3-2][X+1]){
            L_X3=X;
            break;
          }
        }
       Fit(L_X1,L_X3,L_Y1,L_Y3);
          DetermineScanLine();      //È·¶¨É¨ÃèÐÐ
         NormalControl();
         poserror = poserror / 1.18;
      }
      else{
               poserror=(DI.ui8_RPoint[20]+DI.ui8_RPoint[21]+DI.ui8_RPoint[22]
               +DI.ui8_LPoint[20]+DI.ui8_LPoint[21]+DI.ui8_LPoint[22])/6-40;
      }


 }

}
void RRingTreament2(void)
{
    if ((R_Ring_flag1 || R_Ring_delay_flag1))
    {
      R_Ring_flag1 = 0;
      R_Ring_delay_flag1 = 1;
      Record_Dis = 1;   //¼ÇÂ¼¾àÀë
      if (
              //(street_len_75 >= 20 && street_len_75 <= 55
          // && !DI.ui8_ImageArray[55 - street_len_75][74] && !DI.ui8_ImageArray[53 - street_len_75][74]
           //&& !DI.ui8_ImageArray[51 - street_len_75][74] && Distance >= 10) ||
              Distance>Rring_dis)
      {
        Record_Dis = 0;
        Distance = 0;
        R_Ring_delay_flag1 = 0;
        R_Ring_flag2 = 1;
        Record_Angle = 1; //¼ÇÂ¼½Ç¶È
        SumAngle = 0;
        BeeOff;
        //Statu = Stops;
      }
    }
      if (R_Ring_flag2 || R_Ring_delay_flag2)
      {
        R_Ring_flag2 = 0;
        R_Ring_delay_flag2 = 1;
        MID_POINT=40;
        Record_Angle = 1;
        if (SumAngle <= -Rring_ang1)    //»ý·ÖÈë»·½áÊø
        {
    //      BeeOff;
            MID_POINT=40;
          R_Ring_delay_flag2 = 0;
          R_Ring_flag3 = 1;
          SumAngle = 0;
          //Statu = Stops;
        }
      }

      if (R_Ring_flag3 || R_Ring_delay_flag3)
      {
        R_Ring_flag3 = 0;
        R_Ring_delay_flag3 = 1;
        MID_POINT=40;
        BeeOn;
        Record_Angle = 1;
        if (SumAngle <= -Rring_ang2)    //»ý·ÖÈë»·½áÊø
        {
    //      BeeOff;
            MID_POINT=40;
          R_Ring_delay_flag3 = 0;
          R_Ring_flag4 = 1;
          SumAngle = 0;
          //Statu = Stops;
        }
      }
      if (R_Ring_flag4 || R_Ring_delay_flag4)
      {
        R_Ring_flag4 = 0;
        R_Ring_delay_flag4 = 1;
        MID_POINT=40;
        BeeOff;
        Record_Angle = 1;
        if (SumAngle <= -Rring_ang3)    //»ý·ÖÈë»·½áÊø
        {
    //      BeeOff;
            MID_POINT=40;
          R_Ring_delay_flag4 = 0;
          R_Ring_flag5 = 1;
          SumAngle = 0;
          //Statu = Stops;
        }
      }
      if (R_Ring_flag5 || R_Ring_delay_flag5)
      {
        R_Ring_flag5 = 0;
        R_Ring_delay_flag5 = 1;
        Record_Dis = 1;
        BeeOn;
        if (Distance >= Rring_dis2)
        { //(Distance>=90 && Ring_Size==Big) || (Distance>=80 && Ring_Size==Little)
          BeeOff;
          R_Ring_delay_flag5 = 0;
          Ring_flag = 0;
          have_ring = 1;
          Record_Dis = 0;
          Record_Angle = 0;
          Distance = 0;
          SumAngle = 0;
          Statu = Normals;
          //Statu = Stops;
          Ring_times++;
          SPEED_BASE=ring_after_speed;
        }
        // if(Ring_Size==Big){
        if (street_len_75 >= 7)
        {
          int R_Y1 = 54;
          int R_X1 = 79; // 79-(79-DI.ui8_LineWidth[54]-DI.ui8_LPoint[54]);
          int R_Y3 = 59 - street_len_75;
          int R_X3 = 65;
          for (uint8 X = 75; X > 0; X--)
          {
            if (DI.ui8_ImageArray[R_Y3 - 2][X] && DI.ui8_ImageArray[R_Y3 - 2][X - 1])
            {
              R_X3 = X;
              break;
            }
          }
          Fit(R_X1, R_X3, R_Y1, R_Y3);
          DetermineScanLine(); //È·¶¨É¨ÃèÐÐ
          NormalControl();
        }
        else
        {
          poserror = (DI.ui8_RPoint[20] + DI.ui8_RPoint[21] + DI.ui8_RPoint[22] + DI.ui8_LPoint[20] + DI.ui8_LPoint[21] + DI.ui8_LPoint[22]) / 6 - 40;
        }
        /*if(Ring_Size==Big){
          if(poserror<=-20) poserror=-20;
          if(poserror>20) poserror=20;
        }
        else{
          if(poserror<=-40) poserror=-40;
          if(poserror>=40) poserror=40;
        }*/
      }

}
uint8 LRing_Out_Flag = 0;
uint8 L_Y3;
uint8 L_X3;
uint8 L_Y1;
uint8 L_X1;
void LRingTreatment(){
  //³µ»áÅÜµ½Èë»·¿Ú
  if((L_Ring_flag1 || L_Ring_delay_flag1)){
     L_Ring_flag1=0;
     L_Ring_delay_flag1=1;
     Record_Dis=1;      //¼ÇÂ¼¾àÀë


     if(//(street_len_5>20 && street_len_5<=55 && !DI.ui8_ImageArray[57-street_len_5][5] && !DI.ui8_ImageArray[55-street_len_5][5] &&  !DI.ui8_ImageArray[53-street_len_5][5] && Distance>=500)||
/*
             DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 14][DI.ui8_DisposeScopeLeft+5] &&
             DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 12][DI.ui8_DisposeScopeLeft+5 ]&&
             DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 10][DI.ui8_DisposeScopeLeft+5]&&
             !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 6][DI.ui8_DisposeScopeLeft+10] &&
             !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 4][DI.ui8_DisposeScopeLeft+10 ]&&
             !DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 2][DI.ui8_DisposeScopeLeft+10] //&& +DI.ui8_ImageArray[DI.ui8_DisposeScopeDown - 8][DI.ui8_DisposeScopeLeft ]//²»ÄÜÔÙ±È6¸ßÁË
*/


             Distance>Lring_dis
             )
     {
       Record_Dis=0;
       Distance=0;
       L_Ring_delay_flag1=0;
       L_Ring_flag2=1;
       BeeOff;
     }


  }

  //Èë»·,µ½Èë»·½áÊø
   if((L_Ring_flag2 || L_Ring_delay_flag2))
   {
     L_Ring_flag2=0;
     L_Ring_delay_flag2=1;
     MID_POINT=40;
     Record_Angle=1;    //开启角度积分
     if(SumAngle>Lring_ang1)   //积分入环角度
     {
       BeeOff;
       MID_POINT=40;
       L_Ring_delay_flag2=0;
       L_Ring_flag3=1;
     }
     if(street_len_5>20){
         L_Y1=DI.ui8_DisposeScopeDown-1;
         L_X1=DI.ui8_DisposeScopeRight;
             L_Y3=60-street_len_5;
       L_Y3=L_Y3-7;
         L_X3=DI.ui8_LPoint[L_Y3];

       while(1){
                if(L_Y3>=59 || L_X3<=0){
                  break;
                }
                else if(!DI.ui8_ImageArray[L_Y3+1][L_X3])

               { //ÏòÏÂÊÇºÚÉ«
                    L_Y3++;
                }
                else if(!DI.ui8_ImageArray[L_Y3+1][L_X3-1]){
                    L_Y3++;
                    L_X3--;
                }
                else if(!DI.ui8_ImageArray[L_Y3+1][L_X3-2]){
                    L_Y3++;
                    L_X3-=2;
                }
               else if(!DI.ui8_ImageArray[L_Y3+1][L_X3-3]){
                    L_Y3++;
                    L_X3-=3;
                }
                else{
               break;
             }
           }
/*
       if(L_Y3>59)
           L_Y3=59;
       if(L_Y3<0)
           L_Y3=0;
*/
       Vistable_scale=L_Y3;
       for(uint8 ui8_Y=L_Y1;ui8_Y>L_Y3;ui8_Y--)
       {
          DI.ui8_LPoint[ui8_Y]=0; //È«±äµ½±ß½ç´¦,ÓÐÀûÓÚ²¼Ïß
         }
       Fit(L_X1,L_X3,L_Y1,L_Y3);
     }
     else{//ºÚÉ«³öÏÖ
         L_Y1=DI.ui8_DisposeScopeDown-1;
         L_X1=DI.ui8_DisposeScopeRight;
         L_Y3=59-street_len_5;
         L_X3=DI.ui8_LPoint[L_Y3];
       for(uint8 Y=L_Y3-1;Y>DI.ui8_DisposeScopeUp;Y--){
         if(DI.ui8_ImageArray[Y][5] && DI.ui8_ImageArray[Y-1][5]){
           L_Y3=Y;
           break;
         }
       }
       for(uint8 Y=L_Y3-1;Y>DI.ui8_DisposeScopeUp;Y--){
         if(!DI.ui8_ImageArray[Y][5] && !DI.ui8_ImageArray[Y-1][5]){
           L_Y3=Y;
           break;
         }
       }
        L_Y3=L_Y3-7;
       L_X3=DI.ui8_LPoint[L_Y3];
       while(1){
                if(L_Y3>=59 || L_X3<=0){
                  break;
                }
                else if(!DI.ui8_ImageArray[L_Y3+1][L_X3])

               { //ÏòÏÂÊÇºÚÉ«
                    L_Y3++;
                }
                else if(!DI.ui8_ImageArray[L_Y3+1][L_X3-1]){
                    L_Y3++;
                    L_X3--;
                }
                else if(!DI.ui8_ImageArray[L_Y3+1][L_X3-2]){
                    L_Y3++;
                    L_X3-=2;
                }
               else if(!DI.ui8_ImageArray[L_Y3+1][L_X3-3]){
                    L_Y3++;
                    L_X3-=3;
                }
                else{
               break;
             }
           }
       if(L_Y3>59)
           L_Y3=59;
       if(L_Y3<0)
           L_Y3=0;
       Vistable_scale=L_Y3;
       for(uint8 ui8_Y=L_Y1;ui8_Y>L_Y3;ui8_Y--){
          DI.ui8_LPoint[ui8_Y]=0;
         }
       Fit(L_X1,L_X3,L_Y1,L_Y3);

     }


     for(uint8 Y=Vistable_scale;Y>DI.ui8_DisposeScopeUp;Y--){
       if(DI.ui8_ImageArray[Y][3] && DI.ui8_ImageArray[Y][4]){
         for( uint8 X=4;X<DI.ui8_DisposeScopeRight;X++){
           if(!DI.ui8_ImageArray[Y][X]){
              DI.ui8_RPoint[Y]=X;
              DI.ui8_LPoint[Y]=0;
              break;
           }
         }
       }
       else{
         Vistable_scale=Y+1;
         break;
       }
     }

      //Ò»ÖÖÆæ¹ÖµÄÇé¿ö
     if(DI.ui8_ImageArray[59][5] && DI.ui8_ImageArray[58][5] )
       Vistable_scale=59-street_len_5;
/*     if(Vistable_scale>59)
         Vistable_scale=59;
     if(Vistable_scale<0)
         Vistable_scale=0;*/
      DetermineScanLine();      //È·¶¨É¨ÃèÐÐ
      NormalControl();
      if(poserror<=-20) poserror=-20;
      if(poserror>=20) poserror=20;
   }
   //ÏÂÃæÊÇ»·ÖÐ
  if((L_Ring_flag3 || L_Ring_delay_flag3))
  {
     L_Ring_flag3=0;
     L_Ring_delay_flag3=1;
     BeeOn;
     MID_POINT=40;
     if(
    //         (DI.ui8_ImageArray[50][3]&& DI.ui8_ImageArray[53][3] && DI.ui8_ImageArray[55][3] &&
    //         DI.ui8_ImageArray[30][78]&& DI.ui8_ImageArray[32][78] && DI.ui8_ImageArray[34][78]
      //     &&(SumAngle>=130)   )||
             (SumAngle>=Lring_ang2))
     {
         MID_POINT=40;
       L_Ring_delay_flag3=0;
       L_Ring_flag4=1;
       BeeOff;
       SumAngle=0;
     }
     NormalControl();
     if(poserror<=-24) poserror=-24;
     if(poserror>=25) poserror=24;

}
//³ö»·¿Ú³ö»·

   if((L_Ring_flag4 || L_Ring_delay_flag4)){
     L_Ring_flag4=0;
     L_Ring_delay_flag4=1;
     MID_POINT=30;
     if(SumAngle>=Lring_ang3)
     {
         MID_POINT=40;
       L_Ring_delay_flag4=0;
       L_Ring_flag5=1;
       Record_Dis=1;
       BeeOn;
     }

       int L_Y1=DI.ui8_DisposeScopeDown-10; //Ô­ -10
       int L_X1=DI.ui8_DisposeScopeRight-1;
       int L_Y3=59-street_len_5;
       int L_X3=5;
       for(uint8 Y=3;Y<59;Y++){
           if(DI.ui8_ImageArray[10][Y] && DI.ui8_ImageArray[10][Y+1] ){
               L_Y3=Y;
               break;
           }
       }
       //×ó±ßÈ«²¿Îª0
       for(uint8 ui8_Y=L_Y1-1;ui8_Y>L_Y3;ui8_Y--){
          DI.ui8_LPoint[ui8_Y]=0;
         }
       Fit(L_X1,L_X3,L_Y1,L_Y3);

      // DisposeImage();

       Vistable_scale=L_Y3;
        DetermineScanLine();      //È·¶¨É¨ÃèÐÐ
        NormalControl();

  }
     if(L_Ring_flag5 || L_Ring_delay_flag5)
     {
     L_Ring_flag5=0;
     L_Ring_delay_flag5=1;
     if(Distance>=Lring_dis2){ //60¿ÉÄÜÓÐµã¶àÁË
       BeeOff;
       L_Ring_delay_flag5=0;
       Ring_flag=0;
       have_ring=1;
       Record_Dis=0;
       Record_Angle=0;
       Distance=0;
       SumAngle=0;
       LRing_Out_Flag++;
       Statu=Normals;
       Ring_times++;
       SPEED_BASE=ring_after_speed;
     }
    // if(Ring_Size==Big){
       if(street_len_5>=7){
       int L_Y1=54;
       int L_X1=0;//79-(79-DI.ui8_LineWidth[54]-DI.ui8_LPoint[54]);
       int L_Y3=59-street_len_5;
       int L_X3=15;
         for(uint8 X=5;X<79;X++){
           if(DI.ui8_ImageArray[L_Y3-2][X] && DI.ui8_ImageArray[L_Y3-2][X+1]){
             L_X3=X;
             break;
           }
         }
        Fit(L_X1,L_X3,L_Y1,L_Y3);
           DetermineScanLine();      //È·¶¨É¨ÃèÐÐ
          NormalControl();
          poserror = poserror / 1.18;
       }
       else{
                poserror=(DI.ui8_RPoint[20]+DI.ui8_RPoint[21]+DI.ui8_RPoint[22]
                +DI.ui8_LPoint[20]+DI.ui8_LPoint[21]+DI.ui8_LPoint[22])/6-40;
       }


  }
}



//ÓÒ»·µº´¦Àí
void RRingTreatment(void)
{
  if ((R_Ring_flag1 || R_Ring_delay_flag1))
  {
    R_Ring_flag1 = 0;
    R_Ring_delay_flag1 = 1;
    Record_Dis = 1;   //¼ÇÂ¼¾àÀë
    if (
            //(street_len_75 >= 20 && street_len_75 <= 55
        // && !DI.ui8_ImageArray[55 - street_len_75][74] && !DI.ui8_ImageArray[53 - street_len_75][74]
         //&& !DI.ui8_ImageArray[51 - street_len_75][74] && Distance >= 10) ||
            Distance>Rring_dis)
    {
      Record_Dis = 0;
      Distance = 0;
      R_Ring_delay_flag1 = 0;
      R_Ring_flag2 = 1;
      Record_Angle = 1; //¼ÇÂ¼½Ç¶È
      SumAngle = 0;
      BeeOff;
      //Statu = Stops;
    }
  }
  //Èë»·,µ½Èë»·½áÊø
  if (R_Ring_flag2 || R_Ring_delay_flag2)
  {
    R_Ring_flag2 = 0;
    R_Ring_delay_flag2 = 1;
    MID_POINT=40;
    if (SumAngle <= -Rring_ang1)    //»ý·ÖÈë»·½áÊø
    {
//      BeeOff;
        MID_POINT=40;
      R_Ring_delay_flag2 = 0;
      R_Ring_flag3 = 1;
      SumAngle = 0;
      //Statu = Stops;
    }
  if (street_len_75 > 20)
    {
      uint8 R_Y1 = DI.ui8_DisposeScopeDown - 1;
      uint8 R_X1 = DI.ui8_LPoint[R_Y1];//DI.ui8_DisposeScopeLeft + 1
      uint8 R_Y3 = 60 - street_len_75;
      R_Y3 = R_Y3 - 8;
      uint8 R_X3 = DI.ui8_RPoint[R_Y3];
      while (1)
      {
        if (R_Y3 >= 59 || R_X3 >= 79)
        {
          break;
        }
        else if (!DI.ui8_ImageArray[R_Y3 + 1][R_X3])

        { //ÏòÏÂÊÇºÚÉ«
          R_Y3++;
        }
        else if (!DI.ui8_ImageArray[R_Y3 + 1][R_X3 + 1])
        {
          R_Y3++;
          R_X3++;
        }
        else if (!DI.ui8_ImageArray[R_Y3 + 1][R_X3 + 2])
        {
          R_Y3++;
          R_X3 += 2;
        }
        else if (!DI.ui8_ImageArray[R_Y3 + 1][R_X3 + 3])
        {
          R_Y3++;
          R_X3 += 3;
        }
        else
        {
          break;
        }
      }
      Vistable_scale = R_Y3;
      for (uint8 ui8_Y = R_Y1 - 1; ui8_Y > R_Y3; ui8_Y--)
      {
        DI.ui8_RPoint[ui8_Y] = 79; //È«±äµ½±ß½ç´¦£¬ÓÐÀûÓÚ²¹Ïß
      }
      Fit(R_X1, R_X3, R_Y1, R_Y3);
//      BeeOn;
    }
    else
    { //ºÚÉ«³öÏÖ
      uint8 R_Y1 = DI.ui8_DisposeScopeDown - 1;
      uint8 R_X1 = DI.ui8_DisposeScopeLeft + 1;
      uint8 R_Y3 = 59 - street_len_75;
      uint8 R_X3 = DI.ui8_RPoint[R_Y3];
      for (uint8 Y = R_Y3 - 1; Y > DI.ui8_DisposeScopeUp; Y--)
      {
        if (DI.ui8_ImageArray[Y][75] && DI.ui8_ImageArray[Y - 1][75])
        {
          R_Y3 = Y;
          break;
        }
      }
      for (uint8 Y = R_Y3 - 1; Y > DI.ui8_DisposeScopeUp; Y--)
      {
        if (!DI.ui8_ImageArray[Y][75] && !DI.ui8_ImageArray[Y - 1][75])
        {
          R_Y3 = Y;
          break;
        }
      }
      R_Y3 = R_Y3 - 7;
      R_X3 = DI.ui8_RPoint[R_Y3];
      while (1)
      {
        if (R_Y3 >= 59 || R_X3 >= 79)
        {
          break;
        }
        else if (!DI.ui8_ImageArray[R_Y3 + 1][R_X3])

        { //ÏòÏÂÊÇºÚÉ«
          R_Y3++;
        }
        else if (!DI.ui8_ImageArray[R_Y3 + 1][R_X3 + 1])
        {
          R_Y3++;
          R_X3++;
        }
        else if (!DI.ui8_ImageArray[R_Y3 + 1][R_X3 + 2])
        {
          R_Y3++;
          R_X3 += 2;
        }
        else if (!DI.ui8_ImageArray[R_Y3 + 1][R_X3 + 3])
        {
          R_Y3++;
          R_X3 += 3;
        }
        else
        {
          break;
        }
      }
      Vistable_scale = R_Y3;
      for (uint8 ui8_Y = R_Y1 - 1; ui8_Y > R_Y3; ui8_Y--)
      {
        DI.ui8_RPoint[ui8_Y] = 79;
      }
      Fit(R_X1, R_X3, R_Y1, R_Y3);
    }
    for (uint8 Y = Vistable_scale; Y > DI.ui8_DisposeScopeUp; Y--)
    {
      if (DI.ui8_ImageArray[Y][78] && DI.ui8_ImageArray[Y][77])
      {
        for (uint8 X = 76; X > DI.ui8_DisposeScopeLeft; X--)
        {
          if (!DI.ui8_ImageArray[Y][X])
          {
            DI.ui8_LPoint[Y] = X;
            DI.ui8_RPoint[Y] = 79;
            break;
          }
        }
      }
      else
      {
        Vistable_scale = Y + 1;
        break;
      }
    }
    //Ò»ÖÖÆæ¹ÖµÄÇé¿ö
    if (DI.ui8_ImageArray[59][75] && DI.ui8_ImageArray[58][75])
      Vistable_scale = 59 - street_len_75;

    DetermineScanLine(); //È·¶¨É¨ÃèÐÐ
    NormalControl();
  }

  //  ÏÂÃæÊÇ»·ÖÐ
  if ((R_Ring_flag3 || R_Ring_delay_flag3))
  {
    R_Ring_flag3 = 0;
    R_Ring_delay_flag3 = 1;
    MID_POINT=50;
          BeeOn;
    if ((SumAngle <=-Rring_ang2)
          //  || (DI.ui8_ImageArray[50][75] && DI.ui8_ImageArray[53][75] && DI.ui8_ImageArray[55][75] &&
          //                    DI.ui8_ImageArray[30][1] && DI.ui8_ImageArray[32][1] && DI.ui8_ImageArray[34][1])
                              ) //(DI.ui8_ScanLineToL[5]<=10 && DI.ui8_ScanLineToL[7]<=10  && DI.ui8_ScanLineToL[6]<=10)
    {
//        Statu = Stops;
        MID_POINT=40;
      R_Ring_delay_flag3 = 0;
      R_Ring_flag4 = 1;
      SumAngle = 0;
//      BeeOn;
    }
    NormalControl();
    if(poserror<=-24) poserror=-24;
    if(poserror>=24) poserror=24;
  }
  //³ö»·¿Ú³ö»·
  if ((R_Ring_flag4 || R_Ring_delay_flag4))
  {
    R_Ring_flag4 = 0;
    R_Ring_delay_flag4 = 1;
    MID_POINT=40;
    BeeOff;
    if (SumAngle <= -Rring_ang3)
    { //&& Ring_Size==Little)|| (SumAngle>300 && Ring_Size==Big)
      BeeOff;
        MID_POINT=40;
      R_Ring_delay_flag4 = 0;
      R_Ring_flag5 = 1;
      Record_Dis = 1;
      //Statu = Stops;
    }
    int R_Y1 = DI.ui8_DisposeScopeDown - 1;
    int R_X1 = DI.ui8_DisposeScopeLeft + 1;
    int R_Y3 = 59 - street_len_75;
    int R_X3 = 75;
    while (1)
    {
      if (R_Y1 <= 15 || R_X1 >= 65)
      {
        break;
      }
      else if (!DI.ui8_ImageArray[R_Y1][R_X1 + 1])
      { //ÏòÉÏÊÇºÚÉ«
          R_X1++;
      }
      else if (!DI.ui8_ImageArray[R_Y1 - 1][R_X1 + 1])
      {
          R_Y1--;
          R_X1++;
      }
      else if (!DI.ui8_ImageArray[R_Y1 - 1][R_X1 + 2])
      {
          R_Y1--;
          R_X1 += 2;
      }
      else if (!DI.ui8_ImageArray[R_Y1 - 1][R_X1 + 3])
      {
          R_Y1--;
          R_X1 += 3;
      }
      else
      {
        break;
      }
    }
    if(R_Y1 < 15)   //whileÕÒ´íµãÁË£¬Ê¹ÓÃÔ­µã
    {
        R_Y1 = DI.ui8_DisposeScopeDown - 10;
        R_X1 = DI.ui8_DisposeScopeLeft + 1;
    }
//    for (uint8 Y = 3; Y < 59; Y++)
//    {
//      if (DI.ui8_ImageArray[72][Y] && DI.ui8_ImageArray[72][Y + 1])
//      {
//        R_Y3 = Y;
//        break;
//      }
//    }
    for (uint8 Y = 59; Y > 15; Y--)
    {
        if (!DI.ui8_ImageArray[Y][74] && !DI.ui8_ImageArray[Y - 1][74])
        {
          R_Y3 = Y;
          break;
        }
    }
    //ÓÒ±ßÈ«²¿Îª0
    for (uint8 ui8_Y = R_Y1 - 1; ui8_Y > R_Y3; ui8_Y--)
    {
      DI.ui8_RPoint[ui8_Y] = 79;
    }
    //ÓÒÏß
    Fit(R_X1, R_X3, R_Y1, R_Y3);
    Vistable_scale = R_Y3;
    DetermineScanLine(); //È·¶¨É¨ÃèÐÐ
    NormalControl();
  }

  if (R_Ring_flag5 || R_Ring_delay_flag5)
  {
    R_Ring_flag5 = 0;
    R_Ring_delay_flag5 = 1;
    BeeOn;
    if (Distance >= Rring_dis2)
    { //(Distance>=90 && Ring_Size==Big) || (Distance>=80 && Ring_Size==Little)
      BeeOff;
      R_Ring_delay_flag5 = 0;
      Ring_flag = 0;
      have_ring = 1;
      Record_Dis = 0;
      Record_Angle = 0;
      Distance = 0;
      SumAngle = 0;
      Statu = Normals;
      //Statu = Stops;
      Ring_times++;
      SPEED_BASE=ring_after_speed;
    }
    // if(Ring_Size==Big){
    if (street_len_75 >= 7)
    {
      int R_Y1 = 54;
      int R_X1 = 79; // 79-(79-DI.ui8_LineWidth[54]-DI.ui8_LPoint[54]);
      int R_Y3 = 59 - street_len_75;
      int R_X3 = 65;
      for (uint8 X = 75; X > 0; X--)
      {
        if (DI.ui8_ImageArray[R_Y3 - 2][X] && DI.ui8_ImageArray[R_Y3 - 2][X - 1])
        {
          R_X3 = X;
          break;
        }
      }
      Fit(R_X1, R_X3, R_Y1, R_Y3);
      DetermineScanLine(); //È·¶¨É¨ÃèÐÐ
      NormalControl();
    }
    else
    {
      poserror = (DI.ui8_RPoint[20] + DI.ui8_RPoint[21] + DI.ui8_RPoint[22] + DI.ui8_LPoint[20] + DI.ui8_LPoint[21] + DI.ui8_LPoint[22]) / 6 - 40;
    }
    /*if(Ring_Size==Big){
      if(poserror<=-20) poserror=-20;
      if(poserror>20) poserror=20;
    }
    else{
      if(poserror<=-40) poserror=-40;
      if(poserror>=40) poserror=40;
    }*/
  }
}

//ÆÂµÀ
void RampsTreatment()
{
  if (Distance >= 140)
  {
    Distance = 0;
    Record_Dis = 0;
//    BeeOff;
    Statu = Normals;
    have_podao = 1;
  }
}

//Ð±Ê®×Ö
void Xie_CrossTreatment()
{
//  BeeOn;
  if (Xie_Cross_flag0)
  {
    if (street_len_5 > street_len_75)
    { //²¹ÓÒÏß
      int L_Y1 = DI.ui8_DisposeScopeDown - 10;
      int L_X1 = DI.ui8_DisposeScopeRight - 1;
      int L_Y3 = 59 - street_len_5;
      int L_X3 = 5;
      for (uint8 ui8_Y = L_Y1 - 1; ui8_Y > L_Y3; ui8_Y--)
      { //×ó±ßÈ«Îª0
        DI.ui8_LPoint[ui8_Y] = 0;
      }
      Fit(L_X1, L_X3, L_Y1, L_Y3);
      Vistable_scale = L_Y3;
      DetermineScanLine(); //È·¶¨É¨ÃèÐÐ
      NormalControl();
    }
    else
    {
      int L_Y1 = DI.ui8_DisposeScopeDown - 10;
      int L_X1 = DI.ui8_DisposeScopeLeft + 1;
      int L_Y3 = 59 - street_len_75;
      int L_X3 = 75;
      for (uint8 ui8_Y = L_Y1 - 1; ui8_Y > L_Y3; ui8_Y--)
      { //×ó±ßÈ«Îª0
        DI.ui8_RPoint[ui8_Y] = 79;
      }
      Fit(L_X1, L_X3, L_Y1, L_Y3);
      Vistable_scale = L_Y3;
      DetermineScanLine(); //È·¶¨É¨ÃèÐÐ
      NormalControl();
    }
    if (Distance >= 10)
    {
      Distance = 0;
      Record_Dis = 0;
      Statu = Normals;
      Xie_Cross_flag0 = 0;
 //     BeeOff;
    }
  }
}
/*
 //×´Ì¬´¦Àí
void Treatment_Status(){
  switch(Statu)
 {
   case Zebra:  ZebraTreatment();  break;//Èë¿â´¦Àí
   case Cross:  CrossTreatment();  break;
   case LRing:  LRingTreatment();  break;
   case RRing:  RRingTreatment();  break;
   case Lukou:  ChaLuTreatment();  break;
   case Ramps:  RampsTreatment();  break;
   default:    NormalTreatment ();
 }
}
*/
uint8 Chalu_num = 0;  //¼ÆÊý
uint8 CHALU_TIME = 1; // 1:Ïò×ó¹Õ 2:ÏòÓÒ¹Õ
void ChaLuTreatment(void)
{
  if (ChaLu_flag0 || ChaLu_delay_flag0)
  {
    ChaLu_flag0 = 0;
 //   BeeOff;
    ChaLu_delay_flag0 = 1;
    if (street_len_40 <= 50)
    {
      ChaLu_flag1 = 1;
      ChaLu_delay_flag0 = 0;
      Distance = 0;
      SumAngle = 0;
      // Statu = Stops;
    }
  }

  if (ChaLu_flag1 || ChaLu_delay_flag1)
  {
    Record_Angle = 1;
    Record_Dis = 1;
 //   BeeOn;
    ChaLu_flag1 = 0;
    ChaLu_delay_flag1 = 1;
    if ((SumAngle >= 60 || SumAngle <= -60) || Distance >= 18)// || Distance >= 40)SumAngle >= 60 || SumAngle <= -60
    {
      SumAngle = 0;
      Record_Angle = 0;
      Distance = 0;
 //     BeeOff;
      ChaLu_delay_flag1 = 0;
      ChaLu_flag2 = 1;
      //Statu = Stops;
    }
    //ÏòÓÒ
    else
    {
      if (CHALU_TIME == 2)
      {
        Fits(DI.ui8_DisposeScopeLeft + 1, 40, DI.ui8_DisposeScopeDown, 59 - street_len_40);
        Ladder();
        FitRoad(); //»¹Ã»ÓÐ¿¼ÂÇ»·µº
        Get_len();
        Vistable_scale = 59 - street_len_40;
        DetermineScanLine(); //È·¶¨É¨ÃèÐÐ
        NormalControl();
        if (poserror >= 45)
          poserror = 45;
        if (poserror <= 15)
          poserror = 15;
      }
      //Ïò×ó
      else
      {
        Fits(DI.ui8_DisposeScopeRight - 1, 40, DI.ui8_DisposeScopeDown, 59 - street_len_40);
        Ladder();
        FitRoad(); //»¹Ã»ÓÐ¿¼ÂÇ»·µº
        Get_len();
        Vistable_scale = 59 - street_len_40;
        DetermineScanLine(); //È·¶¨É¨ÃèÐÐ
        NormalControl();
        if (poserror <= -45)
          poserror = -45;
        if (poserror >= -15)
          poserror = -15;
      }
    }
  }

  if (ChaLu_flag2 || ChaLu_delay_flag2)
  {
    ChaLu_flag2 = 0;
    ChaLu_delay_flag2 = 1;
//    if (((DI.ui8_ImageArray[40][5] && DI.ui8_ImageArray[45][5] && DI.ui8_ImageArray[50][5]
//            && DI.ui8_ImageArray[59 - street_len_5][50]
//            && DI.ui8_ScanLineToL[5] > 50 && DI.ui8_ScanLineToL[6] > 50 && DI.ui8_ScanLineToL[7] > 50
//            //&& DI.ui8_ImageArray[59 - street_len_5][41] && DI.ui8_ImageArray[59 - street_len_5][42]
//            && CHALU_TIME == 2 && Distance >= 80)
//            || (DI.ui8_ImageArray[40][75] && DI.ui8_ImageArray[45][75] && DI.ui8_ImageArray[50][75]
//            //&& DI.ui8_ImageArray[59 - street_len_75][40] && DI.ui8_ImageArray[59 - street_len_75][41]
//            && DI.ui8_ScanLineToR[4] > 50 && DI.ui8_ScanLineToR[5] > 50 && DI.ui8_ScanLineToR[6] > 50 && DI.ui8_ScanLineToR[7] > 50
//            && DI.ui8_ImageArray[59 - street_len_75][30] && CHALU_TIME == 1 && Distance >= 70)) ||
//        (Distance >= 74 && CHALU_TIME == 1) || (Distance >= 82 && CHALU_TIME == 2)) //92 98
        if (//((DI.ui8_ImageArray[40][5] && DI.ui8_ImageArray[45][5] && DI.ui8_ImageArray[50][5]
                //&& DI.ui8_ImageArray[59 - street_len_5][50]
//                && DI.ui8_ScanLineToL[5] > 50 && DI.ui8_ScanLineToL[6] > 50 && DI.ui8_ScanLineToL[7] > 50
                //&& DI.ui8_ImageArray[59 - street_len_5][41] && DI.ui8_ImageArray[59 - street_len_5][42]
                //&& CHALU_TIME == 2 && Distance >= 70)
                //|| (DI.ui8_ImageArray[40][75] && DI.ui8_ImageArray[45][75] && DI.ui8_ImageArray[50][75]
                //&& DI.ui8_ImageArray[59 - street_len_75][40] && DI.ui8_ImageArray[59 - street_len_75][41]
//                && DI.ui8_ScanLineToR[4] > 50 && DI.ui8_ScanLineToR[5] > 50 && DI.ui8_ScanLineToR[6] > 50 && DI.ui8_ScanLineToR[7] > 50
                //&& DI.ui8_ImageArray[59 - street_len_75][30] && CHALU_TIME == 1 && Distance >= 70)) ||
            (Distance >= 72 && CHALU_TIME == 1) || (Distance >= 76 && CHALU_TIME == 2)) //92 98
    {
 //     BeeOn;
      ChaLu_delay_flag2 = 0;
      ChaLu_flag3 = 1;
      //Statu = Stops;
    }
    else
    {
      NormalControl();
    }
  }
  if (ChaLu_flag3 || ChaLu_delay_flag3)
  {
 //   BeeOff;
    ChaLu_flag3 = 0;
    ChaLu_delay_flag3 = 1;
    if (Distance >= 100)// && Distance >= 15)(SumAngle >= 60 || SumAngle <= -60)
    {
  //    BeeOff;
      ChaLu_delay_flag3 = 0;
      Record_Dis = 0;
      Distance = 0;
      Statu = Normals;
      //Statu = Stops;
      CHALU_TIME++;
      Chalu_num++;
      // if (CHALU_TIME == 2)
      // CHALU_TIME = 0;
    }
    //Ïò×ó
    else
    {
      if (CHALU_TIME == 2&&street_len_40<=30)//&& street_len_5 >= 10
      {
        Fits(DI.ui8_DisposeScopeLeft + 1, 40, DI.ui8_DisposeScopeDown, 59 - street_len_40);
        Ladder();
        FitRoad(); //»¹Ã»ÓÐ¿¼ÂÇ»·µº
        Get_len();
        Vistable_scale = 59 - street_len_40;
        DetermineScanLine(); //È·¶¨É¨ÃèÐÐ
        NormalControl();
        if (poserror >= 45)
          poserror = 45;
        if (poserror <= 15)
          poserror = 15;
        //Statu = Stops;
      }
      else if (CHALU_TIME == 1&&street_len_40<=30)//&& street_len_75 >= 10
      {
        Fits(DI.ui8_DisposeScopeRight - 1, 40, DI.ui8_DisposeScopeDown, 59 - street_len_40);
        Ladder();
        FitRoad(); //»¹Ã»ÓÐ¿¼ÂÇ»·µº
        Get_len();
        Vistable_scale = 59 - street_len_40;
        DetermineScanLine(); //È·¶¨É¨ÃèÐÐ
        NormalControl();
        if (poserror <= -45)
          poserror = -45;
        if (poserror >= -15)
          poserror = -15;
      }
      else{
          NormalControl();
          poserror = poserror *1.03;
      }
    }
  }
}

/*
void CrossTreatment(void){
    if(Cross_flag0||Cross_delay_flag0){
      BeeOn;
      Cross_flag0=0;
      Cross_delay_flag0=1;
      Record_Dis=1;
      if(Distance>=100){
         BeeOff;
          Cross_delay_flag0=0;
          Distance = 0;
          Record_Dis = 0;
          NormalControl();
          Statu=Normals;
          SC++;
          if(SC>=Special_Count) {SC=0;Run=0;}
           Cross_num = 0;
      }else{NormalControl();}//ÔÚ²íÂ·ÄÚ²¿Õý³£´¦Àí
   }
//   if(Cross_flag1||Cross_delay_flag1){
//      Cross_flag1=0;
//      Cross_delay_flag1=1;
//      Record_Angle = 1;
//      if( (SumAngle>=700 || SumAngle<=-700)&&ChaLu_Judge()==1 ){
//          Cross_delay_flag1=0;
//          Cross_flag2=1;
//          SumAngle=0;
//          Record_Angle = 0;
//        NormalControl();
//      }else{NormalControl();}//ÔÚ²íÂ·ÄÚ²¿Õý³£´¦Àí
//   }
//   if(Cross_flag2||Cross_delay_flag2){
//      BeeOn;
//      Cross_flag2=0;
//      Cross_delay_flag2=1;
//      Record_Dis=1;
//      if(Distance>=30){
//          BeeOff;
//          Cross_delay_flag2=0;
//          Distance = 0;
//          Record_Dis = 0;
//          NormalControl();
//          Statu=Normals;
//          SC++;
//          if(SC>=Special_Count) {SC=0;Run=0;}
//           Cross_num = 0;
//      }else{NormalControl();}//ÔÚ²íÂ·ÄÚ²¿Õý³£´¦Àí
//   }
}
*/


