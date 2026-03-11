#ifndef SYSTEMCONFIG_H
#define SYSTEMCONFIG_H

#define ele_cross   1
#define ele_lcircle 2
#define ele_rcircle 3
#define ele_slope   4

#define hc05_buffer_size 64
#define beep_on     P43=0
#define beep_off    P43=1
#include "headfile.h"
#include "CCD.h"
#include "Gryo.h"
#include "MENU.h"
#include "Motor.h"
#include "Servo.h"
//#include "iic_oled.h"

typedef struct 
{
    uint8 last_state;
    uint8 small_S;
    uint8 huge_S;
    uint8 stright;
    uint8 pre;
}car_state;
/*BDLC*/
extern int BDLC_PWM;
/*Zebra*/
extern uint8 zebra_num;
/*Servo&PID*/
extern uint8 Spid_Sel;
extern int16 Servo_Out;
extern float UFF[7];
extern float P_Rate;
extern float ServoP;
extern float ServoD;
extern float MaxP;
extern int Servo_Mid;
extern int Servo_pc;
/*Motor&PID*/
extern float LMp,LMi,LMd,RMp,RMi,RMd;
extern float lvsp,rvsp,ele_count,dv,l_dv,r_dv;
extern int tarsp,lsetv,rsetv;
extern int runflag;
extern uint8 cross_tarsp,circle_tarsp,slope_tarsp;
extern uint16 stright_tarsp,normal_tarsp;
/*flag*/
extern uint8 l_num;
extern uint8 r_num;
extern uint8 cross_nco;
extern uint8 ele_num;
extern uint8 lcir_num;
extern uint8 rcir_num;
extern uint8 cross_num;
extern uint8 ele_num;
extern uint8 last_ele;
extern uint8 slope_nco;
extern uint8 slope_num;
extern float a_time;
/*GRYO*/
extern float yaw_count,pitch,yaw,pitch_err;
/*Cross*/
extern uint8 cross_err,crossf_max;
/*Judge*/
extern uint8 up_limit;
extern uint8 down_limit;
extern uint8 up_lose;
extern uint8 down_lose;
/*Process*/
extern uint8 icm_in;
extern uint8 icm_out;
extern uint8 cir_inp;
extern uint8 cir_outp;
extern uint8 cir_incheck;
extern uint8 cir_outcheck;
/*Camera*/
extern int8 ccd_offset;
extern int8 ccd1_offset;
extern float down_arr;
extern float up_arr;
extern float down_dec;
extern float up_dec;
extern float eCCD[3];
extern uint8 keepline;
extern uint8 incirline;
extern uint8 outcirline;
extern uint8 lp[3];
extern uint8 rp[3];
extern uint8 old_mid;
extern uint8 old_mid1;
extern uint8 lpu;
extern uint8 rpu;
extern uint8 lpd;
extern uint8 rpd;
extern uint8 Comapre_0;
extern uint8 Comapre_1;
extern uint8 Slope_Data;
extern uint16 threshold;
extern uint16 threshold1;
extern float myabs(float x);
extern void Device_initialize();
extern void SystemCtrl();
void BEM_START();
#endif