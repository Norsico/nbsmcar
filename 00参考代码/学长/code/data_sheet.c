/*
 * data_sheet.c
 *
 *  Created on: 2025年2月26日
 *      Author: 15958
 */


#include "data_sheet.h"
int Ring_type[6];//当前要过的圆环类型，设定了最多6个圆环
int Ring_times;//当前已通过的圆环次数
int Ring_mode=2;//圆环循迹方式,1为旧版本,2为新版本
int DB_times;//当前已过单边桥次数
int DB_type[5];//当前要过的单边桥类型,设定了最多5个单边桥,1为五厘米,2为三厘米
int Ringtypes;//小圆环为1，中圆环为3，大圆环为2
int jumptypes;//跳过为1，绕过为2
int ramp_length[5];//坡道长度
int ramp_times;//坡道次数
int luzhang_types[5];//路障为1，斑马线为2
int luzhang_times;//路障通过次数
int speed_extra1=0;
int jump_ready;//是否开启横断
int ramp_ready;//是否开启横断
int lz_no_dis=700;//路障不处理长度
/////////////////////过各元素的速度///////////////////
int ramp_speed=450;//过坡道的速度
int ring_speed=490;//过圆环的速度
int jump_speed=500;//过横断的速度
int DBBridge_speed=300;
int DBBridge_stop_speed=-400;
/////////////////////过完各元素的速度///////////////////
int ramp_after_speed;//过完坡道的速度
int ring_after_speed;//过完圆环的速度
int jump_after_speed;//过完跳跃的速度
int DBBridge_after_speed;//过完单边桥的速度
int luzhang_after_speed;//过完小路障的速度
/////////////////////过完各元素的速度///////////////////


////////////////////单边桥参数//////////////////////////
int dbbridge_stop_time=700;//过单边桥减速时间
int dbbridge_pass_time=2100;//最后通过单边桥的时间

int dbbridge_stop_time3=300;//过3cm单边桥减速时间
int dbbridge_pass_time3=1500;//最后通过3cm单边桥的时间
////////////////////左圆环参数/////////////////////////
int Lring_dis;//积分入环距离
int Lring_ang1;//积分入环角度
int Lring_ang2;//积分出环角度1
int Lring_ang3;//积分出环角度2
int Lring_dis2;//积分出环距离

////////////////////右圆环参数/////////////////////////
int Rring_dis;//积分入环距离
int Rring_ang1;//积分入环角度
int Rring_ang2;//积分出环角度1
int Rring_ang3;//积分出环角度2
int Rring_dis2;//积分出环距离

////////////////////横断参数//////////////////////////
int jump_linelength=42;
int vs_jump=35;//跳跃点

////////////////////单边桥参数////////////////////////

///////////////////////////////////////////旧循迹方式的参数////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
///////////////////左大圆环参数///////////////////////

int Lbig_ring_dis11=1750;//积分入环距离    1850
int Lbig_ring_ang11=44;//积分入环角度   44
int Lbig_ring_ang12=130;//积分出环角度1  130
int Lbig_ring_ang13=45;//积分出环角度2  45
int Lbig_ring_dis12=2000;//积分出环距离   2000



///////////////////左小圆环参数///////////////////////

int Lsmall_ring_dis11=1350;//积分入环距离    1200
int Lsmall_ring_ang11=30;//积分入环角度   44
int Lsmall_ring_ang12=100;//积分出环角度1  100
int Lsmall_ring_ang13=45;//积分出环角度2  45
int Lsmall_ring_dis12=2000;//积分出环距离   2000

///////////////////右大圆环参数///////////////////////

int Rbig_ring_dis11=1750;//积分入环距离
int Rbig_ring_ang11=30;//积分入环角度
int Rbig_ring_ang12=100;//积分出环角度1
int Rbig_ring_ang13=45;//积分出环角度2
int Rbig_ring_dis12=2000;//积分出环距离

///////////////////右小圆环参数///////////////////////

int Rsmall_ring_dis11=1100;//积分入环距离    1100
int Rsmall_ring_ang11=30;//积分入环角度   -30
int Rsmall_ring_ang12=75;//积分出环角度1  -75
int Rsmall_ring_ang13=40;//积分出环角度2  -40
int Rsmall_ring_dis12=2000;//积分出环距离   2000

///////////////////////////////////////////旧循迹方式的参数////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////以下是新循迹方式的参数////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////

///////////////////左大圆环参数///////////////////////

int Lbig_ring_dis21=1750;//积分入环距离    1850
int Lbig_ring_ang21=30;//积分入环角度   44
int Lbig_ring_ang22=90;//积分出环角度1  130
int Lbig_ring_ang23=80;//积分出环角度2  45
int Lbig_ring_dis22=1700;//积分出环距离   2000

///////////////////左小圆环参数///////////////////////

int Lsmall_ring_dis21=1100;//积分入环距离    1200
int Lsmall_ring_ang21=30;//积分入环角度   44
int Lsmall_ring_ang22=110;//积分出环角度1  100
int Lsmall_ring_ang23=50;//积分出环角度2  45
int Lsmall_ring_dis22=2000;//积分出环距离   2000
int Lring_errmax=21;//左圆环误差最大值
///////////////////右大圆环参数///////////////////////

int Rbig_ring_dis21=1750;//积分入环距离
int Rbig_ring_ang21=30;//积分入环角度
int Rbig_ring_ang22=90;//积分出环角度1
int Rbig_ring_ang23=40;//积分出环角度2
int Rbig_ring_dis22=1500;//积分出环距离

///////////////////右小圆环参数///////////////////////

int Rsmall_ring_dis21=1100;//积分入环距离    1100
int Rsmall_ring_ang21=30;//积分入环角度   -30
int Rsmall_ring_ang22=90;//积分出环角度1  -75
int Rsmall_ring_ang23=40;//积分出环角度2  -40
int Rsmall_ring_dis22=2000;//积分出环距离   2000
int Rring_errmax=20;//右圆环误差最大值

void speed_base_init(void)
{
     ramp_after_speed=SPEED_BASE;//过完坡道的速度
     ring_after_speed=SPEED_BASE;//过完圆环的速度
     jump_after_speed=SPEED_BASE;//过完跳跃的速度
     DBBridge_after_speed=SPEED_BASE;//过完单边桥的速度
     luzhang_after_speed=SPEED_BASE;//过完小路障的速度
     ramp_length[0]=3500;
     ramp_length[1]=3500;
     ramp_length[2]=3500;
     ramp_length[3]=3500;
}


void Ring_data(void)
{
    if(Ring_mode==1)
    {
        if(Ring_type[Ring_times]==1)
        {
             Lring_dis=Lsmall_ring_dis11;//积分入环距离
             Lring_ang1=Lsmall_ring_ang11;//积分入环角度
             Lring_ang2=Lsmall_ring_ang12;//积分出环角度1
             Lring_ang3=Lsmall_ring_ang13;//积分出环角度2
             Lring_dis2=Lsmall_ring_dis12;//积分出环距离
             Rring_dis=Rsmall_ring_dis11;//积分入环距离
             Rring_ang1=Rsmall_ring_ang11;//积分入环角度
             Rring_ang2=Rsmall_ring_ang12;//积分出环角度1
             Rring_ang3=Rsmall_ring_ang13;//积分出环角度2
             Rring_dis2=Rsmall_ring_dis12;//积分出环距离
        }
        if(Ring_type[Ring_times]==2)
        {
             Lring_dis=Lbig_ring_dis11;//积分入环距离
             Lring_ang1=Lbig_ring_ang11;//积分入环角度
             Lring_ang2=Lbig_ring_ang12;//积分出环角度1
             Lring_ang3=Lbig_ring_ang13;//积分出环角度2
             Lring_dis2=Lbig_ring_dis12;//积分出环距离
             Rring_dis=Rbig_ring_dis11;//积分入环距离
             Rring_ang1=Rbig_ring_ang11;//积分入环角度
             Rring_ang2=Rbig_ring_ang12;//积分出环角度1
             Rring_ang3=Rbig_ring_ang13;//积分出环角度2
             Rring_dis2=Rbig_ring_dis12;//积分出环距离
        }
    }
    if(Ring_mode==2)
    {
        if(Ring_type[Ring_times]==1)
        {
             Lring_dis=Lsmall_ring_dis21;//积分入环距离
             Lring_ang1=Lsmall_ring_ang21;//积分入环角度
             Lring_ang2=Lsmall_ring_ang22;//积分出环角度1
             Lring_ang3=Lsmall_ring_ang23;//积分出环角度2
             Lring_dis2=Lsmall_ring_dis22;//积分出环距离
             Rring_dis=Rsmall_ring_dis21;//积分入环距离
             Rring_ang1=Rsmall_ring_ang21;//积分入环角度
             Rring_ang2=Rsmall_ring_ang22;//积分出环角度1
             Rring_ang3=Rsmall_ring_ang23;//积分出环角度2
             Rring_dis2=Rsmall_ring_dis22;//积分出环距离
        }
        if(Ring_type[Ring_times]==2)
        {
             Lring_dis=Lbig_ring_dis21;//积分入环距离
             Lring_ang1=Lbig_ring_ang21;//积分入环角度
             Lring_ang2=Lbig_ring_ang22;//积分出环角度1
             Lring_ang3=Lbig_ring_ang23;//积分出环角度2
             Lring_dis2=Lbig_ring_dis22;//积分出环距离
             Rring_dis=Rbig_ring_dis21;//积分入环距离
             Rring_ang1=Rbig_ring_ang21;//积分入环角度
             Rring_ang2=Rbig_ring_ang22;//积分出环角度1
             Rring_ang3=Rbig_ring_ang23;//积分出环角度2
             Rring_dis2=Rbig_ring_dis22;//积分出环距离
        }
    }
}

