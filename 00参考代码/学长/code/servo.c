/*
 * servo.c
 *
 *  Created on: 2025年1月20日
 *      Author: 15958
 */
#include "servo.h"
#include "zf_common_headfile.h"
int  height_out,height_outl,height_outr,fuzhu_Vertical_out;      //翻滚角输出
int compensation_out;//补偿输出
int fuzhuhuan_count;
int servo_count;
float fuzhu_out;       //辅助环输出
float zhuanxiang_out,zhuanxiang_l,zhuanxiang_r;
float high_out=0;        //小车高度
int high_mode=0;         //小车高度模式，0为低模式，1为高模式
int high_out_max;      //小车过单边桥的输出
int g_y;              //陀螺仪翻滚角速度
float sv_p=1.3,sv_i=0,g_p=0.0;         //舵机翻滚角度环1.3
float sr_p=-0,sr_i=0;//0.4
float fz_p=0.18,fz_i;                     //辅助环0.20
float fz_p_base=0.10;
float fz_p_high;
float fzz_p=0,fzz_d=0;
float zx_p=0.0;
int16 pwm_ph1, pwm_ph4;
float LF_SPEED_ANGLE=0,LB_SPEED_ANGLE=0,RF_SPEED_ANGLE=0,RB_SPEED_ANGLE=0;
float LF_SPEED;
float LB_SPEED;
float RF_SPEED;
float RB_SPEED;
int jump_flag,jump_cnt,jump_flag2,jump_flag3,jump_flag4;
int servo_Vertical(float Med,float Angle,float gyro_Y)//翻滚角度环
{
    int temp;
    static float inte,ERR,ERR_OUT,ERR_LAST,l_angle;
    inte+=(Angle-Med);
//    if(l_angle<0&&Angle>=0)inte=0;
//    if(l_angle>=0&&Angle<0)inte=0;
//    l_angle=Angle;
    inte=inte>250?250:(inte<(-250)?(-250):inte);
    temp=sv_p*(Angle-Med)+sv_i*inte+g_p*gyro_Y;
    if(temp>30)
        temp=30;
    if(temp<-30)
        temp=-30;
    return temp;
}
int servo_roll_compensation(float angle,float gyro_x)//补偿角度环
{
    int temp;
    static float inte,ERR,ERR_OUT,ERR_LAST,l_angle;
    inte+=angle;
//    if(l_angle<0&&Angle>=0)inte=0;
//    if(l_angle>=0&&Angle<0)inte=0;
//    l_angle=Angle;
    inte=inte>2000?2000:(inte<(-2000)?(-2000):inte);
    temp=sr_p*angle+sr_i*inte;

    return -temp;
}
float fuzhu_Vertical(float Med,float Angle,float gyro_Y)//直立环
{
    int temp;
    static float l_angle;
/*
    inte+=inte+=-(Angle-Med);;

    if(l_angle<0&&angle>=0)inte=0;
    if(l_angle>=0&&angle<0)inte=0;

    l_angle=Angle;
*/
    temp=fzz_p*(Angle-Med)+fzz_d*gyro_Y;
    return temp;
}

float fuzhu(int Target,int encoder_L,int encoder_R)
{
    static int err;
    static float Err_LowOut_last,Encoder_S,l_angle;
    static float a=0.3;
    fz_i=fz_p/200;//p是i的200倍
    float temp;//结果
    Err_LowOut_last=Err_LowOut;//更新上次

    Err_LowOut=(1-a)*err+a*Err_LowOut_last;//误差低通滤波

    Encoder_S+=Err_LowOut;//积分

    Encoder_S=Encoder_S>10000?10000:(Encoder_S<(-10000)?(-10000):Encoder_S);//积分限幅

    err=(encoder_L+encoder_R)/2+g_x*0-Target;//计算本次滤波前误差

    temp=1.0*Err_LowOut*fz_p+fz_i*Encoder_S;//pi环计算输出

    //Err_LowOut_last=err;

    if(jump_flag3==1)
        temp=0;
    if(temp>30)
        temp=30;
    if(temp<-30)
        temp=-30;
    return temp;
    }

float zhuanxiang(int encoder_L,int encoder_R)
{
    float temp,err,pers;
    err=encoder_L-encoder_R;
    pers=1.0*((1.0*encoder_L+1.0*encoder_R)/2-300)/(SPEED_BASE-300);
    if(pers<0)
        pers=0;
    if(pers>1)
        pers=1;
        temp=zx_p*err;
    if(temp>1.0*5*pers)
        temp=1.0*5*pers;
    if(temp<-1.0*5*pers)
        temp=-1.0*5*pers;
    return temp;
    }
void LB_SERVO_ANGLE(float angle)
{
    if(angle<0)
        angle=0;
    if(angle>130)
        angle=130;
    pwm_set_duty(LB_SERVO_MOTOR_PWM, (uint32)LB_SERVO_MOTOR_DUTY(140-angle));
}

void LF_SERVO_ANGLE(float angle)
{
    if(angle<0)
        angle=0;
    if(angle>130)
        angle=130;
    pwm_set_duty(LF_SERVO_MOTOR_PWM, (uint32)LF_SERVO_MOTOR_DUTY(angle+40));
}
void RF_SERVO_ANGLE(float angle)
{
    if(angle<0)
        angle=0;
    if(angle>130)
        angle=130;
    pwm_set_duty(RF_SERVO_MOTOR_PWM, (uint32)RF_SERVO_MOTOR_DUTY(140-angle));
}
void RB_SERVO_ANGLE(float angle)
{
    if(angle<0)
        angle=0;
    if(angle>130)
        angle=130;
    pwm_set_duty(RB_SERVO_MOTOR_PWM, (uint32)RB_SERVO_MOTOR_DUTY(angle+40));
}

void ALL_SERVO_ANGLE(float lbangle,float lfangle,float rbangle,float rfangle)
{
    LB_SERVO_ANGLE(lbangle);
    LF_SERVO_ANGLE(lfangle);
    RB_SERVO_ANGLE(rbangle);
    RF_SERVO_ANGLE(rfangle);

}
void servo_init(void)
{
    pwm_init(LB_SERVO_MOTOR_PWM, LB_SERVO_MOTOR_FREQ, 0);
    pwm_init(LF_SERVO_MOTOR_PWM, LF_SERVO_MOTOR_FREQ, 0);
    pwm_init(RB_SERVO_MOTOR_PWM, RB_SERVO_MOTOR_FREQ, 0);
    pwm_init(RF_SERVO_MOTOR_PWM, RF_SERVO_MOTOR_FREQ, 0);
}

void servo_speed(void)//舵机速度
{
    LF_SPEED_ANGLE+=LF_SPEED;
    LB_SPEED_ANGLE+=LB_SPEED;
    RF_SPEED_ANGLE+=RF_SPEED;
    RB_SPEED_ANGLE+=RB_SPEED;
    if(LF_SPEED_ANGLE>100)
        LF_SPEED_ANGLE=100;
    if(LB_SPEED_ANGLE>100)
        LB_SPEED_ANGLE=100;
    if(RF_SPEED_ANGLE>100)
        RF_SPEED_ANGLE=100;
    if(RB_SPEED_ANGLE>100)
        RB_SPEED_ANGLE=100;
}

void jump(void)
{

    if(jump_flag==1)
    {

            jump_cnt++;
        if(jump_flag2==0)
        {
            LF_SPEED_ANGLE=LB_SPEED_ANGLE=RF_SPEED_ANGLE=RB_SPEED_ANGLE=-40;

            jump_flag3=1;     //用来关闭速度环
        }
        if(jump_cnt>30&&jump_cnt<80)
        {
            LF_SPEED_ANGLE=LB_SPEED_ANGLE=RF_SPEED_ANGLE=RB_SPEED_ANGLE=100;
//            jump_flag4=1;
        }
        if(jump_cnt>80&&jump_cnt<200)
        {
            LF_SPEED_ANGLE=LB_SPEED_ANGLE=RF_SPEED_ANGLE=RB_SPEED_ANGLE=-40;
//            jump_flag4=1;
        }
        if(jump_cnt>200)
        {
            LF_SPEED_ANGLE=LB_SPEED_ANGLE=RF_SPEED_ANGLE=RB_SPEED_ANGLE=0;
            jump_flag2=1;
            if(jump_cnt>300)
            {

             //jump_flag2=0;
             jump_flag3=0;

            }
            if(jump_cnt>600)
            {
                jump_flag2=0;
               jump_cnt=0;
             jump_flag=0;
             jump_flag4=0;

            }
        }
    }


}
void control_s(void )    //舵机控制,两毫秒中断
{
    fuzhuhuan_count++;
    servo_count++;
    g_y=imu660ra_gyro_transition(imu660ra_gyro_y);//翻滚角
//    height_out=servo_Vertical(0,euler_angle.pitch,g_y);
      height_out=servo_Vertical(0,euler_angle.pitch,g_y);//euler_angle.pitch
    compensation_out= servo_roll_compensation(g_y,0);
    fuzhu_Vertical_out= fuzhu_Vertical(Med_Angle,angle,g_x);//直立环
    if(high_mode==1)
    {
        fz_p=fz_p_high;
    }
    else
    {
        fz_p=fz_p_base;//fz_p_base
    }
    if(fuzhuhuan_count>14)
    {
        servo_speed();
        fuzhuhuan_count=0;
        fuzhu_out=fuzhu(Target_Speed,motor_value.receive_left_speed_data,-motor_value.receive_right_speed_data);
        if(high_mode==0)
        {
            high_out-=5;
            if(high_out<45)
                high_out=45;
            height_out=0;
        }
        if(high_mode==1)
        {

            high_out+=5;
            height_out=servo_Vertical(0,euler_angle.pitch,g_y);

            if(height_out>0)
            {
                height_outl=height_out;
                height_outr=-0.6*height_out;

            }
            if(height_out<0)
            {
                height_outr=-height_out;
                height_outl=0.6*height_out;


            }

            if(high_out>high_out_max)
                high_out=high_out_max;
        }
    }

   zhuanxiang_out=zhuanxiang(motor_value.receive_left_speed_data,-motor_value.receive_right_speed_data);
    if(zhuanxiang_out>0)
    {
        zhuanxiang_l=0.8*zhuanxiang_out;
        zhuanxiang_r=-1.5*zhuanxiang_out;
    }
    if(zhuanxiang_out<0)
    {
        zhuanxiang_r=-0.8*zhuanxiang_out;
        zhuanxiang_l=1.5*zhuanxiang_out;
    }

                 //高度补正(基础角度),元素补正，         不知道，         平面保持补正，不知道，      重量补正，    不知道
        ALL_SERVO_ANGLE( high_out+compensation_out+LB_SPEED_ANGLE+fuzhu_out+zhuanxiang_l-height_outl+fuzhu_Vertical_out
                        ,high_out-compensation_out+LF_SPEED_ANGLE-fuzhu_out+zhuanxiang_l-height_outl-fuzhu_Vertical_out
                        ,high_out+compensation_out+RB_SPEED_ANGLE+fuzhu_out+zhuanxiang_r-height_outr+fuzhu_Vertical_out
                        ,high_out-compensation_out+RF_SPEED_ANGLE-fuzhu_out+zhuanxiang_r-height_outr-fuzhu_Vertical_out);





/*
    servo_control_table(4,-euler_angle.roll,&pwm_ph1, &pwm_ph4);
    if(10000 == pwm_ph4 || 10000 == pwm_ph1)       //限幅防卡死
    {
        zf_assert(10000 == pwm_ph4 || 10000 == pwm_ph1);
        pwm_set_duty(LF_SERVO_MOTOR_PWM, SERVO1_MID);
        pwm_set_duty(LB_SERVO_MOTOR_PWM, SERVO2_MID);
        pwm_set_duty(RF_SERVO_MOTOR_PWM, SERVO3_MID);
        pwm_set_duty(RB_SERVO_MOTOR_PWM, SERVO4_MID);
    }
    pwm_set_duty(LF_SERVO_MOTOR_PWM, SERVO1_MID + pwm_ph4);
    pwm_set_duty(LB_SERVO_MOTOR_PWM, SERVO2_MID - pwm_ph1);
    pwm_set_duty(RF_SERVO_MOTOR_PWM, SERVO3_MID - pwm_ph4);
    pwm_set_duty(RB_SERVO_MOTOR_PWM, SERVO4_MID + pwm_ph1);
*/


//    ALL_SERVO_ANGLE(0,0,height_out,height_out);
}
