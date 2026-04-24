#include "zf_common_headfile.h"
#include "pid.h"
//要改的参数
int SPEED_BASE=500;
int SPEED;
int Target_Speed=0;
int suduhuan_count;
int jiaosuduhuan_count;
int jiaoduhuan_count;

float z_p=-380,z_d=-4.4,z_i=6;            //直立环 z_p=-200,z_d=-4.0,z_i=6      220
float s_p=0.0, s_i;             //速度环s_p=0.16      0.09      0.13   0.08   0.05
float Turn_p=45,Turn_d=850;         //转向环，p45,d25
float Turn_p2=2.0,Turn_gkd=-6.6;       //转向环
float a_p=-2.9,a_d=0;              //角速度环 a_p=-3.5,a_d=0
float p_p=200.1,p_d=5;                 //转向角度闭环，过单边桥和横断的时候使用。
int PWM_MAX=5000;
int PWM_MIN=-5000;
float g_x,g_z;                  //俯仰角速度
//float ;              //翻滚角速度,偏航角速度
float g_x_last;             //上一次俯仰角速度
int PWM_out;
int py=50;
int turn_limit=23;
//编码器数据变量
int Encoder_Left,Encoder_Right;
int tracking_way=0;         //循迹方式，0为摄像头循迹，1为角度环循迹
//闭环控制中间变量
int Turn_out,Target_turn,MOTO1,MOTO2;
int Turn_l,Turn_r;
int32 out_speed,out_angle,out_angular,out_zxangle;//输出：直立环，速度环,角速度环,转向角度闭环
float Med_Angle=-0;//平衡时角度值偏移量（机械中值）
int errorout;//转向输出
int errorout_last;//上一次转向输出

//直立环PD控制器
//输入：期望角度、真实角度、角速度
int Vertical(float Med,float Angle,float gyro_Y)
{
    int temp;
    static float inte,l_angle;
/*
    inte+=inte+=-(Angle-Med);;

    if(l_angle<0&&angle>=0)inte=0;
    if(l_angle>=0&&angle<0)inte=0;

    l_angle=Angle;
*/
    temp=(z_p-90.0/60.0*high_out)*(Angle-Med)+z_d*gyro_Y+z_i*inte;
    if(temp>=6800)temp=6800;
    if(temp<=-6800)temp=-6800;
    return temp;
}

//角速度环pd控制器
int angular_velocity(void)
{
    int output;
    output=g_x*a_p+(g_x-g_x_last)*a_d;
    return output;


}

//速度环PI控制器
//输入：期望速度、左编码器、右编码器
int Err,Err_LowOut,temp;
float angle;//陀螺仪的角度
int Velocity(int Target,int encoder_L,int encoder_R)
{
    static int Err_LowOut_last,Encoder_S,l_angle;
    static float a=0.3;
    s_i=s_p/200;
    //1、计算偏差值
    Err=(encoder_L+encoder_R)/2-Target;
    //2、低通滤波
/*
    Err_LowOut_last=Err_LowOut;
    Err_LowOut=(1-a)*Err+a*Err_LowOut_last;
*/
    Err_LowOut=Err;
    //当经过平衡点时积分清零


/*
    if(l_angle<0&&angle>=0)Encoder_S=0;
    if(l_angle>=0&&angle<0)Encoder_S=0;
*/


    l_angle=angle;
    //3、积分
    Encoder_S+=Err_LowOut;
    if(jump_flag3==1)
        Encoder_S=0;
    //4、积分限幅(-10000~10000)
    Encoder_S=Encoder_S>10000?10000:(Encoder_S<(-10000)?(-10000):Encoder_S);
    //5、速度环计算
    temp=(s_p-0.04/60.0*high_out)*Err_LowOut+s_i*Encoder_S;

    if(temp>=20)temp=20;
    if(temp<=-20)temp=-20;

    temp*=150;
    if(jump_flag3==1)
        temp=0;
    return temp;
}

//转向环PD控制器
//输入：角速度、角度值
int Turn(float gyro_Z,int Target_turn)//误差范围+-45
{
    int temp;
/*
    if(Target_turn>turn_limit)//21
        Target_turn=turn_limit;
    if(Target_turn<-turn_limit)
        Target_turn=-turn_limit;
*/

    temp=Turn_p*Target_turn+Target_turn*abs(Target_turn)*Turn_p2+(Target_turn-errorout_last)*Turn_d-Turn_gkd*gyro_Z;

    errorout_last=Target_turn;

    if(temp> 1500)temp= 1500;
    if(temp<-1500)temp=-1500;

    return temp;
}
//转向角度闭环PD控制器
//输入：期望角度、真实角度、角速度
int zx_Vertical(float Med,float Angle,float gyro_Y)
{
    int temp;
    static float inte,l_angle;
/*
    inte+=inte+=-(Angle-Med);;

    if(l_angle<0&&angle>=0)inte=0;
    if(l_angle>=0&&angle<0)inte=0;

    l_angle=Angle;
*/
    temp=p_p*(Angle-Med)+p_d*gyro_Y;
    return temp;
}
//限幅函数
void Limit(int *motoA,int *motoB)
{
    if(*motoA>PWM_MAX)*motoA=PWM_MAX;
    if(*motoA<PWM_MIN)*motoA=PWM_MIN;
    if(*motoB>PWM_MAX)*motoB=PWM_MAX;
    if(*motoB<PWM_MIN)*motoB=PWM_MIN;
}

void Limit_1(int *limit)
{
    if(*limit>PWM_MAX)*limit=PWM_MAX;
    if(*limit<PWM_MIN)*limit=PWM_MIN;
}

//绝对值函数
int abs(int p)
{
    if(p>0)
        return p;
    else
        return -p;
}

void Control(void)  //每隔2ms调用一次
{

    jiaosuduhuan_count++;
    jiaoduhuan_count++;
    suduhuan_count++;

    g_x=imu660ra_gyro_transition(imu660ra_gyro_x); //俯仰角
    g_y=imu660ra_gyro_transition(imu660ra_gyro_y);
    g_z=imu660ra_gyro_transition(imu660ra_gyro_z);
    //2、将数据传入PID控制器，计算输出结果，即左右电机转速值
    angle=euler_angle.roll;
/*
    if(high_mode==1)
    {
        s_p=0.10;//0.08
    }
    else
    {
        s_p=0.0;
    }
*/
//    Target_Speed=outside_velocity(0,motor_value.receive_left_speed_data,-motor_value.receive_right_speed_data);
        if(suduhuan_count>9)
        {
            suduhuan_count=0;

            out_angle=Velocity(Target_Speed,motor_value.receive_left_speed_data,-motor_value.receive_right_speed_data);//直立环的输出角度
        }

        if(jiaoduhuan_count>3)
        {
           out_speed=Vertical(Med_Angle,angle,g_x);       //速度环的输出速度angle/67.03
           jiaoduhuan_count=0;
        }

    if(jiaosuduhuan_count>0)
    {
        out_angular= angular_velocity();
        jiaosuduhuan_count=0;
    }
    if(tracking_way==0)
    {
        Turn_out=Turn(imu660ra_gyro_transition(imu660ra_gyro_z),poserror);    //   poserror            //转向环的输出速度
        out_zxangle=0;
        now_angle=euler_angle.yaw;
    }
    if(tracking_way==1)
    {
         out_zxangle= zx_Vertical(now_angle,euler_angle.yaw,g_z);
         Turn_out=0;
    }

    PWM_out=out_speed+Turn_out+out_angle;//3个环最终的值
    if(Turn_out>0)
    {
        Turn_l=1*Turn_out;//0.9
        Turn_r=-Turn_out;
    }
    if(Turn_out<0)
    {
        Turn_r=-1*Turn_out;//0.9
        Turn_l=Turn_out;
    }


    MOTO1=out_speed+out_angular+out_angle+out_zxangle-Turn_r;//+Turn_out-Turn_r
    MOTO2=out_speed+out_angular+out_angle-out_zxangle-Turn_l;//-Turn_out-Turn_l
    Limit(&MOTO1,&MOTO2);//限幅
    small_driver_set_duty(-MOTO1,MOTO2);
}
