/*
 * motor.c
 *
 *  Created on: 2024年3月1日
 *      Author: Kurumi
 */


#include <image.h>
#include "zf_common_headfile.h"
#include "motor.h"
#include "math.h"
#include "steer.h"
#include "PID.h"
//extern int rukubz;
extern SystemDatatypdef SystemData;
extern ImageStatustypedef ImageStatus;                //图像的全局变量
extern PID_Datatypedef SteerPIDdata;    //舵机的PID参数
extern ImageFlagtypedef ImageFlag;
extern int8 menu_flag;
extern int8 menu_flag_1;
extern int8 menu_flag_;


extern int16 bianmaqi_rings;  //编码器圆环累加



uint16 fan_pwm_Straight;      //负压风扇基础pwm
uint16  fan_pwm_out1;         //负压风扇1输出
uint16  fan_pwm_out2;         //负压风扇2输出
uint16  fan_pwm_out;          //负压风扇输出
float   fan_pwm_coff;         //非直道负压风扇系数
float   fan_pwm_coff_;         //非直道负压风扇系数
uint16 fan_pwm_out1_max;       //负压风扇1最大输出
uint16 fan_pwm_out2_max;       //负压风扇2最大输出


int fan_pin = 50;                      //负压风扇频率


uint8 gyro_data_flag;    //陀螺仪启用标志位
int  Angle;//以PWM差的形式表示打角，>0右转，小于0左转


#define fan_pin1 ATOM0_CH4_P02_4        //左负压风扇引脚
#define fan_pin2 ATOM1_CH6_P02_6        //右负压风扇引脚


//    ATOM0_CH5_P02_5 ATOM1_CH5_P02_5
//    ATOM1_CH6_P02_6 ATOM0_CH6_P02_6
//    ATOM0_CH4_P02_4 ATOM1_CH4_P02_4
//    ATOM0_CH7_P02_7 ATOM1_CH7_P02_7


extern

#define PWM_CH1             ATOM0_CH0_P21_2 //左轮正
#define PWM_CH2             ATOM0_CH1_P21_3 //左轮反
#define PWM_CH3             ATOM0_CH2_P21_4 //右轮正
#define PWM_CH4             ATOM0_CH3_P21_5 //右轮反

int rukujishu;

int run_flag = 0;//发车标志位初始化


int   bmq_record=0;//记录编码器

int16 speed1, speed2;

int S3010_Duty;



int R_MT_PWM_Ctrl;
int L_MT_PWM_Ctrl;


extern float icm_x;                    //陀螺仪相关
extern float icm_y;
extern float icm_z;
extern float icm_x_last;
extern float icm_y_last;
extern float icm_z_last;
extern float icm_x_out;
extern float icm_y_out;
extern float icm_z_out;


//速度决策2
uint16 Speed_Goal;
uint16 Speed_Goal_;

//左轮
float Speed_Goal_l;//目标速度
int16 Speed_Encoder_l;//当前速度

int16 Speed_Encoder_l_;


float Speed_P_l,Speed_I_l,Speed_D_l;
int16 Speed_Erro_l;//当前误差

extern int16 Speed_PID_OUT_l;//占空比输出
int16 Speed_Lasterro_l;//上次误差
int16 Speed_Preverro_l;//上上次误差

//右轮
float Speed_Goal_r;//目标速度
int16 Speed_Encoder_r;//当前速度

int16 Speed_Encoder_r_;


float Speed_P_r,Speed_I_r,Speed_D_r;
int16 Speed_Erro_r;//当前误差

extern int16 Speed_PID_OUT_r;//占空比输出
int16 Speed_Lasterro_r;//上次误差
int16 Speed_Preverro_r;//上上次误差


int Progrise = 400;                    //调谐 跃升  Let's give you power！

void MOTOR_init()
{
    //pwm_init(ATOM0_CH7_P02_7, 50, 1000);    新库
    pwm_init(ATOM0_CH1_P21_3, 17000,0);//左轮初始化
    gpio_init(P21_2, GPO, 1, GPO_PUSH_PULL);   // 设置DRV方向引脚为为推挽输出
    pwm_init(ATOM1_CH3_P21_5, 17000,0);//右轮初始化
    gpio_init(P21_4, GPO, 1, GPO_PUSH_PULL);// 设置DRV方向引脚为为推挽输出
}

void Encoder_init()
{
    encoder_dir_init(TIM5_ENCODER,TIM5_ENCODER_CH1_P10_3,TIM5_ENCODER_CH2_P10_1);
    encoder_dir_init(TIM2_ENCODER,TIM2_ENCODER_CH1_P33_7,TIM2_ENCODER_CH2_P33_6);
   // encoder_init_dir(TIM1_ENCOEDER, TIM1_CH1_ENCOEDER_E9, TIM1_CH2_ENCOEDER_E11);  //编码器左初始化
    //encoder_init_dir(TIM9_ENCOEDER, TIM9_CH1_ENCOEDER_D9, TIM9_CH2_ENCOEDER_D11);  //编码器右初始化
}


void speed_measure()      //电机速度控制
{

                  speed1 = encoder_get_count(TIM2_ENCODER);     // 定时器取值
                  encoder_clear_count(TIM2_ENCODER);                    // 定时器清空
                  speed2 =  -encoder_get_count(TIM5_ENCODER);     // 定时器取值
                  encoder_clear_count(TIM5_ENCODER);

              Speed_Encoder_l_= 0.08f*speed1 + 0.92f*Speed_Encoder_l_;
              Speed_Encoder_r_= 0.08f*speed2 + 0.92f*Speed_Encoder_r_;
//              Speed_Encoder_l_= 0.28f*speed1 + 0.72f*Speed_Encoder_l_;
//              Speed_Encoder_r_= 0.28f*speed2 + 0.72f*Speed_Encoder_r_;
//              Speed_Encoder_l_= speed1;
//              Speed_Encoder_r_= speed2;
              printf("%d,%d,%d\n",speed1,speed2,Speed_Goal);  //vofa上位机
//           Speed_decision();   // //驱动二套代码赋值     这个优先级高于flash赋值
           straight_speed();   // 直道加速    开了记得改参数
           Dif_spd_calc(Speed_Goal,&Speed_Goal_l,&Speed_Goal_r);  //差速
           banmaxian();

           if(ImageStatus.straight_acc == 0)  //非直道
//           if(1)
           {
               Progrise = 0;
               PID_play();
           }
           else
           {
               Progrise ++;
               speed_PIDL();   // Road to Glory has to Lead to Growin' path to change one to two
           }
}

float  Dif_spd_rat; //差速比    6.3
float  tffkz;
float  bend_Dif_spd_rat; //差速
void Speed_decision()    //驱动二套代码赋值     这个优先级高于flash赋值
{

    Speed_P_r = Speed_P_l;
    Speed_I_r = Speed_I_l;
    Speed_D_r = Speed_D_l;
    Speed_P_l = 80;       //130 30 5    100 2 5
    Speed_I_l = 20;
    Speed_D_l = 0;      //70 7.3 20
    Speed_P_r = 80;
    Speed_I_r = 20;
    Speed_D_r = 0;


}
void speed_PIDL(void)
{
    Speed_Encoder_l  = speed1;//编码器采集当前速度
    Speed_Encoder_r  = speed2;//编码器采集当前速度

    if(Progrise<60)          //弯道切换直道，强行加速
    {
        Speed_Encoder_l = 0;
        Speed_Encoder_r = 0;
    }
//    Speed_Encoder_l_= 0.08f*Speed_Encoder_l + 0.92f*Speed_Encoder_l_;
   // gpt12_clear(COUNTER1_TIM);
//    Speed_Goal_l=filter_l(Speed_Goal_l);        //滤波
//    Speed_Erro_r=filter_l(Speed_Erro_r);        //滤波
//    if(speeddd>2000||SystemData.Stop == 1) //斑马线停车、初始停车
//        {
//            Speed_Goal_l= 0 ;
//            Speed_Goal_r= 0 ;
//        }
//        if(speeddd>100)     //斑马线累加之后再停负压风扇
//        {
//            fan_pwm_Straight = 500;         //直道负压风扇占空比给0
//        }
        if(SystemData.Stop == 1) //斑马线停车、初始停车
        {
            Speed_Goal_l= 0 ;
            Speed_Goal_r= 0 ;
        }
        if(speeddd>100)     //斑马线累加之后再停负压风扇
        {
            Speed_Goal_l= 0 ;
            Speed_Goal_r= 0 ;

        }
        if(speeddd>700)
        {
            fan_pwm_Straight = 500;
        }

    Speed_Erro_l     =  Speed_Goal_l - Speed_Encoder_l;   //编码器反馈目标值与当前的差值
    Speed_Erro_r     =  Speed_Goal_r - Speed_Encoder_r;   //编码器反馈目标值与当前的差值

//        Speed_Erro_l = filter_l(Speed_Erro_l);//不知道有没有用的滤波




    Speed_PID_OUT_l += (Speed_P_l * (Speed_Erro_l - Speed_Lasterro_l)  +
                      Speed_I_l *  Speed_Erro_l  +
                      Speed_D_l * (Speed_Erro_l - 2 * Speed_Lasterro_l + Speed_Preverro_l));
    Speed_PID_OUT_r += (Speed_P_r * (Speed_Erro_r - Speed_Lasterro_r)  +
                          Speed_I_r *  Speed_Erro_r  +
                          Speed_D_r * (Speed_Erro_r - 2 * Speed_Lasterro_r + Speed_Preverro_r));



    if(Speed_PID_OUT_l<-8888)
        Speed_PID_OUT_l = -8888;
    if(Speed_PID_OUT_l>8888)
        Speed_PID_OUT_l = 8888;
    Speed_Preverro_l =  Speed_Lasterro_l;
    Speed_Lasterro_l =  Speed_Erro_l;

    if(Speed_PID_OUT_r<-8888)
            Speed_PID_OUT_r = -8888;
        if(Speed_PID_OUT_r>8888)
            Speed_PID_OUT_r = 8888;
        Speed_Preverro_r =  Speed_Lasterro_r;
        Speed_Lasterro_r =  Speed_Erro_r;
//     Speed_PID_OUT_l = 650;//固定pwm值


        pwm_set_duty(fan_pin1,fan_pwm_Straight);   //负压风扇1输出 ATOM0_CH5_P02_5
        pwm_set_duty(fan_pin2,fan_pwm_Straight);   //负压风扇2输出 ATOM1_CH6_P02_6



    if(Speed_PID_OUT_l>=0)
    {

                gpio_set_level(P21_4, 1);
                pwm_set_duty(ATOM1_CH3_P21_5,Speed_PID_OUT_l);//正转
//P02_6
//ATOM0_CH7_P02_7
    }
    else
    {

                gpio_set_level(P21_4, 0);
                pwm_set_duty(ATOM1_CH3_P21_5,-Speed_PID_OUT_l);//反转
//P02_4
//ATOM0_CH5_P02_5
    }



        if(Speed_PID_OUT_r>=0)
        {

                    gpio_set_level(P21_2, 1);
                    pwm_set_duty(ATOM0_CH1_P21_3,Speed_PID_OUT_r); //正转
        }
        else
        {

                   gpio_set_level(P21_2,0);
                   pwm_set_duty(ATOM0_CH1_P21_3,-Speed_PID_OUT_r); //反转
        }
    }

//差速

extern float err_s; //主动偏差系数与被动偏差系数
float fc = 20.5f;      //截止频率
float fc_out = 1.0f;   //截止频率
float Ts = 0.005f;     //采样周期
float pi = 3.14159f;   //π
float alpha = 0;       //滤波系数
float alpha_out = 0;   //滤波系数

/************************ 滤波器初始化 alpha *****************************/
void low_pass_filter_init(void)                                                //数值初始化
{
    float b = 2.0 * pi * fc * Ts;
    alpha = b / (b + 1);

    b = 2.0 * pi * fc_out * Ts;
    alpha_out = b / (b + 1);
}

float filter_r(float value)                                                     //右轮滤波
{
  static float out_last = 0; //上一次滤波值
  float out;

  /***************** 如果第一次进入，则给 out_last 赋值 ******************/
  static char fisrt_flag = 1;
  if (fisrt_flag == 1)
  {
    fisrt_flag = 0;
    out_last = value;
  }

  /*************************** 一阶滤波 *********************************/
  out = out_last + alpha * (value - out_last);
  out_last = out;

  return out;
}

float filter_l(float value)                                                //左轮滤波
{
  static float out_last = 0; //上一次滤波值
  float out;

  /***************** 如果第一次进入，则给 out_last 赋值 ******************/
  static char fisrt_flag = 1;
  if (fisrt_flag == 1)
  {
    fisrt_flag = 0;
    out_last = value;
  }

  /*************************** 一阶滤波 *********************************/
  out = out_last + alpha * (value - out_last);
  out_last = out;

  return out;
}






#define LTA_Long_Turning_Arc                  (0.0067425f)         //车辆转弯弧长



void Dif_spd_calc(int tar_speeed,float* L_speed,float* R_speed)//
{
    float err;
    err = Speed_Goal - (speed2 +speed1)/2;   //被动差速

    uint16  Differential_storage=0;
    int Pulse_differential;
    float SteerTurnDuty, AcquareSpeed, SteerTurnAngle, ErrorSpeed, SteerDuty;
if(ImageStatus.Det_True-39>0)
        SteerTurnDuty = ImageStatus.Right_Line;
else
        SteerTurnDuty = ImageStatus.Left_Line;

    SteerDuty = fabs(SteerTurnDuty);
    if(SteerDuty > 20.0)      SteerDuty = 20.0;

    AcquareSpeed = ((Speed_Encoder_l_ + Speed_Encoder_r_) / 2)*80;
    if(ImageStatus.Road_type == LeftCirque||ImageStatus.Road_type== RightCirque)
        bend_Dif_spd_rat = /*icm_z_out * 1.1 */tffkz;     //检测到圆环把陀螺仪值赋给差速
    else
        bend_Dif_spd_rat = Dif_spd_rat;
    Differential_storage=bend_Dif_spd_rat;
    SteerTurnAngle = Differential_storage * SteerDuty;

    ErrorSpeed = LTA_Long_Turning_Arc * SteerTurnAngle * AcquareSpeed;
    Pulse_differential = ErrorSpeed / 80 ;

    if(SteerTurnDuty < 0)
    {
        Pulse_differential = -Pulse_differential;
    }
//    *L_speed=tar_speeed + err*err_s + Pulse_differential;             //差速一加一减
//    *R_speed=tar_speeed + err*err_s - Pulse_differential;
    if(ImageStatus.Det_True-39>0)                              //差速单减
    {
        *L_speed=tar_speeed + err*err_s;
        *R_speed=tar_speeed + err*err_s - Pulse_differential;
    }
    if(ImageStatus.Det_True-39<0)
    {
        *L_speed=tar_speeed + err*err_s + Pulse_differential;
        *R_speed=tar_speeed + err*err_s;
    }
    if(ImageStatus.Det_True-39==0)
        {
            *L_speed=tar_speeed + err*err_s;
            *R_speed=tar_speeed + err*err_s;
        }
}



int speeddd;

void PID_play()
{

    pid_Speed_l.target_val= Speed_Goal_l ;
    pid_Speed_r.target_val= Speed_Goal_r ;


//    pid_Speed_l.target_val= 130 ;
//    pid_Speed_r.target_val= 130 ;

//    if(speeddd>100||SystemData.Stop == 1) //斑马线停车、初始停车
    if(SystemData.Stop == 1) //斑马线停车、初始停车
        {
//            pid_Speed_l.target_val= 0 ;
//            pid_Speed_r.target_val= 0 ;
        }
        if(speeddd>700)     //斑马线累加之后再停负压风扇
            {
        //        pid_Speed_l.target_val= 0 ;
        //        pid_Speed_r.target_val= 0 ;

                pid_Speed_l.target_val= 0 ;
                pid_Speed_r.target_val= 0 ;
                if(speeddd>1000)
                {
                    fan_pwm_Straight = 500;
                    pwm_set_duty(fan_pin1,500);   //负压风扇1输出 ATOM0_CH5_P02_5
                    pwm_set_duty(fan_pin2,500);   //负压风扇2输出 ATOM1_CH6_P02_6
        //            pid_Speed_l.target_val= 0 ;
        //            pid_Speed_r.target_val= 0 ;
                }
        //        fan_pwm_out2 = 0;
            }

     pid_Speed_l.target_val=filter_l(pid_Speed_l.target_val);        //滤波
     pid_Speed_r.target_val=filter_r(pid_Speed_r.target_val);



     PID_Motor(&pid_Speed_l,Speed_Encoder_l_);
     PID_Motor(&pid_Speed_r,Speed_Encoder_r_);

     L_MT_PWM_Ctrl=(int32)pid_Speed_l.actual_val;
     R_MT_PWM_Ctrl=(int32)pid_Speed_r.actual_val;



//     fan_pwm_coff_ = fan_pwm_coff;   //转接

//     if(ImageStatus.straight_acc == 1)      //直道系数变为0，只保留基础值
//             fan_pwm_coff_ = 0;
//
//     fan_pwm_out1 = (L_MT_PWM_Ctrl - 120) * fan_pwm_coff_ + fan_pwm_Straight;    //左风扇pwm输出
//     fan_pwm_out2 = (R_MT_PWM_Ctrl - 120) * fan_pwm_coff_ + fan_pwm_Straight;    //右风扇pwm输出
//
//     if(fan_pwm_out1>fan_pwm_out1_max)
//         fan_pwm_out1 = fan_pwm_out1_max;   //负压风扇限幅
//     if(fan_pwm_out2>fan_pwm_out2_max)
//         fan_pwm_out2 = fan_pwm_out2_max;

     pwm_set_duty(fan_pin1,fan_pwm_Straight);   //负压风扇1输出 ATOM0_CH5_P02_5
     pwm_set_duty(fan_pin2,fan_pwm_Straight);   //负压风扇2输出 ATOM1_CH6_P02_6

//     fan_speedcontrl();     //负压风扇速度决策

                   if(L_MT_PWM_Ctrl>=0)
                       {

                                   gpio_set_level(P21_4, 1);
                                   pwm_set_duty(ATOM1_CH3_P21_5,L_MT_PWM_Ctrl);//正转
                   //P02_6
                   //ATOM0_CH7_P02_7
                       }
                       else
                       {

                                   gpio_set_level(P21_4, 0);
                                   pwm_set_duty(ATOM1_CH3_P21_5,-L_MT_PWM_Ctrl);//反转
                       }


                   if(R_MT_PWM_Ctrl>=0)
                   {

                               gpio_set_level(P21_2, 1);
                               pwm_set_duty(ATOM0_CH1_P21_3,R_MT_PWM_Ctrl); //正转
                   }
                   else
                   {

                              gpio_set_level(P21_2,0);
                              pwm_set_duty(ATOM0_CH1_P21_3,-R_MT_PWM_Ctrl); //反转
                   }
}

void fan_speedcontrl()    //负压风扇速度决策
{
//         fan_pwm_coff_ = fan_pwm_coff;   //转接

         if(ImageStatus.straight_acc == 1)      //直道系数变为0，只保留基础值
                 fan_pwm_coff_ = 0;
         else
                 fan_pwm_coff_ = 1;

         fan_pwm_out1 = (L_MT_PWM_Ctrl - Speed_Goal_) * fan_pwm_coff_ + 800;    //左风扇pwm输出
         fan_pwm_out2 = (R_MT_PWM_Ctrl - Speed_Goal_) * fan_pwm_coff_ + 800;    //右风扇pwm输出

         if(fan_pwm_out1>fan_pwm_out1_max)
             fan_pwm_out1 = fan_pwm_out1_max;   //负压风扇限幅
         if(fan_pwm_out2>fan_pwm_out2_max)
             fan_pwm_out2 = fan_pwm_out2_max;

    pwm_set_duty(fan_pin1,fan_pwm_out1);   //负压风扇1输出 ATOM0_CH5_P02_5
    pwm_set_duty(fan_pin2,fan_pwm_out2);   //负压风扇2输出 ATOM1_CH6_P02_6
}

int isp114;
int isp114_yu = 3;

typedef struct {
    uint8_t key1_down;
    uint8_t key1_long;
    uint8_t key1_short;
    uint8_t key1_out;
    uint8_t key2_down;
    uint8_t key2_long;
    uint8_t key2_short;
    uint8_t key2_out;
    uint8_t key3_down;
    uint8_t key3_long;
    uint8_t key3_short;
    uint8_t key3_out;
    uint8_t key4_down;
    uint8_t key4_long;
    uint8_t key4_short;
    uint8_t key4_out;
    uint8_t key1;
    uint8_t key2;
    uint8_t key3;
    uint8_t key4;


} key_t;

key_t Button;
int8 key_time = 10;
void key_readd(void)
{
    if(
              Button.key1_short == 1||Button.key1_long == 1
            ||Button.key2_short == 1||Button.key2_long == 1
            ||Button.key3_short == 1||Button.key3_long == 1
            ||Button.key4_short == 1||Button.key4_long == 1
    )                       //检测到短按或者长按后进行累加
    {
        key_time++;
    }
    if(key_time >= 5)   //空出170ms后再进行判断
    {
        if(gpio_get_level(P22_0) == 0)
            {
                Button.key1_out++;   //输出标志位
                Button.key1++;
            }
            if(Button.key1>0)    //如果之前没有按按键则不会检测
            {
                if(gpio_get_level(P22_0) == 1)
                    {
                        Button.key1 = 0;

                    }
                if(Button.key1 >= 40 && Button.key1_out >= 40)   //长按
                            {
                                Button.key1_long = 1;
                                Button.key1_out = 0;
                                Button.key1 = 0;

//                                SystemData.Stop = 1;//停车标志位
//                                run_flag = 0;

//                                speeddd=0;  //编码器停车积分
//                                ImageStatus.Zebra_read = 0;//斑马线标志位清0
//                                SystemData.Stop = 0;//停车标志位
//                                run_flag = 1;

                                key_time = 0;

                            }
                if(Button.key1 < 5 && Button.key1_out >= 5)   //短按
                            {
                                Button.key1_short = 1;
                                Button.key1_out = 0;
                                Button.key1 = 0;

                                menu_long_back();   //菜单退出标志位
//                                                                speeddd=0;  //编码器停车积分
//                                                                ImageStatus.Zebra_read = 0;//斑马线标志位清0
//                                                                SystemData.Stop = 0;//停车标志位
//                                                                run_flag = 1;

                                key_time = 0;
                            }
            }

            if(gpio_get_level(P22_1) == 0)
                                   {
                                       Button.key2_out++;   //输出标志位
                                       Button.key2++;
                                   }
                                   if(Button.key2>0)    //如果之前没有按按键则不会检测
                                   {
                                       if(gpio_get_level(P22_1) == 1)
                                           {
                                               Button.key2 = 0;

                                           }
                                       if(Button.key2 >= 40 && Button.key2_out >= 40)   //长按
                                                   {
                                                       Button.key2_long = 1;
                                                       Button.key2_out = 0;
                                                       Button.key2 = 0;

                                                       speeddd=0;  //编码器停车积分
                                                       ImageStatus.Zebra_read = 0;//斑马线标志位清0
                                                       SystemData.Stop = 0;//停车标志位
                                                       run_flag = 1;

                                                       key_time = 0;

                                                   }
                                       if(Button.key2 < 5 && Button.key2_out >= 5)   //短按
                                                   {
                                                       Button.key2_short = 1;
                                                       Button.key2_out = 0;
                                                       Button.key2 = 0;

                                                       menu_jump();

                                                       key_time = 0;
                                                   }
                                   }

                                   if(gpio_get_level(P22_2) == 0)
                                               {
                                                   Button.key3_out++;   //输出标志位
                                                   Button.key3++;
                                               }
                                               if(Button.key3>0)    //如果之前没有按按键则不会检测
                                               {
                                                   if(gpio_get_level(P22_2) == 1)
                                                       {
                                                           Button.key3 = 0;

                                                       }
                                                   if(Button.key3 >= 40 && Button.key3_out >= 40)   //长按
                                                               {
                                                                   Button.key3_long = 1;
                                                                   Button.key3_out = 0;
                                                                   Button.key3 = 0;




                                                                   key_time = 0;

                                                               }
                                                   if(Button.key3 < 5 && Button.key3_out >= 5)   //短按
                                                               {
                                                                   Button.key3_short = 1;
                                                                   Button.key3_out = 0;
                                                                   Button.key3 = 0;
//                                                                   isp114_yu++;
//                                                                   isp114 = isp114_yu % 3;
                                                                   menu_fall();

                                                                   key_time = 0;  //清0放在判断后面


                                                               }
                                               }

                                               if(gpio_get_level(P22_3) == 0)
                                                           {
                                                               Button.key4_out++;   //输出标志位
                                                               Button.key4++;
                                                           }
                                                           if(Button.key4>0)    //如果之前没有按按键则不会检测
                                                           {
                                                               if(gpio_get_level(P22_3) == 1)
                                                                   {
                                                                       Button.key4 = 0;

                                                                   }
                                                               if(Button.key4 >= 40 && Button.key4_out >= 40)   //长按
                                                                           {
                                                                               Button.key4_long = 1;
                                                                               Button.key4_out = 0;
                                                                               Button.key4 = 0;

                                                                               if(menu_flag_1 == 9 && menu_flag_ == 3)
                                                                               {
//                                                                                   flash_set_buffer_preset_0(1);
                                                                               }
                                                                               menu_long_back();

                                                                               key_time = 0;

                                                                           }
                                                               if(Button.key4 < 5 && Button.key4_out >= 5)   //短按
                                                                           {
                                                                               Button.key4_short = 1;
                                                                               Button.key4_out = 0;
                                                                               Button.key4 = 0;

                                                                               menu_back();

                                                                               key_time = 0;
                                                                           }
                                                           }
    }


}
void key_Start_run(void)
{

    if(gpio_get_level(P22_0) == 0 && run_flag == 0)   //发车
    {

        SystemData.Stop = 0;//停车标志位
        run_flag = 1;
        system_delay_ms(100);

    }
    if(gpio_get_level(P22_1) == 0 && run_flag == 1)   //停车&重置停车标志位
    {
        SystemData.Stop = 1;//停车标志位
        run_flag = 0;
        speeddd=0;  //编码器停车积分
        ImageStatus.Zebra_read = 0;//斑马线标志位清0
        system_delay_ms(100);
    }
    if(Button.key3_short == 1)      //屏幕1数据显示
       {
        isp114_yu++;
        isp114 = isp114_yu % 3;//根据余数判断显示的内容
        key_time = 0;
        Button.key3_short =0;
       }
}
void banmaxian()
{
    if (ImageStatus.Zebra_read == 1)
    {
//        SystemData.Stop = 1;

            speeddd += (speed1+speed2)/2;

    }
//    if(
//            ImageFlag.image_element_rings_flag == 1||
////            ImageFlag.image_element_rings_flag == 2||
//            ImageFlag.image_element_rings_flag == 3
//     )
//    {
//        bianmaqi_rings += abs((Speed_Encoder_l_+Speed_Encoder_r_)/2);
//    }
//    if(
//              icm_x>140
//            ||icm_x<-140
//            )
//    {
//        speeddd += (Speed_Encoder_l_+Speed_Encoder_r_)/2;
//    }
}



void gyro_data()     //陀螺仪数据获取
{
//    if(ImageStatus.Road_type == LeftCirque||ImageStatus.Road_type== RightCirque)
        gyro_data_flag = 1;
    if(gyro_data_flag)
    {
        icm20602_get_gyro();   //陀螺仪数据获取
        icm_x +=  abs(icm20602_gyro_transition(icm20602_gyro_x)*0.02);
        icm_y +=  abs(icm20602_gyro_transition(icm20602_gyro_y)*0.02);
        icm_z +=  abs(icm20602_gyro_transition(icm20602_gyro_z)*0.02);

        icm_x_last = icm_x;
        icm_y_last = icm_y;
        icm_z_last = icm_z;

        icm_x_out =icm_x- icm_x_last;
        icm_y_out =icm_y- icm_y_last;
        icm_z_out =icm_z- icm_z_last;
    }
}


void fan_init()
{

//           pwm_init(fan_pin1,fan_pin,fan_pwm_Straight);//负压风扇1初始化   ATOM0_CH4_P02_4
//           pwm_init(fan_pin2,fan_pin,fan_pwm_Straight);//负压风扇2初始化  ATOM0_CH7_P02_7
           pwm_init(fan_pin1,fan_pin,fan_pwm_Straight);//负压风扇1初始化   ATOM0_CH4_P02_4
           pwm_init(fan_pin2,fan_pin,fan_pwm_Straight);//负压风扇2初始化  ATOM0_CH7_P02_7


}


void seepd_dif()        //一加一减
{
    float y,x;

    if(S3010_Duty >= steer_middle)//左转
    {
        x = (S3010_Duty - steer_middle) * Left_Speed_Co;       //这个值越大差速越大
        if(x > 100)
        {
            x = 100;
        }
        y = -0.014344 + 0.0078637*x - 0.000014484*x*x;
        Speed_Goal_r = Speed_Goal * (1 + 0.3000 * y);
        Speed_Goal_l = Speed_Goal * (1 - 0.3000 * y);
    }
    else if(S3010_Duty < steer_middle)//右转
    {
        x = (steer_middle - S3010_Duty) * Right_Speed_Co;
        if(x>100)
        {
            x = 100;
        }
        y = -0.014344 + 0.0078637*x - 0.000014484*x*x;
        Speed_Goal_l = Speed_Goal * (1 + 0.3000 * y);
        Speed_Goal_r = Speed_Goal * (1 - 0.3000 * y);
    }
}

void seepd_dif_two()       //单减
{
    float y,x;

    if(S3010_Duty >= steer_middle)//左转
    {
        x = (S3010_Duty - steer_middle) * Left_Speed_Co;
        if(x > 100)
        {
            x = 100;
        }
        y = -0.014344 + 0.0078637*x - 0.000014484*x*x;
        Speed_Goal_r = Speed_Goal;// * (1 + 0.3875 * y)*0.9;// * (1 + 0.3875 * y);
        Speed_Goal_l = Speed_Goal * (1 - 0.3875 * y);
    }
    else if(S3010_Duty < steer_middle)//右转
    {
        x = (steer_middle - S3010_Duty)* Right_Speed_Co;
        if(x>100){
            x = 100;
        }
        y = -0.014344 + 0.0078637*x - 0.000014484*x*x;
        Speed_Goal_l = Speed_Goal;//* (1 + 0.3875 * y)*0.9;//+ (Speed_Goal * (0.3875 * y)* Disf); //* (1 + 0.3875 * y);
        Speed_Goal_r = Speed_Goal * (1 - 0.3875 * y);
    }
}

