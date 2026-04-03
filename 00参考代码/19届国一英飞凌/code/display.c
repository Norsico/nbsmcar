/*
 * tft180.c
 *
 *  Created on: 2024年3月1日
 *      Author: Kurumi
 */

#include <display.h>
#include <image.h>
#include "zf_common_headfile.h"
#include "motor.h"
#include "Fuzzy.h"

extern int sanchachazhi;
extern ImageStatustypedef ImageStatus;                //图像的全局变量
extern PID_Datatypedef SteerPIDdata;    //舵机的PID参数
extern int16 speed1;
extern int16 speed2;
extern uint16 adcleft;
extern uint16 adcmiddle;
extern uint16 adcright;
extern uint16 adcsum;
extern SystemDatatypdef SystemData;
extern int16 Speed_Goal_l;
extern int16 Speed_Goal_r;
extern int ycircle;
extern int xcircle;
extern int axcircle;
extern int aycircle;
extern int bxcircle;
extern int bycircle;
extern float ang_l;
extern int outacc_flag;
extern int Speed_Goal;
extern float bend_Dif_spd_rat;
extern int Speed_Goal_;
extern int isp114;
extern int need_display_menu = 0;
extern uint16 Speed_Goal_add;
extern uint32 exposure_time;
extern int run_flag;
extern uint8 Element_on;
extern uint8 banmaxian_on;
extern int8 preset_read;
extern float icm_x;
extern float icm_y;
extern float icm_z;
extern int16 bianmaqi_rings;
extern uint16 keep_rings_help;//圆环辅助标志位
extern uint16 fan_pwm_Straight;      //负压风扇基础pwm


extern uint8 motor_choose;   //电机方案选择

uint8 menu_flag = 1;//菜单页面标志位
uint8 menu_flag_1 = 1;//二级菜单累加标志位
uint8 menu_flag_; //二级菜单页面标志位
uint8 menu_flag_image;//图像菜单按键累加标志位
uint8 menu_flag_image1;//图像菜单按键累加标志位1
uint8 menu_image_1; //图像数据累加标志位1
uint8 menu_speed_1;  //速度数据累加标志位1
uint8 menu_rings;//圆环选择标志位
uint8 menu_rings_flag;//圆环选择标志位
uint8 half_wide_;   //赛道半宽计数

void display()        //图像显示
{


    if(menu_flag_image == 1 && menu_flag_ == 2)
    ips114_1();
    if(menu_flag_image == 2 && menu_flag_ == 2)
    ips114_2();
    if(menu_flag_image == 3 && menu_flag_ == 2)
    ips114_3();
    if(menu_flag_image == 4 && menu_flag_ == 2)
        ips114_4();
}


void ips114_1()
{
    ips114_show_gray_image(80,0,Pixle[0],LCDW,LCDH,LCDW,LCDH,1);//二值化

//    ips114_show_gray_image(80,0,mt9v03x_image[0],LCDW,LCDH,LCDW,LCDH,0);
//
    drawleftline();     //画左边线
    drawrightline();    //画右边线
    drawcenterline();   //画中线
    ips114_show_string(85,60,"cha");

     ips114_show_int(115,60,ImageStatus.Det_True-39,3);  //图像偏差显示

     ips114_show_int(205,0,speed1,3);           //编码器速度显示
     ips114_show_string(162,0,"left");
     ips114_show_int(205,30,speed2,3);
     ips114_show_string(162,30,"right");

     ips114_show_string(0,60,"speed");
     ips114_show_int(50,60,Speed_Goal,3);         //目标速度
     ips114_show_float(50,80,bend_Dif_spd_rat,3,1);


     ips114_show_int(0,100,SystemData.Stop,1);    //停车标志位
    //    ips114_show_int(50,100,speeddd,5);           //斑马线编码器累加


     ips114_show_int(105,100,ImageStatus.Right_Line,3);
     ips114_show_int(70,100,ImageStatus.Left_Line,3);

//     ips114_show_int(140,100,Straight_Judge(2, 25, 45),3);
     ips114_show_int(130,80,Cirque_or_Cross(1,ImageStatus.Left_Line),3); //测试赛道圆环下白点数
     ips114_show_int(160,80,Cirque_or_Cross(2,ImageStatus.Right_Line),3);
//     ips114_show_int(140,100,icm_x,4);
//     ips114_show_float(140,100,huandaopanduan(1,50,30,15),4,2);

//     ips114_show_int(190,100,bianmaqi_rings,5);    //前瞻行中心线坐标




}

void ips114_2()
{
    //    ips114_displayimage03x(mt9v03x_image[0],LCDW,LCDH);//灰度
    ips114_displayimage03x(mt9v03x_image[0],LCDW,LCDH);//灰度

//    drawleftline();     //画左边线
//    drawrightline();    //画右边线
//    drawcenterline();   //画中线
//    drawoffline();      //画图像顶边/图像中心线
//    drawtowpointUP();   //画前瞻范围上线
//    drawtowpointDOWN(); //画前瞻范围下线
 ips114_show_string(85,60,"cha");
 ips114_show_int(115,60,ImageStatus.Det_True-39,3);  //图像偏差显示

 ips114_show_int(205,0,speed1,3);           //编码器速度显示
 ips114_show_string(162,0,"left");
 ips114_show_int(205,30,speed2,3);
 ips114_show_string(162,30,"right");

 ips114_show_string(0,60,"speed");
 ips114_show_int(50,60,Speed_Goal,3);         //目标速度

 ips114_show_int(0,100,SystemData.Stop,1);    //停车标志位
//    ips114_show_int(50,100,speeddd,5);           //斑马线编码器累加
}
void ips114_3()
{
//    ips114_show_gray_image(80,0,Pixle[0],LCDW,LCDH,LCDW,LCDH,1);//二值化
//        ips114_displayimage03x(mt9v03x_image[0],LCDW,LCDH);//灰度
        ips114_show_gray_image(80,0,Pixle[0],LCDW,LCDH,LCDW,LCDH,1);//二值化
//
        drawleftline();     //画左边线
        drawrightline();    //画右边线
        drawcenterline();   //画中线
        drawoffline();      //画图像顶边/图像中心线
    //    drawtowpointUP();   //画前瞻范围上线
//    //    drawtowpointDOWN(); //画前瞻范围下线
     ips114_show_string(85,60,"cha");
     ips114_show_int(115,60,ImageStatus.Det_True-39,3);  //图像偏差显示
//
//     ips114_show_int(205,0,speed1,3);           //编码器速度显示
//     ips114_show_string(162,0,"left");
//     ips114_show_int(205,30,speed2,3);
//     ips114_show_string(162,30,"right");
//
//     ips114_show_string(0,60,"speed");
//     ips114_show_int(50,60,Speed_Goal,3);         //目标速度
//
//     ips114_show_int(0,100,SystemData.Stop,1);    //停车标志位
//    //    ips114_show_int(50,100,speeddd,5);           //斑马线编码器累加

//    ips114_show_int(30,20,icm20602_gyro_x,3);
//    ips114_show_int(80,20,icm_x,3);
//
//    ips114_show_int(30,50,icm20602_gyro_y,3);
//    ips114_show_int(80,50,icm_y,3);
//
//    ips114_show_int(30,80,icm20602_gyro_z,3);
//    ips114_show_int(80,80,icm_z,3);

    ips114_show_int(100,80,half_wide_count(half_wide_),3);   //赛道半宽
    ips114_show_int(130,80,half_wide_,3); //测试赛道半宽行数
}

void ips114_4()
{
//    draw_black_line_1();  //画边界
//    wave_display();   //显示波形
    ips114_show_int(130,80,Cirque_or_Cross(1,ImageStatus.Left_Line),3); //测试赛道圆环下白点数
    ips114_show_int(160,80,Cirque_or_Cross(2,ImageStatus.Right_Line),3);

}
void drawleftline()
{
    int x;
     for(int i=0;i<=59;i+=1)
     {
        x=ImageDeal[i].LeftBorder;               //之前是搜边界没标出来
        ips114_draw_point(x+80,i,RGB565_RED);
     }
}

void drawrightline()
{
    int x;
      for(int i=0;i<=59;i+=1)
      {
          x=ImageDeal[i].RightBorder;
          ips114_draw_point(x+80,i,RGB565_RED);
      }
}

void drawcenterline()
{
    int x;
     for(int i=0;i<=58;i+=1)
     {
//        x=ImageDeal[i].Center;
        x = (ImageDeal[i].RightBorder + ImageDeal[i].LeftBorder)/2;
        ips114_draw_point(x+80,i,RGB565_BLUE);
     }
}

void drawoffline()
{
    int x;
     for(int i=0;i<=79;i+=1)
     {
         x= half_wide_;//ImageStatus.MiddleLine   ImageStatus.OFFLine
         if(x<0)
             x = 0;
         if(x>60)
             x = 60;

         ips114_draw_point(i+80,x,RGB565_RED);
     }
}

void drawtowpointUP()
{
    int x;
      for(int i=0;i<=79;i+=1)
      {
          x=ImageStatus.TowPoint-5;
          ips114_draw_point(i,x,RGB565_CYAN);
      }
}

void drawtowpointDOWN()
{
    int x;
      for(int i=0;i<=79;i+=1)
      {
          x=ImageStatus.TowPoint+5;
          ips114_draw_point(i,x,RGB565_CYAN);
      }
}



int16 menu()
{
//    ips114_clear();   //清屏
    //主菜单界面
        ips114_show_string(40,20,"Data");
        ips114_show_string(40,45,"Gotcha");
        ips114_show_string(40,70,"Image");
        ips114_show_string(40,95,"Ring");



    while(1)
    {

        switch(menu_flag)
        {
            case 1:
            {
                  ips114_show_string(20,20,"->");
                  ips114_show_string(20,45,"  ");
                  ips114_show_string(20,70,"  ");
                  ips114_show_string(20,95,"  ");
            }break;

            case 2:
            {
                ips114_show_string(20,20,"  ");
                ips114_show_string(20,45,"->");
                ips114_show_string(20,70,"  ");
                ips114_show_string(20,95,"  ");
            }break;
            case 3:
            {
                ips114_show_string(20,20,"  ");
                ips114_show_string(20,45,"  ");
                ips114_show_string(20,70,"->");
                ips114_show_string(20,95,"  ");
            }break;
            case 4:
            {
                ips114_show_string(20,20,"  ");
                ips114_show_string(20,45,"  ");
                ips114_show_string(20,70,"  ");
                ips114_show_string(20,95,"->");
            }break;
        }


        if(menu_flag == 1 && menu_flag_ == 1)   //二级数据菜单
        {
            ips114_clear();   //清屏
            menu_flag_1 = 1;
            while(1)
            {
//                ips114_show_string(25,15,"Steer");               //舵机参数显示
                ips114_show_string(40,30,"Kp");
                ips114_show_float(65,30,SteerPIDdata.P,3,1);
                ips114_show_string(40,55,"Kd");
                ips114_show_float(65,55,SteerPIDdata.D,3,1);
                ips114_show_string(40,80,"_P");                         //非直道舵机KP
                ips114_show_float(65,80,SteerPIDdata.L_P,3,1);         //Speed_Goal_add
                ips114_show_string(40,105,"_D");
                ips114_show_float(65,105,SteerPIDdata.L_D,3,1);   //非直道舵机KD

//                ips114_show_string(25,13,"programme");               //方案使用例
                ips114_show_int(125,15,preset_read,1);
                ips114_show_string(135,30,"Speed");      //目标速度
                ips114_show_int(200,30,Speed_Goal_add,3);
                ips114_show_string(135,55,"Bend");
                ips114_show_int(200,55,Speed_Goal_,3);
                ips114_show_string(135,80,"forword");    //前瞻
                ips114_show_int(200,80,ImageStatus.TowPoint,3);
                ips114_show_string(135,105,"Dif.spd");
                ips114_show_float(200,105,Dif_spd_rat,2,1);          //非直道舵机KP

                if(menu_flag_==0)          //跳出上一级菜单
                    {
                        ips114_clear();
                        ips114_show_string(40,20,"Data");
                        ips114_show_string(40,45,"Gotcha");
                        ips114_show_string(40,70,"Image");
                        ips114_show_string(40,95,"Ring");
                        flash_set_buffer_speed();
                        break;
                    }

            switch(menu_flag_1)
                    {
                        case 1:
                        {
                              ips114_show_string(20,30,"->");
                              ips114_show_string(20,55,"  ");
                              ips114_show_string(20,80,"  ");
                              ips114_show_string(20,105,"  ");
                              ips114_show_string(115,30,"  ");
                              ips114_show_string(115,55,"  ");
                              ips114_show_string(115,80,"  ");
                              ips114_show_string(115,105,"  ");
                              ips114_show_string(145,13,"  ");
                        }break;

                        case 2:
                        {
                              ips114_show_string(20,30,"  ");
                              ips114_show_string(20,55,"->");
                              ips114_show_string(20,80,"  ");
                              ips114_show_string(20,105,"  ");
                              ips114_show_string(115,30,"  ");
                              ips114_show_string(115,55,"  ");
                              ips114_show_string(115,80,"  ");
                              ips114_show_string(115,105,"  ");
                              ips114_show_string(145,13,"  ");
                        }break;
                        case 3:
                        {
                              ips114_show_string(20,30,"  ");
                              ips114_show_string(20,55,"  ");
                              ips114_show_string(20,80,"->");
                              ips114_show_string(20,105,"  ");
                              ips114_show_string(115,30,"  ");
                              ips114_show_string(115,55,"  ");
                              ips114_show_string(115,80,"  ");
                              ips114_show_string(115,105,"  ");
                              ips114_show_string(145,13,"  ");
                        }break;
                        case 4:
                         {
                            ips114_show_string(20,30,"  ");
                            ips114_show_string(20,55,"  ");
                            ips114_show_string(20,80,"  ");
                            ips114_show_string(20,105,"->");
                            ips114_show_string(115,30,"  ");
                            ips114_show_string(115,55,"  ");
                            ips114_show_string(115,80,"  ");
                            ips114_show_string(115,105,"  ");
                            ips114_show_string(145,13,"  ");
                         }break;
                        case 5:
                         {
                             ips114_show_string(20,30,"  ");
                             ips114_show_string(20,55,"  ");
                             ips114_show_string(20,80,"  ");
                             ips114_show_string(20,105,"  ");
                             ips114_show_string(115,30,"->");
                             ips114_show_string(115,55,"  ");
                             ips114_show_string(115,80,"  ");
                             ips114_show_string(115,105,"  ");
                             ips114_show_string(145,13,"  ");
                          }break;
                        case 6:
                          {
                             ips114_show_string(20,30,"  ");
                             ips114_show_string(20,55,"  ");
                             ips114_show_string(20,80,"  ");
                             ips114_show_string(20,105,"  ");
                             ips114_show_string(115,30,"  ");
                             ips114_show_string(115,55,"->");
                             ips114_show_string(115,80,"  ");
                             ips114_show_string(115,105,"  ");
                             ips114_show_string(145,13,"  ");
                          }break;
                        case 7:
                          {
                             ips114_show_string(20,30,"  ");
                             ips114_show_string(20,55,"  ");
                             ips114_show_string(20,80,"  ");
                             ips114_show_string(20,105,"  ");
                             ips114_show_string(115,30,"  ");
                             ips114_show_string(115,55,"  ");
                             ips114_show_string(115,80,"->");
                             ips114_show_string(115,105,"  ");
                             ips114_show_string(145,13,"  ");
                          }break;
                        case 8:
                          {
                             ips114_show_string(20,30,"  ");
                             ips114_show_string(20,55,"  ");
                             ips114_show_string(20,80,"  ");
                             ips114_show_string(20,105,"  ");
                             ips114_show_string(115,30,"  ");
                             ips114_show_string(115,55,"  ");
                             ips114_show_string(115,80,"  ");
                             ips114_show_string(115,105,"->");
                             ips114_show_string(145,13,"  ");
                          }break;
                        case 9:
                           {
                             ips114_show_string(20,30,"  ");
                             ips114_show_string(20,55,"  ");
                             ips114_show_string(20,80,"  ");
                             ips114_show_string(20,105,"  ");
                             ips114_show_string(115,30,"  ");
                             ips114_show_string(115,55,"  ");
                             ips114_show_string(115,80,"  ");
                             ips114_show_string(115,105,"  ");
                             ips114_show_string(145,13,"<<");
                           }break;
                    }
            if(menu_flag_1 == 1 && menu_flag_ == 2)   //选择修改舵机的P值
               {


//                            menu_image_1 = 0;                     //先清一下标志位
                if(menu_speed_1 == 1)///////////////////////////////////////////////////////////////////////////
                   {
                       SteerPIDdata.P = SteerPIDdata.P + 1.5;
                       menu_speed_1 = 0;
                   }
                if(menu_speed_1 == 2)//
                   {
                       SteerPIDdata.P = SteerPIDdata.P - 1;
                       menu_speed_1 = 0;
                   }
               }
            if(menu_flag_1 == 2 && menu_flag_ == 2)   //选择修改舵机的D值
                           {


            //                            menu_image_1 = 0;                     //先清一下标志位
                            if(menu_speed_1 == 1)///////////////////////////////////////////////////////////////////////////
                               {
                                   SteerPIDdata.D = SteerPIDdata.D + 1.5;
                                   menu_speed_1 = 0;
                               }
                            if(menu_speed_1 == 2)//
                               {
                                   SteerPIDdata.D = SteerPIDdata.D - 1;
                                   menu_speed_1 = 0;
                               }
                           }
            if(menu_flag_1 == 3 && menu_flag_ == 2)   //非直道舵机   P
                                       {
                if(menu_speed_1 == 1)///////////////////////////////////////////////////////////////////////////
                                   {
                                       SteerPIDdata.L_P = SteerPIDdata.L_P + 1.5;
                                       menu_speed_1 = 0;
                                   }
                                if(menu_speed_1 == 2)//
                                   {
                                       SteerPIDdata.L_P = SteerPIDdata.L_P - 1;
                                       menu_speed_1 = 0;
                                   }
                                       }
            if(menu_flag_1 == 4 && menu_flag_ == 2)   //选择修改舵机的D值
                                                   {


                //                            menu_image_1 = 0;                     //先清一下标志位
                                if(menu_speed_1 == 1)///////////////////////////////////////////////////////////////////////////
                                   {
                                       SteerPIDdata.L_D = SteerPIDdata.L_D + 1.5;
                                       menu_speed_1 = 0;
                                   }
                                if(menu_speed_1 == 2)//
                                   {
                                       SteerPIDdata.L_D = SteerPIDdata.L_D - 1;
                                       menu_speed_1 = 0;
                                   }
                                                   }
            if(menu_flag_1 == 5 && menu_flag_ == 2)   //直道速度
                                                   {


                                    //                            menu_image_1 = 0;                     //先清一下标志位
                if(menu_speed_1 == 1)///////////////////////////////////////////////////////////////////////////
                                                           {
                                                            Speed_Goal_add = Speed_Goal_add + 5;
                                                               menu_speed_1 = 0;
                                                           }
                                                        if(menu_speed_1 == 2)//
                                                           {
                                                            Speed_Goal_add = Speed_Goal_add - 5;
                                                               menu_speed_1 = 0;
                                                           }
                                                   }
            if(menu_flag_1 == 6 && menu_flag_ == 2)   //弯道速度
                                                   {


                                    //                            menu_image_1 = 0;                     //先清一下标志位
                if(menu_speed_1 == 1)///////////////////////////////////////////////////////////////////////////
                                                                       {
                                                                        Speed_Goal_ = Speed_Goal_ + 5;
                                                                           menu_speed_1 = 0;
                                                                       }
                                                                    if(menu_speed_1 == 2)//
                                                                       {
                                                                        Speed_Goal_ = Speed_Goal_ - 5;
                                                                           menu_speed_1 = 0;
                                                                       }
                                                   }

            if(menu_flag_1 == 7 && menu_flag_ == 2)   //前瞻
                           {


            //                            menu_image_1 = 0;                     //先清一下标志位
                if(menu_speed_1 == 1)///////////////////////////////////////////////////////////////////////////
                                                                       {
                                                                        ImageStatus.TowPoint = ImageStatus.TowPoint + 1;
                                                                           menu_speed_1 = 0;
                                                                       }
                                                                    if(menu_speed_1 == 2)//
                                                                       {
                                                                        ImageStatus.TowPoint = ImageStatus.TowPoint - 1;
                                                                           menu_speed_1 = 0;
                                                                       }
                           }
            if(menu_flag_1 == 8 && menu_flag_ == 2)   //差速
                           {


            //                            menu_image_1 = 0;                     //先清一下标志位
                if(menu_speed_1 == 1)///////////////////////////////////////////////////////////////////////////
                                                                       {
                                                                        Dif_spd_rat = Dif_spd_rat + 0.4;
                                                                           menu_speed_1 = 0;
                                                                       }
                                                                    if(menu_speed_1 == 2)//
                                                                       {
                                                                        Dif_spd_rat = Dif_spd_rat - 0.3;
                                                                           menu_speed_1 = 0;
                                                                       }
                           }
            if(menu_flag_1 == 9 && menu_flag_ == 2)
                 {
                ips114_show_string(33,13,"cun   qv  ");
                   if(menu_speed_1 == 1)
                        {
//                           preset_read++;

                            menu_speed_1 = 0;
                         }
                    if(menu_speed_1 == 2)//
                         {

//                            preset_read--;

                             menu_speed_1 = 0;
                          }
                 }
                        if(menu_flag_1 == 9 && menu_flag_ == 3)
                             {

                               if(menu_speed_1 == 1)
                                   {
//                                          flash_set_buffer_preset(0);             //将原来的数据存到方案0里  加
////                                          ips114_show_string(25,10,"22222222222");
//                                          flash_get_buffer_preset(preset_read);   //取方案
                                   ips114_show_string(33,13,"cun   qv <");

                                          if(preset_read==0)
                                              flash_get_buffer_speed_0();
                                          flash_set_buffer_speed_0();    //存0的方案
                                          if(preset_read==1)
                                          flash_get_buffer_speed_1();
                                          if(preset_read==2)
                                          flash_get_buffer_speed_2();
                                          if(preset_read==3)
                                          flash_get_buffer_speed_3();
                                          if(preset_read==4)
                                          flash_get_buffer_speed_4();
                                          if(preset_read==5)
                                          flash_get_buffer_speed_5();

                                          flash_set_buffer_speed();        //给当前数值存进数据里
                                          menu_speed_1 = 0;
                                    }
                                if(menu_speed_1 == 2)//
                                     {
//                                            flash_set_buffer_preset(preset_read);//存方案       减
//                                            ips114_show_string(25,10,"111111111111");           //0作为上一次方案，存不了
                                    ips114_show_string(33,13,"cun < qv  ");
                                        if(preset_read==1)
                                            flash_set_buffer_speed_1();
                                        if(preset_read==2)
                                            flash_set_buffer_speed_2();
                                        if(preset_read==3)
                                            flash_set_buffer_speed_3();
                                        if(preset_read==4)
                                            flash_set_buffer_speed_4();
                                        if(preset_read==5)
                                            flash_set_buffer_speed_5();

                                        flash_set_buffer_speed();        //给当前数值存进数据里
                                        menu_speed_1 = 0;
                                      }
                             }
            }
        }
        if(menu_flag == 2 && menu_flag_ == 1)               //摄像头图像采集参数设置
        {
            ips114_clear();   //清屏
            menu_flag_1 = 1;
            menu_flag_image1 = 1;
            while(1)
                        {
                            ips114_show_string(25,10,"Image");
                            ips114_show_string(40,30,"ex_time");
                            ips114_show_int(100,30,exposure_time,3);
//                            ips114_show_string(40,55,"zenyi");
//                            ips114_show_int(100,55,MT9V03X_GAIN_DEF,3);
//                            ips114_show_string(40,80,"Re_time");
//                            ips114_show_int(100,80,MT9V03X_AUTO_EXP_DEF,3);
//                            ips114_show_int(100,30,exposure_time,3);
                            ips114_show_string(40,55,"Element");
                            ips114_show_int(135,55,Element_on,1);
                            ips114_show_string(40,80,"banmaxian");
                            ips114_show_int(135,80,banmaxian_on,1);
                            if(menu_flag_==0)
                            {
                                ips114_clear();
                                ips114_show_string(40,20,"Data");
                                ips114_show_string(40,45,"Gotcha");
                                ips114_show_string(40,70,"Image");
                                ips114_show_string(40,95,"Ring");
                                flash_set_buffer_image();
                                break;
                            }

                        switch(menu_flag_image1)
                                {
                                    case 1:
                                    {
                                          ips114_show_string(20,30,"->");
                                          ips114_show_string(20,55,"  ");
                                          ips114_show_string(20,80,"  ");
                                    }break;

                                    case 2:
                                    {
                                          ips114_show_string(20,30,"  ");
                                          ips114_show_string(20,55,"->");
                                          ips114_show_string(20,80,"  ");
                                    }break;
                                    case 3:
                                    {
                                          ips114_show_string(20,30,"  ");
                                          ips114_show_string(20,55,"  ");
                                          ips114_show_string(20,80,"->");
                                    }break;

                                }
                        if(menu_flag_image1 == 1 && menu_flag_ == 2)  //选择第一个值
                        {
//                            menu_image_1 = 0; //先清一下标志位
                                if(menu_image_1 == 1)//
                                {
                                    exposure_time = exposure_time + 10;
                                    menu_image_1 = 0;
                                }
                                if(menu_image_1 == 2)//
                                {
                                     exposure_time = exposure_time - 10;
                                     menu_image_1 = 0;
                                 }
                        }

                        if(menu_flag_image1 == 2 && menu_flag_ == 2)  //选择第一个值
                                                {
                        //                            menu_image_1 = 0; //先清一下标志位
                                                        if(menu_image_1 == 1)//
                                                        {
                                                            Element_on = (Element_on + 1)%2;
                                                            menu_image_1 = 0;
                                                        }
                                                        if(menu_image_1 == 2)//
                                                        {
                                                            Element_on = (Element_on + 1)%2;
                                                             menu_image_1 = 0;
                                                         }
                                                }
                        if(menu_flag_image1 == 3 && menu_flag_ == 2)  //选择第一个值
                                                {
                        //                            menu_image_1 = 0; //先清一下标志位
                                                        if(menu_image_1 == 1)//
                                                        {
                                                            banmaxian_on = (banmaxian_on + 1)%2;
                                                            menu_image_1 = 0;
                                                        }
                                                        if(menu_image_1 == 2)//
                                                        {
                                                            banmaxian_on = (banmaxian_on + 1)%2;
                                                             menu_image_1 = 0;
                                                         }
                                                }
                        }

        }
        if(menu_flag == 3 && menu_flag_ == 1)
        {
            ips114_clear();   //清屏
            menu_flag_1 = 1;
            menu_flag_image=1;

            while(1)
            {
                ips114_show_string(25,10,"Image");          //图像显示设置                                 //图像参数调整
                ips114_show_string(40,30,"black image");
//                ips114_show_float(65,30,SteerPIDdata.P,3,1);
                ips114_show_string(40,55,"Grayscale image");
//                ips114_show_float(65,55,SteerPIDdata.I,3,1);
                ips114_show_string(40,80,"drawline");
//                ips114_show_float(65,80,SteerPIDdata.D,3,1);
                ips114_show_string(40,105,"test");
                switch(menu_flag_image)
                                   {
                                       case 1:
                                       {
                                             ips114_show_string(20,30,"->");
                                             ips114_show_string(20,55,"  ");
                                             ips114_show_string(20,80,"  ");
                                             ips114_show_string(20,105,"  ");
                                       }break;

                                       case 2:
                                       {
                                             ips114_show_string(20,30,"  ");
                                             ips114_show_string(20,55,"->");
                                             ips114_show_string(20,80,"  ");
                                             ips114_show_string(20,105,"  ");
                                       }break;
                                       case 3:
                                       {
                                             ips114_show_string(20,30,"  ");
                                             ips114_show_string(20,55,"  ");
                                             ips114_show_string(20,80,"->");
                                             ips114_show_string(20,105,"  ");
                                       }break;
                                       case 4:
                                    {
                                             ips114_show_string(20,30,"  ");
                                             ips114_show_string(20,55,"  ");
                                             ips114_show_string(20,80,"  ");
                                             ips114_show_string(20,105,"->");
                                     }break;
                                   }
                if(menu_flag_image == 1 && menu_flag_ == 2)
                {
                        SystemData.Stop = 0;         //停车标志位
                        ips114_clear();   //清屏
                        return 1;
                }
                if(menu_flag_image == 2 && menu_flag_ == 2)
                    {
                         ips114_clear();   //清屏
                         return 1;
                     }
                if(menu_flag_image == 3 && menu_flag_ == 2)
                     {
                          ips114_clear();   //清屏
                          return 1;
                     }
                if(menu_flag_image == 4 && menu_flag_ == 2)
                                     {
                                          ips114_clear();   //清屏
                                          return 1;
                                     }
            }
            }
        if(menu_flag == 4 && menu_flag_ == 1)
        {
            ips114_clear();   //清屏
            menu_flag_1 = 1;
            menu_flag_image=1;
            menu_rings = 1;
            while(1)
            {
                ips114_show_string(40,30,"Kp");
                ips114_show_float(70,30,Speed_P_l,3,1);
                ips114_show_string(40,55,"Ki");
                ips114_show_float(70,55,Speed_I_l,3,1);
                ips114_show_string(40,80,"Kz");
                ips114_show_float(70,80,tffkz,3,1);
                ips114_show_string(40,105,"R_c");
                ips114_show_int(70,105,fan_pwm_Straight,4);

                ips114_show_string(135,30,"L_gypo");      //目标速度
                ips114_show_int(200,30,gypo_left,4);
                ips114_show_string(135,55,"R_gypo");
                ips114_show_int(200,55,gypo_right,3);
                ips114_show_string(135,80,"Bian");    //前瞻
                ips114_show_int(200,80,bianmaqi_ring,4);
                ips114_show_string(135,105,"M_c");
                ips114_show_int(200,105,motor_choose,1);          //非直道舵机KP

                if(menu_flag_==0)     //菜单返回
                     {
                          ips114_clear();
                          ips114_show_string(40,20,"Data");
                          ips114_show_string(40,45,"Gotcha");
                          ips114_show_string(40,70,"Image");
                          ips114_show_string(40,95,"Ring");
                          flash_set_buffer_rings();    //存数据
                       break;
                     }

                switch(menu_rings)
                         {
                    case 1:
                                           {
                                                 ips114_show_string(20,30,"->");
                                                 ips114_show_string(20,55,"  ");
                                                 ips114_show_string(20,80,"  ");
                                                 ips114_show_string(20,105,"  ");
                                                 ips114_show_string(115,30,"  ");
                                                 ips114_show_string(115,55,"  ");
                                                 ips114_show_string(115,80,"  ");
                                                 ips114_show_string(115,105,"  ");

                                           }break;

                                           case 2:
                                           {
                                                 ips114_show_string(20,30,"  ");
                                                 ips114_show_string(20,55,"->");
                                                 ips114_show_string(20,80,"  ");
                                                 ips114_show_string(20,105,"  ");
                                                 ips114_show_string(115,30,"  ");
                                                 ips114_show_string(115,55,"  ");
                                                 ips114_show_string(115,80,"  ");
                                                 ips114_show_string(115,105,"  ");

                                           }break;
                                           case 3:
                                           {
                                                 ips114_show_string(20,30,"  ");
                                                 ips114_show_string(20,55,"  ");
                                                 ips114_show_string(20,80,"->");
                                                 ips114_show_string(20,105,"  ");
                                                 ips114_show_string(115,30,"  ");
                                                 ips114_show_string(115,55,"  ");
                                                 ips114_show_string(115,80,"  ");
                                                 ips114_show_string(115,105,"  ");

                                           }break;
                                           case 4:
                                            {
                                               ips114_show_string(20,30,"  ");
                                               ips114_show_string(20,55,"  ");
                                               ips114_show_string(20,80,"  ");
                                               ips114_show_string(20,105,"->");
                                               ips114_show_string(115,30,"  ");
                                               ips114_show_string(115,55,"  ");
                                               ips114_show_string(115,80,"  ");
                                               ips114_show_string(115,105,"  ");

                                            }break;
                                           case 5:
                                            {
                                                ips114_show_string(20,30,"  ");
                                                ips114_show_string(20,55,"  ");
                                                ips114_show_string(20,80,"  ");
                                                ips114_show_string(20,105,"  ");
                                                ips114_show_string(115,30,"->");
                                                ips114_show_string(115,55,"  ");
                                                ips114_show_string(115,80,"  ");
                                                ips114_show_string(115,105,"  ");

                                             }break;
                                           case 6:
                                             {
                                                ips114_show_string(20,30,"  ");
                                                ips114_show_string(20,55,"  ");
                                                ips114_show_string(20,80,"  ");
                                                ips114_show_string(20,105,"  ");
                                                ips114_show_string(115,30,"  ");
                                                ips114_show_string(115,55,"->");
                                                ips114_show_string(115,80,"  ");
                                                ips114_show_string(115,105,"  ");

                                             }break;
                                           case 7:
                                             {
                                                ips114_show_string(20,30,"  ");
                                                ips114_show_string(20,55,"  ");
                                                ips114_show_string(20,80,"  ");
                                                ips114_show_string(20,105,"  ");
                                                ips114_show_string(115,30,"  ");
                                                ips114_show_string(115,55,"  ");
                                                ips114_show_string(115,80,"->");
                                                ips114_show_string(115,105,"  ");

                                             }break;
                                           case 8:
                                             {
                                                ips114_show_string(20,30,"  ");
                                                ips114_show_string(20,55,"  ");
                                                ips114_show_string(20,80,"  ");
                                                ips114_show_string(20,105,"  ");
                                                ips114_show_string(115,30,"  ");
                                                ips114_show_string(115,55,"  ");
                                                ips114_show_string(115,80,"  ");
                                                ips114_show_string(115,105,"->");

                                             }break;

                           }//switch(menu_rings)
                if(menu_rings == 1 && menu_flag_ == 2)  //选择第一个值
                    {

                        if(menu_rings_flag ==1)
                            {
                                //里面放入输出结果即可
                                Speed_P_l = Speed_P_l + 10;
                                menu_rings_flag = 0;
                            }
                        if(menu_rings_flag ==2)
                             {
                                Speed_P_l = Speed_P_l - 5;
                                menu_rings_flag = 0;
                             }
                    }
                if(menu_rings == 2 && menu_flag_ == 2)  //选择第一个值
                                    {

                                        if(menu_rings_flag ==1)
                                            {
                                                //里面放入输出结果即可
                                                Speed_I_l = Speed_I_l + 0.4;
                                                menu_rings_flag = 0;
                                            }
                                        if(menu_rings_flag ==2)
                                             {
                                                Speed_I_l = Speed_I_l - 0.3;
                                                menu_rings_flag = 0;
                                             }
                                    }
                if(menu_rings == 3 && menu_flag_ == 2)  //选择第一个值
                                    {

                                        if(menu_rings_flag ==1)
                                            {
                                                //里面放入输出结果即可
                                                tffkz = tffkz + 0.2;
                                                menu_rings_flag = 0;
                                            }
                                        if(menu_rings_flag ==2)
                                             {
                                                tffkz = tffkz - 0.1;
                                                menu_rings_flag = 0;
                                             }
                                    }
                if(menu_rings == 4 && menu_flag_ == 2)  //选择第一个值
                                    {

                                        if(menu_rings_flag ==1)
                                            {
                                                //里面放入输出结果即可
                                            fan_pwm_Straight = fan_pwm_Straight + 50;
                                                menu_rings_flag = 0;
                                            }
                                        if(menu_rings_flag ==2)
                                             {
                                            fan_pwm_Straight = fan_pwm_Straight - 25;
                                                menu_rings_flag = 0;
                                             }
                                    }
                if(menu_rings == 5 && menu_flag_ == 2)  //选择第一个值
                                    {

                                        if(menu_rings_flag ==1)
                                            {
                                                //里面放入输出结果即可
                                            gypo_left = gypo_left + 40;
                                                menu_rings_flag = 0;
                                            }
                                        if(menu_rings_flag ==2)
                                             {
                                            gypo_left = gypo_left - 10;
                                                menu_rings_flag = 0;
                                             }
                                    }
                if(menu_rings == 6 && menu_flag_ == 2)  //选择第一个值
                                    {

                                        if(menu_rings_flag ==1)
                                            {
                                                //里面放入输出结果即可
                                            gypo_right = gypo_right + 40;
                                                menu_rings_flag = 0;
                                            }
                                        if(menu_rings_flag ==2)
                                             {
                                            gypo_right = gypo_right - 10;
                                                menu_rings_flag = 0;
                                             }
                                    }
                if(menu_rings == 7 && menu_flag_ == 2)  //选择第一个值
                                    {

                                        if(menu_rings_flag ==1)
                                            {
                                                //里面放入输出结果即可
                                                bianmaqi_ring = bianmaqi_ring + 500;
                                                menu_rings_flag = 0;
                                            }
                                        if(menu_rings_flag ==2)
                                             {
                                                bianmaqi_ring = bianmaqi_ring - 400;
                                                menu_rings_flag = 0;
                                             }
                                    }
                if(menu_rings == 8 && menu_flag_ == 2)  //选择第一个值
                                    {

                                        if(menu_rings_flag ==1)
                                            {
                                                //里面放入输出结果即可
                                                ips114_show_string(170,8,"   ");
                                                motor_choose = 1;
                                                ips114_show_string(170,8,"new");
                                                menu_rings_flag = 0;
                                            }
                                        if(menu_rings_flag ==2)
                                             {
                                                ips114_show_string(170,8,"   ");
                                                motor_choose = 0;
                                                ips114_show_string(170,8,"old");
                                                menu_rings_flag = 0;
                                             }
                                    }
            }//while(1)
        }
    }
}















void menu_jump()      //按键 增加
{

    half_wide_++;

    if(menu_flag_==0)
    menu_flag++;

    if(menu_flag_==1)
    menu_flag_1++;

    if(menu_flag == 4 && menu_flag_ == 1)
           menu_rings++;

    if(menu_flag == 3 && menu_flag_ == 1)
        menu_flag_image++;//图像菜单按键累加标志位
    if(menu_flag == 2 && menu_flag_ == 1)

    menu_flag_image1++;//图像菜单按键累加标志位1

    if((menu_flag_image1 == 1 && menu_flag_ == 2)
     ||(menu_flag_image1 == 2 && menu_flag_ == 2)
     ||(menu_flag_image1 == 3 && menu_flag_ == 2))//选择第一个值
        menu_image_1++;   //图像数据累加标志位

    if(menu_flag == 1 && (menu_flag_ == 2 ||menu_flag_ == 3))//选择第一个值
      menu_speed_1++;   //图像数据累加标志位

    if(menu_flag_1 == 9 && menu_flag_ == 2)
            preset_read++;

    if(menu_flag == 4 && menu_flag_ == 2)
        menu_rings_flag++;

    if(menu_flag==5)
     {
        menu_flag = 1;
     }
    if(menu_flag_1==10)
     {
        menu_flag_1 = 1;
     }
    if(menu_flag_image == 5)
     {
       menu_flag_image = 1;
     }
    if(menu_rings == 9)
    {
        menu_rings = 1;
    }
    if(menu_flag_image1 == 4)
        {
            menu_flag_image1 = 1;
        }
    if(preset_read == 6)
    {
        preset_read = 0;
    }
}

void menu_fall()      //按键减少
{
    half_wide_--;

    if(menu_flag_==0)
    menu_flag--;

    if(menu_flag_==1)
    menu_flag_1--;

    if(menu_flag == 4 && menu_flag_ == 1)
        menu_rings--;

    if(menu_flag == 3 && menu_flag_ == 1)
       menu_flag_image--;//图像菜单按键累加标志位

    if(menu_flag == 2 && menu_flag_ == 1)
        menu_flag_image1--;//图像菜单按键累加标志位1

    if(menu_flag == 4 && menu_flag_ == 2)
       menu_rings_flag+=2;

    if((menu_flag_image1 == 1 && menu_flag_ == 2)
      ||(menu_flag_image1 == 2 && menu_flag_ == 2)
      ||(menu_flag_image1 == 3 && menu_flag_ == 2))//选择第一个值
        menu_image_1+=2;   //图像数据累加标志位

    if(menu_flag == 1 && (menu_flag_ == 2||menu_flag_ == 3))//选择第一个值
          menu_speed_1+=2;   //图像数据累加标志位

    if(menu_flag_1 == 9 && menu_flag_ == 2)
        preset_read--;

   if(menu_flag==0)
      {
          menu_flag = 4;
      }
   if(menu_flag_1==0)
        {
           menu_flag_1 = 9;
        }
   if(menu_flag_image ==0)
       {
           menu_flag_image = 4;
       }
   if(menu_rings == 0)
       {
           menu_rings = 8;
       }
   if(menu_flag_image1 ==0)
      {
       menu_flag_image1 = 3;
      }
   if(preset_read == -1)
   {
       preset_read = 5;
   }
}

void menu_back()    //按键 确定
{
    if(menu_flag_<3)
    menu_flag_++;//二级菜单标志位
//    ips114_clear();   //清屏

}

void menu_long_back()   //按键退出
{

    if(menu_flag_>0)
    {
        menu_flag_--;     //二级菜单标志位累减
    }
}

uint16 wave_x,wave_y;   //限制显示
uint16 wave_x_,wave_y_;   //上一次显示，用于替换白点
int wave_lit;
float wave_rid = 2.2;         //放大系数
int wave_line[180] = {0};
int wave_line_[180] = {0};
int i=180;
void wave_display()                //滤波显示
{
//        system_start(); //定时器开始计时
//    wave_limit();    //数值限制
    wave_y = speed1;
    wave_x = speed2;
        if(wave_x<30)
            wave_x =30;
        if(wave_x>230)
            wave_x = 230;
        if(wave_y<30)
            wave_y =30;
        if(wave_y>130)
            wave_y = 130;

//
        if(speed1>-1)   //满足20ms
        {
            wave_lit++;
            if(wave_lit==1)         //20ms更新一次
            {
                wave_lit=0;  //清空标志位

                draw_speed();       //目标速度显示线

                ips114_show_int(182,0,Speed_Goal,3);  //显示器目标速度数值

//                for(int i=0;i<=179;i++)
//                  {
//                RGB565_RED
//                RGB565_BROWN
//                RGB565_WHITE
                if(i>=1)
                    i--;
                        wave_line[i] = 134-wave_y/wave_rid;
                        wave_line_[i] = 134-wave_x/wave_rid;
//                        ips114_draw_point(i+1,wave_line[i],RGB565_WHITE);   //先用白色盖住上一次的值
                        ips114_draw_point(i+1,wave_line[i],RGB565_RED);       //左电机
                        ips114_draw_point(i+1,wave_line_[i],RGB565_BROWN);    //右电机

                        wave_line[i-1] = wave_line[i];
                        wave_line_[i-1] = wave_line_[i];
//                  }
                wave_y_ = wave_y;   //赋值上一次
                wave_x_ = wave_x;   //赋值上一次
//                ips114_draw_point(wave_x_,wave_y_,RGB565_WHITE);
//                ips114_draw_point(wave_x,wave_y,RGB565_BLACK);
//                wave_x_ = wave_x;
//                wave_y_ = wave_y;
                if(i == 0)
                {
                    i=180;
                    ips114_clear();
                }

            }
        }

}

void wave_limit()     //波限幅
{
    if(wave_x<0)
           wave_x =0;
       if(wave_x>180)
           wave_x = 180;
       if(wave_y<0)
           wave_y =0;
       if(wave_y>130)
           wave_y = 130;
}



void draw_speed()
{
      int x;
      for(int i=0;i<=179;i+=1)
      {
          x=Speed_Goal;
          ips114_draw_point(i+1,(135-x/wave_rid)-1,RGB565_BLUE);
      }
}
void draw_black_line_1()
{
          for(int x=0;x<=129;x+=1)
          {
              ips114_draw_point(181,x,RGB565_BLACK);
          }
          for(int i=0;i<=179;i+=1)
          {
               ips114_draw_point(i+1,130,RGB565_BLACK);
          }

}










