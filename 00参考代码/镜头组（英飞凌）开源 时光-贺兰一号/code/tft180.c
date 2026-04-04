/*
 * tft180.c
 *
 *  Created on: 2024Äê3ÔÂ4ÈÕ
 *      Author: xiaoming
 */

#include "C_H.h"
#include "tft180.h"
#include "zf_common_headfile.h"
#include "motor.h"

extern int sanchachazhi;
extern int16 speed1;
extern int16 speed2;
extern uint16 adcleft;
extern uint16 adcmiddle;
extern uint16 adcright;
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
int jishu=0;
extern int run_flag;
extern int ceshi_flag;
extern float variance_acc;
void tft180()
{  //  tft180_displayimage032_zoom(Image_Use[0], LCDW, LCDH, LCDW, LCDH);    //»Ò¶ÈÍ¼Ïñ
   // tft180_displayimage03x(mt9v03x_image[0], 94, 60);
    if(run_flag == 0)
              {
    tft180_show_gray_image(0,0,Pixle[0],LCDW,LCDH,LCDW,LCDH,1);             //¶þÖµ»¯Í¼Ïñ

   //  tft180_displayimage032_zoom(mt9v03x_image_dvp[0], 188, 120, 160, 120);    //»Ò¶ÈÍ¼Ïñ
    drawleftline();     //»­×ó±ßÏß
    drawrightline();    //»­ÓÒ±ßÏß
////    drawcenterline();   //»­ÖÐÏß
//    //draw_math();
    drawoffline();      //»­Í¼Ïñ¶¥±ß
    drawtowpointUP();   //»­Ç°Õ°·¶Î§ÉÏÏß
    drawtowpointDOWN(); //»­Ç°Õ°·¶Î§ÏÂÏß
              }
    tft180_show_string(0,60,"cha");
    tft180_show_int(30,60,ImageStatus.Det_True-39,3);
    tft180_show_string(0,75,"8l");
    tft180_show_int(30,75,speed1,3);         ///°ËÁìÓò×ó¶ªÏß
    tft180_show_string(0,90,"8r");
    tft180_show_int(30,90,ImageStatus.Road_type,3);        //°ËÁìÓòÓÒ¶ªÏß
    tft180_show_string(0,105,"ptl");
    tft180_show_int(30,105,ImageStatus.Left_Line,3);         //±ßÔµ×·Öð×ó¶ªÏß
    tft180_show_string(0,120,"ptr");
    tft180_show_int(30,120,ImageStatus.Right_Line,3);         //±ßÔµ×·ÖðÓÒ¶ªÏß
    tft180_show_string(0,135,"cir");
    tft180_show_int(30,135,ImageFlag.image_element_rings_flag,1);
    tft180_show_string(90,75,"fc");
    tft180_show_int(80,75, ImageStatus.Road_type,3);         ///°ËÁìÓò×ó¶ªÏß
//    tft180_show_string(0,60,"l");
//    tft180_show_uint(30,60,adcleft,4);
////    tft180_show_string(60,75,"adm");
////    tft180_show_uint(90,75,adcmsun,4);
//    tft180_show_string(0,75,"r");
//    tft180_show_uint(30,75,adcright,4);
//    tft180_show_string(0,90,"sum");
//    tft180_show_uint(30,90,adcsum,4);

//    tft180_show_string(60,120,"sz");
//    tft180_show_uint(90,120,ImageStatus.Cross_ture_lenth*OX,3);
//    tft180_show_string(60,105,"p3x");
//    tft180_show_int(90,105,ImageStatus.pansancha_Lenth*OX,3);
    //tft180_show_string(60,120,"cr1");
    //tft180_show_uint(90,120,ImageStatus.Cirque1lenth*OX,4);



}

void drawleftline()
{
    int x;
     for(int i=0;i<=59;i+=1)
     {
        x=ImageDeal[i].LeftBorder;
        tft180_draw_point(x,i,RGB565_RED);
     }
}

void drawrightline()
{
    int x;
      for(int i=0;i<=59;i+=1)
      {
          x=ImageDeal[i].RightBorder;
          tft180_draw_point(x,i,RGB565_RED);
      }
}

void drawcenterline()
{
    int x;
     for(int i=0;i<=58;i+=1)
     {
//        x=ImageDeal[i].Center;
       // x = (ImageDeal[i].RightBorder + ImageDeal[i].LeftBorder)/2;
         x = (ImageDeal[i].RightBorder - ImageDeal[i].LeftBorder)/2;
        //tft180_draw_point(x,i,RGB565_BLUE);
         if(jishu<=58)
         {
          wireless_uart_send_byte(x);
          jishu++;
         }

     }
}

void draw_math()
{
    int x;
    int j=0;
    int y =0;
    for(int i =4;i<=58;i++)
    {
            x = (ImageDeal[i].RightBorder - ImageDeal[i].LeftBorder)/2;
            tft180_show_int(j,y,x,2);
            j+=15;

            if(j>100){j = 0;y+=15;}
    }
}
void drawoffline()
{
    int x;
     for(int i=0;i<=79;i+=1)
     {
         x=ImageStatus.OFFLine;
         tft180_draw_point(i,x,RGB565_RED);
     }
}

void drawtowpointUP()
{
    int x;
      for(int i=0;i<=79;i+=1)
      {
          x=ImageStatus.TowPoint-5;
          tft180_draw_point(i,x,RGB565_CYAN);
      }
}

void drawtowpointDOWN()
{
    int x;
      for(int i=0;i<=79;i+=1)
      {
          x=ImageStatus.TowPoint+5;
          tft180_draw_point(i,x,RGB565_CYAN);
      }
}


