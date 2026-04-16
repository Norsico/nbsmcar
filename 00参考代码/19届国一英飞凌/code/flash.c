/*
 * flash.c
 *
 *  Created on: 2024年5月18日
 *      Author: Amadeus
 */
#include <display.h>
#include <image.h>
#include "zf_common_headfile.h"
#include "flash.h"
#include "motor.h"
#include "Fuzzy.h"

extern uint16 Speed_Goal;
extern uint16 Speed_Goal_;
extern SystemDatatypdef SystemData;
extern PID_Datatypedef SteerPIDdata;    //舵机的PID参数
extern uint32 exposure_time;    //曝光时间参数
extern uint16 Speed_Goal_add;  //直道加速度参数
extern uint8 Element_on;
extern uint8 banmaxian_on;
extern uint16 fan_pwm_Straight;      //负压风扇基础pwm



uint8 die;

int8 preset_read;      //预设值

#define FLASH_SECTION_INDEX             (0)    //存储数据用的和扇区
#define SPEED_PAGE_INDEX                (11)   //速度专用存取扇区
#define IMAGE_PAGE_INDEX                (10)   //图像专用存取扇区
#define PRESET_SPEED_PAGE_INDEX         (9)   //圆环专用存取扇区
#define PRESET_IMAGE_PAGE_INDEX         (8)   //暂时没用的扇区
#define SPEED_PAGE_INDEX_1                (7)   //方案1专用存取扇区
#define SPEED_PAGE_INDEX_2                (6)   //方案2专用存取扇区
#define SPEED_PAGE_INDEX_3                (5)   //方案3专用存取扇区
#define SPEED_PAGE_INDEX_4                (4)   //方案4专用存取扇区
#define SPEED_PAGE_INDEX_5                (3)   //方案5专用存取扇区
#define SPEED_PAGE_INDEX_0                (2)   //上一次方案专用存取扇区

//void s()   //存    在使用菜单时应用一遍
//{
//    flash_buffer_clear();
//    flash_union_buffer[0].uint32_type = 1;
//    if(flash_check(FLASH_SECTION_INDEX,PRESET_SPEED_PAGE_INDEX))
//           {
//               flash_erase_page(FLASH_SECTION_INDEX,PRESET_SPEED_PAGE_INDEX) ;
//           }
//    flash_write_page_from_buffer(FLASH_SECTION_INDEX,PRESET_SPEED_PAGE_INDEX);
//}
//void x()   //取
//{
//    if(flash_check(FLASH_SECTION_INDEX,PRESET_SPEED_PAGE_INDEX))
//                  //判断FLASH扇区中有没有数据，有就把数据从FALSH指定扇区页码放入到缓冲区
//              {
//                  flash_read_page_to_buffer(FLASH_SECTION_INDEX,PRESET_SPEED_PAGE_INDEX);
//                  die = flash_union_buffer[0].uint32_type+1;
//              }
//}
void flash_set_buffer_rings()    //存圆环数据
{
     //第一步清除缓冲区
       flash_buffer_clear();
     //第二步把数据存到缓冲区
       flash_union_buffer[0].float_type = Speed_P_l;     //Speed_Goal  弯道期望速度
       flash_union_buffer[1].float_type = Speed_I_l;      //ImageStatus.TowPoint 前瞻
       flash_union_buffer[2].float_type = tffkz;     //Dif_spd_rat   差速比
       flash_union_buffer[3].int16_type = gypo_left;      //SteerPIDdata.P  舵机P
       flash_union_buffer[4].int16_type = gypo_right;      //SteerPIDdata.D   舵机D
       flash_union_buffer[5].int16_type = bianmaqi_ring;     //Speed_Goal_add  直道增加期望速度
       flash_union_buffer[6].uint16_type = fan_pwm_Straight;      //SteerPIDdata.L_P  非直道舵机P
       flash_union_buffer[7].uint8_type = motor_choose;      //SteerPIDdata.L_D   非直道舵机D

     //第三步判断FLASH里有没有数据，有就把FLASH数据擦除/把数据存到缓冲区
       if(flash_check(FLASH_SECTION_INDEX,PRESET_SPEED_PAGE_INDEX))
       {//判断是否有数据
           flash_erase_page(FLASH_SECTION_INDEX,PRESET_SPEED_PAGE_INDEX) ;
       }//擦除这一页
     //第四步存到FLASH指定扇区
       flash_write_page_from_buffer(FLASH_SECTION_INDEX,PRESET_SPEED_PAGE_INDEX);
     //存取过程结束 —— 成功存取
}
void flash_set_buffer_speed()    //存 速度数据
{
     //第一步清除缓冲区
       flash_buffer_clear();
     //第二步把数据存到缓冲区
       flash_union_buffer[0].uint16_type = Speed_Goal_;     //Speed_Goal  弯道期望速度
       flash_union_buffer[1].uint8_type = ImageStatus.TowPoint;      //ImageStatus.TowPoint 前瞻
       flash_union_buffer[2].float_type = Dif_spd_rat;     //Dif_spd_rat   差速比
       flash_union_buffer[3].float_type = SteerPIDdata.P;      //SteerPIDdata.P  舵机P
       flash_union_buffer[4].float_type = SteerPIDdata.D;      //SteerPIDdata.D   舵机D
       flash_union_buffer[5].uint16_type = Speed_Goal_add;     //Speed_Goal_add  直道增加期望速度
       flash_union_buffer[6].float_type = SteerPIDdata.L_P;      //SteerPIDdata.L_P  非直道舵机P
       flash_union_buffer[7].float_type = SteerPIDdata.L_D;      //SteerPIDdata.L_D   非直道舵机D

     //第三步判断FLASH里有没有数据，有就把FLASH数据擦除/把数据存到缓冲区
       if(flash_check(FLASH_SECTION_INDEX,SPEED_PAGE_INDEX))
       {//判断是否有数据
           flash_erase_page(FLASH_SECTION_INDEX,SPEED_PAGE_INDEX) ;
       }//擦除这一页
     //第四步存到FLASH指定扇区
       flash_write_page_from_buffer(FLASH_SECTION_INDEX,SPEED_PAGE_INDEX);
     //存取过程结束 —— 成功存取
}

void flash_set_buffer_image()    //存图像数据
{
     //第一步清除缓冲区
       flash_buffer_clear();
     //第二步把数据存到缓冲区
       flash_union_buffer[0].uint32_type = exposure_time;     //exposure_time  曝光时间
//       flash_union_buffer[1].int16_type = 32;   //MT9V03X_GAIN_DEF 图像增益
//       flash_union_buffer[2].int16_type = 0;     //MT9V03X_AUTO_EXP_DEF   自动曝光时间
       flash_union_buffer[1].uint8_type = Element_on;
       flash_union_buffer[2].uint8_type = banmaxian_on;

     //第三步判断FLASH里有没有数据，有就把FLASH数据擦除/把数据存到缓冲区
       if(flash_check(FLASH_SECTION_INDEX,IMAGE_PAGE_INDEX))
       {//判断是否有数据
           flash_erase_page(FLASH_SECTION_INDEX,IMAGE_PAGE_INDEX) ;
       }//擦除这一页
     //第四步存到FLASH指定扇区
       flash_write_page_from_buffer(FLASH_SECTION_INDEX,IMAGE_PAGE_INDEX);
     //存取过程结束 —— 成功存取
}

void flash_get_buffer_rings()   //取圆环数据
{
           //取数据过程
           if(flash_check(FLASH_SECTION_INDEX,PRESET_SPEED_PAGE_INDEX))
               //判断FLASH扇区中有没有数据，有就把数据从FALSH指定扇区页码放入到缓冲区
           {
               flash_read_page_to_buffer(FLASH_SECTION_INDEX,PRESET_SPEED_PAGE_INDEX);
               Speed_P_l = flash_union_buffer[0].float_type;           //Speed_Goal  弯道期望速度
               Speed_I_l = flash_union_buffer[1].float_type; //ImageStatus.TowPoint 前瞻
               tffkz = flash_union_buffer[2].float_type;          //Dif_spd_rat   差速比
               gypo_left = flash_union_buffer[3].int16_type;      //SteerPIDdata.P  舵机P
               gypo_right = flash_union_buffer[4].int16_type;      //SteerPIDdata.D   舵机D
               bianmaqi_ring = flash_union_buffer[5].int16_type;      //Speed_Goal_add  直道增加期望速度
               fan_pwm_Straight = flash_union_buffer[6].uint16_type;      //SteerPIDdata.L_P  非直道舵机P
               motor_choose = flash_union_buffer[7].uint8_type;      //SteerPIDdata.L_D   非直道舵机D
           }


}
void flash_get_buffer_speed()   //取  速度数据
{
           //取数据过程
           if(flash_check(FLASH_SECTION_INDEX,SPEED_PAGE_INDEX))
               //判断FLASH扇区中有没有数据，有就把数据从FALSH指定扇区页码放入到缓冲区
           {
               flash_read_page_to_buffer(FLASH_SECTION_INDEX,SPEED_PAGE_INDEX);
               Speed_Goal_ = flash_union_buffer[0].int16_type;           //Speed_Goal  弯道期望速度
               ImageStatus.TowPoint = flash_union_buffer[1].uint8_type; //ImageStatus.TowPoint 前瞻
               Dif_spd_rat = flash_union_buffer[2].float_type;          //Dif_spd_rat   差速比
               SteerPIDdata.P = flash_union_buffer[3].float_type;      //SteerPIDdata.P  舵机P
               SteerPIDdata.D = flash_union_buffer[4].float_type;      //SteerPIDdata.D   舵机D
               Speed_Goal_add = flash_union_buffer[5].uint16_type;      //Speed_Goal_add  直道增加期望速度
               SteerPIDdata.L_P = flash_union_buffer[6].float_type;      //SteerPIDdata.L_P  非直道舵机P
               SteerPIDdata.L_D = flash_union_buffer[7].float_type;      //SteerPIDdata.L_D   非直道舵机D
           }


}

void flash_get_buffer_image()   //取  速度数据
{
           //取数据过程
           if(flash_check(FLASH_SECTION_INDEX,IMAGE_PAGE_INDEX))
               //判断FLASH扇区中有没有数据，有就把数据从FALSH指定扇区页码放入到缓冲区
           {
               flash_read_page_to_buffer(FLASH_SECTION_INDEX,IMAGE_PAGE_INDEX);
               exposure_time = flash_union_buffer[0].uint32_type;
//               MT9V03X_GAIN_DEF = flash_union_buffer[1].int16_type;   //MT9V03X_GAIN_DEF 图像增益
               Element_on = flash_union_buffer[1].uint8_type;     //MT9V03X_AUTO_EXP_DEF   自动曝光时间
               banmaxian_on = flash_union_buffer[2].uint8_type;
           }



}


void flash_set_buffer_speed_1()    //存 速度数据
{
     //第一步清除缓冲区
       flash_buffer_clear();
     //第二步把数据存到缓冲区
       flash_union_buffer[0].uint16_type = Speed_Goal_;     //Speed_Goal  弯道期望速度
       flash_union_buffer[1].uint8_type = ImageStatus.TowPoint;      //ImageStatus.TowPoint 前瞻
       flash_union_buffer[2].float_type = Dif_spd_rat;     //Dif_spd_rat   差速比
       flash_union_buffer[3].float_type = SteerPIDdata.P;      //SteerPIDdata.P  舵机P
       flash_union_buffer[4].float_type = SteerPIDdata.D;      //SteerPIDdata.D   舵机D
       flash_union_buffer[5].uint16_type = Speed_Goal_add;     //Speed_Goal_add  直道增加期望速度
       flash_union_buffer[6].float_type = SteerPIDdata.L_P;      //SteerPIDdata.L_P  非直道舵机P
       flash_union_buffer[7].float_type = SteerPIDdata.L_D;      //SteerPIDdata.L_D   非直道舵机D

     //第三步判断FLASH里有没有数据，有就把FLASH数据擦除/把数据存到缓冲区
       if(flash_check(FLASH_SECTION_INDEX,SPEED_PAGE_INDEX_1))
       {//判断是否有数据
           flash_erase_page(FLASH_SECTION_INDEX,SPEED_PAGE_INDEX_1) ;
       }//擦除这一页
     //第四步存到FLASH指定扇区
       flash_write_page_from_buffer(FLASH_SECTION_INDEX,SPEED_PAGE_INDEX_1);
     //存取过程结束 —— 成功存取
}
void flash_get_buffer_speed_1()   //取  速度数据
{
           //取数据过程
           if(flash_check(FLASH_SECTION_INDEX,SPEED_PAGE_INDEX_1))
               //判断FLASH扇区中有没有数据，有就把数据从FALSH指定扇区页码放入到缓冲区
           {
               flash_read_page_to_buffer(FLASH_SECTION_INDEX,SPEED_PAGE_INDEX_1);
               Speed_Goal_ = flash_union_buffer[0].int16_type;           //Speed_Goal  弯道期望速度
               ImageStatus.TowPoint = flash_union_buffer[1].uint8_type; //ImageStatus.TowPoint 前瞻
               Dif_spd_rat = flash_union_buffer[2].float_type;          //Dif_spd_rat   差速比
               SteerPIDdata.P = flash_union_buffer[3].float_type;      //SteerPIDdata.P  舵机P
               SteerPIDdata.D = flash_union_buffer[4].float_type;      //SteerPIDdata.D   舵机D
               Speed_Goal_add = flash_union_buffer[5].uint16_type;      //Speed_Goal_add  直道增加期望速度
               SteerPIDdata.L_P = flash_union_buffer[6].float_type;      //SteerPIDdata.L_P  非直道舵机P
               SteerPIDdata.L_D = flash_union_buffer[7].float_type;      //SteerPIDdata.L_D   非直道舵机D
           }


}
void flash_set_buffer_speed_2()    //存 速度数据
{
     //第一步清除缓冲区
       flash_buffer_clear();
     //第二步把数据存到缓冲区
       flash_union_buffer[0].uint16_type = Speed_Goal_;     //Speed_Goal  弯道期望速度
       flash_union_buffer[1].uint8_type = ImageStatus.TowPoint;      //ImageStatus.TowPoint 前瞻
       flash_union_buffer[2].float_type = Dif_spd_rat;     //Dif_spd_rat   差速比
       flash_union_buffer[3].float_type = SteerPIDdata.P;      //SteerPIDdata.P  舵机P
       flash_union_buffer[4].float_type = SteerPIDdata.D;      //SteerPIDdata.D   舵机D
       flash_union_buffer[5].uint16_type = Speed_Goal_add;     //Speed_Goal_add  直道增加期望速度
       flash_union_buffer[6].float_type = SteerPIDdata.L_P;      //SteerPIDdata.L_P  非直道舵机P
       flash_union_buffer[7].float_type = SteerPIDdata.L_D;      //SteerPIDdata.L_D   非直道舵机D

     //第三步判断FLASH里有没有数据，有就把FLASH数据擦除/把数据存到缓冲区
       if(flash_check(FLASH_SECTION_INDEX,SPEED_PAGE_INDEX_2))
       {//判断是否有数据
           flash_erase_page(FLASH_SECTION_INDEX,SPEED_PAGE_INDEX_2) ;
       }//擦除这一页
     //第四步存到FLASH指定扇区
       flash_write_page_from_buffer(FLASH_SECTION_INDEX,SPEED_PAGE_INDEX_2);
     //存取过程结束 —— 成功存取
}
void flash_get_buffer_speed_2()   //取  速度数据
{
           //取数据过程
           if(flash_check(FLASH_SECTION_INDEX,SPEED_PAGE_INDEX_2))
               //判断FLASH扇区中有没有数据，有就把数据从FALSH指定扇区页码放入到缓冲区
           {
               flash_read_page_to_buffer(FLASH_SECTION_INDEX,SPEED_PAGE_INDEX_2);
               Speed_Goal_ = flash_union_buffer[0].int16_type;           //Speed_Goal  弯道期望速度
               ImageStatus.TowPoint = flash_union_buffer[1].uint8_type; //ImageStatus.TowPoint 前瞻
               Dif_spd_rat = flash_union_buffer[2].float_type;          //Dif_spd_rat   差速比
               SteerPIDdata.P = flash_union_buffer[3].float_type;      //SteerPIDdata.P  舵机P
               SteerPIDdata.D = flash_union_buffer[4].float_type;      //SteerPIDdata.D   舵机D
               Speed_Goal_add = flash_union_buffer[5].uint16_type;      //Speed_Goal_add  直道增加期望速度
               SteerPIDdata.L_P = flash_union_buffer[6].float_type;      //SteerPIDdata.L_P  非直道舵机P
               SteerPIDdata.L_D = flash_union_buffer[7].float_type;      //SteerPIDdata.L_D   非直道舵机D
           }


}
void flash_set_buffer_speed_3()    //存 速度数据
{
     //第一步清除缓冲区
       flash_buffer_clear();
     //第二步把数据存到缓冲区
       flash_union_buffer[0].uint16_type = Speed_Goal_;     //Speed_Goal  弯道期望速度
       flash_union_buffer[1].uint8_type = ImageStatus.TowPoint;      //ImageStatus.TowPoint 前瞻
       flash_union_buffer[2].float_type = Dif_spd_rat;     //Dif_spd_rat   差速比
       flash_union_buffer[3].float_type = SteerPIDdata.P;      //SteerPIDdata.P  舵机P
       flash_union_buffer[4].float_type = SteerPIDdata.D;      //SteerPIDdata.D   舵机D
       flash_union_buffer[5].uint16_type = Speed_Goal_add;     //Speed_Goal_add  直道增加期望速度
       flash_union_buffer[6].float_type = SteerPIDdata.L_P;      //SteerPIDdata.L_P  非直道舵机P
       flash_union_buffer[7].float_type = SteerPIDdata.L_D;      //SteerPIDdata.L_D   非直道舵机D

     //第三步判断FLASH里有没有数据，有就把FLASH数据擦除/把数据存到缓冲区
       if(flash_check(FLASH_SECTION_INDEX,SPEED_PAGE_INDEX_3))
       {//判断是否有数据
           flash_erase_page(FLASH_SECTION_INDEX,SPEED_PAGE_INDEX_3) ;
       }//擦除这一页
     //第四步存到FLASH指定扇区
       flash_write_page_from_buffer(FLASH_SECTION_INDEX,SPEED_PAGE_INDEX_3);
     //存取过程结束 —— 成功存取
}
void flash_get_buffer_speed_3()   //取  速度数据
{
           //取数据过程
           if(flash_check(FLASH_SECTION_INDEX,SPEED_PAGE_INDEX_3))
               //判断FLASH扇区中有没有数据，有就把数据从FALSH指定扇区页码放入到缓冲区
           {
               flash_read_page_to_buffer(FLASH_SECTION_INDEX,SPEED_PAGE_INDEX_3);
               Speed_Goal_ = flash_union_buffer[0].int16_type;           //Speed_Goal  弯道期望速度
               ImageStatus.TowPoint = flash_union_buffer[1].uint8_type; //ImageStatus.TowPoint 前瞻
               Dif_spd_rat = flash_union_buffer[2].float_type;          //Dif_spd_rat   差速比
               SteerPIDdata.P = flash_union_buffer[3].float_type;      //SteerPIDdata.P  舵机P
               SteerPIDdata.D = flash_union_buffer[4].float_type;      //SteerPIDdata.D   舵机D
               Speed_Goal_add = flash_union_buffer[5].uint16_type;      //Speed_Goal_add  直道增加期望速度
               SteerPIDdata.L_P = flash_union_buffer[6].float_type;      //SteerPIDdata.L_P  非直道舵机P
               SteerPIDdata.L_D = flash_union_buffer[7].float_type;      //SteerPIDdata.L_D   非直道舵机D
           }


}
void flash_set_buffer_speed_4()    //存 速度数据
{
     //第一步清除缓冲区
       flash_buffer_clear();
     //第二步把数据存到缓冲区
       flash_union_buffer[0].uint16_type = Speed_Goal_;     //Speed_Goal  弯道期望速度
       flash_union_buffer[1].uint8_type = ImageStatus.TowPoint;      //ImageStatus.TowPoint 前瞻
       flash_union_buffer[2].float_type = Dif_spd_rat;     //Dif_spd_rat   差速比
       flash_union_buffer[3].float_type = SteerPIDdata.P;      //SteerPIDdata.P  舵机P
       flash_union_buffer[4].float_type = SteerPIDdata.D;      //SteerPIDdata.D   舵机D
       flash_union_buffer[5].uint16_type = Speed_Goal_add;     //Speed_Goal_add  直道增加期望速度
       flash_union_buffer[6].float_type = SteerPIDdata.L_P;      //SteerPIDdata.L_P  非直道舵机P
       flash_union_buffer[7].float_type = SteerPIDdata.L_D;      //SteerPIDdata.L_D   非直道舵机D

     //第三步判断FLASH里有没有数据，有就把FLASH数据擦除/把数据存到缓冲区
       if(flash_check(FLASH_SECTION_INDEX,SPEED_PAGE_INDEX_4))
       {//判断是否有数据
           flash_erase_page(FLASH_SECTION_INDEX,SPEED_PAGE_INDEX_4) ;
       }//擦除这一页
     //第四步存到FLASH指定扇区
       flash_write_page_from_buffer(FLASH_SECTION_INDEX,SPEED_PAGE_INDEX_4);
     //存取过程结束 —— 成功存取
}
void flash_get_buffer_speed_4()   //取  速度数据
{
           //取数据过程
           if(flash_check(FLASH_SECTION_INDEX,SPEED_PAGE_INDEX_4))
               //判断FLASH扇区中有没有数据，有就把数据从FALSH指定扇区页码放入到缓冲区
           {
               flash_read_page_to_buffer(FLASH_SECTION_INDEX,SPEED_PAGE_INDEX_4);
               Speed_Goal_ = flash_union_buffer[0].int16_type;           //Speed_Goal  弯道期望速度
               ImageStatus.TowPoint = flash_union_buffer[1].uint8_type; //ImageStatus.TowPoint 前瞻
               Dif_spd_rat = flash_union_buffer[2].float_type;          //Dif_spd_rat   差速比
               SteerPIDdata.P = flash_union_buffer[3].float_type;      //SteerPIDdata.P  舵机P
               SteerPIDdata.D = flash_union_buffer[4].float_type;      //SteerPIDdata.D   舵机D
               Speed_Goal_add = flash_union_buffer[5].uint16_type;      //Speed_Goal_add  直道增加期望速度
               SteerPIDdata.L_P = flash_union_buffer[6].float_type;      //SteerPIDdata.L_P  非直道舵机P
               SteerPIDdata.L_D = flash_union_buffer[7].float_type;      //SteerPIDdata.L_D   非直道舵机D
           }


}
void flash_set_buffer_speed_5()    //存 速度数据
{
     //第一步清除缓冲区
       flash_buffer_clear();
     //第二步把数据存到缓冲区
       flash_union_buffer[0].uint16_type = Speed_Goal_;     //Speed_Goal  弯道期望速度
       flash_union_buffer[1].uint8_type = ImageStatus.TowPoint;      //ImageStatus.TowPoint 前瞻
       flash_union_buffer[2].float_type = Dif_spd_rat;     //Dif_spd_rat   差速比
       flash_union_buffer[3].float_type = SteerPIDdata.P;      //SteerPIDdata.P  舵机P
       flash_union_buffer[4].float_type = SteerPIDdata.D;      //SteerPIDdata.D   舵机D
       flash_union_buffer[5].uint16_type = Speed_Goal_add;     //Speed_Goal_add  直道增加期望速度
       flash_union_buffer[6].float_type = SteerPIDdata.L_P;      //SteerPIDdata.L_P  非直道舵机P
       flash_union_buffer[7].float_type = SteerPIDdata.L_D;      //SteerPIDdata.L_D   非直道舵机D

     //第三步判断FLASH里有没有数据，有就把FLASH数据擦除/把数据存到缓冲区
       if(flash_check(FLASH_SECTION_INDEX,SPEED_PAGE_INDEX_5))
       {//判断是否有数据
           flash_erase_page(FLASH_SECTION_INDEX,SPEED_PAGE_INDEX_5) ;
       }//擦除这一页
     //第四步存到FLASH指定扇区
       flash_write_page_from_buffer(FLASH_SECTION_INDEX,SPEED_PAGE_INDEX_5);
     //存取过程结束 —— 成功存取
}
void flash_get_buffer_speed_5()   //取  速度数据
{
           //取数据过程
           if(flash_check(FLASH_SECTION_INDEX,SPEED_PAGE_INDEX_5))
               //判断FLASH扇区中有没有数据，有就把数据从FALSH指定扇区页码放入到缓冲区
           {
               flash_read_page_to_buffer(FLASH_SECTION_INDEX,SPEED_PAGE_INDEX_5);
               Speed_Goal_ = flash_union_buffer[0].int16_type;           //Speed_Goal  弯道期望速度
               ImageStatus.TowPoint = flash_union_buffer[1].uint8_type; //ImageStatus.TowPoint 前瞻
               Dif_spd_rat = flash_union_buffer[2].float_type;          //Dif_spd_rat   差速比
               SteerPIDdata.P = flash_union_buffer[3].float_type;      //SteerPIDdata.P  舵机P
               SteerPIDdata.D = flash_union_buffer[4].float_type;      //SteerPIDdata.D   舵机D
               Speed_Goal_add = flash_union_buffer[5].uint16_type;      //Speed_Goal_add  直道增加期望速度
               SteerPIDdata.L_P = flash_union_buffer[6].float_type;      //SteerPIDdata.L_P  非直道舵机P
               SteerPIDdata.L_D = flash_union_buffer[7].float_type;      //SteerPIDdata.L_D   非直道舵机D
           }


}
void flash_set_buffer_speed_0()    //存 速度数据
{
     //第一步清除缓冲区
       flash_buffer_clear();
     //第二步把数据存到缓冲区
       flash_union_buffer[0].uint16_type = Speed_Goal_;     //Speed_Goal  弯道期望速度
       flash_union_buffer[1].uint8_type = ImageStatus.TowPoint;      //ImageStatus.TowPoint 前瞻
       flash_union_buffer[2].float_type = Dif_spd_rat;     //Dif_spd_rat   差速比
       flash_union_buffer[3].float_type = SteerPIDdata.P;      //SteerPIDdata.P  舵机P
       flash_union_buffer[4].float_type = SteerPIDdata.D;      //SteerPIDdata.D   舵机D
       flash_union_buffer[5].uint16_type = Speed_Goal_add;     //Speed_Goal_add  直道增加期望速度
       flash_union_buffer[6].float_type = SteerPIDdata.L_P;      //SteerPIDdata.L_P  非直道舵机P
       flash_union_buffer[7].float_type = SteerPIDdata.L_D;      //SteerPIDdata.L_D   非直道舵机D

     //第三步判断FLASH里有没有数据，有就把FLASH数据擦除/把数据存到缓冲区
       if(flash_check(FLASH_SECTION_INDEX,SPEED_PAGE_INDEX_0))
       {//判断是否有数据
           flash_erase_page(FLASH_SECTION_INDEX,SPEED_PAGE_INDEX_0) ;
       }//擦除这一页
     //第四步存到FLASH指定扇区
       flash_write_page_from_buffer(FLASH_SECTION_INDEX,SPEED_PAGE_INDEX_0);
     //存取过程结束 —— 成功存取
}
void flash_get_buffer_speed_0()   //取  速度数据
{
           //取数据过程
           if(flash_check(FLASH_SECTION_INDEX,SPEED_PAGE_INDEX_0))
               //判断FLASH扇区中有没有数据，有就把数据从FALSH指定扇区页码放入到缓冲区
           {
               flash_read_page_to_buffer(FLASH_SECTION_INDEX,SPEED_PAGE_INDEX_0);
               Speed_Goal_ = flash_union_buffer[0].int16_type;           //Speed_Goal  弯道期望速度
               ImageStatus.TowPoint = flash_union_buffer[1].uint8_type; //ImageStatus.TowPoint 前瞻
               Dif_spd_rat = flash_union_buffer[2].float_type;          //Dif_spd_rat   差速比
               SteerPIDdata.P = flash_union_buffer[3].float_type;      //SteerPIDdata.P  舵机P
               SteerPIDdata.D = flash_union_buffer[4].float_type;      //SteerPIDdata.D   舵机D
               Speed_Goal_add = flash_union_buffer[5].uint16_type;      //Speed_Goal_add  直道增加期望速度
               SteerPIDdata.L_P = flash_union_buffer[6].float_type;      //SteerPIDdata.L_P  非直道舵机P
               SteerPIDdata.L_D = flash_union_buffer[7].float_type;      //SteerPIDdata.L_D   非直道舵机D
           }


}
void gpio_read_set()
{
    if(gpio_get_level(P33_11)==1)    //上
    {
        fan_init();           //负压风扇初始化
    }
    else                             //下
    {

    }
    if(gpio_get_level(P33_12)==1)   //上
    {

    }
    else                             //下
    {
//          icm20602_init();      //陀螺仪初始化
    }
}







