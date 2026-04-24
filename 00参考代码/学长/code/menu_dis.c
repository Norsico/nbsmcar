/*
 * menu_dis.c
 *
 *  Created on: 2025年5月11日
 *      Author: 26672
 */
#include "zf_common_headfile.h"

Menu_var *cur_dis=var1;//显示内存,默认是大菜单结构体
char cur_dis_len=0;
char all_cur_dis_len;

int* int_var;       //中间变量，用于储存void*类型指针
float* float_var;   //中间变量，用于储存void*类型指针
char* char_var;     //中间变量，用于储存void*类型指针
int16* short_var;

char mouse=0;       //显示屏指示器位置
char all_mouse=0;   //实际指示位置




void menu_dis_init(){//菜单初始化
    all_cur_dis_len=var_len[len_var_len-1];
    cur_dis = option;
    mouse = 0;
}

void menu_var_chg(){//参数修改
    if(bt_left){
        if(cur_dis[all_mouse].type!=no){
            switch(cur_dis[all_mouse].type){
                case int1:  int_var=   (int*)   (cur_dis[all_mouse].ptr);*int_var  -=cur_dis[all_mouse].unit; break;
                case char1: char_var=  (char*)  (cur_dis[all_mouse].ptr);*char_var -=cur_dis[all_mouse].unit; break;
                case float1:float_var= (float*) (cur_dis[all_mouse].ptr);*float_var-=cur_dis[all_mouse].unit; break;
                case int161:short_var= (int16*) (cur_dis[all_mouse].ptr);*short_var-=cur_dis[all_mouse].unit; break;

                case no:break;
            }
        }
        bt_left=0;
    }

    if(bt_right){
        if(cur_dis[all_mouse].type!=no){
            switch(cur_dis[all_mouse].type){
                case int1:  int_var=   (int*)   (cur_dis[all_mouse].ptr);*int_var  +=cur_dis[all_mouse].unit; break;
                case char1: char_var=  (char*)  (cur_dis[all_mouse].ptr);*char_var +=cur_dis[all_mouse].unit; break;
                case float1:float_var= (float*) (cur_dis[all_mouse].ptr);*float_var+=cur_dis[all_mouse].unit; break;
                case int161:short_var= (int16*) (cur_dis[all_mouse].ptr);*short_var+=cur_dis[all_mouse].unit; break;
                case no:break;
            }
        }
        bt_right=0;
    }
}

void menu_mouse(){
    if(bt_prs&&
      (bt_up||
       bt_down)){
        ips200_clear();
        if(bt_up){
            all_mouse--;//向上翻
            bt_up=0;
        }

        if(bt_down){
            all_mouse++;//向下翻
            bt_down=0;
        }

        if(all_mouse<0) all_mouse=all_cur_dis_len-1;
        if(all_mouse>all_cur_dis_len-1) all_mouse=0;//切换界面

        mouse = all_mouse%11;

        bt_prs=0;
    }
}

void menu_itf_switch(){//菜单界面切换
    if(bt_ok){
        if(cur_dis[all_mouse].type==no){
            all_cur_dis_len=var_len[all_mouse];
            cur_dis = (Menu_var*)cur_dis[all_mouse].ptr;
            mouse = 0;
            all_mouse=0;
            ips200_clear();
        }
        bt_ok=0;
    }
    if(bt_bk){
        if(cur_dis[all_mouse].type!=no){
            all_cur_dis_len=var_len[len_var_len-1];
            cur_dis = option;
            mouse = 0;
            all_mouse=0;
            ips200_clear();
            //存数据
            var_save();
        }
        bt_bk=0;
    }
    if(all_cur_dis_len<=11){
            cur_dis_len = all_cur_dis_len;//当前显示列表长度
        }else{
            cur_dis_len = all_cur_dis_len-((all_mouse+1)/11);//当前显示列表长度
        }
}

void menu_display(){//显示函数

    menu_mouse();
    menu_itf_switch();
    ips200_show_string(0,dis_head+dis_unit*mouse*2, "->");
    menu_var_chg();

    for(int i=0;i<cur_dis_len;i++){
        //固定显示部分,翻页刷新
        ips200_show_string(dis_L_lmt,dis_head+dis_unit*i*2, cur_dis[i].name);
        //反复刷新部分
        if(cur_dis[i].type!=no){
            switch(cur_dis[i].type){
                case int1:   int_var=   (int*)   (cur_dis[i].ptr); ips200_show_int  (150, dis_head+dis_unit*i*2,(*int_var), 5);     break;
                case float1: float_var= (float*) (cur_dis[i].ptr); ips200_show_float(150, dis_head+dis_unit*i*2,(*float_var), 5,4); break;
                case char1:  char_var=  (char*)  (cur_dis[i].ptr); ips200_show_int  (150, dis_head+dis_unit*i*2,(*char_var),3);     break;
                case int161: short_var= (int16*) (cur_dis[i].ptr); ips200_show_int  (150, dis_head+dis_unit*i*2,(*short_var),3);    break;
                case no:  break;//无参数的不显示
            }
        }
    }


}




