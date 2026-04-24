/*
 * menu.h
 *
 *  Created on: 2025年5月11日
 *      Author: 26672
 */

#ifndef CODE_MENU_H_
#define CODE_MENU_H_

enum var_type{
    int1,
    float1,
    char1,
    no,
    int161,
    uint81,
    uint161
};

typedef struct{
 char name[40];//变量名
 void* ptr;//变量指针
 float unit;//增减量
 enum var_type type;

}Menu_var;//变量结构体

extern Menu_var var1[];
extern Menu_var option[];
extern uint8 var_len[];
extern uint8 len_var_len;
extern uint8 data_buff[255];

extern void var_save();
extern void var_load();


#endif /* CODE_MENU_H_ */
