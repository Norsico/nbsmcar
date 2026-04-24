/*
 * menu_dis.h
 *
 *  Created on: 2025年5月11日
 *      Author: 26672
 */
#ifndef CODE_MENU_DIS_H_
#define CODE_MENU_DIS_H_

#define dis_unit 8 //每个字符都是16x8的点阵

#define dis_head 100
#define dis_botm (2*11*dis_unit+dis_head)
#define dis_L_lmt (2*dis_unit)
#define dis_R_lmt (240-1-dis_unit)

extern char cur_dis_len;

void menu_dis_init();
void menu_display();

#endif



