/*
 * display.h
 *
 *  Created on: 2024年3月1日
 *      Author: Kurumi
 */



#ifndef DISPLAY_H_
#define DISPLAY_H_

void display(void);
void ips114(void);
void drawleftline(void);          //画左边线
void drawrightline(void);         //画右边线
void drawcenterline(void);        //画中线
void drawoffline(void);           //画图像顶边
void drawtowpointUP(void);        //画前瞻范围上线
void drawtowpointDOWN(void);      //画前瞻范围下线
void menu_jump();
void menu_fall();
void menu_back();
void menu_long_back();
void ips114_1(void);
void ips114_2(void);
void ips114_3(void);
void ips114_4(void);
void wave_display(void);
void draw_speed(void);
void draw_black_line_1(void);
#endif /* DISPLAY_H_ */
