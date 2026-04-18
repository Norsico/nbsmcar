/*
 * tft180.h
 *
 *  Created on: 2024年3月3日
 *      Author: xiaoming
 */

#ifndef TFT180_H_
#define TFT180_H_

void tft180(void);
void drawleftline(void);          //画左边线
void drawrightline(void);         //画右边线
void drawcenterline(void);        //画中线
void drawoffline(void);           //画图像顶边
void drawtowpointUP(void);        //画前瞻范围上线
void drawtowpointDOWN(void);      //画前瞻范围下线
void draw_math(void);                 //显示半宽
#endif /* TFT180_H_ */
