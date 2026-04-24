/*
 * image_process.h
 *
 *  Created on: 2025年2月25日
 *      Author: 15958
 */

#ifndef CODE_IMAGE_PROCESS_H_
#define CODE_IMAGE_PROCESS_H_

#include "Platform_Types.h"                  //不加这个头文件没法使用外部声明extern 会导致识别不到

#define LCDH 60//40                              //用于图像处理的图像高度（行）
#define LCDW 80                              //用于图像处理的图像宽度（列）
extern  uint8 Image_Use[LCDH][LCDW];      //用来存储压缩之后灰度图像的二维数组
extern  uint8 Pixle[LCDH][LCDW];          //图像处理时真正处理的二值化图像数组
extern  uint8 Pixle_hb[LCDH][LCDW];
extern uint8 Threshold;                                //通过大津法计算出来的前20行二值化阈值
void Image_Compress(void);
uint8 Get_Threshold(uint8* image,uint16 col, uint16 row);
void Get_BinaryImage(void);
void Pixle_Filter();
void Image_Process(void);
#endif /* CODE_IMAGE_PROCESS_H_ */
