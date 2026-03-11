#ifndef IIC_OLED_H
#define IIC_OLED_H

#include "stdlib.h"	 
#include "headfile.h"

#define OLED_MODE 0
#define SIZE 8
#define XLevelL		0x00
#define XLevelH		0x10
#define Max_Column	128
#define Max_Row		64
#define	Light_Set	0xFF 
#define X_WIDTH 	128
#define Y_WIDTH 	64	    						  
//-----------------OLED IIC端口操作----------------  					   

#define OLED_SCLK_Clr() P25=0
#define OLED_SCLK_Set() P25=1

#define OLED_SDIN_Clr() P24=0
#define OLED_SDIN_Set() P24=1

 		     
#define OLED_CMD  0	//命令模式
#define OLED_DATA 1	//数据模式


//OLED操作方法
void OLED_WR_Byte(unsigned dat,unsigned cmd);  
void OLED_Display_On(void);
void OLED_Display_Off(void);	   							   		    
void MyOLED_Init(void);
void OLED_Clear(void);
// void OLED_DrawPoint(uint8 x,uint8 y,uint8 t);
// void OLED_Fill(uint8 x1,uint8 y1,uint8 x2,uint8 y2,uint8 dot);
void OLED_ShowChar(uint8 x,uint8 y,uint8 chr,uint8 Char_Size);
void OLED_ShowNumber(uint8 x, uint8 y, uint8 num, uint8 len, uint8 size);
void OLED_ShowString(uint8 x,uint8 y, char *p,uint8 Char_Size);	 
void MyOLED_Set_Pos(uint8  x, uint8  y);
void OLED_ShowCHinese(uint8 x, uint8 y);
void OLED_DrawBMP(unsigned char x0, unsigned char y0,unsigned char x1, unsigned char y1,unsigned char BMP[]);
void Delay_50ms(unsigned int Del_50ms);
void Delay_1ms(unsigned int Del_1ms);
void fill_picture(unsigned char fill_Data);
// void Picture(void);
void IIC_Start(void);
void IIC_Stop(void);
void Write_IIC_Command(unsigned char IIC_Command);
void Write_IIC_Data(unsigned char IIC_Data);
void Write_IIC_Byte(unsigned char IIC_Byte);

void IIC_Wait_Ack(void);
#endif
