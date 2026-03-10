/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��һȺ��179029047(����)  ��Ⱥ��244861897(����)  ��Ⱥ��824575535
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ����������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		board
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ790875685)
 * @version    		�鿴doc��version�ļ� �汾˵��
 * @Software 		MDK FOR C251 V5.60
 * @Target core		STC32F12K
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2020-4-14
 ********************************************************************************************************************/



#ifndef __BOARD_H
#define __BOARD_H
#include "common.h"




#define EXTERNAL_CRYSTA_ENABLE  0			// ʹ���ⲿ����0Ϊ��ʹ�ã�1Ϊʹ�ã�����ʹ���ڲ�����
#define PRINTF_ENABLE			1			// printfʹ�ܣ�0Ϊʧ�ܣ�1Ϊʹ��
#define ENABLE_IAP 				1			// ʹ������һ�����ع��ܣ�0Ϊʧ�ܣ�1Ϊʹ��

#define DEBUG_UART 			  	UART_1
#define DEBUG_UART_BAUD 	  	115200
#define DEBUG_UART_RX_PIN  		UART1_RX_P30
#define DEBUG_UART_TX_PIN  		UART1_TX_P31
#define DEBUG_UART_TIM			TIM_2

#if (1==PRINTF_ENABLE)
	char putchar(char c);
#endif

#define SET_P54_RESRT 	  (RSTCFG |= 1<<4)	//����P54Ϊ��λ����



#define SYSTEM_CLOCK_500K 		( 500000   )	
#define SYSTEM_CLOCK_3M   		( 3000000  )
#define SYSTEM_CLOCK_55_296M	( 5529600  )
#define SYSTEM_CLOCK_6M			( 6000000  )
#define SYSTEM_CLOCK_11_0592M	( 11059200 )
#define SYSTEM_CLOCK_12M 		( 12000000 )
#define SYSTEM_CLOCK_22_1184M	( 22118400 )
#define SYSTEM_CLOCK_24M		( 24000000 )
#define SYSTEM_CLOCK_30M		( 30000000 )
#define SYSTEM_CLOCK_32M		( 32000000 )
#define SYSTEM_CLOCK_40M		( 40000000 )
#define SYSTEM_CLOCK_45_1584M	( 45158400 )
#define SYSTEM_CLOCK_48M		( 48000000 )
#define SYSTEM_CLOCK_50M		( 50803200 )
#define SYSTEM_CLOCK_52M		( 52000000 )
#define SYSTEM_CLOCK_56M		( 56000000 )
#define SYSTEM_CLOCK_60M		( 60000000 )
//#define SYSTEM_CLOCK_64M		( 64000000 )


typedef enum
{
	IRCBAND_20M = 0,		
	IRCBAND_24M,
	IRCBAND_44M,
	IRCBAND_72M,
}ircband_sel_enum;


extern int32 sys_clk;
void clock_init(uint32 clock);
void board_init(void);
void DisableGlobalIRQ(void);
void EnableGlobalIRQ(void);

#endif

