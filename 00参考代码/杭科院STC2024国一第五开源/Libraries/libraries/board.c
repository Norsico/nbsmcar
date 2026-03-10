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


#include "board.h"
#include "zf_uart.h"
#include "zf_tim.h"
#include "zf_delay.h"

//�ں�Ƶ��
int32 sys_clk = 0;

//-------------------------------------------------------------------------------------------------------------------
//  @brief      STC32G����ϵͳƵ��
//  @param      NULL          	��ֵ
//  @return     void        	ϵͳƵ��
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
uint32 set_clk(uint32 clock)
{
	uint32 temp_sys_clk = 0;
	
	
    switch(clock)
    {
		case SYSTEM_CLOCK_500K:
        //ѡ��500KHz
        CLKDIV = 0x04;
        IRTRIM = T24M_ADDR;
        VRTRIM = VRT24M_ADDR;
        IRCBAND = IRCBAND_24M;
        WTST = 0;       //���ó����ȡ�ȴ����ƼĴ���
		_nop_();
        _nop_();
        _nop_();
        _nop_();
        CLKDIV = 48;
		temp_sys_clk = clock;
        break;

    case SYSTEM_CLOCK_3M:
        //ѡ��3MHz
        CLKDIV = 0x04;
        IRTRIM = T24M_ADDR;
        VRTRIM = VRT24M_ADDR;
        IRCBAND = IRCBAND_24M;
        WTST = 0;       //���ó����ȡ�ȴ����ƼĴ���
		_nop_();
		_nop_();
		_nop_();
		_nop_();
        CLKDIV = 8;
		temp_sys_clk = clock;
        break;

    case SYSTEM_CLOCK_55_296M:
        //ѡ��5.5296MHz
        CLKDIV = 0x04;
        IRTRIM = T22M_ADDR;
        VRTRIM = VRT24M_ADDR;
        IRCBAND = IRCBAND_24M;
        WTST = 0;       //���ó����ȡ�ȴ����ƼĴ���
		_nop_();
		_nop_();
		_nop_();
		_nop_();
        CLKDIV = 4;
		temp_sys_clk = clock;
        break;

    case SYSTEM_CLOCK_6M:
        //ѡ��6MHz
        CLKDIV = 0x04;
        IRTRIM = T24M_ADDR;
        VRTRIM = VRT24M_ADDR;
        IRCBAND = IRCBAND_24M;
		_nop_();
		_nop_();
		_nop_();
		_nop_();
        WTST = 0;       //���ó����ȡ�ȴ����ƼĴ���
        CLKDIV = 4;
		temp_sys_clk = clock;
        break;

    case SYSTEM_CLOCK_11_0592M:
        //ѡ��11.0592MHz
        CLKDIV = 0x04;
        IRTRIM = T22M_ADDR;
        VRTRIM = VRT24M_ADDR;
        IRCBAND = IRCBAND_24M;
        WTST = 0;       //���ó����ȡ�ȴ����ƼĴ���
		_nop_();
		_nop_();
		_nop_();
		_nop_();
        CLKDIV = 2;
		temp_sys_clk = clock;
        break;

    case SYSTEM_CLOCK_12M:
        //ѡ��12MHz
        CLKDIV = 0x04;
        IRTRIM = T24M_ADDR;
        VRTRIM = VRT24M_ADDR;
        IRCBAND = IRCBAND_24M;
        WTST = 0;       //���ó����ȡ�ȴ����ƼĴ���
		_nop_();
		_nop_();
		_nop_();
		_nop_();
        CLKDIV = 2;
		temp_sys_clk = clock;
        break;

    case SYSTEM_CLOCK_22_1184M:
        //ѡ��22.1184MHz
        CLKDIV = 0x04;
        IRTRIM = T22M_ADDR;
        VRTRIM = VRT24M_ADDR;
        IRCBAND = IRCBAND_24M;
        WTST = 0;       //���ó����ȡ�ȴ����ƼĴ���
		_nop_();
		_nop_();
		_nop_();
		_nop_();
        CLKDIV = 1;
		temp_sys_clk = clock;
        break;

    case SYSTEM_CLOCK_24M:
        //ѡ��24MHz
        CLKDIV = 0x04;
        IRTRIM = T24M_ADDR;
        VRTRIM = VRT24M_ADDR;
        IRCBAND = IRCBAND_24M;
        WTST = 0;       //���ó����ȡ�ȴ����ƼĴ���
		_nop_();
		_nop_();
		_nop_();
		_nop_();
        CLKDIV = 1;
		temp_sys_clk = clock;
        break;

    case SYSTEM_CLOCK_30M:
        //ѡ��30MHz
        CLKDIV = 0x04;
        IRTRIM = T60M_ADDR;
        VRTRIM = VRT44M_ADDR;
        IRCBAND = IRCBAND_44M;
        WTST = 0;       //���ó����ȡ�ȴ����ƼĴ���
		_nop_();
		_nop_();
		_nop_();
		_nop_();
        CLKDIV = 2;
		temp_sys_clk = clock;
        break;

    case SYSTEM_CLOCK_32M:
        //ѡ��32MHz
        CLKDIV = 0x04;
        IRTRIM = T64M_ADDR;
        VRTRIM = VRT44M_ADDR;
        IRCBAND = IRCBAND_44M;
        WTST = 0;       //���ó����ȡ�ȴ����ƼĴ���
		_nop_();
		_nop_();
		_nop_();
		_nop_();
        CLKDIV = 2;
		temp_sys_clk = clock;
        break;

    case SYSTEM_CLOCK_40M:
        //ѡ��40MHz
        CLKDIV = 0x04;
        IRTRIM = T40M_ADDR;
        VRTRIM = VRT44M_ADDR;
        IRCBAND = IRCBAND_44M;
        WTST = 1;       //���ó����ȡ�ȴ����ƼĴ���
		_nop_();
		_nop_();
		_nop_();
		_nop_();
        CLKDIV = 1;
		temp_sys_clk = clock;
        break;

    case SYSTEM_CLOCK_45_1584M:
        //ѡ��45.1584MHz
        CLKDIV = 0x04;
        IRTRIM = T45M_ADDR;
        VRTRIM = VRT44M_ADDR;
        IRCBAND = IRCBAND_44M;
        WTST = 1;       //���ó����ȡ�ȴ����ƼĴ���
		_nop_();
		_nop_();
		_nop_();
		_nop_();
        CLKDIV = 1;
		temp_sys_clk = clock;
        break;

    case SYSTEM_CLOCK_48M:
        //ѡ��48MHz
        CLKDIV = 0x04;
        IRTRIM = T48M_ADDR;
        VRTRIM = VRT44M_ADDR;
        IRCBAND = IRCBAND_44M;
        WTST = 1;       //���ó����ȡ�ȴ����ƼĴ���
		_nop_();
		_nop_();
		_nop_();
		_nop_();
        CLKDIV = 1;
		temp_sys_clk = clock;
        break;

    case SYSTEM_CLOCK_50M:
        //ѡ��50.8032MHz
        CLKDIV = 0x04;
        IRTRIM = T50M_ADDR;
        VRTRIM = VRT44M_ADDR;
        IRCBAND = IRCBAND_44M;
        WTST = 1;       //���ó����ȡ�ȴ����ƼĴ���
		_nop_();
		_nop_();
		_nop_();
		_nop_();
        CLKDIV = 1;
		temp_sys_clk = clock;
        break;

    case SYSTEM_CLOCK_52M:
        //ѡ��52MHz
        CLKDIV = 0x04;
        IRTRIM = T52M_ADDR;
        VRTRIM = VRT44M_ADDR;
        IRCBAND = IRCBAND_44M;
        WTST = 1;       //���ó����ȡ�ȴ����ƼĴ���
		_nop_();
		_nop_();
		_nop_();
		_nop_();
        CLKDIV = 1;
		temp_sys_clk = clock;
        break;

    case SYSTEM_CLOCK_56M:
        //ѡ��56MHz
        CLKDIV = 0x04;
        IRTRIM = T56M_ADDR;
        VRTRIM = VRT44M_ADDR;
        IRCBAND = IRCBAND_44M;
        WTST = 1;       //���ó����ȡ�ȴ����ƼĴ���
		_nop_();
		_nop_();
		_nop_();
		_nop_();
        CLKDIV = 1;
		temp_sys_clk = clock;
        break;

    case SYSTEM_CLOCK_60M:
        //ѡ��60MHz
        CLKDIV = 0x04;
        IRTRIM = T60M_ADDR;
        VRTRIM = VRT44M_ADDR;
        IRCBAND = IRCBAND_44M;
        WTST = 1;       //���ó����ȡ�ȴ����ƼĴ���
		_nop_();
		_nop_();
		_nop_();
		_nop_();
        CLKDIV = 1;
		temp_sys_clk = clock;
        break;

//    case SYSTEM_CLOCK_64M:
//        //ѡ��64MHz
//        CLKDIV = 0x04;
//        IRTRIM = T64M_ADDR;
//        VRTRIM = VRT44M_ADDR;
//        IRCBAND = IRCBAND_44M;
//        WTST = 2;       //���ó����ȡ�ȴ����ƼĴ���
//        CLKDIV = 1;
//        break;

    default:
        //ѡ��56MHz
        CLKDIV = 0x04;
        IRTRIM = T56M_ADDR;
        VRTRIM = VRT44M_ADDR;
        IRCBAND = IRCBAND_44M;
        WTST = 1;       //���ó����ȡ�ȴ����ƼĴ���
		_nop_();
		_nop_();
		_nop_();
		_nop_();
        CLKDIV = 1;
		temp_sys_clk = SYSTEM_CLOCK_56M;
        break;
	}
	
	return temp_sys_clk;
}






#if (1 == PRINTF_ENABLE)      //��ʼ�����Դ���
//�ض���printf ���� ֻ�����uint16
    char putchar(char c)
    {
        uart_putchar(UART_1, c);//���Լ�ʵ�ֵĴ��ڴ�ӡһ�ֽ����ݵĺ����滻������

        return c;
    }
#endif

void DisableGlobalIRQ(void)
{
	EA = 0;
}


void EnableGlobalIRQ(void)
{
	EA = 1;
}

void enalbe_icache(void)
{
	EA = 0;
	_nop_();
	_nop_();
	TA = 0xaa; 		//д�봥����������1
					//�˴������������κ�ָ��
	TA = 0x55; 		//д�봥����������2
					//�˴������������κ�ָ��
	ICHECR = 0x01; 	//д������ʱ�رգ������޸�ICHECR �е�EN λ
					//EN λ�ٴν���д����״̬
	_nop_();
	_nop_();
	EA = 1;
}

void disalbe_icache(void)
{
	EA = 0;
	_nop_();
	_nop_();
	TA = 0xaa; 		//д�봥����������1
					//�˴������������κ�ָ��
	TA = 0x55; 		//д�봥����������2
					//�˴������������κ�ָ��
	ICHECR = 0x00; 	//д������ʱ�رգ������޸�ICHECR �е�EN λ
					//EN λ�ٴν���д����״̬
	_nop_();
	_nop_();
	EA = 1;
}


void ICacheOn() //��ICACHE ����
{
	_nop_();
	_nop_();
	TA = 0xaa; //д�봥����������1
	//�˴������������κ�ָ��
	TA = 0x55; //д�봥����������2
	//�˴������������κ�ָ��
	ICHECR = 0x01; //д������ʱ�رգ������޸�ICHECR �е�EN λ
	//EN λ�ٴν���д����״̬
	_nop_();
	_nop_();
}

void clock_init(uint32 clock)
{
	P_SW2 = 0x80; // ���������ַ����
	
	#if (1 == EXTERNAL_CRYSTA_ENABLE)
	{
		sys_clk = clock;
	}
	#else
	{
		sys_clk = set_clk(clock);
	}
	#endif

}


void board_init(void)
{

	SET_P54_RESRT;			// ʹP54Ϊ��λ����

	
	P0M0 = 0x00;
	P0M1 = 0x00;
	P1M0 = 0x00;
	P1M1 = 0x00;
	P2M0 = 0x00;
	P2M1 = 0x00;
	P3M0 = 0x00;
	P3M1 = 0x00;
	P4M0 = 0x00;
	P4M1 = 0x00;
	P5M0 = 0x00;
	P5M1 = 0x00;   
//	P6M0 = 0x00;
//	P6M1 = 0x00;
//	P7M0 = 0x00;
//	P7M1 = 0x00;
	

#if (1 == EXTERNAL_CRYSTA_ENABLE)
{

	XOSCCR = 0xc0; 			//�����ⲿ����
	while(!(XOSCCR & 1)); 	//�ȴ�ʱ���ȶ�

	CLKDIV = 0x00; 			//ʱ�Ӳ���Ƶ
	CLKSEL = 0x01; 			//ѡ���ⲿ����
}

#endif

	delay_init();		

	if(sys_clk > SYSTEM_CLOCK_32M)
	{
		ICacheOn();
	}
	


	ADCCFG = 0;
	AUXR = 0;
	SCON = 0;
	S2CON = 0;
	S3CON = 0;
	S4CON = 0;
	P_SW1 = 0;
	IE2 = 0;
	TMOD = 0;
//	MCLKOCR =1<<7| 10;
	uart_init(DEBUG_UART, DEBUG_UART_RX_PIN, DEBUG_UART_TX_PIN, DEBUG_UART_BAUD, DEBUG_UART_TIM);
	EnableGlobalIRQ();
}