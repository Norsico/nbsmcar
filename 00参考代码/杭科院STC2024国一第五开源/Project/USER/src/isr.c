#include "SystemConfig.h"
extern ele flag;
extern float oci;
volatile uint32 SysTickFlag = 0;

void TM0_Isr() interrupt 1
{
	SysTickFlag++;

}

void System_Odometer()
{
    get_angle();
    if (flag.all   == 1)      {ele_count   += (float)(lvsp+rvsp)/225;yaw_count+=myabs(yaw);}    
    if (flag.all   == 0)      {ele_count   = 0;yaw_count=0;}  
    if (!runflag)             {oci+=(float)((lvsp+rvsp)/225);}
    else                      {oci=0;}
}

void System_Timer()
{
}

void TM1_Isr() interrupt 3
{
    // Slope_Data=adc_once(ADC_P11, ADC_12BIT);
    Get_Speed();
    System_Odometer();
    Speed_Judge();
    mt_akermann();
    // LMotor_PI(lsetv,0);
    // RMotor_PI(rsetv,0);
}
void TM2_Isr() interrupt 12
{
	TIM2_CLEAR_FLAG;  
    if(!flag.cross&&!flag.slope){Servo_PID(eCCD[0]);}
    // if( flag.cross&&!flag.slope){Servo_PID(eCCD[1]*0.3);}
    if(oci>120){Em_Stop();} //a_time+=0.005;
}

void TM3_Isr() interrupt 19
{
	TIM3_CLEAR_FLAG; 
	
}

void TM4_Isr() interrupt 20
{
	TIM4_CLEAR_FLAG; 
	ccd_collect();	 
    
}

//UART1�ж�
void UART1_Isr() interrupt 4
{
    uint8 res;
	static uint8 dwon_count;
    if(UART1_GET_TX_FLAG)
    {
        UART1_CLEAR_TX_FLAG;
        busy[1] = 0;
    }
    if(UART1_GET_RX_FLAG)
    {
        UART1_CLEAR_RX_FLAG;
        res = SBUF;
        //�����Զ�����
        if(res == 0x7F)
        {
            if(dwon_count++ > 20)
                IAP_CONTR = 0x60;
        }
        else
        {
            dwon_count = 0;
        }
    }
}

//UART2�ж�
void UART2_Isr() interrupt 8
{
    if(UART2_GET_TX_FLAG)
	{
        UART2_CLEAR_TX_FLAG;
		busy[2] = 0;
	}
    if(UART2_GET_RX_FLAG)
	{
        UART2_CLEAR_RX_FLAG;
		//�������ݼĴ���Ϊ��S2BUF

	}
}


//UART3�ж�
void UART3_Isr() interrupt 17
{
    if(UART3_GET_TX_FLAG)
	{
        UART3_CLEAR_TX_FLAG;
		busy[3] = 0;
	}
    if(UART3_GET_RX_FLAG)
	{
        UART3_CLEAR_RX_FLAG;
		//�������ݼĴ���Ϊ��S3BUF

	}
}


//UART4�ж�
void UART4_Isr() interrupt 18
{
    if(UART4_GET_TX_FLAG)
	{
        UART4_CLEAR_TX_FLAG;
		busy[4] = 0;
	}
    if(UART4_GET_RX_FLAG)
	{
        UART4_CLEAR_RX_FLAG;
		//�������ݼĴ���Ϊ��S4BUF;
		if(wireless_type == WIRELESS_SI24R1)
        {
            wireless_uart_callback();           //����ת���ڻص�����
        }
        else if(wireless_type == WIRELESS_CH9141)
        {
            bluetooth_ch9141_uart_callback();   //����ת���ڻص�����
        }

	}
}

#define LED P52
void INT0_Isr() interrupt 0
{
	LED = 0;	//����LED
}
void INT1_Isr() interrupt 2
{

}
void INT2_Isr() interrupt 10
{
	INT2_CLEAR_FLAG;  //����жϱ��?
}
void INT3_Isr() interrupt 11
{
	INT3_CLEAR_FLAG;  //����жϱ��?
}

void INT4_Isr() interrupt 16
{
	INT4_CLEAR_FLAG;  //����жϱ��?
}


extern void pit_callback(void);


//void  INT0_Isr()  interrupt 0;
//void  TM0_Isr()   interrupt 1;
//void  INT1_Isr()  interrupt 2;
//void  TM1_Isr()   interrupt 3;
//void  UART1_Isr() interrupt 4;
//void  ADC_Isr()   interrupt 5;
//void  LVD_Isr()   interrupt 6;
//void  PCA_Isr()   interrupt 7;
//void  UART2_Isr() interrupt 8;
//void  SPI_Isr()   interrupt 9;
//void  INT2_Isr()  interrupt 10;
//void  INT3_Isr()  interrupt 11;
//void  TM2_Isr()   interrupt 12;
//void  INT4_Isr()  interrupt 16;
//void  UART3_Isr() interrupt 17;
//void  UART4_Isr() interrupt 18;
//void  TM3_Isr()   interrupt 19;
//void  TM4_Isr()   interrupt 20;
//void  CMP_Isr()   interrupt 21;
//void  I2C_Isr()   interrupt 24;
//void  USB_Isr()   interrupt 25;
//void  PWM1_Isr()  interrupt 26;
//void  PWM2_Isr()  interrupt 27;