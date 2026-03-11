#include "headfile.h"

int encoder_L=0,encoder_R=0;
float encoder_integral=0,encoder=0,encoder_integra2=0;
/*******************************************
*************编码器计数函数*****************
*******************************************/
void EncoderCount(void)
{
	   //读取采集到的编码器脉冲数
    encoder_L = ctimer_count_read(SPEEDL_PLUSE);
		encoder_R = ctimer_count_read(SPEEDR_PLUSE);
     //计数器清零
    ctimer_count_clean(SPEEDL_PLUSE);
		ctimer_count_clean(SPEEDR_PLUSE);
		//采集方向信息
		if(1 == SPEEDL_DIR)    
		{
			encoder_L = -encoder_L;
		}

		if(1 == SPEEDR_DIR)    
		{
			encoder_R = -encoder_R;
		}
		encoder=(encoder_L+encoder_R)*0.5;
		encoder_integral=encoder_integral+encoder*0.02;	
		encoder_integra2=encoder_integra2+encoder*0.02;	
}