#include  "SystemConfig.h"
extern ele flag;
extern LADRC_NUM LADRC;
float oci =0;

uint16 sum=0;
void Show_Data()
{

	
}   

void main()
{
	clock_init(SYSTEM_CLOCK_56M);
	board_init();
	// Ladrc_Init(&LADRC);
	// Ladrc_Reset(&LADRC);
	Device_initialize();

    while(1)
    {
		LMOTOR_FORWARD;pwm_duty(LMOTOR_PWM, 800);
		RMOTOR_FORWARD;pwm_duty(RMOTOR_PWM, 800);
		delay_ms(2000);
		LMOTOR_RETREAT;pwm_duty(LMOTOR_PWM, 800);
		RMOTOR_RETREAT;pwm_duty(RMOTOR_PWM, 800);
		delay_ms(2000);
		// Show_Data();
		// ccd_mode(compare,KEEP_NORMAL_LINE);
        // elements();
		// SystemCtrl();
    }
}

void Car_Debug()
{
	runflag=1;
}

void BEM_START()
{
	clear(0x00);
	pwm_init(PWMB_CH2_P01,333,3000);
	pwm_init(PWMB_CH1_P00,333,3000);
	delay_ms(1000);	
	pwm_duty(PWMB_CH2_P01,5500);
	pwm_duty(PWMB_CH1_P00,5500);
	delay_ms(1000);	
	pwm_duty(PWMB_CH2_P01,BDLC_PWM);
	pwm_duty(PWMB_CH1_P00,BDLC_PWM);
	delay_ms(1000);	
	runflag=0;		
}

void SystemCtrl()
{	
	if (SetMenu)					{Menu_display();}
	if (IMAGE_M)					{img1_get();image_ori_bin();}
	if (ST_CAR&&key_right)			{if(runflag)BEM_START();}		   
	if (DEBUG_CAR)					
	{
		runflag=1;
		ele_num=0;
		oci=0;
		l_num=0;
		r_num=0;
		slope_nco=0;
		cross_nco=0;
		last_ele=0;
	}
	if (TEST_CAR)
	{
		if (key_right)
		oled_init_spi(); 
	}
	
}

