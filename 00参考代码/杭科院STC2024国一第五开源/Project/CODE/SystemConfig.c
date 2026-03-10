#include "SystemConfig.h"
extern LADRC_NUM LADRC;
extern ele flag;

/*BDLC*/
int BDLC_PWM=6250;

/*ZEBRA*/
uint8 zebra_num=10;

/*Circle*/
uint8 lpu=46;
uint8 rpu=78;
uint8 lpd=35;
uint8 rpd=85;

/*Judge*/
uint8 up_limit=8;
uint8 down_limit=8;
uint8 up_lose=11;
uint8 down_lose=11;

/*Process*/
uint8 icm_in=50;
uint8 icm_out=65;
uint8 cir_inp=20;
uint8 cir_outp=18;
uint8 cir_incheck=70;
uint8 cir_outcheck=150;
 
uint8 Slope_Data = 0;
/*Cross*/
uint8 cross_err = 15;
uint8 crossf_max= 8;

uint8 Spid_Sel=2;
float P_Rate=5; 
float ServoP=18;//basic
float MaxP=18;
float ServoD=100;
int Servo_Mid=5070;
int Servo_pc=550;
int16 Servo_Out=0;
float UFF[7] = { 14.1,
                 14.3, 
                 14.7, 
                 15.2, 
                 15.9, 
                 16.6,
                 16.8 };

/*SpeedJudge*/
float dv=120;//77
float l_dv=30;//77
float r_dv=30;//77
int tarsp;

uint16 stright_tarsp=270;//
uint16 normal_tarsp=230;//
uint8 cross_tarsp=0;
uint8 circle_tarsp=225;
uint8 slope_tarsp=160; 

// uint16 stright_tarsp=200;//
// uint16 normal_tarsp=180;//
// uint8 cross_tarsp=0;
// uint8 circle_tarsp=180;
// uint8 slope_tarsp=100; 

/*Motor&ENCONDER*/

/*PI For Speed*/
float LMp=32;  	float RMp=32;
float LMi=13.5;  float RMi=13.5;
float LMd=5;    float RMd=5;


int lsetv = 0,rsetv = 0;
float lvsp  = 0,rvsp  = 0;

/*Flag*/
float a_time=0;
int runflag=1;

uint8 l_num=0;
uint8 r_num=0;
uint8 slope_nco=0;
uint8 cross_nco=0;
uint8 last_ele=0;
uint8 ele_num=0;

uint8 lcir_num=2;
uint8 rcir_num=2;
uint8 cross_num=5;
uint8 slope_num=2;

/*GRYO DATA*/
float pitch = 0;
float yaw = 0;
float yaw_count=0;
float pitch_err=0;

/*Camera*/
uint16 threshold=0,threshold1=0;
uint8 old_mid=63,old_mid1=63;
uint8 Comapre_0 = 33;
uint8 Comapre_1 = 35;
int8 ccd_offset=-3;
int8 ccd1_offset=-1;
uint8 lp[3]={0};
uint8 rp[3]={127};
uint8 keepline=65;
uint8 incirline=70;
uint8 outcirline=70;
float eCCD[3] = {0};
float ele_count=0;
float down_arr=0.8;
float up_arr=0.5 ;
float down_dec=0;
float up_dec=0;//10




float myabs(float x)
{
    if (x>=0)   {return  x;}
    else        {return -x;}
}


void Device_initialize()
{
    oled_init_spi(); 
	      
    ccd_init();             
    Servo_init();           
	Motor_Init();           
    // adc_init(ADC_P11,0x01); 
    adc_init(ADC_P17,0x01);
	gpio_mode(P5_1,GPO_PP); 
    gpio_mode(P5_2,GPIO);  
    gpio_mode(P1_1,GPIO);
    icm20602_init(); 

    tarsp=normal_tarsp;     //ORGINAL TARSPEED
    flag.start=1;           //FLAGINIT
    flag.cross=0;
    flag.lcircle=0;
    flag.rcircle=0;
    flag.all=0;
    flag.slope=0;           



    pit_timer_ms(TIM_0,5);	
    pit_timer_ms(TIM_1,5);
    pit_timer_ms(TIM_2,5);
    // pit_timer_ms(TIM_3,5);  

    // delay_ms(2000); 
}

