#include "SystemConfig.h"
extern int lspeed;
extern int rspeed;
extern ele flag;

void Motor_Init()
{
    gpio_mode(P3_2,GPIO);
    gpio_mode(P0_6,GPIO);
    ctimer_count_init(CTIM0_P34);
    ctimer_count_init(CTIM3_P04);

    gpio_mode(LMOTOR_DIR,GPO_PP);
    gpio_mode(RMOTOR_DIR,GPO_PP);
       
    pwm_init(LMOTOR_PWM,17000,0);
    pwm_init(RMOTOR_PWM,17000,0);
}

void Get_Speed()
{
    if(L_ENCONDER_FORWARD)  {lvsp= 1*ctimer_count_read(CTIM0_P34);ctimer_count_clean(CTIM0_P34);}
    else                    {lvsp=-1*ctimer_count_read(CTIM0_P34);ctimer_count_clean(CTIM0_P34);}
    if(R_ENCONDER_FORWARD)  {rvsp= 1*ctimer_count_read(CTIM3_P04);ctimer_count_clean(CTIM3_P04);}
    else                    {rvsp=-1*ctimer_count_read(CTIM3_P04);ctimer_count_clean(CTIM3_P04);}
}


void LMotor_PI(int16 target,int16 eyaw)
{
    static float lMotor_Out;
    static float llast_err; 
    static float llast_derivative; 
    float err;
    err = target-lvsp;

    lMotor_Out+=LMp*(err-llast_err)+LMi*err+LMd*((err-llast_err)-llast_derivative);
        llast_err=err;
    llast_derivative=err-llast_err;

    // if (flag.start==1)      {lMotor_Out=0;}

    if(lMotor_Out> Motor_MAX)    {lMotor_Out= Motor_MAX;}
    if(lMotor_Out< Motor_MIN)    {lMotor_Out= Motor_MIN;}

    if (lMotor_Out>=0)      {LMOTOR_FORWARD;pwm_duty(LMOTOR_PWM, lMotor_Out);}
    if (lMotor_Out<0)       {LMOTOR_RETREAT;pwm_duty(LMOTOR_PWM,-lMotor_Out);}
}

void RMotor_PI(int16 target,int16 eyaw)
{
    static float rMotor_Out;
    static float rlast_err; 
    static float rlast_derivative; 
    float err;
    err = target-rvsp-eyaw;

    rMotor_Out+=RMp*(err-rlast_err)+RMi*err+RMd*((err-rlast_err)-rlast_derivative);
    rlast_err=err;
    rlast_derivative=err-rlast_err;
    // if (flag.start==1)      {rMotor_Out=0;}

    if(rMotor_Out> Motor_MAX)    {rMotor_Out= Motor_MAX;}
    if(rMotor_Out< Motor_MIN)    {rMotor_Out= Motor_MIN;}

    if (rMotor_Out>=0)      {RMOTOR_FORWARD;pwm_duty(RMOTOR_PWM, rMotor_Out);}
    if (rMotor_Out<0)       {RMOTOR_RETREAT;pwm_duty(RMOTOR_PWM,-rMotor_Out);}
}

void START_CAR()
{
    while (flag.start)
    {
        flag.start=1;
        runflag=1;
        flag.all=1;
        // if (ele_count<8)
        // {
        //     CCD_Process(KEEP_NORMAL_LINE);
        //     Motor_Forward(start_tarsp,start_tarsp);
        // }
        // else
        // {          

            flag.all=0;
            ele_count=0;
            break;
        // }
    }
    
}

void mt_akermann()
{ 
    float turn_angle    =   0;
    float v_dif         =   0;

    uint8 bdlc_kp=2.5;

    uint16 BDLC_L=0;
    uint16 BDLC_R=0;
    turn_angle  =   (Servo_Mid-Servo_Out)/7.5;
    v_dif       =   dv*0.01*tan(turn_angle*3.1415926/180)*0.424;

    if (turn_angle>= 20) {turn_angle= 20;}
    if (turn_angle<=-20) {turn_angle=-20;}
                                                   
    if (turn_angle>0)   {lsetv=tarsp;rsetv=(1-v_dif)*tarsp;}//(1+v_dif)*
    else                {rsetv=tarsp;lsetv=(1+v_dif)*tarsp;}//(1-v_dif)*

    // if (turn_angle>=0)  //right 
    // {
    //     BDLC_L=BDLC_PWM;
    //     BDLC_R=BDLC_PWM+turn_angle*bdlc_kp;
    // }
    // if (turn_angle<0)             
    // {
    //     BDLC_L=BDLC_PWM-turn_angle*bdlc_kp;
    //     BDLC_R=BDLC_PWM;
    // }

    // if (BDLC_L>=6250){BDLC_L=6250;}
    // if (BDLC_R>=6250){BDLC_R=6250;}
        
    // if (BDLC_L<=5500){BDLC_L=5500;}
    // if (BDLC_R<=5500){BDLC_R=5500;}


    // if (!runflag)
    // {
    //     pwm_duty(PWMB_CH1_P00,BDLC_L);
    //     pwm_duty(PWMB_CH2_P01,BDLC_R);  
    // }
    
   
	
}

void Speed_Judge()
{
    static float addspeed_count=0;
    if (myabs(eCCD[1])<=5&&myabs(eCCD[0])<=3&&rp[1]-lp[1]<rpu-lpu+6&&!flag.lcircle&&!flag.rcircle&&!flag.slope)
    {
        addspeed_count+=((float)lvsp+(float)rvsp)/225;
        if (addspeed_count>10)   {tarsp = stright_tarsp;} 
        else  tarsp=normal_tarsp;
    } 
    else if (!flag.lcircle&&!flag.rcircle&&!flag.slope)
    {
        addspeed_count=0;
        tarsp=normal_tarsp;                                
    }
    if (flag.lcircle||flag.rcircle)
    {
        tarsp=circle_tarsp;
    }
    if (flag.slope)
    {
        tarsp=slope_tarsp;
    } 
    if (runflag)
    {
        tarsp=0;
    }
}