#include "SystemConfig.h"

uint8 old_ccd[128]={0};
uint8 old_img[128]={0};
uint8 old_img1[128]={0};
uint8 old_img2[128]={0};

// uint32 FLASH_DATA[200];
int menupagenum =9;
int keymode=1;
int func=0;
float Dvi=1;
void (*fx)(void);
void voidfx();
void mainbackground();
void Servo_Menu();
void Motor_Menu();
void Circle_Judge_Menu();
void Circle_Process_Menu();
void Camera_Menu();
void Speed_Planning_Menu();
void mode_set();

void voidfx(){}
/*
    int Num;
    int Next;
    int Back;
    int Enter;
    int Esc;
    void (*fx)();
*/
Menu table[100]=
{
    //num,next,back,enter,exit
    /*Main_Menu*/
    {PG0+0,PG0+1,PG0+7,PG1,PG0+0,(*mainbackground)},
    {PG0+1,PG0+2,PG0+0,PG2,PG0+1,(*mainbackground)},
    {PG0+2,PG0+3,PG0+1,PG3,PG0+2,(*mainbackground)},
    {PG0+3,PG0+4,PG0+2,PG4,PG0+3,(*mainbackground)},
    {PG0+4,PG0+5,PG0+3,PG5,PG0+4,(*mainbackground)},
    {PG0+5,PG0+6,PG0+4,PG6,PG0+5,(*mainbackground)},
    {PG0+6,PG0+7,PG0+5,PG7,PG0+6,(*mainbackground)},
    {PG0+7,PG0+0,PG0+6,PG8,PG0+7,(*mainbackground)},
        //num,next,pre,enter,exit
        /*Servo*/
        {PG1+0,PG1+1,PG1+7,PG1+0,0,(*Servo_Menu)},
        {PG1+1,PG1+2,PG1+0,PG1+1,0,(*Servo_Menu)},
        {PG1+2,PG1+3,PG1+1,PG1+2,0,(*Servo_Menu)},
        {PG1+3,PG1+4,PG1+2,PG1+3,0,(*Servo_Menu)},
        {PG1+4,PG1+5,PG1+3,PG1+4,0,(*Servo_Menu)},
        {PG1+5,PG1+6,PG1+4,PG1+5,0,(*Servo_Menu)},
        {PG1+6,PG1+7,PG1+5,PG1+6,0,(*Servo_Menu)},
        {PG1+7,PG1+0,PG1+6,PG1+7,0,(*Servo_Menu)},

        {PG2+0,PG2+1,PG2+7,PG2+0,1,(*Motor_Menu)},
        {PG2+1,PG2+2,PG2+0,PG2+1,1,(*Motor_Menu)},
        {PG2+2,PG2+3,PG2+1,PG2+2,1,(*Motor_Menu)},
        {PG2+3,PG2+4,PG2+2,PG2+3,1,(*Motor_Menu)},
        {PG2+4,PG2+5,PG2+3,PG2+4,1,(*Motor_Menu)},
        {PG2+5,PG2+6,PG2+4,PG2+5,1,(*Motor_Menu)},
        {PG2+6,PG2+7,PG2+5,PG2+6,1,(*Motor_Menu)},
        {PG2+7,PG2+0,PG2+6,PG2+7,1,(*Motor_Menu)},

        {PG3+0,PG3+1,PG3+7,PG3+0,2,(*Circle_Judge_Menu)},
        {PG3+1,PG3+2,PG3+0,PG3+1,2,(*Circle_Judge_Menu)},
        {PG3+2,PG3+3,PG3+1,PG3+2,2,(*Circle_Judge_Menu)},
        {PG3+3,PG3+4,PG3+2,PG3+3,2,(*Circle_Judge_Menu)},
        {PG3+4,PG3+5,PG3+3,PG3+4,2,(*Circle_Judge_Menu)},
        {PG3+5,PG3+6,PG3+4,PG3+5,2,(*Circle_Judge_Menu)},
        {PG3+6,PG3+7,PG3+5,PG3+6,2,(*Circle_Judge_Menu)},
        {PG3+7,PG3+0,PG3+6,PG3+7,2,(*Circle_Judge_Menu)},

        {PG4+0,PG4+1,PG4+7,PG4+0,3,(*Circle_Process_Menu)},
        {PG4+1,PG4+2,PG4+0,PG4+1,3,(*Circle_Process_Menu)},
        {PG4+2,PG4+3,PG4+1,PG4+2,3,(*Circle_Process_Menu)},
        {PG4+3,PG4+4,PG4+2,PG4+3,3,(*Circle_Process_Menu)},
        {PG4+4,PG4+5,PG4+3,PG4+4,3,(*Circle_Process_Menu)},
        {PG4+5,PG4+6,PG4+4,PG4+5,3,(*Circle_Process_Menu)},
        {PG4+6,PG4+7,PG4+5,PG4+6,3,(*Circle_Process_Menu)},
        {PG4+7,PG4+0,PG4+6,PG4+7,3,(*Circle_Process_Menu)},

        {PG5+0,PG5+1,PG5+7,PG5+0,4,(*Speed_Planning_Menu)},
        {PG5+1,PG5+2,PG5+0,PG5+1,4,(*Speed_Planning_Menu)},
        {PG5+2,PG5+3,PG5+1,PG5+2,4,(*Speed_Planning_Menu)},
        {PG5+3,PG5+4,PG5+2,PG5+3,4,(*Speed_Planning_Menu)},
        {PG5+4,PG5+5,PG5+3,PG5+4,4,(*Speed_Planning_Menu)},
        {PG5+5,PG5+6,PG5+4,PG5+5,4,(*Speed_Planning_Menu)},
        {PG5+6,PG5+7,PG5+5,PG5+6,4,(*Speed_Planning_Menu)},
        {PG5+7,PG5+0,PG5+6,PG5+7,4,(*Speed_Planning_Menu)},

        {PG6+0,PG6+1,PG6+7,PG6+0,5,(*Camera_Menu)},
        {PG6+1,PG6+2,PG6+0,PG6+1,5,(*Camera_Menu)},
        {PG6+2,PG6+3,PG6+1,PG6+2,5,(*Camera_Menu)},
        {PG6+3,PG6+4,PG6+2,PG6+3,5,(*Camera_Menu)},
        {PG6+4,PG6+5,PG6+3,PG6+4,5,(*Camera_Menu)},
        {PG6+5,PG6+6,PG6+4,PG6+5,5,(*Camera_Menu)},
        {PG6+6,PG6+7,PG6+5,PG6+6,5,(*Camera_Menu)},
        {PG6+7,PG6+0,PG6+6,PG6+7,5,(*Camera_Menu)},

        {PG7+0,PG7+1,PG7+7,PG7+0,6,(*mode_set)},
        {PG7+1,PG7+2,PG7+0,PG7+1,6,(*mode_set)},
        {PG7+2,PG7+3,PG7+1,PG7+2,6,(*mode_set)},
        {PG7+3,PG7+4,PG7+2,PG7+3,6,(*mode_set)},
        {PG7+4,PG7+5,PG7+3,PG7+4,6,(*mode_set)},
        {PG7+5,PG7+6,PG7+4,PG7+5,6,(*mode_set)},
        {PG7+6,PG7+7,PG7+5,PG7+6,6,(*mode_set)},
        {PG7+7,PG7+0,PG7+6,PG7+7,6,(*mode_set)},

        // {PG8+0,PG8+1,PG8+7,PG8+0,1,(*Circle_Process_Menu)},
        // {PG8+1,PG8+2,PG8+0,PG8+1,1,(*Circle_Process_Menu)},
        // {PG8+2,PG8+3,PG8+1,PG8+2,1,(*Circle_Process_Menu)},
        // {PG8+3,PG8+4,PG8+2,PG8+3,1,(*Circle_Process_Menu)},
        // {PG8+4,PG8+5,PG8+3,PG8+4,1,(*Circle_Process_Menu)},
        // {PG8+5,PG8+6,PG8+4,PG8+5,1,(*Circle_Process_Menu)},
        // {PG8+6,PG8+7,PG8+5,PG8+6,1,(*Circle_Process_Menu)},
        // {PG8+7,PG8+0,PG8+6,PG8+7,1,(*Circle_Process_Menu)},
};

void SelectdotShowing(int set)
{
    uint8 i = 0;
    if (keymode==1)
    {
        for (i = NumStart; i <NumEnd; i++)
        {
            if (i!=(func-set)) {show_str(0,i," ");}
            if (i==(func-set)) {show_str(0,i,">");}
        }
    }
    if (keymode==2)
    {
        for (i = NumStart; i <NumEnd; i++)
        {
            if (i!=(func-set)) {show_str(0,i," ");}
            if (i==(func-set)) {show_str(0,i,"-");}
        }
    }
}

void mainbackground()
{
    SelectdotShowing(PG0);
    show_str(x_main,line0 ,"ServoPID       ");
    show_str(x_main,line1 ,"MotorPID       ");
    show_str(x_main,line2 ,"CircleJudge    ");
    show_str(x_main,line3 ,"CircleProcess  ");
    show_str(x_main,line4 ,"Speed_Planning ");
    show_str(x_main,line5 ,"Camera         ");
    show_str(x_main,line6 ,"RunMode        ");
    show_str(x_main,line7 ,"Setting        ");
    
    if (func==7&&key_press){show_str(56,line7,"flashed!");delay_ms(200);clear(0x00);}//set eep  
}

void Servo_Menu()
{
    SelectdotShowing(PG1);
    show_str(x_main,line0 ,"UFF0        ");show_float(x_sec, line0 , UFF[0] , 2, 1);
    show_str(x_main,line1 ,"UFF1        ");show_float(x_sec, line1 , UFF[1] , 2, 1);
    show_str(x_main,line2 ,"UFF2        ");show_float(x_sec, line2 , UFF[2] , 2, 1);
    show_str(x_main,line3 ,"UFF3        ");show_float(x_sec, line3 , UFF[3] , 2, 1);
    show_str(x_main,line4 ,"UFF4        ");show_float(x_sec, line4 , UFF[4] , 2, 1);
    show_str(x_main,line5 ,"UFF5        ");show_float(x_sec, line5 , UFF[5] , 2, 1);
    show_str(x_main,line6 ,"UFF6        ");show_float(x_sec, line6 , UFF[6] , 2, 1);
    show_str(x_main,line7 ,"Servo_Mid   ");show_int  (x_sec, line7 , Servo_Mid);


    // if (Spid_Sel<=0){Spid_Sel=0;}
    // if (Spid_Sel>=2){Spid_Sel=2;}

    // show_str(x_main,line7 ,"PID_MODE    ");                                       
    // switch (Spid_Sel)
    // {
    //     case 0:show_str(x_sec,line7 ,"Npid");break;
    //     case 1:show_str(x_sec,line7 ,"Ppid");break;
    //     case 2:show_str(x_sec,line7 ,"Fpid");break;   
    //     default:break;
    // }
    
}

void Motor_Menu()
{
    SelectdotShowing(PG2);
    show_str(x_main,line0 ,"BDLC_PWM  ");show_int  (x_sec, line0 , BDLC_PWM);
    show_str(x_main,line1 ,"DV        ");show_float(x_sec, line1 , dv,3,1);
    show_str(x_main,line2 ,"Lp        ");show_float(x_sec, line2 , LMp,3,1);
    show_str(x_main,line3 ,"Li        ");show_float(x_sec, line3 , LMi,3,1);
    show_str(x_main,line4 ,"Rp        ");show_float(x_sec, line4 , RMp,3,1);
    show_str(x_main,line5 ,"Ri        ");show_float(x_sec, line5 , RMi,3,1);
    show_str(x_main,line6 ,"Zebra_Num ");show_int  (x_sec, line6 , zebra_num);
    show_str(x_main,line7 ,"Servo_Mid ");show_int  (x_sec, line7 , Servo_Mid);
    // if (BDLC_PWM>=6300){BDLC_PWM=0;}
}
 
void Circle_Judge_Menu()
{
    SelectdotShowing(PG3);
    show_str(x_main,line0 ,"Up_Limit   ");show_int  (x_sec, line0 , up_limit);
    show_str(x_main,line1 ,"Down_Limit ");show_int  (x_sec, line1 , down_limit);
    show_str(x_main,line2 ,"Up_lose    ");show_int  (x_sec, line2 , up_lose);
    show_str(x_main,line3 ,"Down_Lose  ");show_int  (x_sec, line3 , down_lose);
    show_str(x_main,line4 ,"Cross_Lose ");show_int  (x_sec, line4 , cross_err);
    show_str(x_main,line5 ,"Cross_Limit");show_int  (x_sec, line5 , crossf_max);
}

void Circle_Process_Menu()
{
    SelectdotShowing(PG4);
    show_str(x_main,line0 ,"ICM_IN     ");   show_int  (x_sec, line0 , icm_in);
    show_str(x_main,line1 ,"ICM_OUT    ");   show_int  (x_sec, line1 , icm_out);
    show_str(x_main,line2 ,"Cir_INP    ");   show_int  (x_sec, line2 , cir_inp);
    show_str(x_main,line3 ,"Cir_OUTP   ");   show_int  (x_sec, line3 , cir_outp);
    show_str(x_main,line4 ,"Cir_InCheck");   show_int  (x_sec, line4 , cir_incheck);    
    show_str(x_main,line5 ,"Cir_OutCheck");  show_int  (x_sec, line5 , cir_outcheck);

}

void Speed_Planning_Menu()
{
    SelectdotShowing(PG5);
    show_str(x_main,line0 ,"normalvel ");    show_int  (x_sec, line0 , normal_tarsp);
    show_str(x_main,line1 ,"circlrvel ");    show_int  (x_sec, line1 , circle_tarsp);    
    show_str(x_main,line2 ,"slopevel  ");    show_int  (x_sec, line2 , slope_tarsp);
    show_str(x_main,line3 ,"stright_dv");    show_int  (x_sec, line3 , stright_tarsp);
}

void Camera_Menu()
{
    SelectdotShowing(PG6);
    show_str(x_main,line0 ,"Comapre_0 ");show_int  (x_sec, line0 , Comapre_0);
    show_str(x_main,line1 ,"Comapre_1 ");show_int  (x_sec, line1 , Comapre_1);
    show_str(x_main,line2 ,"DOWN_DEC  ");show_int  (x_sec, line2 , down_dec);
    show_str(x_main,line3 ,"UP_DEC    ");show_int  (x_sec, line3 , up_dec);
    show_str(x_main,line4 ,"KEEPLINE  ");show_int  (x_sec, line4 , keepline);
    show_str(x_main,line5 ,"INCIRLINE ");show_int  (x_sec, line5 , incirline);
    show_str(x_main,line6 ,"OUTCIRLINE");show_int  (x_sec, line6 , outcirline);
    show_str(x_main,line7 ,"CCD_OFFSET");show_int  (x_sec, line7 , ccd_offset);
}

void mode_set()
{
    SelectdotShowing(PG7);
    show_str(x_main,line0 ,"run_mode  ");//show_int  (x_sec, line0 , Comapre_0);
    show_str(x_main,line1 ,"lcir_num  ");show_int  (x_sec, line1 , lcir_num);
    show_str(x_main,line2 ,"rcir_num  ");show_int  (x_sec, line2 , rcir_num);
    show_str(x_main,line3 ,"cross_num ");show_int  (x_sec, line3 , cross_num);
    show_str(x_main,line4 ,"slope_num ");show_int  (x_sec, line4 , slope_num);
    show_str(x_main,line5 ,"direction ");//show_int  (x_sec, line5 , incirline);
    show_str(x_main,line6 ,"OUTCIRLINE");//show_int  (x_sec, line6 , outcirline);   
}

void EEPROMSetData(void)
{
    if(keymode==2)
    {
        switch (func)
        {
            case PG1+0:UFF[0]  +=Dvi*0.1;       break;
            case PG1+1:UFF[1]  +=Dvi*0.1;       break;
            case PG1+2:UFF[2]  +=Dvi*0.1;       break;
            case PG1+3:UFF[3]  +=Dvi*0.1;       break;
            case PG1+4:UFF[4]  +=Dvi*0.1;       break;
            case PG1+5:UFF[5]  +=Dvi*0.1;       break;
            case PG1+6:UFF[6]  +=Dvi*0.1;       break;
            case PG1+7:Servo_Mid +=Dvi*5;       break;

            case PG2+0:BDLC_PWM+=Dvi*10;        break;
            case PG2+1:dv+=Dvi*1;               break;
            case PG2+2:LMp+=Dvi*1;              break;
            case PG2+3:LMi+=Dvi*0.1;            break;
            case PG2+4:RMp+=Dvi*1;              break;
            case PG2+5:RMi+=Dvi*0.1;            break;
            case PG2+6:zebra_num+=Dvi*1;        break;
            

            case PG3+0:up_limit+=Dvi*1;         break;
            case PG3+1:down_limit+=Dvi*1;       break;
            case PG3+2:up_lose+=Dvi;            break;
            case PG3+3:down_lose+=Dvi;          break;
            case PG3+4:cross_err+=Dvi;          break;
            case PG3+5:crossf_max+=Dvi;         break;

            case PG4+0:icm_in+=Dvi;             break;
            case PG4+1:icm_out+=Dvi;            break;
            case PG4+2:cir_inp+=Dvi;            break;
            case PG4+3:cir_outp+=Dvi;           break;
            case PG4+4:cir_incheck+=Dvi;        break;
            case PG4+5:cir_outcheck+=Dvi*1;     break;

            case PG5+0:normal_tarsp+=Dvi*1;     break;
            case PG5+1:circle_tarsp+=Dvi*1;     break;
            case PG5+2:slope_tarsp+=Dvi*1;      break;
            case PG5+3:stright_tarsp+=Dvi*10;   break;


            case PG6+0:Comapre_0+=Dvi*1;        break;
            case PG6+1:Comapre_1+=Dvi*1;        break;
            case PG6+2:down_dec+=Dvi;           break;
            case PG6+3:up_dec+=Dvi;             break;
            case PG6+4:keepline+=Dvi;           break;
            case PG6+5:incirline+=Dvi;          break;
            case PG6+6:outcirline+=Dvi;         break;
            case PG6+7:ccd_offset+=Dvi;         break;

            case PG7+0:cross_num+=Dvi*1;        break;
            case PG7+1:lcir_num+=Dvi*1;         break;
            case PG7+2:rcir_num+=Dvi;           break;
        }
    }Dvi=0;
}

void image_ori_bin()
{
    uint8 i = 0;
    uint8 j=0,t=0,y=0;

	for ( i = 0; i < 128; i++)
	{
        j = image[i];
		if (j!=old_img[i])
		{
			oled_putpixel_spi(i,j,0x00);
			oled_putpixel_spi(i,old_img[i],0xff);
			old_img[i]=j;
		}
	}
    for ( i = 0; i < 128; i++)
	{
        t = image1[i];
		if (t!=old_img1[i])
		{
			oled_putpixel_spi(i,t,0x00);
			oled_putpixel_spi(i,old_img1[i],0xff);
			old_img1[i]=t;
		}
	}
    for ( i = 0; i < 128; i++)
	{
        y = image2[i];
		if (y!=old_img2[i])
		{
			oled_putpixel_spi(i,y,0x01);
			oled_putpixel_spi(i,old_img2[i],0xff);
			old_img1[i]=y;
		}
	}
    if (P11)
	{
		show_str(0,0,"N");
	}
	else
	{
		show_str(0,0,"Y");
	}
    show_int(0,1,lp[1]);show_int(43,1,eCCD[1]);show_int(85,1,rp[1]);
    show_int(0,2,lp[0]);show_int(43,2,eCCD[0]);show_int(85,2,rp[0]);
    if (key_right)
    {

        lpu=lp[1];rpu=rp[1];
        lpd=lp[0];rpd=rp[0]; 
        delay_ms(100);

    }
    

}


int Key_Scan()
{

    if (keymode==2&&key_left&&func>=PG1&&func<=PG8+8) {keymode=1;}
    if (keymode==1&&key_press&&func>=PG1&&func<=PG8+8){keymode=2;}
    
    if(keymode==1&&(key_up||key_down||key_left||key_right||key_press))
    {
        delay_ms(8);
        
        if      (key_up)    return UP;
        else if (key_down)  return DOWN;
        else if (key_left)  {clear(0x00);return LEFT;}
        else if (key_right) return RIGHT;
        else if (key_press) {clear(0x00);return PRESS;}
    }

    if (keymode==2&&(key_up||key_down))
    {
        delay_ms(8);
        if      (key_up)    return ADD;
        else if (key_down)  return DEC;
    }

    return 0;
}

void Menu_display()
{
    if (Key_Scan()==DOWN)   {func=table[func].Next;}
    if (Key_Scan()==UP)     {func=table[func].Back;}
    if (Key_Scan()==PRESS)  {delay_ms(10);func=table[func].Enter;}
    if (Key_Scan()==LEFT)   {func=table[func].Esc;}
    if (Key_Scan()==ADD)    {Dvi= 1;}
    if (Key_Scan()==DEC)    {Dvi=-1;}

    fx=table[func].fx;
    (*fx)();
    EEPROMSetData();
}

