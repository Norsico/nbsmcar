#include "SystemConfig.h"
uint8 i =0,j=0,r_cir_in=0,r_cir_out=0,l_cir_in=0,l_cir_out=0,white_block_count=0;

uint8 image[128]={0};
uint8 image1[128]={0};
uint8 image2[128]={0};
uint8 fuzzy_image[64]={0};
uint8 lp_index=0,rp_index=0;
#define ax 62
#define hei 6
#define bai 7
#define hei1 4
#define bai1 5
#define hei2 2
#define bai2 3
ele flag;

uint8 find_edge_point(uint16 data0,uint16 data1,uint8 compare_data)
{
    int16 sum =data0-data1;
    if ((sum<<7/(data0+data1))>compare_data)
    {
        return 1 ;
    }

    return 0 ;
}

void search_midline(uint8 input)
{
    int8 clp,crp=0;
    uint8 i =0;
    static uint8 l_lp,l_rp,l_eCCD;
    for ( i = old_mid; i >= 6; i--)
    {//&&find_edge_point(ccd_data_ch1[i-1],ccd_data_ch1[i-5],Comapre_0)
        if (find_edge_point(ccd_data_ch1[i]  ,ccd_data_ch1[i-5],Comapre_0)
            )
        {
            lp[0]=i-5;break;
        }
        if (i<=7) {lp[0]=0;break;}
    }

    for ( i = old_mid; i <= 122; i++)
    {//&&find_edge_point(ccd_data_ch1[i+1],ccd_data_ch1[i+5],Comapre_0)
        if (find_edge_point(ccd_data_ch1[i]  ,ccd_data_ch1[i+5],Comapre_0)
            )
        {
            rp[0]=i+5;break;
        } 
        if (i>=120) {rp[0]=127;break;}
    }

    for ( i = old_mid1; i >= 6; i--)
    {
        if (find_edge_point(ccd_data_ch2[i]  ,ccd_data_ch2[i-5],Comapre_1)&&
            find_edge_point(ccd_data_ch2[i-1],ccd_data_ch2[i-5],Comapre_1))
        {
            lp[1]=i-5;break;
        }
        if (i<=7) {lp[1]=0;break;}
    }

    for ( i = old_mid1; i <= 122; i++)
    {
        if (find_edge_point(ccd_data_ch2[i]  ,ccd_data_ch2[i+5],Comapre_1)&&
            find_edge_point(ccd_data_ch2[i+1],ccd_data_ch2[i+5],Comapre_1))
        {
            rp[1]=i+5;break;
        } 
        if (i>=120) {rp[1]=127;break;}
    }
    old_mid=(lp[0]+rp[0])/2;
    old_mid1=(lp[1]+rp[1])/2;
    // if (old_mid1<lpu+20){old_mid1=lpu+20;}
    // if (old_mid1>rpu-20){old_mid1=rpu-20;}
        
    
    // eCCD[0]=63-0.5*(lp[0]+rp[0]);
    // eCCD[1]=63-0.5*(lp[1]+rp[1]);

    if (input!=KEEP_NORMAL_LINE)
    {
        if (input==KEEP_RIGHT_LINE)         {eCCD[0]=63-0.5*(2*rp[0]-keepline);}
        if (input==KEEP_LEFT_LINE)          {eCCD[0]=63-0.5*(2*lp[0]+keepline);}

        if (input==KEEP_INR_LINE)           {eCCD[0]=63-0.5*(2*rp[0]-incirline);}
        if (input==KEEP_INL_LINE)           {eCCD[0]=63-0.5*(2*lp[0]+incirline);}

        if (input==KEEP_OUTR_LINE)          {eCCD[0]=63-0.5*(2*rp[0]-outcirline);}
        if (input==KEEP_OUTL_LINE)          {eCCD[0]=63-0.5*(2*lp[0]+outcirline);}

        if (input==KEEP_CLOSE_LINE)                       
        {        
            if ((64-lp[0])>=rp[0]-64)   {clp=rp[0]-keepline;eCCD[0]=63-0.5*(clp+rp[0]);}
            else                        {crp=lp[0]+keepline;eCCD[0]=63-0.5*(lp[0]+crp);}
        }
    }
    else
    {
        eCCD[0]=64-0.5*(lp[0]+rp[0])+ccd_offset;
    }

    if (input==KEEP_CROSS_LINE)                       
    {        
        if ((64-lp[1])>=rp[1]-64)   {clp=rp[1]-34;eCCD[1]=63-0.5*(clp+rp[1])+ccd1_offset;}
        else                        {crp=lp[1]+34;eCCD[1]=63-0.5*(lp[1]+crp)+ccd1_offset;}
        eCCD[0]=64-0.5*(lp[0]+rp[0])+ccd_offset;
    }
    else
    {
        eCCD[1]=63-0.5*(lp[1]+rp[1])+ccd1_offset;
    }

}
void CCD_Process(uint8 input)
{
    uint8 i=0;
    uint16 min=65535,max=0;
    int8 clp,crp=0;
    uint8 dw=(rpd-lpd)/2;
    int16 image_sum=0;
    int16 threshold_avg=0;

    for ( i = 0; i < 128; i+=2)
    {
        image_sum+=ccd_data_ch1[i];
    }

    threshold_avg=image_sum>>6;
    
    for ( i = 0; i < 128; i+=2)
    {
        if (ccd_data_ch1[i]>max)        {max=ccd_data_ch1[i];}
        if (ccd_data_ch1[i]<min)        {min=ccd_data_ch1[i];}
    }
    // threshold=(uint32)((max-min)*100/(max+min)*0.5);
    threshold=((max+min)/2)*(1-down_arr)+threshold_avg*down_arr-down_dec;
    
    for ( i = 0; i < 128; i++)
    {
        if (ccd_data_ch1[i]>=threshold)  {image[i]=bai ;}
        else                             {image[i]=hei ;}  
    }  
    
    for ( i = old_mid; i >= 2; i--)  
    {
        if(image[i]>image[i-1]&&image[i]>image[i-2])
        {
            lp[0]=i-2;
            break;
        }
        if (i<= 2){lp[0]=0;break;}
    }
    for ( i = old_mid; i <=125; i++) 
    {
        if(image[i]>image[i+1]&&image[i]>image[i+2])
        {
            rp[0]=i+2;
            break;
        }
        if (i>= 126){rp[0]=128;break;}
    }

    old_mid=(lp[0]+rp[0])*0.5;
    
    // if (old_mid<7)  {old_mid=7;}
    // if (old_mid>120){old_mid=120;}


    if (input!=KEEP_NORMAL_LINE)
    {
        if (input==KEEP_RIGHT_LINE)         {eCCD[0]=63-0.5*(2*rp[0]-keepline);}
        if (input==KEEP_LEFT_LINE)          {eCCD[0]=63-0.5*(2*lp[0]+keepline);}

        if (input==KEEP_INR_LINE)           {eCCD[0]=63-0.5*(2*rp[0]-incirline);}
        if (input==KEEP_INL_LINE)           {eCCD[0]=63-0.5*(2*lp[0]+incirline);}

        if (input==KEEP_OUTR_LINE)          {eCCD[0]=63-0.5*(2*rp[0]-outcirline);}
        if (input==KEEP_OUTL_LINE)          {eCCD[0]=63-0.5*(2*lp[0]+outcirline);}

        if (input==KEEP_CLOSE_LINE)                       
        {        
            if ((64-lp[0])>=rp[0]-64)   {clp=rp[0]-keepline;eCCD[0]=63-0.5*(clp+rp[0]);}
            else                        {crp=lp[0]+keepline;eCCD[0]=63-0.5*(lp[0]+crp);}
        }
    }
    else
    {
        eCCD[0]=64-0.5*(lp[0]+rp[0]);
    }
}
void CCD1_Process()
{
    uint8 i=0;
    uint16 min=3000,max=0;
    int16 image_sum=0;
    int16 threshold_avg=0;

    for ( i = 0; i < 128; i+=2)
    {
        image_sum+=ccd_data_ch2[i];
    }

    threshold_avg=image_sum>>6;  

    for ( i = 0; i < 128; i+=2)
    {
        if (ccd_data_ch2[i]>max)        {max=ccd_data_ch2[i];}
        if (ccd_data_ch2[i]<min)        {min=ccd_data_ch2[i];}
    }

    threshold1=(max+min)/2*(1-up_arr)+threshold_avg*up_arr-up_dec;//
    for ( i = 0; i < 128; i++)
    {
        if (ccd_data_ch2[i]>=threshold1)  {image1[i]=bai1 ;}//?
        else                              {image1[i]=hei1 ;}//?  
    }

    if (image1[old_mid]==hei1)
    {
        for ( i = 0; i < 64; i++)
        {
            if (image1[old_mid+i]==bai1){old_mid1=64+i+4;break;}
            if (image1[old_mid-i]==bai1){old_mid1=64-i-4;break;}    
        }
        if (old_mid1>=122)   {old_mid1=122;}
        if (old_mid1<=5)     {old_mid1=5;} 
    } 

    for ( i = old_mid1; i > 1; i--)  
    {
        if(image1[i]>image1[i-1]&&image1[i]>image1[i-2])
        {
            lp[1]=i-2;
            break;
        }
        if (i<= 2){lp[1]=0;break;}
    }
    for ( i = old_mid; i <126; i++) 
    {
        if(image1[i]>image1[i+1]&&image1[i]>image1[i+2])
        {
            rp[1]=i+2;
            break;
        }
        if (i>= 126){rp[1]=128;break;}
    }
    // old_mid1=(lp[1]+rp[1])*0.5;
    // if (old_mid1>=122)   {old_mid1=122;}
    // if (old_mid1<=5)     {old_mid1=5;} 

    eCCD[1]=64-0.5*(lp[1]+rp[1]); 
}

void img_get()
{
    uint8 i=0;
    uint16 min=65535,max=0;
    int16 image_sum=0;
    int16 threshold_avg=0;

    for ( i = 0; i < 128; i+=2)
    {
        image_sum+=ccd_data_ch1[i];
    }

    threshold_avg=image_sum>>6;
    
    for ( i = 0; i < 128; i+=2)
    {
        if (ccd_data_ch1[i]>max)        {max=ccd_data_ch1[i];}
        if (ccd_data_ch1[i]<min)        {min=ccd_data_ch1[i];}
    }
    // threshold=(uint32)((max-min)*100/(max+min)*0.5);
    threshold=((max+min)/2)*(1-down_arr)+threshold_avg*down_arr-down_dec;
    
    for ( i = 0; i < 128; i++)
    {
        if (ccd_data_ch1[i]>=threshold)  {image[i]=bai ;}
        else                             {image[i]=hei ;}  
    }
}

void img1_get()
{
    uint8 i=0;
    uint16 min=3000,max=0;
    int16 image_sum=0;
    int16 threshold_avg=0;

    for ( i = 0; i < 128; i+=2)
    {
        image_sum+=ccd_data_ch2[i];
    }

    threshold_avg=image_sum>>6;  

    for ( i = 0; i < 128; i+=2)
    {
        if (ccd_data_ch2[i]>max)        {max=ccd_data_ch2[i];}
        if (ccd_data_ch2[i]<min)        {min=ccd_data_ch2[i];}
    }

    threshold1=(max+min)/2*(1-up_arr)+threshold_avg*up_arr-up_dec;//
    for ( i = 0; i < 128; i++)
    {
        if (ccd_data_ch2[i]>=threshold1)  {image1[i]=bai1 ;}//?
        else                              {image1[i]=hei1 ;}//?  
    }   
}

void ccd_mode(uint8 mode, uint8 line_mode)
{   
    img_get();
    if (mode == 0)
    {
        search_midline(line_mode);
    }
    else if (mode == 1)
    {
        CCD_Process(line_mode);
        CCD1_Process();
    }
}

uint8 Stright_Judge(int dv)
{
    if (myabs(lp[0]-lpd)<=dv&&(rp[0]-rpd)<=dv)
    {     
        return 1;
    }
    return 0;
}

void Right_Roundabout_Process()
{   
    uint8 rc_count=0;
    uint16 in_check=0;
    uint16 out_check=0;
    uint8 Compare_Width = rpd-lpd+crossf_max;
    flag.rcircle=1;
    flag.all=1;
    while (flag.rcircle)
    {
        switch (rc_count)
        {
        case 0:
            ccd_mode(compare,KEEP_LEFT_LINE);
            if ((rp[0]-lp[0])<=Compare_Width)
            {
                in_check+=1;
            }
            if (in_check>=cir_incheck&&rp[0]>127-cir_inp)//7
            {
                in_check=0;
                rc_count=1;
            }
            break;

        case 1: 
            ccd_mode(compare,KEEP_INR_LINE);  
            if (yaw_count>icm_in)
            {
                rc_count=2; 
                yaw_count=0;
            }
            break;

        case 2:
            ccd_mode(compare,KEEP_LEFT_LINE); 
            
            if (lp[0]<=cir_outp&&yaw_count>120)
            {
                rc_count=3;
                yaw_count=0; 
            }
            break;

        case 3:

            ccd_mode(compare,KEEP_OUTR_LINE); 
            if (yaw_count>icm_out)
            {
                rc_count=4;
                yaw_count=0;
            }

            break;
        
        case 4:
            ccd_mode(compare,KEEP_LEFT_LINE);
            if ((rp[0]-lp[0])<=Compare_Width)
            {
                out_check+=1;
            }
            if (out_check>cir_outcheck)
            {
                rc_count=5;
            }
            break;
        }
        if(lp[0]<cross_err&&rp[0]>127-cross_err&&rc_count==0)
        {
            flag.rcircle=0;
            flag.all=0;
            rc_count=0;
            break;
        }
        if (rc_count>=5||DEBUG_CAR)
        {
            flag.rcircle=0;
            flag.all=0;
            rc_count=0;
            show_str(0,ele_num,"RIGHT_CIRCLE");
            ele_num+=1;
            last_ele=ele_rcircle; 
            r_num+=1;
            break;
        }
    }
}

void Left_Roundabout_Process()
{
    uint8 lc_count=0;
    uint16 in_check=0;
    uint16 out_check=0;
    uint8 Compare_Width = rpd-lpd+crossf_max;
    flag.lcircle=1;
    flag.all=1;
    ele_count=0;

    while (flag.lcircle)
    {
        switch (lc_count)
        {
        case 0://lcircle prein
            ccd_mode(compare,KEEP_RIGHT_LINE);
            if ((rp[0]-lp[0])<=Compare_Width)
            {
                in_check+=1;
            }
            if (in_check>=cir_incheck&&lp[0]<cir_inp)
            {
                lc_count=1;
                ele_count=0;
            }
            break;

        case 1: //in lcircle
            ccd_mode(compare,KEEP_INL_LINE);   
            if (yaw_count>icm_in)
            {
                lc_count=2; 
                yaw_count=0;
            }
            break;

        case 2: //lcircle running
            ccd_mode(compare,KEEP_RIGHT_LINE);
            if (rp[0]>=128-cir_outp&&yaw_count>120)
            {
                lc_count=3;
                yaw_count=0; 
            }
            break;

        case 3: //lcircle out

            ccd_mode(compare,KEEP_OUTL_LINE);
            if (yaw_count>icm_out)
            {
                lc_count=4;
                yaw_count=0;
                ele_count=0;
            }
 
            break;
        
        case 4:
            CCD_Process(KEEP_RIGHT_LINE); 
            // if (ele_count>20)
            // {
            //     lc_count=5;ele_count=0;
            // }
            
            if ((rp[0]-lp[0])<=Compare_Width)
            {
                out_check+=1;
            }
            if (out_check>cir_outcheck)
            {
                lc_count=5;
            }
            break;
        }
        if(lp[0]<cross_err&&rp[0]>127-cross_err&&lc_count==0)
        {
            flag.lcircle=0;
            flag.all=0;
            lc_count=0;
            break;
        }
        if (lc_count>=5||DEBUG_CAR)
        {
            flag.lcircle=0;
            flag.all=0;
            lc_count=0; 
            show_str(0,ele_num,"LEFT_CIRCLE");
            ele_num+=1;
            last_ele=ele_lcircle; 
            l_num+=1;
            break;
        }
    }
}

/**
 * @brief  Circle_Judge powered by ccd
 * @author Pre.fy
 * @date 2024/06/07
 */

void Circle_Judge()
{
    // static uint8 last_rp=0;
    // static uint8 last_lp=0;
    // uint8 drp=rp[0]-last_rp;
    // uint8 dlp=last_lp-lp[0];
    static uint8 rc_c=0;
    static uint8 lc_c=0;
    // if (dlp<=0){dlp=0;}
    // if (drp<=0){drp=0;}

    /*
        Left_Roundabout //名字叫的这么可爱嘻嘻嘻反正是我取的
        nmm的是我呆码写错了啊啊啊啊啊啊
        有bug你是怎么跑起来的？逻辑完全不对啊 6.19
        ************************心静嗯 6.18
        你左端再255一个试试 uint8嗯
        我镜头减多了。。。。
        是不是有病啊环岛
        跑起来了啊啊啊啊啊啊啊逆子你真棒6.20
        误判了
        你还是死掉吧逆子6.21
        改bug了 复活吧 我的逆子6.22
    */
    // if ((rp[0]-lp[0])<=(rpd-lpd)+minfolder)//宽度限幅 崩溃啊啊啊啊啊啊啊草泥马阈值6.12   
    if (myabs(lp[1]-lpu)<=up_limit&&rp[1]>=127-up_lose&&r_num<rcir_num-0.1)
    {
        rc_c=1;flag.all=1;ele_count=0;
    }
    
    switch (rc_c)
    {
    case 1:
        if (myabs(lp[1]-lpu)>up_limit||myabs(lp[0]-lpd)>down_limit||(lp[1]-lp[0])>(lpu-lpd)+down_limit) 
        {
            rc_c=0;flag.all=0;break;
        }
        if (ele_count>30)
        {
            rc_c=0;flag.all=0;break;
        }
        
        if (rp[0]>127-down_lose)//drp>20
        {
            flag.all=0;ele_count=0;Right_Roundabout_Process();
        }
        
        break;
    
    default:
        break;
    }

    if (myabs(rp[1]-rpu)<=up_limit&&lp[1]<=up_lose&&l_num<lcir_num-0.1)
    {
        lc_c=1;flag.all=1;ele_count=0;
    }
    switch (lc_c)
    {
    case 1:
        if (myabs(rp[1]-rpu)>up_limit||myabs(rp[0]-rpd)>down_limit||(rp[0]-rp[1])>rpd-rpu+down_limit)
        {
            lc_c=0;flag.all=0;break;
        }
        if (ele_count>30)
        {
            lc_c=0;flag.all=0;break;
        }
        if (lp[0]<down_lose)
        {
            flag.all=0;ele_count=0;Left_Roundabout_Process();
        }
        
        break;
    
    default:
        break;
    }
    
    // last_lp=lp[0];
    // last_rp=rp[0];
}

void Cross_Process()
{
    static uint8 cross_check=0;
    uint8 Track_Width = rp[0]-lp[0];
    uint8 Width_Compare = rpd-lpd+crossf_max;
    uint8 cross_pre=0;
	// if(lp[1]<cross_err&&rp[1]>127-cross_err)//Track_Width>Width_Compare&&myabs(eCCD[0])<=5
    // {
    //     cross_pre=1;flag.all=1;ele_count=0;
    // }
    // while (cross_pre)
    // { Track_Width = rp[0]-lp[0];
    //     ccd_mode(compare,KEEP_NORMAL_LINE);     
    //     if(lp[0]<cross_err&&rp[0]>127-cross_err || DEBUG_CAR)//Track_Width>Width_Compare
    //     {
    //         flag.cross=1;ele_count=0;break;
    //     }
    //     if (ele_count>10)//||Track_Width < Width_Compare
    //     {
    //         cross_pre=0;flag.all=0;ele_count=0;break;
    //     }    
    // }   
    if(lp[0]<cross_err&&rp[0]>127-cross_err&&cross_nco<cross_num-0.1)//Track_Width>Width_Compare
    {
        flag.cross=1;flag.all=1;ele_count=0;
    }        
    while (flag.cross)
    {
        Track_Width = rp[0]-lp[0];
        ccd_mode(binary,KEEP_NORMAL_LINE);
        Servo_PID(eCCD[1]*0.5);
        if (Track_Width < Width_Compare || DEBUG_CAR)
        {
            cross_check+=1;
        }
        if (cross_check>=20)//||ele_count>30
        {
            cross_check=0;
            flag.cross=0;
            show_str(0,ele_num,"CROSS");
            ele_num+=1;  
            cross_nco+=1; 
            last_ele=ele_cross;         
            break;
        }
        
    }
}

void zebra()
{
    uint8 i=0;
    uint8 zebra_count=0;
    ele_count=0;    

    for ( i = lpd; i < rpd; i++)
    {
        if (image[i]!=image[i+1])
        {
            zebra_count+=1; 
        }
    }
    // show_int(0,0,zebra_count);
    if (zebra_count>=zebra_num)    {flag.zebra=1;flag.all=1;}//&&Stright_Judge(7)
    else                                    {flag.zebra=0;}
    
    while (flag.zebra)
    {

        CCD_Process(KEEP_NORMAL_LINE);
        if (ele_count>40||DEBUG_CAR)
        {
            runflag=1;
            pwm_freq(PWMB_CH2_P01,333,0);
		    pwm_freq(PWMB_CH1_P00,333,0);

            show_str(100,7,"END");
            ele_num+=1;             
            break;
        }
    }
}

void Em_Stop()
{
    if (P52==1)  
    {
        runflag=1;
        pwm_duty(PWMB_CH2_P01,0);
        pwm_duty(PWMB_CH1_P00,0);           
    } 
    
}

void slope_judge()
{
    static uint8 slope_check=0;
    static uint8 slope_c=0;
    static float slope_icm=0;
    // uint8 slope_check=0;
    // if (Slope_Data<=40&&Slope_Data>5) //&&rp[0]-lp[0]>rpd-lpd+crossf_max
    // {
    //     flag.slope=1;
    //     pitch_err=0;
    // }
    // if (Slope_Data>200
    //     &&lp[1]<=lpu&&rp[1]>=rpu&&myabs(eCCD[0])<=8)//&&myabs(eCCD[1])<=10  
    // {//cross_nco>=2
    //     flag.slope=1;
    //     pitch_err=0;
    // }
    
    // if ((lp[0]>lpd-8&&lp[0]<lpd-3)&&(rp[0]<rpd+8&&rp[0]>rpd+3)&&
    //      myabs(eCCD[0])<=4&&myabs(eCCD[1])<=5
    //      &&rp[1]-lp[1]<65)
    // {
    //     flag.slope=1;
    //     pitch_err=0;


    // }
    if (!P11&&slope_nco<slope_num-0.1&&myabs(eCCD[0])<=10)
    {//&&lp[0]>lpd&&rp[0]>=rpd&&last_ele==ele_cross
        slope_check+=1;
    }
    else
    {
        slope_check=0;
    }

    if (slope_check>=10)
    {
        flag.slope=1;
        pitch_err=0;
    }
    

    // if (pitch>=0.5&&last_ele==ele_cross&&slope_nco<slope_num-0.1)
    // {
    //     flag.slope=1;
    //     pitch_err=0;
    // }
    

    
    while (flag.slope)
    {
        // slope_icm+=myabs(pitch);
        // show_float(0,0,slope_icm,4,2);
        ccd_mode(binary,KEEP_NORMAL_LINE);
        Servo_PID(eCCD[0]*0.3); 
        switch (slope_c)
        {
            case 0:
                // if (slope_icm>440)
                // {
                //     slope_c=1;
                // }
            
                if (pitch_err>10)
                {
                    slope_c=1;
                }
                
                break;
            case 1:
                if (pitch_err<-10)
                {
                    slope_c=2;
                }

                break;
            case 2:
                if (pitch_err>-5)
                {
                    slope_c=3;
                }

                break;                
            case 3:
                if (myabs(pitch)<=0.20)
                {
                    slope_check+=1;
                }
                if (slope_check>=100)
                {
                    slope_c=4;
                }
                
        }    
        if (slope_c>=4||DEBUG_CAR)
        {
            flag.slope=0;
            pitch_err=0;
            slope_c=0;
            slope_icm=0;
            show_str(0,ele_num,"SLOPE"); 
            ele_num+=1;
            slope_nco+=1;
            break;
        }
    } 
}

void elements()
{   
    if (!runflag)
    {
        slope_judge();
        Cross_Process();
        Circle_Judge();
        zebra();
    }
}


