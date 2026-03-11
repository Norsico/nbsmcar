void CCD2_Process()
{
    uint8 i=0;
    uint16 min=3000,max=0;
    int16 image_sum=0;
    int16 threshold_avg=0;

    for ( i = 0; i < 128; i+=2)
    {
        image_sum+=ccd_data_ch3[i];
    }

    threshold_avg=image_sum>>6;  

    for ( i = 0; i < 127; i+=2)
    {
        if (ccd_data_ch3[i]>max)        {max=ccd_data_ch3[i];}
        if (ccd_data_ch3[i]<min)        {min=ccd_data_ch3[i];}
    }

    threshold2=(max+min)/2*(1-0.1)+threshold_avg*0.1;//
    for ( i = 0; i < 128; i++)
    {
        if (ccd_data_ch2[i]>threshold2)  {image2[i]=bai2 ;}//?
        else                             {image2[i]=hei2 ;}//?  
    }

    // if (image2[old_mid]==hei)
    // {
    //     for ( i = 0; i < 64; i++)
    //     {
    //         if (image2[old_mid+i]==bai){old_mid1=64+i+2;break;}
    //         if (image2[old_mid-i]==bai){old_mid1=64-i-2;break;}    
    //     }
    //     if (old_mid2>=122)   {old_mid2=122;}
    //     if (old_mid2<=5)     {old_mid2=5;} 
    // } 

    for ( i = old_mid2; i > 0; i--)  
    {
        if(image2[i]>image2[i-1]&&image2[i]>image2[i-2])
        {
            lp[2]=i-1;
            break;
        }
    }
    for ( i = old_mid2; i <127; i++) 
    {
        if(image2[i]>image2[i+1]&&image2[i]>image2[i+2])
        {
            rp[2]=i+1;
            break;
        }
    }
    old_mid2=(lp[2]+rp[2])*0.5;
    if (old_mid2>=122)   {old_mid2=122;}
    if (old_mid2<=5)     {old_mid2=5;} 

    eCCD[2]=63-0.5*(lp[2]+rp[2]); 
}


void Right_Roundabout_Process()
{   
    static uint8 last_rp1;
    uint8 circin_statu=0;
    uint8 cirin_flag=0;
    flag.rcircle=1;flag.all=1;ele_count=0;beep_on;
    while (flag.rcircle)
    { 
        show_str(0,1,"R_Circle_In ");
        show_int(0,2,circin_statu);
        CCD_Process(KEEP_LEFT_LINE);

        if ((rp[0]-lp[0])<=(rpd-lpd+15)){circin_statu+=1;}
        if (circin_statu>=70&&rp[0]>105||DEBUG_CAR){r_cir_in=1;ele_count=0;break;}  //&&rp[0]>=rpd+10   

    }//ele_count>ENcir_in
    while (r_cir_in)
    { 
        show_str(0,1,"R_Circle_Run");
        //ele_count<Servo_Keep
        CCD1_Process();
        if (ele_count<1.2) {eCCD[0]=-35;}//CCD_Process(KEEP_RIGHT_LINE);
        else             {CCD_Process(KEEP_NORMAL_LINE);cirin_flag+=1;}    
        if (lp[0]<=30&&cirin_flag>=60||DEBUG_CAR)    
        {
            ele_count=0;
            r_cir_out=1;
            r_cir_in=0;
            break;
        }                                       
    }
    while (r_cir_out)
    { 
        show_str(0,1,"R_Circle_Out");
        show_int(0,2,rp[0]);
        CCD1_Process();
        if (ele_count<1.2)
        {
            // CCD_Process(KEEP_RIGHT_LINE);
            eCCD[0]=-35;
        }
        else
        {
            CCD_Process(KEEP_LEFT_LINE);
            if (myabs(eCCD[0])<=5||DEBUG_CAR)  
            {
                flag.all=0;
                r_cir_out=0;
                show_str(0,1,"R_Circle_End");
                break;
            }//
        }     
    }flag.rcircle=0;beep_off;
}



void search_midline(uint8 input)
{
    uint8 i =0;
    static uint8 l_lp,l_rp,l_eCCD;
    for ( i = old_mid; i > 6; i--)
    {
        if (find_edge_point(ccd_data_ch1[i],ccd_data_ch1[i-5],Comapre_0))
        {
            lp[0]=i-5;break;
        }
        if (i<=1) {lp[0]=1;break;}
    }

    for ( i = old_mid; i < 122; i++)
    {
        if (find_edge_point(ccd_data_ch1[i],ccd_data_ch1[i+5],Comapre_0))
        {
            rp[0]=i+5;break;
        } 
        if (i>=126) {rp[0]=126;break;}
    }

    for ( i = 64; i > 6; i--)
    {
        if (find_edge_point(ccd_data_ch2[i],ccd_data_ch2[i-5],Comapre_1))
        {
            lp[1]=i-5;break;
        }
        if (i<=1) {lp[1]=1;break;}
    }

    for ( i = 64; i < 122; i++)
    {
        if (find_edge_point(ccd_data_ch2[i],ccd_data_ch2[i+5],Comapre_1))
        {
            rp[1]=i+5;break;
        } 
        if (i>=126) {rp[1]=126;break;}
    }

    l_lp=lp[0];l_rp=rp[0];


    old_mid=(lp[0]+rp[0])/2;
    old_mid1=(lp[1]+rp[1])/2;
    if (old_mid<15){old_mid=15;}
    if (old_mid>127-15){old_mid=127-15;}
        
    
    
    if (input==KEEP_RIGHT_LINE)         {lp[0]=rp[0]-52;}
    if (input==KEEP_LEFT_LINE)          {rp[0]=lp[0]+52;}
    eCCD[0]=63-0.5*(lp[0]+rp[0]);
    eCCD[1]=63-0.5*(lp[1]+rp[1]);

    if (lp[0]==7&&rp[0]==121)
    {
        eCCD[0]=l_eCCD;
    }

    l_eCCD=eCCD[0];
}

    if (lp[1]<minfolder)                         
    if (myabs(rp[1]-rpu)<max_folder_err) 
    if (Stright_Judge(max_folder_err))
    if (rp[1]>127-minfolder)
    if (myabs(lp[1]-lpu)<max_folder_err)
    if (Stright_Judge(max_folder_err))  