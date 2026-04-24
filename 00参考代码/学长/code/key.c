#include "key.h"
#include "zf_common_headfile.h"

/*°´¼ü×´Ì¬ÐÅÏ¢*/
uint8 key1_status = 1;
uint8 key2_status = 1;
uint8 key3_status = 1;
uint8 key4_status = 1;
uint8 key5_status = 1;
uint8 key6_status = 1;
uint8 key1_last_status;
uint8 key2_last_status;
uint8 key3_last_status;
uint8 key4_last_status;
uint8 key5_last_status;
uint8 key6_last_status;

uint8 Lprs_key1=0;
uint8 Lprs_key2=0;
uint8 Lprs_key3=0;
uint8 Lprs_key4=0;
uint8 Lprs_key5=0;
uint8 Lprs_key6=0;

uint8 Lprs_key1_flag=0;
uint8 Lprs_key2_flag=0;
uint8 Lprs_key3_flag=0;
uint8 Lprs_key4_flag=0;
uint8 Lprs_key5_flag=0;
uint8 Lprs_key6_flag=0;

uint8 prs_times=0;
/*¿ª¹Ø±êÖ¾Î»*/
uint8 key1_flag;
uint8 key2_flag;
uint8 key3_flag;
uint8 key4_flag;
uint8 key5_flag;
uint8 key6_flag;
uint8 test1,test2,test3,test4;
uint8 Start_Flag = 0;

uint8 bt_left;
uint8 bt_right;
uint8 bt_up;
uint8 bt_down;
uint8 bt_ok;
uint8 bt_bk;
uint8 bt_prs=1;

void key_scan(void)
{
    key1_last_status = key1_status;
    key2_last_status = key2_status;
    key3_last_status = key3_status;
    key4_last_status = key4_status;
    key5_last_status = key5_status;
    key6_last_status = key6_status;

    /*******¶ÁÈ¡µ±Ç°°´¼ü×´Ì¬*******/
    key1_status = gpio_get_level(KEY1);
    key2_status = gpio_get_level(KEY2);
    key3_status = gpio_get_level(KEY3);
    key4_status = gpio_get_level(KEY4);
    key5_status = gpio_get_level(KEY5);
    key6_status = gpio_get_level(KEY6);
    /*******¶ÁÈ¡°´¼ü±êÖ¾Î»*******/
    if(!key1_status && key1_last_status)
    {key1_flag = 1;}else key1_flag=0;
    if(!key2_status && key2_last_status)
    {key2_flag = 1;}else key2_flag=0;
    if(!key3_status && key3_last_status)
    {key3_flag = 1;}else key3_flag=0;
    if(!key4_status && key4_last_status)
    {key4_flag = 1;}else key4_flag=0;
    if(!key5_status && key5_last_status)
    {key5_flag = 1;}else key5_flag=0;
    if(!key6_status && key6_last_status)
    {key6_flag = 1;}else key6_flag=0;

    if(key1_status&&
        key2_status&&
        key3_status&&
        key4_status&&
        key5_status&&
        key6_status
    ){prs_times=0;}

    if(!key1_status&&Lprs_key1<200)Lprs_key1++;
    if(key1_status)Lprs_key1=0;
    if(Lprs_key1==200)Lprs_key1_flag=1;
    else Lprs_key1_flag=0;
    if(!key2_status&&Lprs_key2<200)Lprs_key2++;
    if(key2_status)Lprs_key2=0;
    if(Lprs_key2==200)Lprs_key2_flag=1;
    else Lprs_key2_flag=0;
    if(!key3_status&&Lprs_key3<200)Lprs_key3++;
    if(key3_status)Lprs_key3=0;
    if(Lprs_key3==200)Lprs_key3_flag=1;
    else Lprs_key3_flag=0;
    if(!key4_status&&Lprs_key4<200)Lprs_key4++;
    if(key4_status)Lprs_key4=0;
    if(Lprs_key4==200)Lprs_key4_flag=1;
    else Lprs_key4_flag=0;
    if(!key5_status&&Lprs_key5<200)Lprs_key5++;
    if(key5_status)Lprs_key5=0;
    if(Lprs_key5==200)Lprs_key5_flag=1;
    else Lprs_key5_flag=0;
    if(!key6_status&&Lprs_key6<200)Lprs_key6++;
    if(key6_status)Lprs_key6=0;
    if(Lprs_key6==200)Lprs_key6_flag=1;
    else Lprs_key6_flag=0;
//


}
void key(void)
{
//        Run=1;

  if(Lprs_key1_flag){
     if(prs_times>=50)
     {prs_times=0;
     key1_flag=1;}
     else prs_times++;
  }

  if(Lprs_key2_flag){
       if(prs_times>=50)
       {prs_times=0;
       key2_flag=1;}
       else prs_times++;
    }

  if(Lprs_key3_flag){
       if(prs_times>=50)
       {prs_times=0;
       key3_flag=1;}
       else prs_times++;
    }

  if(Lprs_key4_flag){
       if(prs_times>=50)
       {prs_times=0;
       key4_flag=1;}
       else prs_times++;
    }

  if(Lprs_key5_flag){
       if(prs_times>=50)
       {prs_times=0;
       key5_flag=1;}
       else prs_times++;
    }

  if(Lprs_key6_flag){
       if(prs_times>=50)
       {prs_times=0;
       key6_flag=1;}
       else prs_times++;
    }



  if(key1_flag||
        key2_flag||
        key3_flag||
        key4_flag||
        key5_flag||
        key6_flag){
         bt_prs=1;
      }

  if(key1_flag){bt_up=1;   Run=1;  key1_flag=0;}
  if(key2_flag){bt_down=1;  key2_flag=0;}
  if(key3_flag){bt_ok=1;    key3_flag=0;}
  if(key4_flag){bt_bk=1;    key4_flag=0;}
  if(key5_flag){bt_left=1;  key5_flag=0;}
  if(key6_flag){bt_right=1; key6_flag=0;}

}

