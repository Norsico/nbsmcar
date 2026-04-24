#ifndef _KEY_h
#define _KEY_h
#include "zf_common_headfile.h"

#define BEEP_PIN   P33_10       //¶¨Òå·äÃùÆ÷Òý½Å
//#define BeeOn                 gpio_set_level(BEEP_PIN, 1)
//#define BeeOff                gpio_set_level(BEEP_PIN, 0)


/*°´¼ü*/
#define KEY1    P20_6//P23_1
#define KEY2    P20_7//P22_0
#define KEY3    P11_2
#define KEY4    P11_3
#define KEY5    P10_1
#define KEY6    P10_3

/*°´¼ü×´Ì¬ÐÅÏ¢*/
extern uint8 key1_status;
extern uint8 key2_status;
extern uint8 key3_status;
extern uint8 key4_status;
extern uint8 key5_status;
extern uint8 key6_status;
extern uint8 key1_last_status;
extern uint8 key2_last_status;
extern uint8 key3_last_status;
extern uint8 key4_last_status;
extern uint8 key5_last_status;
extern uint8 key6_last_status;

extern uint8 Lprs_key1;
extern uint8 Lprs_key2;
extern uint8 Lprs_key3;
extern uint8 Lprs_key4;
extern uint8 Lprs_key5;
extern uint8 Lprs_key6;

extern uint8 Lprs_key1_flag;
extern uint8 Lprs_key2_flag;
extern uint8 Lprs_key3_flag;
extern uint8 Lprs_key4_flag;
extern uint8 Lprs_key5_flag;
extern uint8 Lprs_key6_flag;

extern uint8 prs_times;
/*¿ª¹Ø±êÖ¾Î»*/
extern uint8 key1_flag;
extern uint8 key2_flag;
extern uint8 key3_flag;
extern uint8 key4_flag;
extern uint8 key5_flag;
extern uint8 key6_flag;

extern uint8 bt_left;
extern uint8 bt_right;
extern uint8 bt_up;
extern uint8 bt_down;
extern uint8 bt_ok;
extern uint8 bt_bk;
extern uint8 bt_prs;
/*ÆðÅÜ±êÖ¾Î»*/
extern uint8 Start_Flag;

void key_scan(void);
void key(void);
#endif
