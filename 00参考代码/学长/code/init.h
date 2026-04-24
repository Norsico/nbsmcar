#include "zf_common_headfile.h"
#ifndef CODE_INIT_H_
#define CODE_INIT_H_

#define SERVO_1                 (ATOM0_CH0_P21_2)       //定义主板上舵机1对应引脚
#define SERVO_2                 (ATOM0_CH1_P21_3)       //定义主板上舵机2对应引脚
#define SERVO_3                 (ATOM0_CH2_P21_4)       //定义主板上舵机3对应引脚
#define SERVO_4                 (ATOM0_CH3_P21_5)       //定义主板上舵机4对应引脚
#define SERVO_FREQ              (300)                   //定义主板上舵机频率
#define SERVO1_MID              (4633)                    //舵机1中值     左下
#define SERVO2_MID              (4333)                    //舵机2中值     左上
#define SERVO3_MID              (4367)                    //舵机3中值     右上
#define SERVO4_MID              (4533)                    //舵机4中值     右下



void all_init(void);
void servo_init_2(void);


#endif /* CODE_INIT_H_ */
