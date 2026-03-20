#ifndef _APP_KEY_H_
#define _APP_KEY_H_

#include "zf_common_typedef.h"
#include "zf_driver_gpio.h"

/* 按键引脚定义 */
#define KEY1_PIN (IO_P32)
#define KEY2_PIN (IO_PB2)
#define KEY3_PIN (IO_PB3)
#define KEY4_PIN (IO_PB4)

/* 激光笔引脚 */
#define LASER_PIN (IO_P67)

/* 激光笔点亮时间 (ms) */
#define LASER_ON_TIME 500

/* 按键状态 */
typedef enum {
    KEY_RELEASE = 1,
    KEY_PRESS = 0
} key_state_t;

/* 函数声明 */
void key_init(void);
void key_scan(void);
void laser_on(uint32 ms);
void laser_process(void);

#endif
