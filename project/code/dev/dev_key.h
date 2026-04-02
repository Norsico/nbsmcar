#ifndef _DEV_KEY_H_
#define _DEV_KEY_H_

#include "zf_driver_gpio.h"

/* 按键引脚定义 */
#define KEY1_PIN (IO_PB2)
#define KEY2_PIN (IO_PB3)
#define KEY3_PIN (IO_PB4)
#define KEY4_PIN (IO_P32)

/* 按键序号 */
enum {
	KEY1=0,
	KEY2,
	KEY3,
	KEY4,
	KEY_MAX
};
/* 按键状态 */
typedef enum {
	KEY_IDLE = 0,
	KEY_SHORT,
	KEY_LONG
} key_state_t;


/* 按键结构体 */
typedef struct {
	uint8 level:1; // 电平
	key_state_t state;
	uint16 interval; // 抬起间隔
} key_info_t;


extern key_info_t keys_info[KEY_MAX];

/* 函数声明 */
void key_init(void);
void key_update(void);


#endif