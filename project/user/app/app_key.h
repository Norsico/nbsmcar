#ifndef _APP_KEY_H_
#define _APP_KEY_H_

#include "dev_key.h"

// 按键事件处理函数类型定义
typedef void(*key_event_handler_t)(key_state_t state);

void key_event_init(void);
void key_event_poll(void);

#endif
