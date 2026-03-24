#include "app_key.h"
#include "zf_common_headfile.h"
#include "system_state.h"

static key_event_handler_t key_event_table[KEY_MAX];

/****************** 按键事件函数 ******************/
static void key1_handler(key_state_t state){
	switch(state){
		case KEY_IDLE:
			break;
		case KEY_SHORT:
			gpio_set_level(LED_DEBUG,0);
			break;
		case KEY_LONG:
			break;
	}
}
static void key2_handler(key_state_t state){
	switch(state){
		case KEY_IDLE:
			break;
		case KEY_SHORT:
			break;
		case KEY_LONG:
			gpio_set_level(LED_DEBUG,1);
			break;
	}
}
static void key3_handler(key_state_t state){
	switch(state){
		case KEY_IDLE:
			break;
		case KEY_SHORT:
			gpio_set_level(LED_DEBUG,0);
			break;
		case KEY_LONG:
			break;
	}
}
static void key4_handler(key_state_t state){
	switch(state){
		case KEY_IDLE:
			break;
		case KEY_SHORT:
			break;
		case KEY_LONG:
			gpio_set_level(LED_DEBUG,1);
			break;
	}
}

/************ 处理和使用逻辑 ***********/

// 按键事件注册函数
static void key_event_register(uint8 key_num,key_event_handler_t handler){
	if(key_num<KEY_MAX) key_event_table[key_num] = handler;
}
// 按键事件初始化函数（进行初始化）
void key_event_init(void){
	key_event_register(KEY1,key1_handler);
	key_event_register(KEY2,key2_handler);
	key_event_register(KEY3,key3_handler);
	key_event_register(KEY4,key4_handler);
}
// 按键事件轮询处理（在每次更新之后）
void key_event_poll(void){
	// 直接使用全局变量，但不允许修改
	uint8 i;
	key_event_handler_t handler = (key_event_handler_t)NULL;
	for(i=0;i<KEY_MAX;i++){
		handler = key_event_table[i]; // 提取函数
		if(handler != NULL){
			handler(keys_info[i].state); // 使用函数
			handler = (key_event_handler_t)NULL;
		}
	}
}

