#include "app_key.h"
#include "app_display.h"
#include "dev_other.h"
#include "zf_common_headfile.h"
#include "system_state.h"

static key_event_handler_t key_event_table[KEY_MAX];

static void key1_handler(key_state_t state){
	if(!switch_ui_enabled()) return;
	switch(state){
		case KEY_IDLE:
			break;
		case KEY_SHORT:
			display_menu_move_down();
			break;
		case KEY_LONG:
			break;
	}
}
static void key2_handler(key_state_t state){
	if(!switch_ui_enabled()) return;
	switch(state){
		case KEY_IDLE:
			break;
		case KEY_SHORT:
			display_menu_back();
			break;
		case KEY_LONG:
			break;
	}
}
static void key3_handler(key_state_t state){
	if(!switch_ui_enabled()) return;
	switch(state){
		case KEY_IDLE:
			break;
		case KEY_SHORT:
			display_menu_enter();
			break;
		case KEY_LONG:
			break;
	}
}
static void key4_handler(key_state_t state){
	if(!switch_ui_enabled()) return;
	switch(state){
		case KEY_IDLE:
			break;
		case KEY_SHORT:
			display_menu_move_up();
			break;
		case KEY_LONG:
			break;
	}
}
static void key_event_register(uint8 key_num,key_event_handler_t handler){
	if(key_num<KEY_MAX) key_event_table[key_num] = handler;
}
void key_event_init(void){
	key_event_register(KEY1,key1_handler);
	key_event_register(KEY2,key2_handler);
	key_event_register(KEY3,key3_handler);
	key_event_register(KEY4,key4_handler);
}
void key_event_poll(void){
	uint8 i;
	key_event_handler_t handler = (key_event_handler_t)NULL;
	for(i=0;i<KEY_MAX;i++){
		handler = key_event_table[i];
		if(handler != NULL){
			handler(keys_info[i].state);
			handler = (key_event_handler_t)NULL;
		}
	}
}
