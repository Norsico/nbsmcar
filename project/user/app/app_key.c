#include "app_key.h"
#include "app_display.h"
#include "dev_other.h"
#include "zf_common_headfile.h"
#include "system_state.h"

#define KEY_HOLD_REPEAT_START_COUNT   (800 / KEY_SCAN_PERIOD)
#define KEY_HOLD_REPEAT_PERIOD_COUNT  (5)

static key_event_handler_t key_event_table[KEY_MAX];
static uint16 key_hold_count[KEY_MAX];

static void key_hold_repeat_reset(uint8 key_num)
{
	if(key_num < KEY_MAX)
	{
		key_hold_count[key_num] = 0;
	}
}

static void key_hold_repeat_process(void)
{
	uint8 i;

	if(!switch_ui_enabled())
	{
		for(i = 0; i < KEY_MAX; i++)
		{
			key_hold_count[i] = 0;
		}
		return;
	}

	for(i = 0; i < KEY_MAX; i++)
	{
		if(0 != keys_info[i].level)
		{
			key_hold_count[i] = 0;
			continue;
		}

		if(key_hold_count[i] < 0xFFFF)
		{
			key_hold_count[i]++;
		}

		if(key_hold_count[i] < KEY_HOLD_REPEAT_START_COUNT)
		{
			continue;
		}

		if((KEY_HOLD_REPEAT_START_COUNT != key_hold_count[i]) &&
		   (0 != ((key_hold_count[i] - KEY_HOLD_REPEAT_START_COUNT) % KEY_HOLD_REPEAT_PERIOD_COUNT)))
		{
			continue;
		}

		if(KEY3 == i)
		{
			display_menu_move_up_fast();
		}
		else if(KEY4 == i)
		{
			display_menu_move_down_fast();
		}
	}
}

static void key1_handler(key_state_t state){
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
			display_menu_move_up();
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
			display_menu_move_down();
			break;
		case KEY_LONG:
			break;
		}
}
static void key_event_register(uint8 key_num,key_event_handler_t handler){
	if(key_num<KEY_MAX) key_event_table[key_num] = handler;
}
void key_event_init(void){
	uint8 i;

	key_event_register(KEY1,key1_handler);
	key_event_register(KEY2,key2_handler);
	key_event_register(KEY3,key3_handler);
	key_event_register(KEY4,key4_handler);
	for(i = 0; i < KEY_MAX; i++)
	{
		key_hold_count[i] = 0;
	}
}
void key_event_poll(void){
	uint8 i;
	key_event_handler_t handler = (key_event_handler_t)NULL;

	key_hold_repeat_process();
	for(i=0;i<KEY_MAX;i++){
		if(KEY_IDLE != keys_info[i].state)
		{
			key_hold_repeat_reset(i);
		}
		handler = key_event_table[i];
		if(handler != NULL){
			handler(keys_info[i].state);
			handler = (key_event_handler_t)NULL;
		}
	}
}
