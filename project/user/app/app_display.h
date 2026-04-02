/*
 * app_display.h - 显示任务模块
 *
 * 用于屏幕显示任务管理
 */

#ifndef __APP_DISPLAY_H__
#define __APP_DISPLAY_H__

#include "dev_display.h"

void display_menu_init(void);
void display_menu_render(void);
void display_menu_move_up(void);
void display_menu_move_down(void);
void display_menu_move_up_fast(void);
void display_menu_move_down_fast(void);
void display_menu_enter(void);
void display_menu_back(void);
uint8 display_menu_in_camera_view(void);

#endif /* __APP_DISPLAY_H__ */
