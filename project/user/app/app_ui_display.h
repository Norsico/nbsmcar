/*
 * app_ui_display.h - UI 显示与按键控制
 *
 * 负责页面状态、绘制调度和交互处理
 */

#ifndef __APP_UI_DISPLAY_H__
#define __APP_UI_DISPLAY_H__

#include "dev_display.h"

/* 页面初始化与重绘。 */
void display_menu_init(void);
void display_menu_render(void);
/* 菜单导航与调参增减。 */
void display_menu_move_up(void);
void display_menu_move_down(void);
void display_menu_move_up_fast(void);
void display_menu_move_down_fast(void);
/* 页面确认与返回。 */
void display_menu_enter(void);
void display_menu_back(void);
void display_menu_go_root(void);
/* 主循环查询当前 UI 状态。 */
uint8 display_menu_in_camera_view(void);

#endif /* __APP_UI_DISPLAY_H__ */
