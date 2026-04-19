/*
 * app_ui_library.h - UI 通用绘制组件
 *
 * 负责屏幕底层绘制复用，不参与页面状态和参数逻辑
 */

#ifndef _APP_UI_LIBRARY_H_
#define _APP_UI_LIBRARY_H_

#include "dev_display.h"

/* 基础文本与图形绘制。 */
void ui_library_show_percent(uint16 x, uint16 y, uint8 percent);
void ui_library_draw_rect(uint16 x, uint16 y, uint16 w, uint16 h, uint16 color);
void ui_library_draw_rect_frame(uint16 x, uint16 y, uint16 w, uint16 h, uint16 color);
void ui_library_draw_dash_line(uint16 x, uint16 y, uint16 w, uint16 dash_w, uint16 gap_w, uint16 color);
/* 常用 UI 组件。 */
void ui_library_draw_title(const char *title);
void ui_library_draw_list_item(uint16 y, const char *label, uint8 selected);
void ui_library_draw_battery(uint8 percent);
/* 数值转字符串显示。 */
void ui_library_format_uint16(uint16 value, char *text);
void ui_library_format_tenths(uint16 value, char *text);

#endif
