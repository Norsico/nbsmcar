/*
 * app_display_pro.h - 显示任务模块 (IPS200Pro高级版)
 *
 * 用于屏幕显示任务管理
 */

#ifndef __APP_DISPLAY_PRO_H__
#define __APP_DISPLAY_PRO_H__

/************ 头文件 ************/
#include "zf_common_typedef.h"
#include "zf_common_font.h"
#include "zf_device_ips200pro.h"

/************ 宏定义 ************/
// 字体大小选择 (根据 FONT_SIZE_12 ~ FONT_SIZE_28)
#define DISPLAY_FONT_SIZE    FONT_SIZE_16

// 屏幕分辨率 (横屏模式)
#define SCREEN_WIDTH         320
#define SCREEN_HEIGHT        240

// 行高 = 字体高度
#define LINE_HEIGHT          DISPLAY_FONT_SIZE
// 最大行数
#define MAX_LINE             (SCREEN_HEIGHT / LINE_HEIGHT)

/************ 接口函数 ************/
// 显示模块初始化
void display_pro_init(void);

// 显示各状态对应的画面
void display_pro_show_init(void);       // 初始化状态
void display_pro_show_prepare(void);    // 准备状态
void display_pro_show_running(void);    // 运行状态
void display_pro_show_stopped(void);    // 停止状态
void display_pro_show_emergency(void); // 紧急状态

#endif /* __APP_DISPLAY_PRO_H__ */
