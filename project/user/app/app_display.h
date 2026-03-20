/*
 * app_display.h - 显示任务模块
 *
 * 用于屏幕显示任务管理
 */

#ifndef __APP_DISPLAY_H__
#define __APP_DISPLAY_H__

/************ 头文件 ************/
#include "zf_common_typedef.h"
#include "zf_common_font.h"
#include "zf_device_ips200.h"

/************ 接口函数 ************/
// 显示模块初始化
void display_init(void);

// 显示各状态对应的画面
void display_show_init(void);       // 初始化状态
void display_show_prepare(void);    // 准备状态
void display_show_running(void);    // 运行状态
void display_show_stopped(void);    // 停止状态
void display_show_emergency(void);  // 紧急状态

#endif /* __APP_DISPLAY_H__ */
