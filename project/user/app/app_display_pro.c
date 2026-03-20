/*
 * app_display_pro.c - 显示任务模块 (IPS200Pro高级版)
 *
 * 用于屏幕显示任务管理
 * 注意: ips200pro 是面向对象的 UI 系统，需要创建 Label 控件来显示文字
 */

#include "app_display_pro.h"

/************ 私有变量 ************/
static uint16 display_count = 0;

// 页面和标签ID
static uint16 page_main;
static uint16 label_status;
static uint16 label_count;
static uint16 label_hint;

/************ 接口函数 ************/

// 显示模块初始化
void display_pro_init(void)
{
    // 初始化 IPS200Pro 屏幕
    // 参数: 标题, 标题位置, 标题大小
    page_main = ips200pro_init("SmartCar", IPS200PRO_TITLE_BOTTOM, DISPLAY_FONT_SIZE);

    // 设置显示方向为横屏 (IPS200PRO_CROSSWISE = 横屏)
    ips200pro_set_direction(IPS200PRO_CROSSWISE);

    // 设置默认字体大小
    ips200pro_set_default_font(DISPLAY_FONT_SIZE);

    // 创建标签控件 (x, y, width, height)
    label_status = ips200pro_label_create(0, 0, 320, LINE_HEIGHT);
    label_count  = ips200pro_label_create(0, 1*LINE_HEIGHT, 320, LINE_HEIGHT);
    label_hint   = ips200pro_label_create(0, 2*LINE_HEIGHT, 320, LINE_HEIGHT);

    // 设置页面背景色
    ips200pro_set_color(page_main, COLOR_BACKGROUND, RGB565_WHITE);
}

// 显示初始化状态
void display_pro_show_init(void)
{
    // 设置页面背景色
    ips200pro_set_color(page_main, COLOR_BACKGROUND, RGB565_WHITE);

    // 更新标签内容
    ips200pro_label_show_string(label_status, "System Init...");
    ips200pro_label_printf(label_count, "%d", display_count);
    ips200pro_label_show_string(label_hint, "");
}

// 显示准备状态
void display_pro_show_prepare(void)
{
    ips200pro_set_color(page_main, COLOR_BACKGROUND, RGB565_WHITE);

    ips200pro_label_show_string(label_status, "=== READY ===");
    ips200pro_label_show_string(label_hint, "Press K1 to Start");
    ips200pro_label_show_string(label_count, "System OK!");
}

// 显示运行状态
void display_pro_show_running(void)
{
    ips200pro_set_color(page_main, COLOR_BACKGROUND, RGB565_BLACK);
    ips200pro_set_color(label_status, COLOR_FOREGROUND, RGB565_GREEN);
    ips200pro_set_color(label_count, COLOR_FOREGROUND, RGB565_GREEN);
    ips200pro_set_color(label_hint, COLOR_FOREGROUND, RGB565_GREEN);

    ips200pro_label_show_string(label_status, "RUNNING");
    ips200pro_label_printf(label_count, "Count: %d", display_count);
    ips200pro_label_show_string(label_hint, "");

    // 恢复颜色
    ips200pro_set_color(label_status, COLOR_FOREGROUND, RGB565_RED);
}

// 显示停止状态
void display_pro_show_stopped(void)
{
    ips200pro_set_color(page_main, COLOR_BACKGROUND, RGB565_WHITE);
    ips200pro_set_color(label_status, COLOR_FOREGROUND, RGB565_BLUE);
    ips200pro_set_color(label_count, COLOR_FOREGROUND, RGB565_BLUE);
    ips200pro_set_color(label_hint, COLOR_FOREGROUND, RGB565_BLUE);

    ips200pro_label_show_string(label_status, "=== STOPPED ===");
    ips200pro_label_printf(label_count, "Count: %d", display_count);
    ips200pro_label_show_string(label_hint, "Press K1 to Continue");

    // 恢复颜色
    ips200pro_set_color(label_status, COLOR_FOREGROUND, RGB565_RED);
}

// 显示紧急状态
void display_pro_show_emergency(void)
{
    ips200pro_set_color(page_main, COLOR_BACKGROUND, RGB565_RED);
    ips200pro_set_color(label_status, COLOR_FOREGROUND, RGB565_WHITE);
    ips200pro_set_color(label_count, COLOR_FOREGROUND, RGB565_WHITE);
    ips200pro_set_color(label_hint, COLOR_FOREGROUND, RGB565_WHITE);

    ips200pro_label_show_string(label_status, "!!! EMERGENCY !!!");
    ips200pro_label_show_string(label_count, "System Stopped!");
    ips200pro_label_show_string(label_hint, "Check System!");

    // 恢复颜色
    ips200pro_set_color(label_status, COLOR_FOREGROUND, RGB565_RED);
}
