/*
 * app_display.c - 显示任务模块
 *
 * 优化说明：
 * 1. 只在状态改变时清屏重绘背景
 * 2. 静态文字只绘制一次
 * 3. 动态内容（数值）增量更新
 */

#include "app_display.h"
#include "system_state.h"

/************ 引用外部变量 ************/
// 引用 main.c 中的全局系统状态
extern volatile system_state_t g_system_state;

/************ 私有变量 ************/
static uint16 display_count = 0;
static uint16 display_last_count = 0;                  // 记录上次数值
static system_state_t display_last_state = SYS_INIT;   // 记录上次显示的状态

/************ 接口函数 ************/

// 显示模块初始化
void display_init(void)
{
    // 初始化 IPS200 屏幕
    ips200_init();

    // 设置显示方向为横屏 (2 = IPS200_CROSSWISE)
    ips200_set_dir(IPS200_CROSSWISE);

    // 设置文字颜色和背景色
    ips200_set_color(RGB565_RED, RGB565_WHITE);

    // 清屏
    ips200_clear(RGB565_WHITE);
}

// 绘制静态背景（只在状态改变时调用）
static void display_draw_background(system_state_t state)
{
    switch(state)
    {
        case SYS_INIT:
            ips200_clear(RGB565_WHITE);
            ips200_show_string(0, 0, "System Init...");
            break;

        case SYS_PREPARE:
            ips200_clear(RGB565_WHITE);
            ips200_show_string(0, 0, "=== READY ===");
            ips200_show_string(0, 1*16, "Press K1 to Start");
            ips200_show_string(0, 2*16, "System OK!");
            break;

        case SYS_RUNNING:
            ips200_clear(RGB565_BLACK);
            ips200_set_color(RGB565_GREEN, RGB565_BLACK);
            ips200_show_string(0, 0, "RUNNING");
            // 预留传感器显示位置
            // ips200_show_string(0, 2*16, "Speed:");
            // ips200_show_string(0, 3*16, "Angle:");
            ips200_set_color(RGB565_RED, RGB565_BLACK);
            break;

        case SYS_STOPED:
            ips200_clear(RGB565_WHITE);
            ips200_set_color(RGB565_BLUE, RGB565_WHITE);
            ips200_show_string(0, 0, "=== STOPPED ===");
            ips200_show_string(0, 2*16, "Press K1 to Continue");
            ips200_set_color(RGB565_RED, RGB565_WHITE);
            break;

        case SYS_EMERGENCY:
            ips200_clear(RGB565_RED);
            ips200_set_color(RGB565_WHITE, RGB565_RED);
            ips200_show_string(0, 0, "!!! EMERGENCY !!!");
            ips200_show_string(0, 1*16, "System Stopped!");
            ips200_show_string(0, 2*16, "Check System!");
            ips200_set_color(RGB565_RED, RGB565_WHITE);
            break;

        default:
            break;
    }
}

// 显示初始化状态
void display_show_init(void)
{
    // INIT 状态无条件显示（首次显示）
    if(g_system_state == SYS_INIT)
    {
        display_draw_background(SYS_INIT);
        display_last_state = SYS_INIT;
    }
    // 状态改变时重绘背景
    else if(g_system_state != display_last_state)
    {
        display_draw_background(SYS_INIT);
        display_last_state = g_system_state;
    }

    // 只更新变化的数值
    if(display_count != display_last_count)
    {
        // 清除旧数值区域
        ips200_show_string(0, 1*16, "            ");
        ips200_show_uint16(0, 1*16, display_count);
        display_last_count = display_count;
    }
}

// 显示准备状态
void display_show_prepare(void)
{
    if(g_system_state != display_last_state)
    {
        display_draw_background(SYS_PREPARE);
        display_last_state = g_system_state;
    }
}

// 显示运行状态
void display_show_running(void)
{
    if(g_system_state != display_last_state)
    {
        display_draw_background(SYS_RUNNING);
        display_last_state = g_system_state;
    }

    // 只更新变化的数值
    if(display_count != display_last_count)
    {
        ips200_show_string(0, 1*16, "            ");
        ips200_show_uint16(0, 1*16, display_count);
        display_last_count = display_count;
    }
}

// 显示停止状态
void display_show_stopped(void)
{
    if(g_system_state != display_last_state)
    {
        display_draw_background(SYS_STOPED);
        display_last_state = g_system_state;
    }

    if(display_count != display_last_count)
    {
        ips200_show_string(0, 1*16, "            ");
        ips200_show_uint16(0, 1*16, display_count);
        display_last_count = display_count;
    }
}

// 显示紧急状态
void display_show_emergency(void)
{
    if(g_system_state != display_last_state)
    {
        display_draw_background(SYS_EMERGENCY);
        display_last_state = g_system_state;
    }
}
