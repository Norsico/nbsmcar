#include "dev_display.h"

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