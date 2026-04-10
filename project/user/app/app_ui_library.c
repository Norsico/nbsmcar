/*
 * app_ui_library.c - UI 通用绘制组件
 */

#include "app_ui_library.h"

#define UI_LIBRARY_TITLE_Y           (0)
#define UI_LIBRARY_LIST_MARK_X       (0)
#define UI_LIBRARY_LIST_LABEL_X      (16)
#define UI_LIBRARY_BAT_X             (168)
#define UI_LIBRARY_BAT_Y             (2)
#define UI_LIBRARY_BAT_W             (36)
#define UI_LIBRARY_BAT_H             (10)
#define UI_LIBRARY_BAT_CAP_W         (3)
#define UI_LIBRARY_BAT_TEXT_X        (132)

void ui_library_show_percent(uint16 x, uint16 y, uint8 percent)
{
    char text[4];

    /* 百分比只保留整数显示，给电量条和状态字复用。 */
    if(percent >= 100)
    {
        text[0] = '1';
        text[1] = '0';
        text[2] = '0';
        text[3] = '\0';
    }
    else if(percent >= 10)
    {
        text[0] = (char)('0' + percent / 10);
        text[1] = (char)('0' + percent % 10);
        text[2] = '\0';
    }
    else
    {
        text[0] = (char)('0' + percent);
        text[1] = '\0';
    }

    ips200_show_string(x, y, "   ");
    ips200_show_string(x, y, text);
}

void ui_library_draw_rect(uint16 x, uint16 y, uint16 w, uint16 h, uint16 color)
{
    uint16 row = 0;
    uint16 col = 0;

    /* IPS200 没有现成填充矩形接口，这里直接逐点铺色。 */
    for(row = 0; row < h; row++)
    {
        for(col = 0; col < w; col++)
        {
            ips200_draw_point(x + col, y + row, color);
        }
    }
}

void ui_library_draw_rect_frame(uint16 x, uint16 y, uint16 w, uint16 h, uint16 color)
{
    uint16 col = 0;
    uint16 row = 0;

    for(col = 0; col < w; col++)
    {
        ips200_draw_point(x + col, y, color);
        ips200_draw_point(x + col, y + h - 1, color);
    }

    for(row = 0; row < h; row++)
    {
        ips200_draw_point(x, y + row, color);
        ips200_draw_point(x + w - 1, y + row, color);
    }
}

void ui_library_draw_dash_line(uint16 x, uint16 y, uint16 w, uint16 dash_w, uint16 gap_w, uint16 color)
{
    uint16 offset = 0;
    uint16 draw_w = 0;

    /* 参数页分隔线统一走这里，避免页面里重复算虚线节拍。 */
    while(offset < w)
    {
        draw_w = dash_w;
        if((uint16)(offset + draw_w) > w)
        {
            draw_w = (uint16)(w - offset);
        }

        ui_library_draw_rect((uint16)(x + offset), y, draw_w, 1, color);
        offset = (uint16)(offset + dash_w + gap_w);
    }
}

void ui_library_draw_title(const char *title)
{
    ips200_set_color(RGB565_BLUE, RGB565_WHITE);
    ips200_show_string(0, UI_LIBRARY_TITLE_Y, title);
}

void ui_library_draw_list_item(uint16 y, const char *label, uint8 selected)
{
    if(selected)
    {
        ips200_set_color(RGB565_WHITE, RGB565_BLUE);
        ips200_show_string(UI_LIBRARY_LIST_MARK_X, y, "> ");
        ips200_show_string(UI_LIBRARY_LIST_LABEL_X, y, label);
    }
    else
    {
        ips200_set_color(RGB565_BLACK, RGB565_WHITE);
        ips200_show_string(UI_LIBRARY_LIST_MARK_X, y, "  ");
        ips200_show_string(UI_LIBRARY_LIST_LABEL_X, y, label);
    }
}

void ui_library_draw_battery(uint8 percent)
{
    uint16 fill_w = 0;

    /* 电量区域每次都整块重画，保证数字位数变化时不会残影。 */
    ips200_set_color(RGB565_BLACK, RGB565_WHITE);
    ui_library_draw_rect(UI_LIBRARY_BAT_X,
                         UI_LIBRARY_BAT_Y,
                         UI_LIBRARY_BAT_W + UI_LIBRARY_BAT_CAP_W + 18,
                         UI_LIBRARY_BAT_H + 2,
                         RGB565_WHITE);
    ui_library_draw_rect_frame(UI_LIBRARY_BAT_X, UI_LIBRARY_BAT_Y, UI_LIBRARY_BAT_W, UI_LIBRARY_BAT_H, RGB565_BLACK);
    ui_library_draw_rect(UI_LIBRARY_BAT_X + UI_LIBRARY_BAT_W,
                         UI_LIBRARY_BAT_Y + 3,
                         UI_LIBRARY_BAT_CAP_W,
                         UI_LIBRARY_BAT_H - 6,
                         RGB565_BLACK);

    if(percent > 0)
    {
        fill_w = ((uint16)(UI_LIBRARY_BAT_W - 2) * percent) / 100U;
        if(fill_w > 0)
        {
            ui_library_draw_rect(UI_LIBRARY_BAT_X + 1, UI_LIBRARY_BAT_Y + 1, fill_w, UI_LIBRARY_BAT_H - 2, RGB565_GREEN);
        }
    }

    ips200_show_string(UI_LIBRARY_BAT_TEXT_X, UI_LIBRARY_TITLE_Y, "    ");
    ui_library_show_percent(UI_LIBRARY_BAT_TEXT_X, UI_LIBRARY_TITLE_Y, percent);
    ips200_show_string(UI_LIBRARY_BAT_TEXT_X + 24, UI_LIBRARY_TITLE_Y, "%");
}

void ui_library_format_uint16(uint16 value, char *text)
{
    /* UI 里这类数值最长只显示 4 位，超出直接收口。 */
    if(value >= 10000)
    {
        value = 9999;
    }

    if(value >= 1000)
    {
        text[0] = (char)('0' + (value / 1000U));
        text[1] = (char)('0' + ((value / 100U) % 10U));
        text[2] = (char)('0' + ((value / 10U) % 10U));
        text[3] = (char)('0' + (value % 10U));
        text[4] = '\0';
    }
    else if(value >= 100)
    {
        text[0] = (char)('0' + (value / 100U));
        text[1] = (char)('0' + ((value / 10U) % 10U));
        text[2] = (char)('0' + (value % 10U));
        text[3] = '\0';
    }
    else if(value >= 10)
    {
        text[0] = (char)('0' + (value / 10U));
        text[1] = (char)('0' + (value % 10U));
        text[2] = '\0';
    }
    else
    {
        text[0] = (char)('0' + value);
        text[1] = '\0';
    }
}

void ui_library_format_tenth(int16 value_tenth, char *text)
{
    uint16 abs_value = 0;
    uint16 int_part = 0;
    uint16 frac_part = 0;
    uint8 text_index = 0;

    if(value_tenth < 0)
    {
        text[text_index++] = '-';
        abs_value = (uint16)(-value_tenth);
    }
    else
    {
        abs_value = (uint16)value_tenth;
    }

    int_part = abs_value / 10U;
    frac_part = abs_value % 10U;
    if(int_part >= 100)
    {
        text[text_index++] = (char)('0' + (int_part / 100U) % 10U);
        text[text_index++] = (char)('0' + (int_part / 10U) % 10U);
        text[text_index++] = (char)('0' + int_part % 10U);
    }
    else if(int_part >= 10)
    {
        text[text_index++] = (char)('0' + (int_part / 10U) % 10U);
        text[text_index++] = (char)('0' + int_part % 10U);
    }
    else
    {
        text[text_index++] = (char)('0' + int_part);
    }

    text[text_index++] = '.';
    text[text_index++] = (char)('0' + frac_part);
    text[text_index] = '\0';
}
