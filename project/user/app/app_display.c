/*
 * app_display.c - simple two-level menu demo for IPS200
 */

#include "app_display.h"

#include "app_line.h"
#include "dev_adc.h"
#include "dev_flash.h"
#include "dev_servo.h"

#define MENU_LINE_HEIGHT            (16)
#define MENU_ROOT_ITEM_COUNT        (2)
#define MENU_PARAM_MENU_ITEM_COUNT  (2)
#define MENU_TITLE_Y                (0)
#define MENU_ROOT_LIST_Y            (16)
#define MENU_SUBMENU_LIST_Y         (24)
#define MENU_SCREEN_W               (188)
#define MENU_PARAM_INFO_Y           (24)
#define MENU_PARAM_INFO_LINE_Y      (44)
#define MENU_PARAM_LIST_Y           (56)
#define MENU_PARAM_ROW_STEP         (24)
#define MENU_PARAM_LABEL_X          (18)
#define MENU_PARAM_VALUE_X          (136)
#define MENU_PARAM_ACCENT_X         (8)
#define MENU_PARAM_ACCENT_Y_OFFSET  (2)
#define MENU_PARAM_ACCENT_W         (3)
#define MENU_PARAM_ACCENT_H         (14)
#define MENU_PARAM_DIVIDER_W        (108)
#define MENU_PARAM_FAST_STEP_MUL    (10)
#define MENU_PARAM_LABEL_CLEAR      "            "
#define MENU_PARAM_VALUE_CLEAR      "      "
#define MENU_PARAM_INFO_CLEAR       "                       "
#define MENU_BAT_X                  (168)
#define MENU_BAT_Y                  (2)
#define MENU_BAT_W                  (36)
#define MENU_BAT_H                  (10)
#define MENU_BAT_CAP_W              (3)
#define MENU_BAT_TEXT_X             (132)
#define BATTERY_EMPTY_DECI          (114)
#define BATTERY_FULL_DECI           (126)

typedef enum
{
    DISPLAY_PAGE_ROOT = 0,
    DISPLAY_PAGE_CAMERA,
    DISPLAY_PAGE_PARAM_MENU,
    DISPLAY_PAGE_PARAM_CAMERA,
    DISPLAY_PAGE_PARAM_CASE
} display_page_t;

static const char *g_root_menu_titles[MENU_ROOT_ITEM_COUNT] =
{
    "Camera View",
    "Param Config"
};

static const char *g_param_menu_titles[MENU_PARAM_MENU_ITEM_COUNT] =
{
    "Camera Params",
    "Current Test"
};

static uint8 g_menu_selected = 0;
static uint8 g_param_menu_selected = 0;
static display_page_t g_menu_page = DISPLAY_PAGE_ROOT;
static uint8 g_menu_dirty = 1;
static uint8 g_menu_last_battery_percent = 0xFF;
static flash_param_slot_t g_case_param_selected = FLASH_PARAM_SLOT_FIRST;
static flash_camera_slot_t g_camera_param_selected = FLASH_CAMERA_SLOT_AUTO_EXP;
static uint8 g_param_editing = 0;

static void display_menu_show_percent(uint16 x, uint16 y, uint8 percent)
{
    char text[4];

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

static void display_menu_draw_rect(uint16 x, uint16 y, uint16 w, uint16 h, uint16 color)
{
    uint16 row = 0;
    uint16 col = 0;

    for(row = 0; row < h; row++)
    {
        for(col = 0; col < w; col++)
        {
            ips200_draw_point(x + col, y + row, color);
        }
    }
}

static void display_menu_draw_rect_frame(uint16 x, uint16 y, uint16 w, uint16 h, uint16 color)
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

static void display_menu_draw_dash_line(uint16 x, uint16 y, uint16 w, uint16 dash_w, uint16 gap_w, uint16 color)
{
    uint16 offset = 0;
    uint16 draw_w = 0;

    while(offset < w)
    {
        draw_w = dash_w;
        if((uint16)(offset + draw_w) > w)
        {
            draw_w = (uint16)(w - offset);
        }

        display_menu_draw_rect((uint16)(x + offset), y, draw_w, 1, color);
        offset = (uint16)(offset + dash_w + gap_w);
    }
}

static void display_menu_draw_battery(uint8 force)
{
    uint16 battery = 0;
    uint16 fill_w = 0;
    uint8 percent = 0;

    if(!power_adc_is_ready())
    {
        return;
    }

    battery = (uint16)(power_adc_get_voltage() * 10.0f + 0.5f);
    if(battery <= BATTERY_EMPTY_DECI)
    {
        percent = 0;
    }
    else if(battery >= BATTERY_FULL_DECI)
    {
        percent = 100;
    }
    else
    {
        percent = (uint8)(((battery - BATTERY_EMPTY_DECI) * 100U) / (BATTERY_FULL_DECI - BATTERY_EMPTY_DECI));
    }

    if(!force && (percent == g_menu_last_battery_percent))
    {
        return;
    }

    ips200_set_color(RGB565_BLACK, RGB565_WHITE);
    display_menu_draw_rect(MENU_BAT_X, MENU_BAT_Y, MENU_BAT_W + MENU_BAT_CAP_W + 18, MENU_BAT_H + 2, RGB565_WHITE);
    display_menu_draw_rect_frame(MENU_BAT_X, MENU_BAT_Y, MENU_BAT_W, MENU_BAT_H, RGB565_BLACK);
    display_menu_draw_rect(MENU_BAT_X + MENU_BAT_W, MENU_BAT_Y + 3, MENU_BAT_CAP_W, MENU_BAT_H - 6, RGB565_BLACK);

    if(percent > 0)
    {
        fill_w = ((uint16)(MENU_BAT_W - 2) * percent) / 100U;
        if(fill_w > 0)
        {
            display_menu_draw_rect(MENU_BAT_X + 1, MENU_BAT_Y + 1, fill_w, MENU_BAT_H - 2, RGB565_GREEN);
        }
    }

    ips200_show_string(MENU_BAT_TEXT_X, MENU_TITLE_Y, "    ");
    display_menu_show_percent(MENU_BAT_TEXT_X, MENU_TITLE_Y, percent);
    ips200_show_string(MENU_BAT_TEXT_X + 24, MENU_TITLE_Y, "%");
    g_menu_last_battery_percent = percent;
}

static void display_menu_mark_dirty(void)
{
    g_menu_dirty = 1;
}

static void display_menu_draw_title(const char *title)
{
    ips200_set_color(RGB565_BLUE, RGB565_WHITE);
    ips200_show_string(0, MENU_TITLE_Y, title);
}

static void display_menu_format_param_value(int16 value_tenth, char *text)
{
    uint16 integer_part = 0;
    uint8 decimal_part = 0;

    if(value_tenth < 0)
    {
        value_tenth = 0;
    }

    integer_part = (uint16)(value_tenth / 10);
    decimal_part = (uint8)(value_tenth % 10);

    if(integer_part >= 100)
    {
        integer_part = 99;
        decimal_part = 9;
    }

    if(integer_part >= 10)
    {
        text[0] = (char)('0' + (integer_part / 10));
        text[1] = (char)('0' + (integer_part % 10));
        text[2] = '.';
        text[3] = (char)('0' + decimal_part);
        text[4] = '\0';
    }
    else
    {
        text[0] = (char)('0' + integer_part);
        text[1] = '.';
        text[2] = (char)('0' + decimal_part);
        text[3] = '\0';
    }
}

static void display_menu_format_uint16(uint16 value, char *text)
{
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

static uint16 display_menu_get_row_y(uint8 index)
{
    return (uint16)(MENU_PARAM_LIST_Y + (uint16)index * MENU_PARAM_ROW_STEP);
}

static void display_menu_draw_list_item(uint16 y, const char *label, uint8 selected)
{
    if(selected)
    {
        ips200_set_color(RGB565_WHITE, RGB565_BLUE);
        ips200_show_string(0, y, "> ");
        ips200_show_string(16, y, label);
    }
    else
    {
        ips200_set_color(RGB565_BLACK, RGB565_WHITE);
        ips200_show_string(0, y, "  ");
        ips200_show_string(16, y, label);
    }
}

static void display_menu_draw_root_item(uint8 index)
{
    if(index >= MENU_ROOT_ITEM_COUNT)
    {
        return;
    }

    display_menu_draw_list_item((uint16)(MENU_ROOT_LIST_Y + (uint16)index * MENU_LINE_HEIGHT),
                                g_root_menu_titles[index],
                                (index == g_menu_selected) ? 1 : 0);
}

static void display_menu_draw_param_menu_item(uint8 index)
{
    if(index >= MENU_PARAM_MENU_ITEM_COUNT)
    {
        return;
    }

    display_menu_draw_list_item((uint16)(MENU_SUBMENU_LIST_Y + (uint16)index * MENU_LINE_HEIGHT),
                                g_param_menu_titles[index],
                                (index == g_param_menu_selected) ? 1 : 0);
}

static const char *display_menu_get_case_param_label(flash_param_slot_t slot)
{
    if(FLASH_PARAM_SLOT_FIRST == slot)
    {
        return "first";
    }
    else
    {
        return "second";
    }
}

static const char *display_menu_get_camera_param_label(flash_camera_slot_t slot)
{
    switch(slot)
    {
        case FLASH_CAMERA_SLOT_AUTO_EXP:
            return "auto exp";
        case FLASH_CAMERA_SLOT_EXP_TIME:
            return "exp time";
        case FLASH_CAMERA_SLOT_GAIN:
            return "gain";
        default:
            return "";
    }
}

static void display_menu_get_camera_param_range(flash_camera_slot_t slot,
                                                uint16 *min_value,
                                                uint16 *max_value,
                                                uint16 *step_value)
{
    if(0 == min_value || 0 == max_value || 0 == step_value)
    {
        return;
    }

    switch(slot)
    {
        case FLASH_CAMERA_SLOT_AUTO_EXP:
            *min_value = FLASH_CAMERA_AUTO_EXP_MIN;
            *max_value = FLASH_CAMERA_AUTO_EXP_MAX;
            *step_value = FLASH_CAMERA_AUTO_EXP_STEP;
            break;
        case FLASH_CAMERA_SLOT_EXP_TIME:
            *min_value = FLASH_CAMERA_EXP_TIME_MIN;
            *max_value = FLASH_CAMERA_EXP_TIME_MAX;
            *step_value = FLASH_CAMERA_EXP_TIME_STEP;
            break;
        case FLASH_CAMERA_SLOT_GAIN:
            *min_value = FLASH_CAMERA_GAIN_MIN;
            *max_value = FLASH_CAMERA_GAIN_MAX;
            *step_value = FLASH_CAMERA_GAIN_STEP;
            break;
        default:
            *min_value = 0;
            *max_value = 0;
            *step_value = 0;
            break;
    }
}

static void display_menu_draw_case_param_row(flash_param_slot_t slot)
{
    char value_text[5];
    const char *label_text = 0;
    uint16 row_y = 0;
    uint16 guide_color = RGB565_GRAY;
    uint16 label_color = RGB565_BLACK;
    uint16 value_color = RGB565_BLACK;
    int16 value_tenth = 0;

    row_y = display_menu_get_row_y((uint8)slot);
    label_text = display_menu_get_case_param_label(slot);
    value_tenth = flash_store_get_param_value_tenth(slot);
    display_menu_format_param_value(value_tenth, value_text);

    display_menu_draw_rect(MENU_PARAM_ACCENT_X,
                           (uint16)(row_y + MENU_PARAM_ACCENT_Y_OFFSET),
                           MENU_PARAM_ACCENT_W,
                           MENU_PARAM_ACCENT_H,
                           RGB565_WHITE);

    if(slot == g_case_param_selected)
    {
        guide_color = g_param_editing ? RGB565_RED : RGB565_BLUE;
        label_color = guide_color;
        value_color = guide_color;
        display_menu_draw_rect(MENU_PARAM_ACCENT_X,
                               (uint16)(row_y + MENU_PARAM_ACCENT_Y_OFFSET),
                               MENU_PARAM_ACCENT_W,
                               MENU_PARAM_ACCENT_H,
                               guide_color);
    }
    else
    {
        label_color = RGB565_GRAY;
    }

    ips200_set_color(label_color, RGB565_WHITE);
    ips200_show_string(MENU_PARAM_LABEL_X, row_y, MENU_PARAM_LABEL_CLEAR);
    ips200_show_string(MENU_PARAM_LABEL_X, row_y, label_text);

    ips200_set_color(value_color, RGB565_WHITE);
    ips200_show_string(MENU_PARAM_VALUE_X, row_y, MENU_PARAM_VALUE_CLEAR);
    ips200_show_string(MENU_PARAM_VALUE_X, row_y, value_text);
}

static void display_menu_draw_case_param_info(void)
{
    char min_text[5];
    char max_text[5];
    char step_text[5];

    ips200_set_color(RGB565_BLACK, RGB565_WHITE);
    ips200_show_string(0, MENU_PARAM_INFO_Y, MENU_PARAM_INFO_CLEAR);
    if(!g_param_editing)
    {
        return;
    }

    display_menu_format_param_value(FLASH_PARAM_VALUE_MIN_TENTH, min_text);
    display_menu_format_param_value(FLASH_PARAM_VALUE_MAX_TENTH, max_text);
    display_menu_format_param_value(FLASH_PARAM_VALUE_STEP_TENTH, step_text);

    ips200_set_color(RGB565_GRAY, RGB565_WHITE);
    ips200_show_string(0, MENU_PARAM_INFO_Y, "min");
    ips200_show_string(64, MENU_PARAM_INFO_Y, "max");
    ips200_show_string(136, MENU_PARAM_INFO_Y, "sp");

    ips200_set_color(RGB565_GREEN, RGB565_WHITE);
    ips200_show_string(24, MENU_PARAM_INFO_Y, min_text);

    ips200_set_color(RGB565_RED, RGB565_WHITE);
    ips200_show_string(88, MENU_PARAM_INFO_Y, max_text);

    ips200_set_color(RGB565_BLUE, RGB565_WHITE);
    ips200_show_string(152, MENU_PARAM_INFO_Y, step_text);
}

static void display_menu_draw_case_param_page_full(void)
{
    ips200_clear(RGB565_WHITE);

    display_menu_draw_title("Current Test");
    display_menu_draw_battery(1);
    display_menu_draw_dash_line(0, MENU_PARAM_INFO_LINE_Y, MENU_PARAM_DIVIDER_W, 10, 6, RGB565_GRAY);
    display_menu_draw_case_param_row(FLASH_PARAM_SLOT_FIRST);
    display_menu_draw_case_param_row(FLASH_PARAM_SLOT_SECOND);
    display_menu_draw_case_param_info();
}

static void display_menu_refresh_case_selection(flash_param_slot_t previous_slot)
{
    display_menu_draw_case_param_row(previous_slot);
    display_menu_draw_case_param_row(g_case_param_selected);
}

static void display_menu_refresh_case_mode(void)
{
    display_menu_draw_case_param_row(g_case_param_selected);
    display_menu_draw_case_param_info();
}

static void display_menu_refresh_case_value(void)
{
    display_menu_draw_case_param_row(g_case_param_selected);
    display_menu_draw_case_param_info();
}

static void display_menu_case_param_select_up(void)
{
    flash_param_slot_t previous_slot = g_case_param_selected;

    if(FLASH_PARAM_SLOT_FIRST == g_case_param_selected)
    {
        g_case_param_selected = FLASH_PARAM_SLOT_SECOND;
    }
    else
    {
        g_case_param_selected = (flash_param_slot_t)(g_case_param_selected - 1);
    }

    display_menu_refresh_case_selection(previous_slot);
}

static void display_menu_case_param_select_down(void)
{
    flash_param_slot_t previous_slot = g_case_param_selected;

    g_case_param_selected = (flash_param_slot_t)(g_case_param_selected + 1);
    if(g_case_param_selected >= FLASH_PARAM_SLOT_COUNT)
    {
        g_case_param_selected = FLASH_PARAM_SLOT_FIRST;
    }

    display_menu_refresh_case_selection(previous_slot);
}

static void display_menu_case_param_adjust(int16 delta_tenth)
{
    int16 current_value = 0;
    int16 next_value = 0;

    current_value = flash_store_get_param_value_tenth(g_case_param_selected);
    next_value = (int16)(current_value + delta_tenth);

    if(next_value < FLASH_PARAM_VALUE_MIN_TENTH)
    {
        next_value = FLASH_PARAM_VALUE_MIN_TENTH;
    }
    else if(next_value > FLASH_PARAM_VALUE_MAX_TENTH)
    {
        next_value = FLASH_PARAM_VALUE_MAX_TENTH;
    }

    if(next_value != current_value)
    {
        flash_store_set_param_value_tenth(g_case_param_selected, next_value);
        display_menu_refresh_case_value();
    }
}

static void display_menu_case_param_adjust_by_step_mul(int8 direction, uint8 step_mul)
{
    int32 delta_tenth = 0;

    if(0 == direction || 0 == step_mul)
    {
        return;
    }

    delta_tenth = (int32)FLASH_PARAM_VALUE_STEP_TENTH * (int32)step_mul;
    if(direction < 0)
    {
        delta_tenth = -delta_tenth;
    }

    if(delta_tenth > 32767)
    {
        delta_tenth = 32767;
    }
    else if(delta_tenth < -32768)
    {
        delta_tenth = -32768;
    }

    display_menu_case_param_adjust((int16)delta_tenth);
}

static void display_menu_draw_camera_param_row(flash_camera_slot_t slot)
{
    char value_text[6];
    const char *label_text = 0;
    uint16 row_y = 0;
    uint16 guide_color = RGB565_GRAY;
    uint16 label_color = RGB565_BLACK;
    uint16 value_color = RGB565_BLACK;
    uint16 value = 0;

    row_y = display_menu_get_row_y((uint8)slot);
    label_text = display_menu_get_camera_param_label(slot);
    value = flash_store_get_camera_value(slot);
    display_menu_format_uint16(value, value_text);

    display_menu_draw_rect(MENU_PARAM_ACCENT_X,
                           (uint16)(row_y + MENU_PARAM_ACCENT_Y_OFFSET),
                           MENU_PARAM_ACCENT_W,
                           MENU_PARAM_ACCENT_H,
                           RGB565_WHITE);

    if(slot == g_camera_param_selected)
    {
        guide_color = g_param_editing ? RGB565_RED : RGB565_BLUE;
        label_color = guide_color;
        value_color = guide_color;
        display_menu_draw_rect(MENU_PARAM_ACCENT_X,
                               (uint16)(row_y + MENU_PARAM_ACCENT_Y_OFFSET),
                               MENU_PARAM_ACCENT_W,
                               MENU_PARAM_ACCENT_H,
                               guide_color);
    }
    else
    {
        label_color = RGB565_GRAY;
    }

    ips200_set_color(label_color, RGB565_WHITE);
    ips200_show_string(MENU_PARAM_LABEL_X, row_y, MENU_PARAM_LABEL_CLEAR);
    ips200_show_string(MENU_PARAM_LABEL_X, row_y, label_text);

    ips200_set_color(value_color, RGB565_WHITE);
    ips200_show_string(MENU_PARAM_VALUE_X, row_y, MENU_PARAM_VALUE_CLEAR);
    ips200_show_string(MENU_PARAM_VALUE_X, row_y, value_text);
}

static void display_menu_draw_camera_param_info(void)
{
    char min_text[6];
    char max_text[6];
    char step_text[6];
    uint16 min_value = 0;
    uint16 max_value = 0;
    uint16 step_value = 0;

    ips200_set_color(RGB565_BLACK, RGB565_WHITE);
    ips200_show_string(0, MENU_PARAM_INFO_Y, MENU_PARAM_INFO_CLEAR);
    if(!g_param_editing)
    {
        return;
    }

    display_menu_get_camera_param_range(g_camera_param_selected, &min_value, &max_value, &step_value);
    display_menu_format_uint16(min_value, min_text);
    display_menu_format_uint16(max_value, max_text);
    display_menu_format_uint16(step_value, step_text);

    ips200_set_color(RGB565_GRAY, RGB565_WHITE);
    ips200_show_string(0, MENU_PARAM_INFO_Y, "min");
    ips200_show_string(64, MENU_PARAM_INFO_Y, "max");
    ips200_show_string(136, MENU_PARAM_INFO_Y, "sp");

    ips200_set_color(RGB565_GREEN, RGB565_WHITE);
    ips200_show_string(24, MENU_PARAM_INFO_Y, min_text);

    ips200_set_color(RGB565_RED, RGB565_WHITE);
    ips200_show_string(88, MENU_PARAM_INFO_Y, max_text);

    ips200_set_color(RGB565_BLUE, RGB565_WHITE);
    ips200_show_string(152, MENU_PARAM_INFO_Y, step_text);
}

static void display_menu_draw_camera_param_page_full(void)
{
    ips200_clear(RGB565_WHITE);

    display_menu_draw_title("Camera Params");
    display_menu_draw_battery(1);
    display_menu_draw_dash_line(0, MENU_PARAM_INFO_LINE_Y, MENU_PARAM_DIVIDER_W, 10, 6, RGB565_GRAY);
    display_menu_draw_camera_param_row(FLASH_CAMERA_SLOT_AUTO_EXP);
    display_menu_draw_camera_param_row(FLASH_CAMERA_SLOT_EXP_TIME);
    display_menu_draw_camera_param_row(FLASH_CAMERA_SLOT_GAIN);
    display_menu_draw_camera_param_info();
}

static void display_menu_refresh_camera_selection(flash_camera_slot_t previous_slot)
{
    display_menu_draw_camera_param_row(previous_slot);
    display_menu_draw_camera_param_row(g_camera_param_selected);
}

static void display_menu_refresh_camera_mode(void)
{
    display_menu_draw_camera_param_row(g_camera_param_selected);
    display_menu_draw_camera_param_info();
}

static void display_menu_refresh_camera_value(void)
{
    display_menu_draw_camera_param_row(g_camera_param_selected);
    display_menu_draw_camera_param_info();
}

static void display_menu_camera_param_select_up(void)
{
    flash_camera_slot_t previous_slot = g_camera_param_selected;

    if(FLASH_CAMERA_SLOT_AUTO_EXP == g_camera_param_selected)
    {
        g_camera_param_selected = FLASH_CAMERA_SLOT_GAIN;
    }
    else
    {
        g_camera_param_selected = (flash_camera_slot_t)(g_camera_param_selected - 1);
    }

    display_menu_refresh_camera_selection(previous_slot);
}

static void display_menu_camera_param_select_down(void)
{
    flash_camera_slot_t previous_slot = g_camera_param_selected;

    g_camera_param_selected = (flash_camera_slot_t)(g_camera_param_selected + 1);
    if(g_camera_param_selected >= FLASH_CAMERA_SLOT_COUNT)
    {
        g_camera_param_selected = FLASH_CAMERA_SLOT_AUTO_EXP;
    }

    display_menu_refresh_camera_selection(previous_slot);
}

static void display_menu_camera_param_adjust(int16 delta)
{
    uint16 current_value = 0;
    uint16 min_value = 0;
    uint16 max_value = 0;
    uint16 step_value = 0;
    int32 next_value = 0;

    current_value = flash_store_get_camera_value(g_camera_param_selected);
    display_menu_get_camera_param_range(g_camera_param_selected, &min_value, &max_value, &step_value);
    next_value = (int32)current_value + (int32)delta;

    if(next_value < min_value)
    {
        next_value = min_value;
    }
    else if(next_value > max_value)
    {
        next_value = max_value;
    }

    if((uint16)next_value != current_value)
    {
        if(line_app_set_camera_param_value(g_camera_param_selected, (uint16)next_value))
        {
            display_menu_refresh_camera_value();
        }
    }
}

static void display_menu_camera_param_adjust_by_step_mul(int8 direction, uint8 step_mul)
{
    uint16 min_value = 0;
    uint16 max_value = 0;
    uint16 step_value = 0;
    int32 delta = 0;

    if(0 == direction || 0 == step_mul)
    {
        return;
    }

    display_menu_get_camera_param_range(g_camera_param_selected, &min_value, &max_value, &step_value);
    delta = (int32)step_value * (int32)step_mul;
    if(direction < 0)
    {
        delta = -delta;
    }

    if(delta > 32767)
    {
        delta = 32767;
    }
    else if(delta < -32768)
    {
        delta = -32768;
    }

    display_menu_camera_param_adjust((int16)delta);
}

static void display_menu_draw_root(void)
{
    uint8 i = 0;

    ips200_clear(RGB565_WHITE);
    display_menu_draw_battery(1);

    for(i = 0; i < MENU_ROOT_ITEM_COUNT; i++)
    {
        display_menu_draw_root_item(i);
    }
}

static void display_menu_draw_param_menu(void)
{
    uint8 i = 0;

    ips200_clear(RGB565_WHITE);
    display_menu_draw_title("Param Config");
    display_menu_draw_battery(1);

    for(i = 0; i < MENU_PARAM_MENU_ITEM_COUNT; i++)
    {
        display_menu_draw_param_menu_item(i);
    }
}

static void display_menu_prepare_camera_view(void)
{
    ips200_clear(RGB565_BLACK);
}

void display_menu_init(void)
{
    g_menu_selected = 0;
    g_param_menu_selected = 0;
    g_menu_page = DISPLAY_PAGE_ROOT;
    g_menu_dirty = 1;
    g_menu_last_battery_percent = 0xFF;
    g_case_param_selected = FLASH_PARAM_SLOT_FIRST;
    g_camera_param_selected = FLASH_CAMERA_SLOT_AUTO_EXP;
    g_param_editing = 0;
}

void display_menu_render(void)
{
    if(DISPLAY_PAGE_CAMERA == g_menu_page)
    {
        if(g_menu_dirty)
        {
            display_menu_prepare_camera_view();
            g_menu_dirty = 0;
        }
        return;
    }

    if(!g_menu_dirty)
    {
        display_menu_draw_battery(0);
        return;
    }

    switch(g_menu_page)
    {
        case DISPLAY_PAGE_ROOT:
            display_menu_draw_root();
            break;
        case DISPLAY_PAGE_PARAM_MENU:
            display_menu_draw_param_menu();
            break;
        case DISPLAY_PAGE_PARAM_CAMERA:
            display_menu_draw_camera_param_page_full();
            break;
        case DISPLAY_PAGE_PARAM_CASE:
            display_menu_draw_case_param_page_full();
            break;
        default:
            display_menu_draw_root();
            break;
    }

    g_menu_dirty = 0;
}

void display_menu_move_up(void)
{
    uint8 previous_selected = 0;

    if(DISPLAY_PAGE_PARAM_CAMERA == g_menu_page)
    {
        if(g_param_editing)
        {
            display_menu_camera_param_adjust_by_step_mul(1, 1);
        }
        else
        {
            display_menu_camera_param_select_up();
        }
        return;
    }

    if(DISPLAY_PAGE_PARAM_CASE == g_menu_page)
    {
        if(g_param_editing)
        {
            display_menu_case_param_adjust_by_step_mul(1, 1);
        }
        else
        {
            display_menu_case_param_select_up();
        }
        return;
    }

    if(DISPLAY_PAGE_PARAM_MENU == g_menu_page)
    {
        previous_selected = g_param_menu_selected;
        if(0 == g_param_menu_selected)
        {
            g_param_menu_selected = MENU_PARAM_MENU_ITEM_COUNT - 1;
        }
        else
        {
            g_param_menu_selected--;
        }

        display_menu_draw_param_menu_item(previous_selected);
        display_menu_draw_param_menu_item(g_param_menu_selected);
        return;
    }

    if(DISPLAY_PAGE_ROOT != g_menu_page)
    {
        return;
    }

    previous_selected = g_menu_selected;
    if(0 == g_menu_selected)
    {
        g_menu_selected = MENU_ROOT_ITEM_COUNT - 1;
    }
    else
    {
        g_menu_selected--;
    }

    if(g_menu_dirty)
    {
        display_menu_render();
    }
    else
    {
        display_menu_draw_root_item(previous_selected);
        display_menu_draw_root_item(g_menu_selected);
    }
}

void display_menu_move_down(void)
{
    uint8 previous_selected = 0;

    if(DISPLAY_PAGE_PARAM_CAMERA == g_menu_page)
    {
        if(g_param_editing)
        {
            display_menu_camera_param_adjust_by_step_mul(-1, 1);
        }
        else
        {
            display_menu_camera_param_select_down();
        }
        return;
    }

    if(DISPLAY_PAGE_PARAM_CASE == g_menu_page)
    {
        if(g_param_editing)
        {
            display_menu_case_param_adjust_by_step_mul(-1, 1);
        }
        else
        {
            display_menu_case_param_select_down();
        }
        return;
    }

    if(DISPLAY_PAGE_PARAM_MENU == g_menu_page)
    {
        previous_selected = g_param_menu_selected;
        g_param_menu_selected++;
        if(g_param_menu_selected >= MENU_PARAM_MENU_ITEM_COUNT)
        {
            g_param_menu_selected = 0;
        }

        display_menu_draw_param_menu_item(previous_selected);
        display_menu_draw_param_menu_item(g_param_menu_selected);
        return;
    }

    if(DISPLAY_PAGE_ROOT != g_menu_page)
    {
        return;
    }

    previous_selected = g_menu_selected;
    g_menu_selected++;
    if(g_menu_selected >= MENU_ROOT_ITEM_COUNT)
    {
        g_menu_selected = 0;
    }

    if(g_menu_dirty)
    {
        display_menu_render();
    }
    else
    {
        display_menu_draw_root_item(previous_selected);
        display_menu_draw_root_item(g_menu_selected);
    }
}

void display_menu_move_up_fast(void)
{
    if(!g_param_editing)
    {
        return;
    }

    if(DISPLAY_PAGE_PARAM_CAMERA == g_menu_page)
    {
        display_menu_camera_param_adjust_by_step_mul(1, MENU_PARAM_FAST_STEP_MUL);
    }
    else if(DISPLAY_PAGE_PARAM_CASE == g_menu_page)
    {
        display_menu_case_param_adjust_by_step_mul(1, MENU_PARAM_FAST_STEP_MUL);
    }
}

void display_menu_move_down_fast(void)
{
    if(!g_param_editing)
    {
        return;
    }

    if(DISPLAY_PAGE_PARAM_CAMERA == g_menu_page)
    {
        display_menu_camera_param_adjust_by_step_mul(-1, MENU_PARAM_FAST_STEP_MUL);
    }
    else if(DISPLAY_PAGE_PARAM_CASE == g_menu_page)
    {
        display_menu_case_param_adjust_by_step_mul(-1, MENU_PARAM_FAST_STEP_MUL);
    }
}

void display_menu_enter(void)
{
    if(DISPLAY_PAGE_PARAM_CAMERA == g_menu_page)
    {
        g_param_editing = (uint8)!g_param_editing;
        display_menu_refresh_camera_mode();
        return;
    }

    if(DISPLAY_PAGE_PARAM_CASE == g_menu_page)
    {
        g_param_editing = (uint8)!g_param_editing;
        display_menu_refresh_case_mode();
        return;
    }

    if(DISPLAY_PAGE_PARAM_MENU == g_menu_page)
    {
        g_param_editing = 0;
        if(0 == g_param_menu_selected)
        {
            g_camera_param_selected = FLASH_CAMERA_SLOT_AUTO_EXP;
            g_menu_page = DISPLAY_PAGE_PARAM_CAMERA;
        }
        else
        {
            g_case_param_selected = FLASH_PARAM_SLOT_FIRST;
            g_menu_page = DISPLAY_PAGE_PARAM_CASE;
        }
        display_menu_mark_dirty();
        display_menu_render();
        return;
    }

    if(DISPLAY_PAGE_ROOT != g_menu_page)
    {
        display_menu_back();
        return;
    }

    if(0 == g_menu_selected)
    {
        g_menu_page = DISPLAY_PAGE_CAMERA;
    }
    else
    {
        g_param_menu_selected = 0;
        g_menu_page = DISPLAY_PAGE_PARAM_MENU;
    }

    g_param_editing = 0;
    display_menu_mark_dirty();
    display_menu_render();
}

void display_menu_back(void)
{
    if(DISPLAY_PAGE_ROOT == g_menu_page)
    {
        return;
    }

    if(DISPLAY_PAGE_PARAM_CAMERA == g_menu_page)
    {
        if(g_param_editing)
        {
            g_param_editing = 0;
            display_menu_refresh_camera_mode();
            return;
        }

        g_menu_page = DISPLAY_PAGE_PARAM_MENU;
        display_menu_mark_dirty();
        display_menu_render();
        return;
    }

    if(DISPLAY_PAGE_PARAM_CASE == g_menu_page)
    {
        if(g_param_editing)
        {
            g_param_editing = 0;
            display_menu_refresh_case_mode();
            return;
        }

        g_menu_page = DISPLAY_PAGE_PARAM_MENU;
        display_menu_mark_dirty();
        display_menu_render();
        return;
    }

    if(DISPLAY_PAGE_PARAM_MENU == g_menu_page)
    {
        g_menu_page = DISPLAY_PAGE_ROOT;
        display_menu_mark_dirty();
        car_servo_set_center();
        display_menu_render();
        return;
    }

    g_menu_page = DISPLAY_PAGE_ROOT;
    display_menu_mark_dirty();
    car_servo_set_center();
    display_menu_render();
}

uint8 display_menu_in_camera_view(void)
{
    return (DISPLAY_PAGE_CAMERA == g_menu_page) ? 1 : 0;
}
