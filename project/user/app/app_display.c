/*
 * app_display.c - simple two-level menu demo for IPS200
 */

#include "app_display.h"

#include "dev_adc.h"
#include "dev_flash.h"
#include "dev_servo.h"

#define MENU_LINE_HEIGHT            (16)
#define MENU_ITEM_COUNT             (2)
#define MENU_TITLE_Y                (0)
#define MENU_LIST_Y                 (16)
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
    DISPLAY_PAGE_PARAM
} display_page_t;

static const char *g_menu_titles[MENU_ITEM_COUNT] =
{
    "Camera View",
    "Param Config"
};

static uint8 g_menu_selected = 0;
static display_page_t g_menu_page = DISPLAY_PAGE_ROOT;
static uint8 g_menu_dirty = 1;
static uint8 g_menu_last_battery_percent = 0xFF;
static flash_param_slot_t g_param_selected = FLASH_PARAM_SLOT_FIRST; /* 当前选中的参数行。 */
static uint8 g_param_editing = 0;                                    /* 0 先选行，1 直接改值。 */

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

/* 内部按 0.1 存，显示时还是给屏幕拼成普通小数。 */
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

/* 参数页两行的位置先单独收口，后面加参数时好改。 */
static uint16 display_menu_get_param_row_y(flash_param_slot_t slot)
{
    return (uint16)(MENU_PARAM_LIST_Y + (uint16)slot * MENU_PARAM_ROW_STEP);
}

/* 参数名先放在这里，后面换成真实参数时只改这一处。 */
static const char *display_menu_get_param_label(flash_param_slot_t slot)
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

/* 只重画一行，这样切换选中和改值时不用整页闪。 */
static void display_menu_draw_param_row(flash_param_slot_t slot)
{
    char value_text[5];
    const char *label_text = 0;
    uint16 row_y = 0;
    uint16 guide_color = RGB565_GRAY;
    uint16 label_color = RGB565_BLACK;
    uint16 value_color = RGB565_BLACK;
    int16 value_tenth = 0;

    row_y = display_menu_get_param_row_y(slot);
    label_text = display_menu_get_param_label(slot);
    value_tenth = flash_store_get_param_value_tenth(slot);
    display_menu_format_param_value(value_tenth, value_text);

    display_menu_draw_rect(MENU_PARAM_ACCENT_X,
                           (uint16)(row_y + MENU_PARAM_ACCENT_Y_OFFSET),
                           MENU_PARAM_ACCENT_W,
                           MENU_PARAM_ACCENT_H,
                           RGB565_WHITE);

    if(slot == g_param_selected)
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

/* 进入编辑态后，再把范围和步进放出来，平时不占地方。 */
static void display_menu_draw_param_info(void)
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

/* 参数页第一次进入时整页画一遍，后面就尽量只改局部。 */
static void display_menu_draw_param_page_full(void)
{
    ips200_clear(RGB565_WHITE);

    ips200_set_color(RGB565_BLUE, RGB565_WHITE);
    ips200_show_string(0, MENU_TITLE_Y, "Param Config");
    display_menu_draw_battery(1);

    display_menu_draw_dash_line(0, MENU_PARAM_INFO_LINE_Y, MENU_PARAM_DIVIDER_W, 10, 6, RGB565_GRAY);
    display_menu_draw_param_row(FLASH_PARAM_SLOT_FIRST);
    display_menu_draw_param_row(FLASH_PARAM_SLOT_SECOND);
    display_menu_draw_param_info();
}

/* 只更新选中变化涉及到的那两行。 */
static void display_menu_refresh_param_selection(flash_param_slot_t previous_slot)
{
    display_menu_draw_param_row(previous_slot);
    display_menu_draw_param_row(g_param_selected);
}

/* 编辑状态切换后，当前行和底部信息要一起变。 */
static void display_menu_refresh_param_mode(void)
{
    display_menu_draw_param_row(g_param_selected);
    display_menu_draw_param_info();
}

/* 数值变化时，只重画当前行和底部信息。 */
static void display_menu_refresh_param_value(void)
{
    display_menu_draw_param_row(g_param_selected);
    display_menu_draw_param_info();
}

/* 没进编辑态时，上键就是换行。 */
static void display_menu_param_select_up(void)
{
    flash_param_slot_t previous_slot = g_param_selected;

    if(FLASH_PARAM_SLOT_FIRST == g_param_selected)
    {
        g_param_selected = FLASH_PARAM_SLOT_SECOND;
    }
    else
    {
        g_param_selected = (flash_param_slot_t)(g_param_selected - 1);
    }

    display_menu_refresh_param_selection(previous_slot);
}

/* 没进编辑态时，下键也是换行。 */
static void display_menu_param_select_down(void)
{
    flash_param_slot_t previous_slot = g_param_selected;

    g_param_selected = (flash_param_slot_t)(g_param_selected + 1);
    if(g_param_selected >= FLASH_PARAM_SLOT_COUNT)
    {
        g_param_selected = FLASH_PARAM_SLOT_FIRST;
    }

    display_menu_refresh_param_selection(previous_slot);
}

/* 进了编辑态，上下键就直接改当前值。 */
static void display_menu_param_adjust(int16 delta_tenth)
{
    int16 current_value = 0;
    int16 next_value = 0;

    current_value = flash_store_get_param_value_tenth(g_param_selected);
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
        flash_store_set_param_value_tenth(g_param_selected, next_value);
        display_menu_refresh_param_value();
    }
}

static void display_menu_draw_root_item(uint8 index)
{
    uint16 y = 0;

    if(index >= MENU_ITEM_COUNT)
    {
        return;
    }

    y = MENU_LIST_Y + (uint16)index * MENU_LINE_HEIGHT;
    if(index == g_menu_selected)
    {
        ips200_set_color(RGB565_WHITE, RGB565_BLUE);
        ips200_show_string(0, y, "> ");
        ips200_show_string(16, y, g_menu_titles[index]);
    }
    else
    {
        ips200_set_color(RGB565_BLACK, RGB565_WHITE);
        ips200_show_string(0, y, "  ");
        ips200_show_string(16, y, g_menu_titles[index]);
    }
}

static void display_menu_draw_root(void)
{
    uint8 i = 0;

    ips200_clear(RGB565_WHITE);
    display_menu_draw_battery(1);

    for(i = 0; i < MENU_ITEM_COUNT; i++)
    {
        display_menu_draw_root_item(i);
    }
}

static void display_menu_prepare_camera_view(void)
{
    /* 摄像头页不再叠加菜单文案，只保留图像区域，其余区域统一拉黑。 */
    ips200_clear(RGB565_BLACK);
}

void display_menu_init(void)
{
    g_menu_selected = 0;
    g_menu_page = DISPLAY_PAGE_ROOT;
    g_menu_dirty = 1;
    g_menu_last_battery_percent = 0xFF;
    g_param_selected = FLASH_PARAM_SLOT_FIRST;
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

    if(DISPLAY_PAGE_ROOT == g_menu_page)
    {
        display_menu_draw_root();
    }
    else if(DISPLAY_PAGE_PARAM == g_menu_page)
    {
        display_menu_draw_param_page_full();
    }

    g_menu_dirty = 0;
}

void display_menu_move_up(void)
{
    uint8 previous_selected = 0;

    if(DISPLAY_PAGE_PARAM == g_menu_page)
    {
        /* 参数页里，上键要么切行，要么增大当前值。 */
        if(g_param_editing)
        {
            display_menu_param_adjust(FLASH_PARAM_VALUE_STEP_TENTH);
        }
        else
        {
            display_menu_param_select_up();
        }
        return;
    }

    if(DISPLAY_PAGE_ROOT != g_menu_page)
    {
        return;
    }

    previous_selected = g_menu_selected;
    if(0 == g_menu_selected)
    {
        g_menu_selected = MENU_ITEM_COUNT - 1;
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

    if(DISPLAY_PAGE_PARAM == g_menu_page)
    {
        /* 参数页里，下键要么切行，要么减小当前值。 */
        if(g_param_editing)
        {
            display_menu_param_adjust(-FLASH_PARAM_VALUE_STEP_TENTH);
        }
        else
        {
            display_menu_param_select_down();
        }
        return;
    }

    if(DISPLAY_PAGE_ROOT != g_menu_page)
    {
        return;
    }

    previous_selected = g_menu_selected;
    g_menu_selected++;
    if(g_menu_selected >= MENU_ITEM_COUNT)
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

void display_menu_enter(void)
{
    if(DISPLAY_PAGE_PARAM == g_menu_page)
    {
        /* 参数页里，确认键就是在选行和改值之间来回切。 */
        g_param_editing = (uint8)!g_param_editing;
        display_menu_refresh_param_mode();
        return;
    }

    if(DISPLAY_PAGE_ROOT != g_menu_page)
    {
        display_menu_back();
        return;
    }

    g_menu_page = (display_page_t)(DISPLAY_PAGE_CAMERA + g_menu_selected);
    if(DISPLAY_PAGE_PARAM == g_menu_page)
    {
        g_param_selected = FLASH_PARAM_SLOT_FIRST;
        g_param_editing = 0;
    }
    display_menu_mark_dirty();
    display_menu_render();
}

void display_menu_back(void)
{
    if(DISPLAY_PAGE_ROOT == g_menu_page)
    {
        return;
    }

    if(DISPLAY_PAGE_PARAM == g_menu_page && g_param_editing)
    {
        /* 正在改值时，先退回选行状态。 */
        g_param_editing = 0;
        display_menu_refresh_param_mode();
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
