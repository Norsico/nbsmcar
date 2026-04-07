/*
 * app_display.c - IPS200 菜单与调参页面
 */

#include "app_display.h"

#include "app_line.h"
#include "dev_adc.h"
#include "dev_flash.h"
#include "dev_servo.h"
#include "dev_wheel.h"
#include "system_state.h"

#define MENU_LINE_HEIGHT            (16)
#define MENU_ROOT_ITEM_COUNT        (3)
#define MENU_PARAM_MENU_ITEM_COUNT  (2)
#define MENU_LINE_MENU_ITEM_COUNT   (2)
#define MENU_ARRAY_COUNT(array)     ((uint8)(sizeof(array) / sizeof((array)[0])))
#define MENU_TITLE_Y                (0)
#define MENU_ROOT_LIST_Y            (16)
#define MENU_SUBMENU_LIST_Y         (24)
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
    DISPLAY_PAGE_START,
    DISPLAY_PAGE_PARAM_MENU,
    DISPLAY_PAGE_PARAM_CAMERA,
    DISPLAY_PAGE_PARAM_LINE_MENU,
    DISPLAY_PAGE_PARAM_LINE_PID,
    DISPLAY_PAGE_PARAM_SERVO_LIMIT
} display_page_t;

static const char *g_root_menu_titles[MENU_ROOT_ITEM_COUNT] =
{
    "Camera View",
    "Param Config",
    "Start"
};

static const char *g_param_menu_titles[MENU_PARAM_MENU_ITEM_COUNT] =
{
    "Camera",
    "Line Tune"
};

static const char *g_line_menu_titles[MENU_LINE_MENU_ITEM_COUNT] =
{
    "Line PID",
    "Servo Limit"
};

static const line_tune_slot_t g_line_pid_slots[2] =
{
    LINE_TUNE_SLOT_KP,
    LINE_TUNE_SLOT_KD
};

static const line_tune_slot_t g_line_servo_slots[2] =
{
    LINE_TUNE_SLOT_SERVO_MIN,
    LINE_TUNE_SLOT_SERVO_MAX
};

typedef enum
{
    START_SLOT_SPEED = 0,
    START_SLOT_ENABLE
} start_slot_t;

typedef struct
{
    display_page_t page;
    const char *title;
    const line_tune_slot_t *slot_list;
    uint8 slot_count;
    line_tune_slot_t *selected_slot;
} display_line_page_t;

typedef void (*display_menu_draw_item_t)(uint8 index);

static uint8 g_menu_selected = 0;
static uint8 g_param_menu_selected = 0;
static display_page_t g_menu_page = DISPLAY_PAGE_ROOT;
static uint8 g_menu_dirty = 1;
static uint8 g_menu_last_battery_percent = 0xFF;
static flash_camera_slot_t g_camera_param_selected = FLASH_CAMERA_SLOT_EXP_TIME;
static uint8 g_line_menu_selected = 0;
static line_tune_slot_t g_line_pid_selected = LINE_TUNE_SLOT_KP;
static line_tune_slot_t g_line_servo_selected = LINE_TUNE_SLOT_SERVO_MIN;
static start_slot_t g_start_selected = START_SLOT_SPEED;
static flash_start_page_t g_start_page = {FLASH_START_SPEED_DEFAULT, FLASH_START_ENABLE_DEFAULT, 0};
static uint8 g_param_editing = 0;

static const display_line_page_t g_line_pages[MENU_LINE_MENU_ITEM_COUNT] =
{
    {DISPLAY_PAGE_PARAM_LINE_PID, "Line PID", g_line_pid_slots, MENU_ARRAY_COUNT(g_line_pid_slots), &g_line_pid_selected},
    {DISPLAY_PAGE_PARAM_SERVO_LIMIT, "Servo Limit", g_line_servo_slots, MENU_ARRAY_COUNT(g_line_servo_slots), &g_line_servo_selected}
};

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

static void display_menu_draw_line_menu_item(uint8 index)
{
    if(index >= MENU_LINE_MENU_ITEM_COUNT)
    {
        return;
    }

    display_menu_draw_list_item((uint16)(MENU_SUBMENU_LIST_Y + (uint16)index * MENU_LINE_HEIGHT),
                                g_line_menu_titles[index],
                                (index == g_line_menu_selected) ? 1 : 0);
}

static const display_line_page_t *display_menu_get_line_page(display_page_t page)
{
    uint8 i = 0;

    for(i = 0; i < MENU_ARRAY_COUNT(g_line_pages); i++)
    {
        if(g_line_pages[i].page == page)
        {
            return &g_line_pages[i];
        }
    }

    return 0;
}

static void display_menu_cycle_list_selection(uint8 *selected,
                                              uint8 item_count,
                                              int8 direction,
                                              display_menu_draw_item_t draw_item)
{
    uint8 previous_selected = 0;

    if(0 == selected || 0 == draw_item || 0 == item_count || 0 == direction)
    {
        return;
    }

    previous_selected = *selected;
    if(direction > 0)
    {
        if(0 == *selected)
        {
            *selected = item_count - 1;
        }
        else
        {
            (*selected)--;
        }
    }
    else
    {
        (*selected)++;
        if(*selected >= item_count)
        {
            *selected = 0;
        }
    }

    draw_item(previous_selected);
    draw_item(*selected);
}

/* 把当前选中的巡线参数映射成行号，方便不同子页共用同一套绘制逻辑。 */
static uint8 display_menu_find_line_tune_slot_index(const line_tune_slot_t *slot_list,
                                                    uint8 slot_count,
                                                    line_tune_slot_t slot)
{
    uint8 i = 0;

    for(i = 0; i < slot_count; i++)
    {
        if(slot_list[i] == slot)
        {
            return i;
        }
    }

    return 0;
}

/* 这里统一维护巡线调参项的显示名称，后面改文案只动这一处。 */
static const char *display_menu_get_line_tune_label(line_tune_slot_t slot)
{
    switch(slot)
    {
        case LINE_TUNE_SLOT_KP:
            return "kp";
        case LINE_TUNE_SLOT_KD:
            return "kd";
        case LINE_TUNE_SLOT_SERVO_MIN:
            return "servo min";
        case LINE_TUNE_SLOT_SERVO_MAX:
            return "servo max";
        default:
            return "";
    }
}

/* KP/KD 用 0.1 显示，其它项按整数显示，避免 UI 层到处写分支。 */
static void display_menu_format_line_tune_value(line_tune_slot_t slot, uint16 value, char *text)
{
    switch(slot)
    {
        case LINE_TUNE_SLOT_KP:
        case LINE_TUNE_SLOT_KD:
            display_menu_format_param_value((int16)value, text);
            break;
        default:
            display_menu_format_uint16(value, text);
            break;
    }
}

/* servo min/max 这类成对参数在显示层先收一遍，避免 UI 先送非法组合。 */
static void display_menu_get_line_tune_effective_range(line_tune_slot_t slot,
                                                       uint16 *min_value,
                                                       uint16 *max_value,
                                                       uint16 *step_value)
{
    uint16 pair_value = 0;

    if(0 == min_value || 0 == max_value || 0 == step_value)
    {
        return;
    }

    line_app_get_tune_range(slot, min_value, max_value, step_value);
    switch(slot)
    {
        case LINE_TUNE_SLOT_SERVO_MIN:
            pair_value = line_app_get_tune_value(LINE_TUNE_SLOT_SERVO_MAX);
            pair_value--;
            if(pair_value < *max_value)
            {
                *max_value = pair_value;
            }
            break;
        case LINE_TUNE_SLOT_SERVO_MAX:
            pair_value = line_app_get_tune_value(LINE_TUNE_SLOT_SERVO_MIN);
            pair_value++;
            if(pair_value > *min_value)
            {
                *min_value = pair_value;
            }
            break;
        default:
            break;
    }

    if(*min_value > *max_value)
    {
        *min_value = *max_value;
    }
}

static int16 display_menu_build_step_delta(uint16 step_value, int8 direction, uint8 step_mul)
{
    int32 delta = 0;

    if(0 == direction || 0 == step_mul || 0 == step_value)
    {
        return 0;
    }

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

    return (int16)delta;
}

static const char *display_menu_get_camera_param_label(flash_camera_slot_t slot)
{
    switch(slot)
    {
        case FLASH_CAMERA_SLOT_EXP_TIME:
            return "exp time";
        case FLASH_CAMERA_SLOT_GAIN:
            return "gain";
        default:
            return "";
    }
}

static uint8 display_menu_get_camera_param_row_index(flash_camera_slot_t slot)
{
    switch(slot)
    {
        case FLASH_CAMERA_SLOT_EXP_TIME:
            return 0;
        case FLASH_CAMERA_SLOT_GAIN:
            return 1;
        default:
            return 0;
    }
}

static void display_menu_get_camera_param_range(flash_camera_slot_t slot,
                                                uint16 *min_value,
                                                uint16 *max_value,
                                                uint16 *step_value)
{
    uint16 min_value_local = 0;
    uint16 max_value_local = 0;
    uint16 step_value_local = 0;

    switch(slot)
    {
        case FLASH_CAMERA_SLOT_EXP_TIME:
            min_value_local = FLASH_CAMERA_EXP_TIME_MIN;
            max_value_local = FLASH_CAMERA_EXP_TIME_MAX;
            step_value_local = FLASH_CAMERA_EXP_TIME_STEP;
            break;
        case FLASH_CAMERA_SLOT_GAIN:
            min_value_local = FLASH_CAMERA_GAIN_MIN;
            max_value_local = FLASH_CAMERA_GAIN_MAX;
            step_value_local = FLASH_CAMERA_GAIN_STEP;
            break;
        default:
            break;
    }

    if(0 != min_value)
    {
        *min_value = min_value_local;
    }

    if(0 != max_value)
    {
        *max_value = max_value_local;
    }

    if(0 != step_value)
    {
        *step_value = step_value_local;
    }
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

    row_y = display_menu_get_row_y(display_menu_get_camera_param_row_index(slot));
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

    display_menu_draw_title("Camera");
    display_menu_draw_battery(1);
    display_menu_draw_dash_line(0, MENU_PARAM_INFO_LINE_Y, MENU_PARAM_DIVIDER_W, 10, 6, RGB565_GRAY);
    display_menu_draw_camera_param_row(FLASH_CAMERA_SLOT_EXP_TIME);
    display_menu_draw_camera_param_row(FLASH_CAMERA_SLOT_GAIN);
    display_menu_draw_camera_param_info();
}

static void display_menu_refresh_camera_selection(flash_camera_slot_t previous_slot)
{
    display_menu_draw_camera_param_row(previous_slot);
    display_menu_draw_camera_param_row(g_camera_param_selected);
}

static void display_menu_refresh_camera_current(void)
{
    display_menu_draw_camera_param_row(g_camera_param_selected);
    display_menu_draw_camera_param_info();
}

static void display_menu_camera_param_select_up(void)
{
    flash_camera_slot_t previous_slot = g_camera_param_selected;

    if(FLASH_CAMERA_SLOT_EXP_TIME == g_camera_param_selected)
    {
        g_camera_param_selected = FLASH_CAMERA_SLOT_GAIN;
    }
    else
    {
        g_camera_param_selected = FLASH_CAMERA_SLOT_EXP_TIME;
    }

    display_menu_refresh_camera_selection(previous_slot);
}

static void display_menu_camera_param_select_down(void)
{
    flash_camera_slot_t previous_slot = g_camera_param_selected;

    if(FLASH_CAMERA_SLOT_GAIN == g_camera_param_selected)
    {
        g_camera_param_selected = FLASH_CAMERA_SLOT_EXP_TIME;
    }
    else
    {
        g_camera_param_selected = FLASH_CAMERA_SLOT_GAIN;
    }

    display_menu_refresh_camera_selection(previous_slot);
}

static void display_menu_camera_param_adjust(int16 delta)
{
    uint16 current_value = 0;
    uint16 min_value = 0;
    uint16 max_value = 0;
    int32 next_value = 0;

    current_value = flash_store_get_camera_value(g_camera_param_selected);
    display_menu_get_camera_param_range(g_camera_param_selected, &min_value, &max_value, 0);
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
            display_menu_refresh_camera_current();
        }
    }
}

static void display_menu_camera_param_adjust_by_step_mul(int8 direction, uint8 step_mul)
{
    uint16 step_value = 0;
    int16 delta = 0;

    display_menu_get_camera_param_range(g_camera_param_selected, 0, 0, &step_value);
    delta = display_menu_build_step_delta(step_value, direction, step_mul);
    if(0 == delta)
    {
        return;
    }

    display_menu_camera_param_adjust(delta);
}

static void display_menu_fill_default_start_page(flash_start_page_t *page)
{
    if(0 == page)
    {
        return;
    }

    page->target_speed = FLASH_START_SPEED_DEFAULT;
    page->enable = FLASH_START_ENABLE_DEFAULT;
    page->reserved = 0;
}

/* 启动页就两项，参数规则简单，先在显示层做一层轻量收口。 */
static void display_menu_normalize_start_page(flash_start_page_t *page)
{
    if(0 == page)
    {
        return;
    }

    if(page->target_speed > FLASH_START_SPEED_MAX)
    {
        page->target_speed = FLASH_START_SPEED_MAX;
    }

    if(page->enable > FLASH_START_ENABLE_MAX)
    {
        page->enable = FLASH_START_ENABLE_MAX;
    }

    page->reserved = 0;
}

/* 这里用一份本地校验，避免 flash 读坏后直接把异常值送到后轮目标。 */
static uint8 display_menu_start_page_is_valid(const flash_start_page_t *page)
{
    if(0 == page)
    {
        return 0;
    }

    if(page->target_speed < FLASH_START_SPEED_MIN || page->target_speed > FLASH_START_SPEED_MAX)
    {
        return 0;
    }

    if(page->enable < FLASH_START_ENABLE_MIN || page->enable > FLASH_START_ENABLE_MAX)
    {
        return 0;
    }

    return 1;
}

/* 启动页改值后统一走这里：同步闭环目标值，并按开关切到 PREPARE/RUNNING。 */
static void display_menu_apply_start_page(const flash_start_page_t *page)
{
    uint8 previous_enable = g_start_page.enable;

    if(0 == page)
    {
        return;
    }

    g_start_page.target_speed = page->target_speed;
    g_start_page.enable = page->enable;
    g_start_page.reserved = 0;
    display_menu_normalize_start_page(&g_start_page);

    if(g_start_page.enable)
    {
        if(!previous_enable)
        {
            car_wheel_control_reset();
        }

        /* 当前已知开屏时屏幕刷新会影响后轮闭环，实车用 Start 建议先关屏再跑。 */
        car_wheel_set_target((float)g_start_page.target_speed);
        if(DISPLAY_PAGE_CAMERA != g_menu_page)
        {
            car_servo_set_center();
        }

        if(SYS_EMERGENCY != g_system_state)
        {
            g_system_state = SYS_RUNNING;
        }
    }
    else
    {
        car_wheel_control_reset();
        if(SYS_EMERGENCY != g_system_state)
        {
            g_system_state = SYS_PREPARE;
        }
    }
}

/* 上电先把启动页参数从 flash 拉出来，异常就按默认值重建。 */
static void display_menu_load_start_page_from_flash(void)
{
    flash_start_page_t page;

    flash_store_get_start_page(&page);
    if(!display_menu_start_page_is_valid(&page))
    {
        display_menu_fill_default_start_page(&page);
        flash_store_set_start_page(&page);
    }

    display_menu_apply_start_page(&page);
}

uint8 display_menu_start_is_enabled(void)
{
    return g_start_page.enable;
}

static const char *display_menu_get_start_label(start_slot_t slot)
{
    if(START_SLOT_SPEED == slot)
    {
        return "speed";
    }

    return "enable";
}

static void display_menu_format_start_value(start_slot_t slot, char *text)
{
    if(START_SLOT_SPEED == slot)
    {
        display_menu_format_uint16(g_start_page.target_speed, text);
    }
    else if(g_start_page.enable)
    {
        text[0] = 'o';
        text[1] = 'n';
        text[2] = '\0';
    }
    else
    {
        text[0] = 'o';
        text[1] = 'f';
        text[2] = 'f';
        text[3] = '\0';
    }
}

static void display_menu_draw_start_row(start_slot_t slot)
{
    char value_text[6];
    const char *label_text = 0;
    uint16 row_y = 0;
    uint16 guide_color = RGB565_GRAY;
    uint16 label_color = RGB565_BLACK;
    uint16 value_color = RGB565_BLACK;

    row_y = display_menu_get_row_y((uint8)slot);
    label_text = display_menu_get_start_label(slot);
    display_menu_format_start_value(slot, value_text);

    display_menu_draw_rect(MENU_PARAM_ACCENT_X,
                           (uint16)(row_y + MENU_PARAM_ACCENT_Y_OFFSET),
                           MENU_PARAM_ACCENT_W,
                           MENU_PARAM_ACCENT_H,
                           RGB565_WHITE);

    if(slot == g_start_selected)
    {
        guide_color = (g_param_editing && (START_SLOT_SPEED == slot)) ? RGB565_RED : RGB565_BLUE;
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

static void display_menu_draw_start_info(void)
{
    char min_text[6];
    char max_text[6];
    char step_text[6];

    ips200_set_color(RGB565_BLACK, RGB565_WHITE);
    ips200_show_string(0, MENU_PARAM_INFO_Y, MENU_PARAM_INFO_CLEAR);

    if(START_SLOT_SPEED == g_start_selected)
    {
        if(!g_param_editing)
        {
            return;
        }

        display_menu_format_uint16(FLASH_START_SPEED_MIN, min_text);
        display_menu_format_uint16(FLASH_START_SPEED_MAX, max_text);
        display_menu_format_uint16(FLASH_START_SPEED_STEP, step_text);

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
        return;
    }

    /* 开关项不走编辑态，直接提示确认键就是切换。 */
    ips200_set_color(RGB565_GRAY, RGB565_WHITE);
    ips200_show_string(0, MENU_PARAM_INFO_Y, "ENT");
    ips200_set_color(RGB565_BLUE, RGB565_WHITE);
    ips200_show_string(32, MENU_PARAM_INFO_Y, "toggle");
}

static void display_menu_draw_start_page_full(void)
{
    ips200_clear(RGB565_WHITE);

    display_menu_draw_title("Start");
    display_menu_draw_battery(1);
    display_menu_draw_dash_line(0, MENU_PARAM_INFO_LINE_Y, MENU_PARAM_DIVIDER_W, 10, 6, RGB565_GRAY);
    display_menu_draw_start_row(START_SLOT_SPEED);
    display_menu_draw_start_row(START_SLOT_ENABLE);
    display_menu_draw_start_info();
}

static void display_menu_refresh_start_selection(start_slot_t previous_slot)
{
    display_menu_draw_start_row(previous_slot);
    display_menu_draw_start_row(g_start_selected);
    display_menu_draw_start_info();
}

static void display_menu_refresh_start_mode(void)
{
    display_menu_draw_start_row(g_start_selected);
    display_menu_draw_start_info();
}

static void display_menu_refresh_start_value(void)
{
    display_menu_draw_start_row(START_SLOT_SPEED);
    display_menu_draw_start_row(START_SLOT_ENABLE);
    display_menu_draw_start_info();
}

static void display_menu_start_select_up(void)
{
    start_slot_t previous_slot = g_start_selected;

    if(START_SLOT_SPEED == g_start_selected)
    {
        g_start_selected = START_SLOT_ENABLE;
    }
    else
    {
        g_start_selected = START_SLOT_SPEED;
    }

    display_menu_refresh_start_selection(previous_slot);
}

static void display_menu_start_select_down(void)
{
    display_menu_start_select_up();
}

static void display_menu_start_adjust_speed(int16 delta)
{
    flash_start_page_t page;
    int32 next_speed = 0;

    page = g_start_page;
    next_speed = (int32)page.target_speed + (int32)delta;
    if(next_speed < FLASH_START_SPEED_MIN)
    {
        next_speed = FLASH_START_SPEED_MIN;
    }
    else if(next_speed > FLASH_START_SPEED_MAX)
    {
        next_speed = FLASH_START_SPEED_MAX;
    }

    if((uint16)next_speed == page.target_speed)
    {
        return;
    }

    page.target_speed = (uint16)next_speed;
    display_menu_apply_start_page(&page);
    flash_store_set_start_page(&g_start_page);
}

static void display_menu_start_adjust_speed_by_step_mul(int8 direction, uint8 step_mul)
{
    int16 delta = 0;

    delta = display_menu_build_step_delta(FLASH_START_SPEED_STEP, direction, step_mul);
    if(0 == delta)
    {
        return;
    }

    display_menu_start_adjust_speed(delta);
}

static void display_menu_start_toggle_enable(void)
{
    flash_start_page_t page;

    /* 开关值只有 0/1，确认键直接翻转并立即同步到运行状态和 flash。 */
    page = g_start_page;
    page.enable = (uint8)!page.enable;
    display_menu_apply_start_page(&page);
    flash_store_set_start_page(&g_start_page);
}

static void display_menu_draw_line_tune_row(const line_tune_slot_t *slot_list,
                                            uint8 slot_count,
                                            line_tune_slot_t slot,
                                            line_tune_slot_t selected_slot)
{
    char value_text[6];
    const char *label_text = 0;
    uint16 row_y = 0;
    uint16 guide_color = RGB565_GRAY;
    uint16 label_color = RGB565_BLACK;
    uint16 value_color = RGB565_BLACK;
    uint16 value = 0;
    uint8 row_index = 0;

    row_index = display_menu_find_line_tune_slot_index(slot_list, slot_count, slot);
    if(slot_count > 3)
    {
        row_y = (uint16)(MENU_PARAM_LIST_Y + (uint16)row_index * MENU_LINE_HEIGHT);
    }
    else
    {
        row_y = display_menu_get_row_y(row_index);
    }
    label_text = display_menu_get_line_tune_label(slot);
    value = line_app_get_tune_value(slot);
    display_menu_format_line_tune_value(slot, value, value_text);

    display_menu_draw_rect(MENU_PARAM_ACCENT_X,
                           (uint16)(row_y + MENU_PARAM_ACCENT_Y_OFFSET),
                           MENU_PARAM_ACCENT_W,
                           MENU_PARAM_ACCENT_H,
                           RGB565_WHITE);

    if(slot == selected_slot)
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

/* 巡线调参页沿用 Camera 页那套 min/max/step 提示，编辑时才显示。 */
static void display_menu_draw_line_tune_info(line_tune_slot_t selected_slot)
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

    display_menu_get_line_tune_effective_range(selected_slot, &min_value, &max_value, &step_value);
    display_menu_format_line_tune_value(selected_slot, min_value, min_text);
    display_menu_format_line_tune_value(selected_slot, max_value, max_text);
    display_menu_format_line_tune_value(selected_slot, step_value, step_text);

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

/* Line PID / Servo Limit 两页都复用这一套通用渲染函数。 */
static void display_menu_draw_line_tune_page_full(const char *title,
                                                  const line_tune_slot_t *slot_list,
                                                  uint8 slot_count,
                                                  line_tune_slot_t selected_slot)
{
    uint8 i = 0;

    ips200_clear(RGB565_WHITE);

    display_menu_draw_title(title);
    display_menu_draw_battery(1);
    display_menu_draw_dash_line(0, MENU_PARAM_INFO_LINE_Y, MENU_PARAM_DIVIDER_W, 10, 6, RGB565_GRAY);
    for(i = 0; i < slot_count; i++)
    {
        display_menu_draw_line_tune_row(slot_list, slot_count, slot_list[i], selected_slot);
    }
    display_menu_draw_line_tune_info(selected_slot);
}

/* 切换高亮项时只重刷前后两行，尽量减少屏幕刷新量。 */
static void display_menu_refresh_line_tune_selection(const line_tune_slot_t *slot_list,
                                                     uint8 slot_count,
                                                     line_tune_slot_t previous_slot,
                                                     line_tune_slot_t selected_slot)
{
    display_menu_draw_line_tune_row(slot_list, slot_count, previous_slot, selected_slot);
    display_menu_draw_line_tune_row(slot_list, slot_count, selected_slot, selected_slot);
}

/* 进入或退出编辑态时，只需要刷新当前行和顶部参数提示。 */
static void display_menu_refresh_line_tune_mode(const line_tune_slot_t *slot_list,
                                                uint8 slot_count,
                                                line_tune_slot_t selected_slot)
{
    display_menu_draw_line_tune_row(slot_list, slot_count, selected_slot, selected_slot);
    display_menu_draw_line_tune_info(selected_slot);
}

/* 某些参数会顺带改动它的配对项，所以这里直接整页刷新，避免屏幕显示旧值。 */
static void display_menu_refresh_line_tune_value(const line_tune_slot_t *slot_list,
                                                 uint8 slot_count,
                                                 line_tune_slot_t selected_slot)
{
    uint8 i = 0;

    for(i = 0; i < slot_count; i++)
    {
        display_menu_draw_line_tune_row(slot_list, slot_count, slot_list[i], selected_slot);
    }
    display_menu_draw_line_tune_info(selected_slot);
}

/* 非编辑态时，上键切换上一项；到第一项后回绕到最后一项。 */
static void display_menu_line_tune_select_up(const line_tune_slot_t *slot_list,
                                             uint8 slot_count,
                                             line_tune_slot_t *selected_slot)
{
    uint8 current_index = 0;
    line_tune_slot_t previous_slot = LINE_TUNE_SLOT_KP;

    if(0 == selected_slot || 0 == slot_list || 0 == slot_count)
    {
        return;
    }

    previous_slot = *selected_slot;
    current_index = display_menu_find_line_tune_slot_index(slot_list, slot_count, *selected_slot);
    if(0 == current_index)
    {
        current_index = slot_count - 1;
    }
    else
    {
        current_index--;
    }

    *selected_slot = slot_list[current_index];
    display_menu_refresh_line_tune_selection(slot_list, slot_count, previous_slot, *selected_slot);
}

/* 非编辑态时，下键切换下一项；到最后一项后回绕到第一项。 */
static void display_menu_line_tune_select_down(const line_tune_slot_t *slot_list,
                                               uint8 slot_count,
                                               line_tune_slot_t *selected_slot)
{
    uint8 current_index = 0;
    line_tune_slot_t previous_slot = LINE_TUNE_SLOT_KP;

    if(0 == selected_slot || 0 == slot_list || 0 == slot_count)
    {
        return;
    }

    previous_slot = *selected_slot;
    current_index = display_menu_find_line_tune_slot_index(slot_list, slot_count, *selected_slot);
    current_index++;
    if(current_index >= slot_count)
    {
        current_index = 0;
    }

    *selected_slot = slot_list[current_index];
    display_menu_refresh_line_tune_selection(slot_list, slot_count, previous_slot, *selected_slot);
}

/* 编辑态时真正改值，底层会负责运行时生效和 flash 保存。 */
static void display_menu_line_tune_adjust(line_tune_slot_t slot, int16 delta)
{
    uint16 current_value = 0;
    uint16 min_value = 0;
    uint16 max_value = 0;
    uint16 step_value = 0;
    int32 next_value = 0;

    current_value = line_app_get_tune_value(slot);
    display_menu_get_line_tune_effective_range(slot, &min_value, &max_value, &step_value);
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
        line_app_set_tune_value(slot, (uint16)next_value);
    }
}

/* 长按加速时按 step 的倍数改，保持和 Camera 页一致的手感。 */
static void display_menu_line_tune_adjust_by_step_mul(line_tune_slot_t slot, int8 direction, uint8 step_mul)
{
    uint16 step_value = 0;
    int16 delta = 0;

    line_app_get_tune_range(slot, 0, 0, &step_value);
    delta = display_menu_build_step_delta(step_value, direction, step_mul);
    if(0 == delta)
    {
        return;
    }

    display_menu_line_tune_adjust(slot, delta);
}

static void display_menu_draw_line_page(const display_line_page_t *line_page)
{
    if(0 == line_page)
    {
        return;
    }

    display_menu_draw_line_tune_page_full(line_page->title,
                                          line_page->slot_list,
                                          line_page->slot_count,
                                          *line_page->selected_slot);
}

static void display_menu_refresh_line_page_mode(const display_line_page_t *line_page)
{
    if(0 == line_page)
    {
        return;
    }

    display_menu_refresh_line_tune_mode(line_page->slot_list,
                                        line_page->slot_count,
                                        *line_page->selected_slot);
}

static void display_menu_refresh_line_page_value(const display_line_page_t *line_page)
{
    if(0 == line_page)
    {
        return;
    }

    display_menu_refresh_line_tune_value(line_page->slot_list,
                                         line_page->slot_count,
                                         *line_page->selected_slot);
}

static uint8 display_menu_handle_line_page_move(int8 direction)
{
    const display_line_page_t *line_page = 0;

    line_page = display_menu_get_line_page(g_menu_page);
    if(0 == line_page)
    {
        return 0;
    }

    if(g_param_editing)
    {
        display_menu_line_tune_adjust_by_step_mul(*line_page->selected_slot, direction, 1);
        display_menu_refresh_line_page_value(line_page);
    }
    else if(direction > 0)
    {
        display_menu_line_tune_select_up(line_page->slot_list,
                                         line_page->slot_count,
                                         line_page->selected_slot);
    }
    else
    {
        display_menu_line_tune_select_down(line_page->slot_list,
                                           line_page->slot_count,
                                           line_page->selected_slot);
    }

    return 1;
}

static uint8 display_menu_handle_line_page_fast(int8 direction)
{
    const display_line_page_t *line_page = 0;

    if(!g_param_editing)
    {
        return 0;
    }

    line_page = display_menu_get_line_page(g_menu_page);
    if(0 == line_page)
    {
        return 0;
    }

    display_menu_line_tune_adjust_by_step_mul(*line_page->selected_slot, direction, MENU_PARAM_FAST_STEP_MUL);
    display_menu_refresh_line_page_value(line_page);
    return 1;
}

static uint8 display_menu_handle_line_page_enter(void)
{
    const display_line_page_t *line_page = 0;

    line_page = display_menu_get_line_page(g_menu_page);
    if(0 == line_page)
    {
        return 0;
    }

    g_param_editing = (uint8)!g_param_editing;
    display_menu_refresh_line_page_mode(line_page);
    return 1;
}

static uint8 display_menu_handle_line_page_back(void)
{
    const display_line_page_t *line_page = 0;

    line_page = display_menu_get_line_page(g_menu_page);
    if(0 == line_page)
    {
        return 0;
    }

    if(g_param_editing)
    {
        g_param_editing = 0;
        display_menu_refresh_line_page_mode(line_page);
        return 1;
    }

    line_app_save_tune_page();
    g_menu_page = DISPLAY_PAGE_PARAM_LINE_MENU;
    display_menu_mark_dirty();
    display_menu_render();
    return 1;
}

/* Line Tune 两个子页共用这一条离页保存逻辑，避免调参过程中频繁写 flash。 */
static void display_menu_commit_line_tune_before_leave(void)
{
    if(0 != display_menu_get_line_page(g_menu_page))
    {
        line_app_save_tune_page();
    }
}

/* Line Tune 二级子菜单本身只负责列出分类，不直接进入编辑。 */
static void display_menu_draw_line_menu(void)
{
    uint8 i = 0;

    ips200_clear(RGB565_WHITE);
    display_menu_draw_title("Line Tune");
    display_menu_draw_battery(1);

    for(i = 0; i < MENU_LINE_MENU_ITEM_COUNT; i++)
    {
        display_menu_draw_line_menu_item(i);
    }
}

static void display_menu_draw_root(void)
{
    uint8 i = 0;

    ips200_clear(RGB565_WHITE);
    ips200_set_color(RGB565_RED, RGB565_WHITE);
    ips200_show_string(0, MENU_TITLE_Y, "Home");
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

static void display_menu_move_root_selection(int8 direction)
{
    uint8 previous_selected = 0;

    previous_selected = g_menu_selected;
    if(direction > 0)
    {
        if(0 == g_menu_selected)
        {
            g_menu_selected = MENU_ROOT_ITEM_COUNT - 1;
        }
        else
        {
            g_menu_selected--;
        }
    }
    else
    {
        g_menu_selected++;
        if(g_menu_selected >= MENU_ROOT_ITEM_COUNT)
        {
            g_menu_selected = 0;
        }
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

static void display_menu_open_selected_line_page(void)
{
    const display_line_page_t *line_page = 0;

    if(g_line_menu_selected >= MENU_ARRAY_COUNT(g_line_pages))
    {
        return;
    }

    line_page = &g_line_pages[g_line_menu_selected];
    g_param_editing = 0;
    *line_page->selected_slot = line_page->slot_list[0];
    g_menu_page = line_page->page;
    display_menu_mark_dirty();
    display_menu_render();
}

static void display_menu_prepare_camera_view(void)
{
    ips200_clear(RGB565_BLACK);
}

static void display_menu_enter_root_page(void)
{
    display_menu_commit_line_tune_before_leave();
    g_menu_page = DISPLAY_PAGE_ROOT;
    g_menu_selected = 0;
    g_param_menu_selected = 0;
    g_line_menu_selected = 0;
    g_line_pid_selected = LINE_TUNE_SLOT_KP;
    g_line_servo_selected = LINE_TUNE_SLOT_SERVO_MIN;
    g_start_selected = START_SLOT_SPEED;
    g_param_editing = 0;
    display_menu_mark_dirty();
    car_servo_set_center();
    display_menu_render();
}

void display_menu_init(void)
{
    g_menu_selected = 0;
    g_param_menu_selected = 0;
    g_menu_page = DISPLAY_PAGE_ROOT;
    g_menu_dirty = 1;
    g_menu_last_battery_percent = 0xFF;
    g_camera_param_selected = FLASH_CAMERA_SLOT_EXP_TIME;
    g_line_menu_selected = 0;
    g_line_pid_selected = LINE_TUNE_SLOT_KP;
    g_line_servo_selected = LINE_TUNE_SLOT_SERVO_MIN;
    g_start_selected = START_SLOT_SPEED;
    display_menu_load_start_page_from_flash();
    g_param_editing = 0;
}

void display_menu_render(void)
{
    const display_line_page_t *line_page = 0;

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

    line_page = display_menu_get_line_page(g_menu_page);
    if(0 != line_page)
    {
        display_menu_draw_line_page(line_page);
        g_menu_dirty = 0;
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
        case DISPLAY_PAGE_START:
            display_menu_draw_start_page_full();
            break;
        case DISPLAY_PAGE_PARAM_LINE_MENU:
            display_menu_draw_line_menu();
            break;
        default:
            display_menu_draw_root();
            break;
    }

    g_menu_dirty = 0;
}

void display_menu_move_up(void)
{
    if(!g_ips_enable) return;
    if(DISPLAY_PAGE_START == g_menu_page)
    {
        if(g_param_editing && (START_SLOT_SPEED == g_start_selected))
        {
            display_menu_start_adjust_speed_by_step_mul(1, 1);
            display_menu_refresh_start_value();
        }
        else
        {
            display_menu_start_select_up();
        }
        return;
    }

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

    if(display_menu_handle_line_page_move(1))
    {
        return;
    }

    if(DISPLAY_PAGE_PARAM_MENU == g_menu_page)
    {
        display_menu_cycle_list_selection(&g_param_menu_selected,
                                          MENU_PARAM_MENU_ITEM_COUNT,
                                          1,
                                          display_menu_draw_param_menu_item);
        return;
    }

    if(DISPLAY_PAGE_PARAM_LINE_MENU == g_menu_page)
    {
        display_menu_cycle_list_selection(&g_line_menu_selected,
                                          MENU_LINE_MENU_ITEM_COUNT,
                                          1,
                                          display_menu_draw_line_menu_item);
        return;
    }

    if(DISPLAY_PAGE_ROOT != g_menu_page)
    {
        return;
    }

    display_menu_move_root_selection(1);
}

void display_menu_move_down(void)
{
    if(!g_ips_enable) return;
    if(DISPLAY_PAGE_START == g_menu_page)
    {
        if(g_param_editing && (START_SLOT_SPEED == g_start_selected))
        {
            display_menu_start_adjust_speed_by_step_mul(-1, 1);
            display_menu_refresh_start_value();
        }
        else
        {
            display_menu_start_select_down();
        }
        return;
    }

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

    if(display_menu_handle_line_page_move(-1))
    {
        return;
    }

    if(DISPLAY_PAGE_PARAM_MENU == g_menu_page)
    {
        display_menu_cycle_list_selection(&g_param_menu_selected,
                                          MENU_PARAM_MENU_ITEM_COUNT,
                                          -1,
                                          display_menu_draw_param_menu_item);
        return;
    }

    if(DISPLAY_PAGE_PARAM_LINE_MENU == g_menu_page)
    {
        display_menu_cycle_list_selection(&g_line_menu_selected,
                                          MENU_LINE_MENU_ITEM_COUNT,
                                          -1,
                                          display_menu_draw_line_menu_item);
        return;
    }

    if(DISPLAY_PAGE_ROOT != g_menu_page)
    {
        return;
    }

    display_menu_move_root_selection(-1);
}

void display_menu_move_up_fast(void)
{
    if(!g_ips_enable) return;
    if(!g_param_editing)
    {
        return;
    }

    if(DISPLAY_PAGE_START == g_menu_page)
    {
        if(START_SLOT_SPEED == g_start_selected)
        {
            display_menu_start_adjust_speed_by_step_mul(1, MENU_PARAM_FAST_STEP_MUL);
            display_menu_refresh_start_value();
        }
    }
    else if(DISPLAY_PAGE_PARAM_CAMERA == g_menu_page)
    {
        display_menu_camera_param_adjust_by_step_mul(1, MENU_PARAM_FAST_STEP_MUL);
        display_menu_refresh_camera_current();
    }
    else
    {
        display_menu_handle_line_page_fast(1);
    }
}

void display_menu_move_down_fast(void)
{
    if(!g_ips_enable) return;
    if(!g_param_editing)
    {
        return;
    }

    if(DISPLAY_PAGE_START == g_menu_page)
    {
        if(START_SLOT_SPEED == g_start_selected)
        {
            display_menu_start_adjust_speed_by_step_mul(-1, MENU_PARAM_FAST_STEP_MUL);
            display_menu_refresh_start_value();
        }
    }
    else if(DISPLAY_PAGE_PARAM_CAMERA == g_menu_page)
    {
        display_menu_camera_param_adjust_by_step_mul(-1, MENU_PARAM_FAST_STEP_MUL);
        display_menu_refresh_camera_current();
    }
    else
    {
        display_menu_handle_line_page_fast(-1);
    }
}

void display_menu_enter(void)
{
    if(!g_ips_enable) return;
    if(DISPLAY_PAGE_START == g_menu_page)
    {
        if(START_SLOT_SPEED == g_start_selected)
        {
            g_param_editing = (uint8)!g_param_editing;
            display_menu_refresh_start_mode();
        }
        else
        {
            display_menu_start_toggle_enable();
            display_menu_refresh_start_value();
        }
        return;
    }

    if(DISPLAY_PAGE_PARAM_CAMERA == g_menu_page)
    {
        g_param_editing = (uint8)!g_param_editing;
        display_menu_refresh_camera_current();
        return;
    }

    if(display_menu_handle_line_page_enter())
    {
        return;
    }

    if(DISPLAY_PAGE_PARAM_MENU == g_menu_page)
    {
        g_param_editing = 0;
        if(0 == g_param_menu_selected)
        {
            g_camera_param_selected = FLASH_CAMERA_SLOT_EXP_TIME;
            g_menu_page = DISPLAY_PAGE_PARAM_CAMERA;
        }
        else
        {
            g_line_menu_selected = 0;
            g_menu_page = DISPLAY_PAGE_PARAM_LINE_MENU;
        }
        display_menu_mark_dirty();
        display_menu_render();
        return;
    }

    if(DISPLAY_PAGE_PARAM_LINE_MENU == g_menu_page)
    {
        display_menu_open_selected_line_page();
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
    else if(1 == g_menu_selected)
    {
        g_param_menu_selected = 0;
        g_menu_page = DISPLAY_PAGE_PARAM_MENU;
    }
    else
    {
        g_start_selected = START_SLOT_SPEED;
        g_menu_page = DISPLAY_PAGE_START;
    }

    g_param_editing = 0;
    display_menu_mark_dirty();
    display_menu_render();
}

void display_menu_back(void)
{
    if(!g_ips_enable) return;
    if(DISPLAY_PAGE_ROOT == g_menu_page)
    {
        return;
    }

    if(DISPLAY_PAGE_START == g_menu_page)
    {
        if(g_param_editing)
        {
            g_param_editing = 0;
            display_menu_refresh_start_mode();
            return;
        }

        display_menu_enter_root_page();
        return;
    }

    if(DISPLAY_PAGE_PARAM_CAMERA == g_menu_page)
    {
        if(g_param_editing)
        {
            g_param_editing = 0;
            display_menu_refresh_camera_current();
            return;
        }

        g_menu_page = DISPLAY_PAGE_PARAM_MENU;
        display_menu_mark_dirty();
        display_menu_render();
        return;
    }

    if(display_menu_handle_line_page_back())
    {
        return;
    }

    if(DISPLAY_PAGE_PARAM_MENU == g_menu_page)
    {
        display_menu_enter_root_page();
        return;
    }

    if(DISPLAY_PAGE_PARAM_LINE_MENU == g_menu_page)
    {
        g_menu_page = DISPLAY_PAGE_PARAM_MENU;
        display_menu_mark_dirty();
        display_menu_render();
        return;
    }

    display_menu_enter_root_page();
}

void display_menu_go_root(void)
{
    if(!g_ips_enable) return;
    if(DISPLAY_PAGE_ROOT == g_menu_page)
    {
        if(0 == g_menu_selected)
        {
            return;
        }
    }

    display_menu_enter_root_page();
}

uint8 display_menu_in_camera_view(void)
{
    return (DISPLAY_PAGE_CAMERA == g_menu_page) ? 1 : 0;
}
