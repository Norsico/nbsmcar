/*
 * app_ui_display.c - UI 页面状态与显示逻辑
 */

#include "app_ui_display.h"

#include "search_line.h"
#include "app_line.h"
#include "app_ui_flash.h"
#include "app_ui_library.h"
#include "dev_adc.h"
#include "dev_servo.h"

#define MENU_LINE_HEIGHT            (16)
#define MENU_ROOT_ITEM_COUNT        (3)
#define MENU_PARAM_MENU_ITEM_COUNT  (2)
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
#define BATTERY_EMPTY_DECI          (114)
#define BATTERY_FULL_DECI           (126)

typedef enum
{
    DISPLAY_PAGE_ROOT = 0,
    DISPLAY_PAGE_CAMERA,
    DISPLAY_PAGE_START,
    DISPLAY_PAGE_PARAM_MENU,
    DISPLAY_PAGE_PARAM_CAMERA,
    DISPLAY_PAGE_PARAM_STEER_PD
} display_page_t;

typedef enum
{
    START_SLOT_SPEED = 0,
} start_slot_t;

typedef enum
{
    STEER_PAGE_SLOT_P = 0,
    STEER_PAGE_SLOT_D,
    STEER_PAGE_SLOT_ERR2,
    STEER_PAGE_SLOT_IMU_D,
    STEER_PAGE_SLOT_SERVO_MIN,
    STEER_PAGE_SLOT_SERVO_MAX
} steer_page_slot_t;

typedef void (*display_menu_draw_item_t)(uint8 index);

static const char *g_root_menu_titles[MENU_ROOT_ITEM_COUNT] =
{
    "Camera View",
    "Param Config",
    "Car Speed"
};

static const char *g_param_menu_titles[MENU_PARAM_MENU_ITEM_COUNT] =
{
    "Camera",
    "Steer PD"
};

static uint8 g_menu_selected = 0;
static uint8 g_param_menu_selected = 0;
static display_page_t g_menu_page = DISPLAY_PAGE_ROOT;
static uint8 g_menu_dirty = 1;
static uint8 g_menu_last_battery_percent = 0xFF;
static flash_camera_slot_t g_camera_param_selected = FLASH_CAMERA_SLOT_EXP_TIME;
static steer_page_slot_t g_steer_pd_selected = STEER_PAGE_SLOT_P;
static start_slot_t g_start_selected = START_SLOT_SPEED;
static uint8 g_param_editing = 0;

static void display_menu_mark_dirty(void)
{
    g_menu_dirty = 1;
}

static uint8 display_menu_get_battery_percent(void)
{
    uint16 battery = 0;
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
        percent = (uint8)(((battery - BATTERY_EMPTY_DECI) * 100U) /
                          (BATTERY_FULL_DECI - BATTERY_EMPTY_DECI));
    }

    return percent;
}

static void display_menu_draw_battery(uint8 force)
{
    uint8 percent = 0;

    /* 非强制刷新时只更新电量，避免整页反复重绘。 */
    percent = display_menu_get_battery_percent();
    if(!force && (percent == g_menu_last_battery_percent))
    {
        return;
    }

    ui_library_draw_battery(percent);
    g_menu_last_battery_percent = percent;
}

static uint16 display_menu_get_row_y(uint8 index)
{
    return (uint16)(MENU_PARAM_LIST_Y + (uint16)index * MENU_PARAM_ROW_STEP);
}

static void display_menu_draw_root_item(uint8 index)
{
    if(index >= MENU_ROOT_ITEM_COUNT)
    {
        return;
    }

    ui_library_draw_list_item((uint16)(MENU_ROOT_LIST_Y + (uint16)index * MENU_LINE_HEIGHT),
                              g_root_menu_titles[index],
                              (index == g_menu_selected) ? 1 : 0);
}

static void display_menu_draw_param_menu_item(uint8 index)
{
    if(index >= MENU_PARAM_MENU_ITEM_COUNT)
    {
        return;
    }

    ui_library_draw_list_item((uint16)(MENU_SUBMENU_LIST_Y + (uint16)index * MENU_LINE_HEIGHT),
                              g_param_menu_titles[index],
                              (index == g_param_menu_selected) ? 1 : 0);
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

    /* 菜单项切换只局部重画前后两项，减小 UI 刷新量。 */
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
    value = ui_flash_get_camera_value(slot);
    ui_library_format_uint16(value, value_text);

    ui_library_draw_rect(MENU_PARAM_ACCENT_X,
                         (uint16)(row_y + MENU_PARAM_ACCENT_Y_OFFSET),
                         MENU_PARAM_ACCENT_W,
                         MENU_PARAM_ACCENT_H,
                         RGB565_WHITE);

    /* 当前项蓝色表示选中，红色表示进入编辑态。 */
    if(slot == g_camera_param_selected)
    {
        guide_color = g_param_editing ? RGB565_RED : RGB565_BLUE;
        label_color = guide_color;
        value_color = guide_color;
        ui_library_draw_rect(MENU_PARAM_ACCENT_X,
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
    /* 只有进入编辑态时，才显示当前参数的范围和步进。 */
    if(!g_param_editing)
    {
        return;
    }

    ui_flash_get_camera_range(g_camera_param_selected, &min_value, &max_value, &step_value);
    ui_library_format_uint16(min_value, min_text);
    ui_library_format_uint16(max_value, max_text);
    ui_library_format_uint16(step_value, step_text);

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

    ui_library_draw_title("Camera");
    display_menu_draw_battery(1);
    ui_library_draw_dash_line(0, MENU_PARAM_INFO_LINE_Y, MENU_PARAM_DIVIDER_W, 10, 6, RGB565_GRAY);
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
    display_menu_camera_param_select_up();
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

static void display_menu_camera_param_adjust(int16 delta)
{
    /* 参数页只关心本页刷新，真正的落盘和相机生效交给 ui_flash。 */
    if(ui_flash_adjust_camera_value(g_camera_param_selected, delta))
    {
        display_menu_refresh_camera_current();
    }
}

static void display_menu_camera_param_adjust_by_step_mul(int8 direction, uint8 step_mul)
{
    uint16 step_value = 0;
    int16 delta = 0;

    ui_flash_get_camera_range(g_camera_param_selected, 0, 0, &step_value);
    delta = display_menu_build_step_delta(step_value, direction, step_mul);
    if(0 == delta)
    {
        return;
    }

    display_menu_camera_param_adjust(delta);
}

static const char *display_menu_get_steer_pd_label(steer_page_slot_t slot)
{
    switch(slot)
    {
        case STEER_PAGE_SLOT_P:
            return "steer p";
        case STEER_PAGE_SLOT_D:
            return "steer d";
        case STEER_PAGE_SLOT_ERR2:
            return "err2 k";
        case STEER_PAGE_SLOT_IMU_D:
            return "imu d";
        case STEER_PAGE_SLOT_SERVO_MIN:
            return "servo min";
        case STEER_PAGE_SLOT_SERVO_MAX:
            return "servo max";
        default:
            return "";
    }
}

static uint8 display_menu_get_steer_pd_row_index(steer_page_slot_t slot)
{
    switch(slot)
    {
        case STEER_PAGE_SLOT_P:
            return 0;
        case STEER_PAGE_SLOT_D:
            return 1;
        case STEER_PAGE_SLOT_ERR2:
            return 2;
        case STEER_PAGE_SLOT_IMU_D:
            return 3;
        case STEER_PAGE_SLOT_SERVO_MIN:
            return 4;
        case STEER_PAGE_SLOT_SERVO_MAX:
            return 5;
        default:
            return 0;
    }
}

static void display_menu_draw_steer_pd_row(steer_page_slot_t slot)
{
    char value_text[8];
    const char *label_text = 0;
    uint16 row_y = 0;
    uint16 guide_color = RGB565_GRAY;
    uint16 label_color = RGB565_BLACK;
    uint16 value_color = RGB565_BLACK;
    uint16 value = 0;

    row_y = display_menu_get_row_y(display_menu_get_steer_pd_row_index(slot));
    label_text = display_menu_get_steer_pd_label(slot);
    switch(slot)
    {
        case STEER_PAGE_SLOT_P:
            value = ui_flash_get_steer_pd_value(FLASH_PARAM_SLOT_FIRST);
            ui_library_format_uint16(value, value_text);
            break;
        case STEER_PAGE_SLOT_D:
            value = ui_flash_get_steer_pd_value(FLASH_PARAM_SLOT_SECOND);
            ui_library_format_uint16(value, value_text);
            break;
        case STEER_PAGE_SLOT_ERR2:
            value = ui_flash_get_steer_pd_value(FLASH_PARAM_SLOT_THIRD);
            ui_library_format_tenths(value, value_text);
            break;
        case STEER_PAGE_SLOT_IMU_D:
            value = ui_flash_get_steer_pd_value(FLASH_PARAM_SLOT_FOURTH);
            ui_library_format_tenths(value, value_text);
            break;
        case STEER_PAGE_SLOT_SERVO_MIN:
            value = ui_flash_get_servo_limit_min_value();
            ui_library_format_uint16(value, value_text);
            break;
        case STEER_PAGE_SLOT_SERVO_MAX:
            value = ui_flash_get_servo_limit_max_value();
            ui_library_format_uint16(value, value_text);
            break;
        default:
            value_text[0] = '\0';
            break;
    }

    ui_library_draw_rect(MENU_PARAM_ACCENT_X,
                         (uint16)(row_y + MENU_PARAM_ACCENT_Y_OFFSET),
                         MENU_PARAM_ACCENT_W,
                         MENU_PARAM_ACCENT_H,
                         RGB565_WHITE);

    if(slot == g_steer_pd_selected)
    {
        guide_color = g_param_editing ? RGB565_RED : RGB565_BLUE;
        label_color = guide_color;
        value_color = guide_color;
        ui_library_draw_rect(MENU_PARAM_ACCENT_X,
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

static void display_menu_draw_steer_pd_info(void)
{
    char min_text[8];
    char max_text[8];
    char step_text[8];
    uint16 min_value = 0;
    uint16 max_value = 0;
    uint16 step_value = 0;

    ips200_set_color(RGB565_BLACK, RGB565_WHITE);
    ips200_show_string(0, MENU_PARAM_INFO_Y, MENU_PARAM_INFO_CLEAR);
    if(!g_param_editing)
    {
        return;
    }

    switch(g_steer_pd_selected)
    {
        case STEER_PAGE_SLOT_P:
            ui_flash_get_steer_pd_range(FLASH_PARAM_SLOT_FIRST, &min_value, &max_value, &step_value);
            ui_library_format_uint16(min_value, min_text);
            ui_library_format_uint16(max_value, max_text);
            ui_library_format_uint16(step_value, step_text);
            break;
        case STEER_PAGE_SLOT_D:
            ui_flash_get_steer_pd_range(FLASH_PARAM_SLOT_SECOND, &min_value, &max_value, &step_value);
            ui_library_format_uint16(min_value, min_text);
            ui_library_format_uint16(max_value, max_text);
            ui_library_format_uint16(step_value, step_text);
            break;
        case STEER_PAGE_SLOT_ERR2:
            ui_flash_get_steer_pd_range(FLASH_PARAM_SLOT_THIRD, &min_value, &max_value, &step_value);
            ui_library_format_tenths(min_value, min_text);
            ui_library_format_tenths(max_value, max_text);
            ui_library_format_tenths(step_value, step_text);
            break;
        case STEER_PAGE_SLOT_IMU_D:
            ui_flash_get_steer_pd_range(FLASH_PARAM_SLOT_FOURTH, &min_value, &max_value, &step_value);
            ui_library_format_tenths(min_value, min_text);
            ui_library_format_tenths(max_value, max_text);
            ui_library_format_tenths(step_value, step_text);
            break;
        case STEER_PAGE_SLOT_SERVO_MIN:
        case STEER_PAGE_SLOT_SERVO_MAX:
            ui_flash_get_servo_limit_range(&min_value, &max_value, &step_value);
            ui_library_format_uint16(min_value, min_text);
            ui_library_format_uint16(max_value, max_text);
            ui_library_format_uint16(step_value, step_text);
            break;
        default:
            return;
    }

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

static void display_menu_draw_steer_pd_page_full(void)
{
    ips200_clear(RGB565_WHITE);

    ui_library_draw_title("Steer PD");
    display_menu_draw_battery(1);
    ui_library_draw_dash_line(0, MENU_PARAM_INFO_LINE_Y, MENU_PARAM_DIVIDER_W, 10, 6, RGB565_GRAY);
    display_menu_draw_steer_pd_row(STEER_PAGE_SLOT_P);
    display_menu_draw_steer_pd_row(STEER_PAGE_SLOT_D);
    display_menu_draw_steer_pd_row(STEER_PAGE_SLOT_ERR2);
    display_menu_draw_steer_pd_row(STEER_PAGE_SLOT_IMU_D);
    display_menu_draw_steer_pd_row(STEER_PAGE_SLOT_SERVO_MIN);
    display_menu_draw_steer_pd_row(STEER_PAGE_SLOT_SERVO_MAX);
    display_menu_draw_steer_pd_info();
}

static void display_menu_refresh_steer_pd_selection(steer_page_slot_t previous_slot)
{
    display_menu_draw_steer_pd_row(previous_slot);
    display_menu_draw_steer_pd_row(g_steer_pd_selected);
    display_menu_draw_steer_pd_info();
}

static void display_menu_refresh_steer_pd_current(void)
{
    display_menu_draw_steer_pd_row(g_steer_pd_selected);
    display_menu_draw_steer_pd_info();
}

static void display_menu_steer_pd_select_up(void)
{
    steer_page_slot_t previous_slot = g_steer_pd_selected;

    if(STEER_PAGE_SLOT_P == g_steer_pd_selected)
    {
        g_steer_pd_selected = STEER_PAGE_SLOT_SERVO_MAX;
    }
    else
    {
        g_steer_pd_selected = (steer_page_slot_t)(g_steer_pd_selected - 1);
    }

    display_menu_refresh_steer_pd_selection(previous_slot);
}

static void display_menu_steer_pd_select_down(void)
{
    steer_page_slot_t previous_slot = g_steer_pd_selected;

    if(STEER_PAGE_SLOT_SERVO_MAX == g_steer_pd_selected)
    {
        g_steer_pd_selected = STEER_PAGE_SLOT_P;
    }
    else
    {
        g_steer_pd_selected = (steer_page_slot_t)(g_steer_pd_selected + 1);
    }

    display_menu_refresh_steer_pd_selection(previous_slot);
}

static void display_menu_steer_pd_adjust(int16 delta)
{
    uint8 changed = 0;

    switch(g_steer_pd_selected)
    {
        case STEER_PAGE_SLOT_P:
            changed = ui_flash_adjust_steer_pd_value(FLASH_PARAM_SLOT_FIRST, delta);
            break;
        case STEER_PAGE_SLOT_D:
            changed = ui_flash_adjust_steer_pd_value(FLASH_PARAM_SLOT_SECOND, delta);
            break;
        case STEER_PAGE_SLOT_ERR2:
            changed = ui_flash_adjust_steer_pd_value(FLASH_PARAM_SLOT_THIRD, delta);
            break;
        case STEER_PAGE_SLOT_IMU_D:
            changed = ui_flash_adjust_steer_pd_value(FLASH_PARAM_SLOT_FOURTH, delta);
            break;
        case STEER_PAGE_SLOT_SERVO_MIN:
            changed = ui_flash_adjust_servo_limit_min_value(delta);
            break;
        case STEER_PAGE_SLOT_SERVO_MAX:
            changed = ui_flash_adjust_servo_limit_max_value(delta);
            break;
        default:
            break;
    }

    if(changed)
    {
        display_menu_refresh_steer_pd_current();
    }
}

static void display_menu_steer_pd_adjust_by_step_mul(int8 direction, uint8 step_mul)
{
    uint16 step_value = 0;
    int16 delta = 0;

    switch(g_steer_pd_selected)
    {
        case STEER_PAGE_SLOT_P:
            ui_flash_get_steer_pd_range(FLASH_PARAM_SLOT_FIRST, 0, 0, &step_value);
            break;
        case STEER_PAGE_SLOT_D:
            ui_flash_get_steer_pd_range(FLASH_PARAM_SLOT_SECOND, 0, 0, &step_value);
            break;
        case STEER_PAGE_SLOT_ERR2:
            ui_flash_get_steer_pd_range(FLASH_PARAM_SLOT_THIRD, 0, 0, &step_value);
            break;
        case STEER_PAGE_SLOT_IMU_D:
            ui_flash_get_steer_pd_range(FLASH_PARAM_SLOT_FOURTH, 0, 0, &step_value);
            break;
        case STEER_PAGE_SLOT_SERVO_MIN:
        case STEER_PAGE_SLOT_SERVO_MAX:
            ui_flash_get_servo_limit_range(0, 0, &step_value);
            break;
        default:
            return;
    }
    delta = display_menu_build_step_delta(step_value, direction, step_mul);
    if(0 == delta)
    {
        return;
    }

    display_menu_steer_pd_adjust(delta);
}

static const char *display_menu_get_start_label(start_slot_t slot)
{
    switch(slot)
    {
        case START_SLOT_SPEED:
        default:
            return "speed";
    }
}

static void display_menu_format_start_value(start_slot_t slot, char *text)
{
    flash_start_page_t page;

    ui_flash_get_start_page(&page);
    switch(slot)
    {
        case START_SLOT_SPEED:
        default:
            ui_library_format_uint16(page.target_speed, text);
            break;
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

    ui_library_draw_rect(MENU_PARAM_ACCENT_X,
                         (uint16)(row_y + MENU_PARAM_ACCENT_Y_OFFSET),
                         MENU_PARAM_ACCENT_W,
                         MENU_PARAM_ACCENT_H,
                         RGB565_WHITE);

    if(slot == g_start_selected)
    {
        guide_color = (g_param_editing && (START_SLOT_SPEED == slot)) ? RGB565_RED : RGB565_BLUE;
        label_color = guide_color;
        value_color = guide_color;
        ui_library_draw_rect(MENU_PARAM_ACCENT_X,
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
    uint16 min_value = 0;
    uint16 max_value = 0;
    uint16 step_value = 0;

    ips200_set_color(RGB565_BLACK, RGB565_WHITE);
    ips200_show_string(0, MENU_PARAM_INFO_Y, MENU_PARAM_INFO_CLEAR);

    /* Car Speed 页只显示速度范围提示。 */
    if(START_SLOT_SPEED == g_start_selected)
    {
        if(!g_param_editing)
        {
            return;
        }

        ui_flash_get_start_speed_range(&min_value, &max_value, &step_value);
        ui_library_format_uint16(min_value, min_text);
        ui_library_format_uint16(max_value, max_text);
        ui_library_format_uint16(step_value, step_text);

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
}

static void display_menu_draw_start_page_full(void)
{
    ips200_clear(RGB565_WHITE);

    ui_library_draw_title("Car Speed");
    display_menu_draw_battery(1);
    ui_library_draw_dash_line(0, MENU_PARAM_INFO_LINE_Y, MENU_PARAM_DIVIDER_W, 10, 6, RGB565_GRAY);
    display_menu_draw_start_row(START_SLOT_SPEED);
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
    display_menu_draw_start_info();
}

static void display_menu_start_select_up(void)
{
    display_menu_refresh_start_selection(g_start_selected);
}

static void display_menu_start_select_down(void)
{
    display_menu_start_select_up();
}

static void display_menu_start_adjust_speed_by_step_mul(int8 direction, uint8 step_mul)
{
    uint16 step_value = 0;
    int16 delta = 0;

    ui_flash_get_start_speed_range(0, 0, &step_value);
    delta = display_menu_build_step_delta(step_value, direction, step_mul);
    if(0 == delta)
    {
        return;
    }

    ui_flash_adjust_start_speed(delta);
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
    ui_library_draw_title("Param Config");
    display_menu_draw_battery(1);

    for(i = 0; i < MENU_PARAM_MENU_ITEM_COUNT; i++)
    {
        display_menu_draw_param_menu_item(i);
    }
}

static void display_menu_move_root_selection(int8 direction)
{
    uint8 previous_selected = 0;

    /* 首页菜单保持循环切换，便于单手反复调页。 */
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

static void display_menu_prepare_camera_view(void)
{
    ips200_clear(RGB565_BLACK);
    SearchLine_ResetPreviewOverlay();
}

static void display_menu_toggle_camera_preview(void)
{
    /* 相机页当前只保留二值和原始灰度两档，方向键统一按切换模式处理。 */
    if(LINE_APP_PREVIEW_RAW == line_app_get_preview_mode())
    {
        line_app_set_preview_mode(LINE_APP_PREVIEW_BINARY);
    }
    else
    {
        line_app_set_preview_mode(LINE_APP_PREVIEW_RAW);
    }
}

static void display_menu_enter_root_page(void)
{
    /* 退回首页时统一清掉编辑态，并把主菜单光标复位到第一项。 */
    g_menu_page = DISPLAY_PAGE_ROOT;
    g_menu_selected = 0;
    g_param_menu_selected = 0;
    g_camera_param_selected = FLASH_CAMERA_SLOT_EXP_TIME;
    g_steer_pd_selected = STEER_PAGE_SLOT_P;
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
    g_steer_pd_selected = STEER_PAGE_SLOT_P;
    g_start_selected = START_SLOT_SPEED;
    g_param_editing = 0;
    /* UI 初始化时恢复 Start 页缓存。 */
    ui_flash_init();
}

/* 按当前页面刷新 UI。 */
void display_menu_render(void)
{
    if(DISPLAY_PAGE_CAMERA == g_menu_page)
    {
        if(g_menu_dirty)
        {
            /* 相机页只切黑底，图像预览仍按原节拍交给 line_app_render_frame()。 */
            display_menu_prepare_camera_view();
            g_menu_dirty = 0;
        }
        return;
    }

    /* 普通页面在非脏帧时只刷新电量条，减少闪烁。 */
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
        case DISPLAY_PAGE_PARAM_STEER_PD:
            display_menu_draw_steer_pd_page_full();
            break;
        case DISPLAY_PAGE_START:
            display_menu_draw_start_page_full();
            break;
        default:
            display_menu_draw_root();
            break;
    }

    g_menu_dirty = 0;
}

/* 处理上移输入。 */
void display_menu_move_up(void)
{
    if(DISPLAY_PAGE_CAMERA == g_menu_page)
    {
        display_menu_toggle_camera_preview();
        return;
    }

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

    if(DISPLAY_PAGE_PARAM_STEER_PD == g_menu_page)
    {
        if(g_param_editing)
        {
            display_menu_steer_pd_adjust_by_step_mul(1, 1);
        }
        else
        {
            display_menu_steer_pd_select_up();
        }
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

    if(DISPLAY_PAGE_ROOT != g_menu_page)
    {
        return;
    }

    display_menu_move_root_selection(1);
}

/* 处理下移输入。 */
void display_menu_move_down(void)
{
    if(DISPLAY_PAGE_CAMERA == g_menu_page)
    {
        display_menu_toggle_camera_preview();
        return;
    }

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

    if(DISPLAY_PAGE_PARAM_STEER_PD == g_menu_page)
    {
        if(g_param_editing)
        {
            display_menu_steer_pd_adjust_by_step_mul(-1, 1);
        }
        else
        {
            display_menu_steer_pd_select_down();
        }
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

    if(DISPLAY_PAGE_ROOT != g_menu_page)
    {
        return;
    }

    display_menu_move_root_selection(-1);
}

/* 处理长按上移输入。 */
void display_menu_move_up_fast(void)
{
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
    }
    else if(DISPLAY_PAGE_PARAM_STEER_PD == g_menu_page)
    {
        display_menu_steer_pd_adjust_by_step_mul(1, MENU_PARAM_FAST_STEP_MUL);
    }
}

/* 处理长按下移输入。 */
void display_menu_move_down_fast(void)
{
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
    }
    else if(DISPLAY_PAGE_PARAM_STEER_PD == g_menu_page)
    {
        display_menu_steer_pd_adjust_by_step_mul(-1, MENU_PARAM_FAST_STEP_MUL);
    }
}

/* 处理确认键。 */
void display_menu_enter(void)
{
    /* 进入键同时承担翻页和进入编辑两类操作。 */
    if(DISPLAY_PAGE_START == g_menu_page)
    {
        g_param_editing = (uint8)!g_param_editing;
        display_menu_refresh_start_mode();
        return;
    }

    if(DISPLAY_PAGE_PARAM_CAMERA == g_menu_page)
    {
        g_param_editing = (uint8)!g_param_editing;
        display_menu_refresh_camera_current();
        return;
    }

    if(DISPLAY_PAGE_PARAM_STEER_PD == g_menu_page)
    {
        g_param_editing = (uint8)!g_param_editing;
        display_menu_refresh_steer_pd_current();
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
            g_steer_pd_selected = STEER_PAGE_SLOT_P;
            g_menu_page = DISPLAY_PAGE_PARAM_STEER_PD;
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

/* 处理返回键。 */
void display_menu_back(void)
{
    if(DISPLAY_PAGE_ROOT == g_menu_page)
    {
        return;
    }

    /* 返回键优先退出编辑态，其次退回上一级页面。 */
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

    if(DISPLAY_PAGE_PARAM_STEER_PD == g_menu_page)
    {
        if(g_param_editing)
        {
            g_param_editing = 0;
            display_menu_refresh_steer_pd_current();
            return;
        }

        g_menu_page = DISPLAY_PAGE_PARAM_MENU;
        display_menu_mark_dirty();
        display_menu_render();
        return;
    }

    if(DISPLAY_PAGE_PARAM_MENU == g_menu_page)
    {
        display_menu_enter_root_page();
        return;
    }

    display_menu_enter_root_page();
}

/* 强制回到主菜单。 */
void display_menu_go_root(void)
{
    if(DISPLAY_PAGE_ROOT == g_menu_page)
    {
        if(0 == g_menu_selected)
        {
            return;
        }
    }

    display_menu_enter_root_page();
}

/* 当前是否在相机页。 */
uint8 display_menu_in_camera_view(void)
{
    return (DISPLAY_PAGE_CAMERA == g_menu_page) ? 1 : 0;
}
