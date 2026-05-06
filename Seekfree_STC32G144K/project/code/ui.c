#include "flash.h"
#include "image.h"
#include "ui.h"

#define UI_TITLE_Y                   (0)                    /* 标题行 */
#define UI_STATUS_Y                  (16)                   /* 状态行 */
#define UI_LIST_Y                    (32)                   /* 列表起始 */
#define UI_ROW_HEIGHT                (16)                   /* 行高 */
#define UI_LABEL_X                   (16)                   /* 标签X */
#define UI_VALUE_X                   (152)                  /* 数值X */
#define UI_POWER_X                   (200)                  /* 电量X */
#define UI_CAMERA_INFO_Y             (124)                  /* 图像信息行 */

typedef enum
{
    UI_EVENT_NONE = 0,                                      /* 无事件 */
    UI_EVENT_BACK,                                          /* 返回 */
    UI_EVENT_UP,                                            /* 上 */
    UI_EVENT_DOWN,                                          /* 下 */
    UI_EVENT_ENTER                                          /* 确认 */
} ui_event_t;

typedef enum
{
    UI_PAGE_ROOT = 0,                                       /* 主界面 */
    UI_PAGE_CAMERA_VIEW,                                    /* 相机预览 */
    UI_PAGE_PARAM_MENU,                                     /* 参数页 */
    UI_PAGE_CAMERA_PARAM,                                   /* 相机参数 */
    UI_PAGE_SERVO_PARAM,                                    /* 舵机参数 */
    UI_PAGE_MOTOR_PARAM,                                    /* 电机参数 */
    UI_PAGE_PLAN_SELECT                                     /* 方案选择 */
} ui_page_t;

typedef enum
{
    UI_ROOT_CAMERA_VIEW = 0,                                /* 相机预览 */
    UI_ROOT_PARAM_TUNING,                                   /* 参数调节 */
    UI_ROOT_PLAN_SELECT,                                    /* 方案选择 */
    UI_ROOT_COUNT                                           /* 主界面数量 */
} ui_root_row_t;

typedef enum
{
    UI_PARAM_SERVO = 0,                                     /* 舵机 */
    UI_PARAM_CAMERA,                                        /* 相机 */
    UI_PARAM_MOTOR,                                         /* 电机 */
    UI_PARAM_COUNT                                          /* 参数分类数量 */
} ui_param_row_t;

typedef enum
{
    UI_CAMERA_EXP_TIME = 0,                                 /* 曝光 */
    UI_CAMERA_GAIN,                                         /* 增益 */
    UI_CAMERA_THRESHOLD_OFFSET,                             /* 阈值补偿 */
    UI_CAMERA_FIRE_ROW_MIN,                                 /* 打靶上沿 */
    UI_CAMERA_FIRE_ROW_MAX,                                 /* 打靶下沿 */
    UI_CAMERA_FIRE_CENTER_TOL,                              /* 打靶容差 */
    UI_CAMERA_COUNT                                         /* 相机参数数量 */
} ui_camera_row_t;

typedef enum
{
    UI_SERVO_P = 0,                                         /* 舵机p */
    UI_SERVO_D,                                             /* 舵机d */
    UI_SERVO_ERR2,                                          /* 二次误差 */
    UI_SERVO_ACKERMAN,                                      /* 阿克曼 */
    UI_SERVO_IMU_D,                                         /* 陀螺仪d */
    UI_SERVO_TOW_POINT,                                     /* 前瞻 */
    UI_SERVO_COUNT                                          /* 舵机参数数量 */
} ui_servo_row_t;

typedef enum
{
    UI_MOTOR_TARGET_SPEED = 0,                              /* 目标速度 */
    UI_MOTOR_STRAIGHT_SPEED,                                /* 直道速度 */
    UI_MOTOR_COUNT                                          /* 电机参数数量 */
} ui_motor_row_t;

typedef enum
{
    UI_CAMERA_PREVIEW_BINARY = 0,                           /* 二值图 */
    UI_CAMERA_PREVIEW_RAW                                   /* 灰度图 */
} ui_camera_preview_t;

static const char *ui_root_name[UI_ROOT_COUNT] =
{
    "Camera view",
    "Param tuning",
    "Plan select"
};

static const char *ui_param_name[UI_PARAM_COUNT] =
{
    "servo",
    "camera",
    "motor"
};

static const char *ui_camera_name[UI_CAMERA_COUNT] =
{
    "exp time",
    "gain",
    "th offset",
    "fire r min",
    "fire r max",
    "fire tol"
};

static const char *ui_servo_name[UI_SERVO_COUNT] =
{
    "steer p",
    "steer d",
    "err2 k",
    "ackerman",
    "imu d",
    "tow point"
};

static const char *ui_motor_name[UI_MOTOR_COUNT] =
{
    "target speed",
    "straight speed"
};

static uint8 ui_ready = 0;
static uint8 ui_task_ready = 0;
static uint8 ui_dirty = 1;
static uint8 ui_editing = 0;
static ui_page_t ui_page = UI_PAGE_ROOT;
static uint8 ui_root_selected = 0;
static uint8 ui_param_selected = 0;
static uint8 ui_camera_selected = 0;
static uint8 ui_servo_selected = 0;
static uint8 ui_motor_selected = 0;
static uint8 ui_plan_selected = 0;
static uint8 ui_camera_preview_mode = UI_CAMERA_PREVIEW_BINARY;
static uint8 ui_key_stable_level[4] = {1, 1, 1, 1};
static uint8 ui_key_raw_level[4] = {1, 1, 1, 1};
static uint8 ui_key_debounce_count[4] = {0, 0, 0, 0};
static uint8 ui_key_pressed[4] = {0, 0, 0, 0};
static volatile uint8 ui_key_tick_ready = 0;
static volatile uint8 ui_screen_tick_ready = 0;
static flash_camera_page_t ui_camera_page;
static flash_servo_page_t ui_servo_page;
static flash_motor_page_t ui_motor_page;
static flash_camera_page_t ui_camera_backup;
static flash_servo_page_t ui_servo_backup;
static flash_motor_page_t ui_motor_backup;
static uint8 ui_power_percent = 0;

/* 电量百分比 */
static uint8 ui_power_calc_percent(uint16 voltage_deci)
{
    if(voltage_deci <= UI_POWER_EMPTY_DECI)
    {
        return 0;
    }

    if(voltage_deci >= UI_POWER_FULL_DECI)
    {
        return 100;
    }

    return (uint8)(((uint16)(voltage_deci - UI_POWER_EMPTY_DECI) * 100U) /
                   (UI_POWER_FULL_DECI - UI_POWER_EMPTY_DECI));
}

/* 电量采样 */
static void ui_power_init(void)
{
    uint16 adc_value;
    uint32 voltage_calc;

    adc_init(UI_POWER_ADC_PIN, UI_POWER_ADC_RESOLUTION);
    adc_value = adc_mean_filter_convert(UI_POWER_ADC_PIN, UI_POWER_ADC_SAMPLE_COUNT);
    voltage_calc = (uint32)adc_value * 363U + 2047U;
    ui_power_percent = ui_power_calc_percent((uint16)(voltage_calc / 4095U));
}

/* 按键定时器 */
static void ui_key_pit_handler(void)
{
    ui_key_tick_ready = 1;
}

/* 屏幕定时器 */
static void ui_screen_pit_handler(void)
{
    ui_screen_tick_ready = 1;
}

/* 读当前值 */
static void ui_load_page_value(void)
{
    flash_get_camera_page(&ui_camera_page);
    flash_get_servo_page(&ui_servo_page);
    flash_get_motor_page(&ui_motor_page);
    ui_plan_selected = flash_get_active_plan();
}

/* 备份当前值 */
static void ui_backup_page_value(void)
{
    memcpy(&ui_camera_backup, &ui_camera_page, sizeof(ui_camera_backup));
    memcpy(&ui_servo_backup, &ui_servo_page, sizeof(ui_servo_backup));
    memcpy(&ui_motor_backup, &ui_motor_page, sizeof(ui_motor_backup));
}

/* 恢复当前值 */
static void ui_restore_page_value(void)
{
    memcpy(&ui_camera_page, &ui_camera_backup, sizeof(ui_camera_page));
    memcpy(&ui_servo_page, &ui_servo_backup, sizeof(ui_servo_page));
    memcpy(&ui_motor_page, &ui_motor_backup, sizeof(ui_motor_backup));
}

/* 按键初始化 */
static void ui_key_init(void)
{
    gpio_init(UI_KEY1_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(UI_KEY2_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(UI_KEY3_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(UI_KEY4_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
}

/* 屏幕初始化 */
static void ui_screen_init(void)
{
    ips200_set_dir(IPS200_PORTAIT);
    ips200_init();
}

/* UI定时器初始化 */
static void ui_task_init(void)
{
    if(ui_task_ready)
    {
        return;
    }

    pit_ms_init(UI_KEY_PIT, UI_KEY_PERIOD_MS, ui_key_pit_handler);
    pit_ms_init(UI_SCREEN_PIT, UI_SCREEN_PERIOD_MS, ui_screen_pit_handler);
    interrupt_set_priority(TIM5_IRQn, UI_KEY_PIT_PRIORITY);
    interrupt_set_priority(TIM6_IRQn, UI_SCREEN_PIT_PRIORITY);
    ui_task_ready = 1;
}

/* 当前行数 */
static uint8 ui_get_row_count(void)
{
    switch(ui_page)
    {
        case UI_PAGE_ROOT:
            return UI_ROOT_COUNT;
        case UI_PAGE_PARAM_MENU:
            return UI_PARAM_COUNT;
        case UI_PAGE_CAMERA_PARAM:
            return UI_CAMERA_COUNT;
        case UI_PAGE_SERVO_PARAM:
            return UI_SERVO_COUNT;
        case UI_PAGE_MOTOR_PARAM:
            return UI_MOTOR_COUNT;
        case UI_PAGE_PLAN_SELECT:
            return FLASH_PLAN_COUNT;
        default:
            return 0;
    }
}

/* 选中行指针 */
static uint8 *ui_get_selected_ptr(void)
{
    switch(ui_page)
    {
        case UI_PAGE_ROOT:
            return &ui_root_selected;
        case UI_PAGE_PARAM_MENU:
            return &ui_param_selected;
        case UI_PAGE_CAMERA_PARAM:
            return &ui_camera_selected;
        case UI_PAGE_SERVO_PARAM:
            return &ui_servo_selected;
        case UI_PAGE_MOTOR_PARAM:
            return &ui_motor_selected;
        case UI_PAGE_PLAN_SELECT:
            return &ui_plan_selected;
        default:
            return 0;
    }
}

/* 行移动 */
static void ui_move_selected(int8 direction)
{
    uint8 *selected;
    uint8 row_count;

    selected = ui_get_selected_ptr();
    row_count = ui_get_row_count();

    if((0 == selected) || (0 == row_count))
    {
        return;
    }

    if(direction < 0)
    {
        if(0 == *selected)
        {
            *selected = (uint8)(row_count - 1);
        }
        else
        {
            (*selected)--;
        }
    }
    else if(direction > 0)
    {
        (*selected)++;
        if(*selected >= row_count)
        {
            *selected = 0;
        }
    }

    ui_dirty = 1;
}

/* 无符号数转文本 */
static uint8 ui_format_uint16_text(uint16 value, char *text)
{
    char temp[6];
    uint8 count;
    uint8 index;

    count = 0;
    index = 0;

    do
    {
        temp[count++] = (char)('0' + (value % 10U));
        value = (uint16)(value / 10U);
    } while(value && (count < sizeof(temp)));

    while(count > 0)
    {
        text[index++] = temp[--count];
    }
    text[index] = '\0';

    return index;
}

/* 0.1单位转文本 */
static void ui_format_tenths_text(int16 value, char *text)
{
    uint8 index;
    uint16 abs_value;

    index = 0;
    if(value < 0)
    {
        text[index++] = '-';
        abs_value = (uint16)(-value);
    }
    else
    {
        abs_value = (uint16)value;
    }

    index = (uint8)(index + ui_format_uint16_text((uint16)(abs_value / 10U), &text[index]));
    text[index++] = '.';
    text[index++] = (char)('0' + (abs_value % 10U));
    text[index] = '\0';
}

/* 调相机值 */
static void ui_adjust_camera_value(int8 direction)
{
    int16 step_value;
    int16 value;

    step_value = flash_get_camera_step((flash_camera_slot_t)ui_camera_selected);

    if(UI_CAMERA_EXP_TIME == ui_camera_selected)
    {
        value = (int16)(ui_camera_page.exp_time + (direction < 0 ? -step_value : step_value));
        ui_camera_page.exp_time = flash_limit_camera_value(FLASH_CAMERA_EXP_TIME, value);
    }
    else if(UI_CAMERA_GAIN == ui_camera_selected)
    {
        value = (int16)(ui_camera_page.gain + (direction < 0 ? -step_value : step_value));
        ui_camera_page.gain = flash_limit_camera_value(FLASH_CAMERA_GAIN, value);
    }
    else if(UI_CAMERA_THRESHOLD_OFFSET == ui_camera_selected)
    {
        value = (int16)(ui_camera_page.threshold_offset + (direction < 0 ? -step_value : step_value));
        ui_camera_page.threshold_offset = flash_limit_camera_value(FLASH_CAMERA_THRESHOLD_OFFSET, value);
    }
    else if(UI_CAMERA_FIRE_ROW_MIN == ui_camera_selected)
    {
        value = (int16)(ui_camera_page.fire_row_min + (direction < 0 ? -step_value : step_value));
        ui_camera_page.fire_row_min = flash_limit_camera_value(FLASH_CAMERA_FIRE_ROW_MIN, value);
        if(ui_camera_page.fire_row_min > ui_camera_page.fire_row_max)
        {
            ui_camera_page.fire_row_max = ui_camera_page.fire_row_min;
        }
    }
    else if(UI_CAMERA_FIRE_ROW_MAX == ui_camera_selected)
    {
        value = (int16)(ui_camera_page.fire_row_max + (direction < 0 ? -step_value : step_value));
        ui_camera_page.fire_row_max = flash_limit_camera_value(FLASH_CAMERA_FIRE_ROW_MAX, value);
        if(ui_camera_page.fire_row_max < ui_camera_page.fire_row_min)
        {
            ui_camera_page.fire_row_min = ui_camera_page.fire_row_max;
        }
    }
    else if(UI_CAMERA_FIRE_CENTER_TOL == ui_camera_selected)
    {
        value = (int16)(ui_camera_page.fire_center_tol + (direction < 0 ? -step_value : step_value));
        ui_camera_page.fire_center_tol = flash_limit_camera_value(FLASH_CAMERA_FIRE_CENTER_TOL, value);
    }

    ui_dirty = 1;
}

/* 调舵机值 */
static void ui_adjust_servo_value(int8 direction)
{
    int16 step_value;
    int16 value;

    step_value = flash_get_servo_step((flash_servo_slot_t)ui_servo_selected);

    switch(ui_servo_selected)
    {
        case UI_SERVO_P:
            value = (int16)(ui_servo_page.steer_p + (direction < 0 ? -step_value : step_value));
            ui_servo_page.steer_p = flash_limit_servo_value(FLASH_SERVO_P, value);
            break;
        case UI_SERVO_D:
            value = (int16)(ui_servo_page.steer_d + (direction < 0 ? -step_value : step_value));
            ui_servo_page.steer_d = flash_limit_servo_value(FLASH_SERVO_D, value);
            break;
        case UI_SERVO_ERR2:
            value = (int16)(ui_servo_page.err2_k + (direction < 0 ? -step_value : step_value));
            ui_servo_page.err2_k = flash_limit_servo_value(FLASH_SERVO_ERR2, value);
            break;
        case UI_SERVO_ACKERMAN:
            value = (int16)(ui_servo_page.ackerman + (direction < 0 ? -step_value : step_value));
            ui_servo_page.ackerman = flash_limit_servo_value(FLASH_SERVO_ACKERMAN, value);
            break;
        case UI_SERVO_IMU_D:
            value = (int16)(ui_servo_page.imu_d + (direction < 0 ? -step_value : step_value));
            ui_servo_page.imu_d = flash_limit_servo_value(FLASH_SERVO_IMU_D, value);
            break;
        case UI_SERVO_TOW_POINT:
            value = (int16)(ui_servo_page.tow_point + (direction < 0 ? -step_value : step_value));
            ui_servo_page.tow_point = flash_limit_servo_value(FLASH_SERVO_TOW_POINT, value);
            break;
        default:
            break;
    }

    ui_dirty = 1;
}

/* 调电机值 */
static void ui_adjust_motor_value(int8 direction)
{
    int16 step_value;
    int16 value;

    step_value = flash_get_motor_step((flash_motor_slot_t)ui_motor_selected);

    switch(ui_motor_selected)
    {
        case UI_MOTOR_TARGET_SPEED:
            value = (int16)(ui_motor_page.target_speed + (direction < 0 ? -step_value : step_value));
            ui_motor_page.target_speed = flash_limit_motor_value(FLASH_MOTOR_TARGET_SPEED, value);
            break;
        case UI_MOTOR_STRAIGHT_SPEED:
            value = (int16)(ui_motor_page.straight_speed + (direction < 0 ? -step_value : step_value));
            ui_motor_page.straight_speed = flash_limit_motor_value(FLASH_MOTOR_STRAIGHT_SPEED, value);
            break;
        default:
            break;
    }
    ui_dirty = 1;
}

/* 保存相机值 */
static void ui_save_camera_value(void)
{
    image_set_camera_value(FLASH_CAMERA_EXP_TIME, ui_camera_page.exp_time);
    image_set_camera_value(FLASH_CAMERA_GAIN, ui_camera_page.gain);
    image_set_camera_value(FLASH_CAMERA_THRESHOLD_OFFSET, ui_camera_page.threshold_offset);
    image_set_camera_value(FLASH_CAMERA_FIRE_ROW_MIN, ui_camera_page.fire_row_min);
    image_set_camera_value(FLASH_CAMERA_FIRE_ROW_MAX, ui_camera_page.fire_row_max);
    image_set_camera_value(FLASH_CAMERA_FIRE_CENTER_TOL, ui_camera_page.fire_center_tol);
    ui_editing = 0;
    ui_dirty = 1;
}

/* 保存舵机值 */
static void ui_save_servo_value(void)
{
    if(flash_set_servo_page(&ui_servo_page))
    {
        ui_load_page_value();
        ui_backup_page_value();
    }
    else
    {
        ui_restore_page_value();
    }
    ui_editing = 0;
    ui_dirty = 1;
}

/* 保存电机值 */
static void ui_save_motor_value(void)
{
    if(flash_set_motor_page(&ui_motor_page))
    {
        ui_load_page_value();
        ui_backup_page_value();
    }
    else
    {
        ui_restore_page_value();
    }
    ui_editing = 0;
    ui_dirty = 1;
}

/* 保存当前编辑页 */
static void ui_save_current_page(void)
{
    if(UI_PAGE_CAMERA_PARAM == ui_page)
    {
        ui_save_camera_value();
    }
    else if(UI_PAGE_SERVO_PARAM == ui_page)
    {
        ui_save_servo_value();
    }
    else if(UI_PAGE_MOTOR_PARAM == ui_page)
    {
        ui_save_motor_value();
    }
}

/* 读按键 */
static void ui_read_key_level(void)
{
    ui_key_raw_level[0] = gpio_get_level(UI_KEY1_PIN) ? 1 : 0;
    ui_key_raw_level[1] = gpio_get_level(UI_KEY2_PIN) ? 1 : 0;
    ui_key_raw_level[2] = gpio_get_level(UI_KEY3_PIN) ? 1 : 0;
    ui_key_raw_level[3] = gpio_get_level(UI_KEY4_PIN) ? 1 : 0;
}

/* 按键更新 */
static ui_event_t ui_key_update(void)
{
    ui_event_t event;
    uint8 i;

    event = UI_EVENT_NONE;
    ui_read_key_level();

    for(i = 0; i < 4; i++)
    {
        if(ui_key_raw_level[i] == ui_key_stable_level[i])
        {
            ui_key_debounce_count[i] = 0;
        }
        else
        {
            ui_key_debounce_count[i]++;
            if(ui_key_debounce_count[i] >= UI_KEY_DEBOUNCE_COUNT)
            {
                ui_key_stable_level[i] = ui_key_raw_level[i];
                ui_key_debounce_count[i] = 0;

                if(0 == ui_key_stable_level[i])
                {
                    ui_key_pressed[i] = 1;
                }
                else if(ui_key_pressed[i])
                {
                    ui_key_pressed[i] = 0;

                    switch(i)
                    {
                        case 0:
                            event = UI_EVENT_BACK;
                            break;
                        case 1:
                            event = UI_EVENT_UP;
                            break;
                        case 2:
                            event = UI_EVENT_DOWN;
                            break;
                        case 3:
                            event = UI_EVENT_ENTER;
                            break;
                        default:
                            break;
                    }
                }
            }
        }
    }

    return event;
}

/* 画标题 */
static void ui_draw_title(const char *title)
{
    ips200_set_color(RGB565_BLACK, RGB565_WHITE);
    ips200_show_string(0, UI_TITLE_Y, title);
}

/* 画红标题 */
static void ui_draw_title_red(const char *title)
{
    ips200_set_color(RGB565_RED, RGB565_WHITE);
    ips200_show_string(0, UI_TITLE_Y, title);
}

/* 画步进 */
static void ui_draw_step_value(int16 step_value)
{
    ips200_set_color(RGB565_PURPLE, RGB565_WHITE);
    ips200_show_string(0, UI_STATUS_Y, "step");
    ips200_show_int16(48, UI_STATUS_Y, step_value);
}

/* 画步进文本 */
static void ui_draw_step_text(const char *text)
{
    ips200_set_color(RGB565_PURPLE, RGB565_WHITE);
    ips200_show_string(0, UI_STATUS_Y, "step");
    ips200_show_string(48, UI_STATUS_Y, text);
}

/* 画电量 */
static void ui_draw_power_percent(void)
{
    ips200_set_color(RGB565_BLACK, RGB565_WHITE);
    ips200_show_uint8(UI_POWER_X, UI_TITLE_Y, ui_power_percent);
    ips200_show_string((uint16)(UI_POWER_X + 24), UI_TITLE_Y, "%");
}

/* 相机页底图 */
static void ui_prepare_camera_view(void)
{
    ips200_clear(RGB565_BLACK);
}

/* 画打靶框 */
static void ui_draw_target_ring_overlay(void)
{
    flash_camera_page_t camera_page;
    uint8 found;
    uint8 center_x;
    uint8 center_y;
    uint8 left_x;
    uint8 right_x;
    uint8 top_y;
    uint8 bottom_y;
    uint16 fire_y_min;
    uint16 fire_y_max;
    uint16 fire_x_left;
    uint16 fire_x_right;
    uint16 draw_center_x;
    uint16 draw_center_y;
    uint16 draw_left_x;
    uint16 draw_right_x;
    uint16 draw_top_y;
    uint16 draw_bottom_y;
    int i;
    int draw_x;
    int draw_y;

    flash_get_camera_page(&camera_page);
    fire_y_min = (uint16)(UI_CAMERA_VIEW_Y +
                          (((uint16)camera_page.fire_row_min * UI_CAMERA_VIEW_H) + (LCDH / 2)) / LCDH);
    fire_y_max = (uint16)(UI_CAMERA_VIEW_Y +
                          (((uint16)camera_page.fire_row_max * UI_CAMERA_VIEW_H) + (LCDH / 2)) / LCDH);
    fire_x_left = (uint16)(UI_CAMERA_VIEW_X +
                           ((((uint16)(ImageSensorMid - camera_page.fire_center_tol)) * UI_CAMERA_VIEW_W) + (LCDW / 2)) / LCDW);
    fire_x_right = (uint16)(UI_CAMERA_VIEW_X +
                            ((((uint16)(ImageSensorMid + camera_page.fire_center_tol)) * UI_CAMERA_VIEW_W) + (LCDW / 2)) / LCDW);

    for(i = (int)fire_x_left; i <= (int)fire_x_right; i++)
    {
        ips200_draw_point((uint16)i, fire_y_min, RGB565_BLUE);
        ips200_draw_point((uint16)i, fire_y_max, RGB565_BLUE);
    }
    for(i = (int)fire_y_min; i <= (int)fire_y_max; i++)
    {
        ips200_draw_point(fire_x_left, (uint16)i, RGB565_BLUE);
        ips200_draw_point(fire_x_right, (uint16)i, RGB565_BLUE);
    }

    found = image_get_target_ring_found();
    if(!found)
    {
        return;
    }

    image_get_target_ring_box(&center_x, &center_y, &left_x, &right_x, &top_y, &bottom_y);

    draw_center_x = (uint16)(UI_CAMERA_VIEW_X + (((uint16)center_x * UI_CAMERA_VIEW_W) + (LCDW / 2)) / LCDW);
    draw_center_y = (uint16)(UI_CAMERA_VIEW_Y + (((uint16)center_y * UI_CAMERA_VIEW_H) + (LCDH / 2)) / LCDH);
    draw_left_x = (uint16)(UI_CAMERA_VIEW_X + (((uint16)left_x * UI_CAMERA_VIEW_W) + (LCDW / 2)) / LCDW);
    draw_right_x = (uint16)(UI_CAMERA_VIEW_X + (((uint16)right_x * UI_CAMERA_VIEW_W) + (LCDW / 2)) / LCDW);
    draw_top_y = (uint16)(UI_CAMERA_VIEW_Y + (((uint16)top_y * UI_CAMERA_VIEW_H) + (LCDH / 2)) / LCDH);
    draw_bottom_y = (uint16)(UI_CAMERA_VIEW_Y + (((uint16)bottom_y * UI_CAMERA_VIEW_H) + (LCDH / 2)) / LCDH);

    for(i = (int)draw_left_x; i <= (int)draw_right_x; i++)
    {
        ips200_draw_point((uint16)i, draw_top_y, RGB565_MAGENTA);
        ips200_draw_point((uint16)i, draw_bottom_y, RGB565_MAGENTA);
    }
    for(i = (int)draw_top_y; i <= (int)draw_bottom_y; i++)
    {
        ips200_draw_point(draw_left_x, (uint16)i, RGB565_MAGENTA);
        ips200_draw_point(draw_right_x, (uint16)i, RGB565_MAGENTA);
    }
    for(i = -4; i <= 4; i++)
    {
        draw_x = (int)draw_center_x + i;
        draw_y = (int)draw_center_y + i;

        if(draw_x < (int)UI_CAMERA_VIEW_X)
        {
            draw_x = UI_CAMERA_VIEW_X;
        }
        else if(draw_x >= (int)(UI_CAMERA_VIEW_X + UI_CAMERA_VIEW_W))
        {
            draw_x = (int)(UI_CAMERA_VIEW_X + UI_CAMERA_VIEW_W - 1);
        }

        if(draw_y < (int)UI_CAMERA_VIEW_Y)
        {
            draw_y = UI_CAMERA_VIEW_Y;
        }
        else if(draw_y >= (int)(UI_CAMERA_VIEW_Y + UI_CAMERA_VIEW_H))
        {
            draw_y = (int)(UI_CAMERA_VIEW_Y + UI_CAMERA_VIEW_H - 1);
        }

        ips200_draw_point((uint16)draw_x, draw_center_y, RGB565_RED);
        ips200_draw_point(draw_center_x, (uint16)draw_y, RGB565_RED);
    }
}

/* 画辅助线 */
static void ui_draw_camera_overlay(void)
{
    uint8 row;
    uint8 row_start;
    uint8 row_end;
    uint8 boundary_start;
    int left_col;
    int right_col;
    int center_col;
    int boundary_col;
    uint16 x;
    uint16 y;

    row_start = ImageStatus.OFFLine;
    if(row_start >= LCDH)
    {
        return;
    }

    row_end = (uint8)(LCDH - 1);
    for(row = row_start; row <= row_end; row++)
    {
        left_col = ImageDeal[row].LeftBorder;
        right_col = ImageDeal[row].RightBorder;
        center_col = ImageDeal[row].Center;

        if(left_col < 0)
        {
            left_col = 0;
        }
        else if(left_col >= LCDW)
        {
            left_col = LCDW - 1;
        }

        if(right_col < 0)
        {
            right_col = 0;
        }
        else if(right_col >= LCDW)
        {
            right_col = LCDW - 1;
        }

        if(center_col < 0)
        {
            center_col = 0;
        }
        else if(center_col >= LCDW)
        {
            center_col = LCDW - 1;
        }

        y = (uint16)(UI_CAMERA_VIEW_Y + ((uint16)row * UI_CAMERA_VIEW_H + (LCDH / 2)) / LCDH);

        x = (uint16)(UI_CAMERA_VIEW_X + (((uint16)left_col * UI_CAMERA_VIEW_W) + (LCDW / 2)) / LCDW);
        ips200_draw_point(x, y, RGB565_GREEN);
        if(y < (uint16)(UI_CAMERA_VIEW_Y + UI_CAMERA_VIEW_H - 1))
        {
            ips200_draw_point(x, (uint16)(y + 1), RGB565_GREEN);
        }

        x = (uint16)(UI_CAMERA_VIEW_X + (((uint16)right_col * UI_CAMERA_VIEW_W) + (LCDW / 2)) / LCDW);
        ips200_draw_point(x, y, RGB565_GREEN);
        if(y < (uint16)(UI_CAMERA_VIEW_Y + UI_CAMERA_VIEW_H - 1))
        {
            ips200_draw_point(x, (uint16)(y + 1), RGB565_GREEN);
        }

        x = (uint16)(UI_CAMERA_VIEW_X + (((uint16)center_col * UI_CAMERA_VIEW_W) + (LCDW / 2)) / LCDW);
        ips200_draw_point(x, y, RGB565_RED);
        if(y < (uint16)(UI_CAMERA_VIEW_Y + UI_CAMERA_VIEW_H - 1))
        {
            ips200_draw_point(x, (uint16)(y + 1), RGB565_RED);
        }
    }

    if(ImageStatus.OFFLineBoundary < 0)
    {
        boundary_start = 0;
    }
    else if(ImageStatus.OFFLineBoundary >= (LCDH - 1))
    {
        boundary_start = (uint8)(LCDH - 2);
    }
    else
    {
        boundary_start = (uint8)ImageStatus.OFFLineBoundary;
    }

    for(row = boundary_start; row < (uint8)(LCDH - 1); row++)
    {
        y = (uint16)(UI_CAMERA_VIEW_Y + ((uint16)row * UI_CAMERA_VIEW_H + (LCDH / 2)) / LCDH);

        boundary_col = ImageDeal[row].LeftBoundary_First;
        if(boundary_col < 0)
        {
            boundary_col = 0;
        }
        else if(boundary_col >= LCDW)
        {
            boundary_col = LCDW - 1;
        }
        x = (uint16)(UI_CAMERA_VIEW_X + (((uint16)boundary_col * UI_CAMERA_VIEW_W) + (LCDW / 2)) / LCDW);
        ips200_draw_point(x, y, RGB565_YELLOW);

        boundary_col = ImageDeal[row].RightBoundary_First;
        if(boundary_col < 0)
        {
            boundary_col = 0;
        }
        else if(boundary_col >= LCDW)
        {
            boundary_col = LCDW - 1;
        }
        x = (uint16)(UI_CAMERA_VIEW_X + (((uint16)boundary_col * UI_CAMERA_VIEW_W) + (LCDW / 2)) / LCDW);
        ips200_draw_point(x, y, RGB565_YELLOW);

        boundary_col = ImageDeal[row].LeftBoundary;
        if(boundary_col < 0)
        {
            boundary_col = 0;
        }
        else if(boundary_col >= LCDW)
        {
            boundary_col = LCDW - 1;
        }
        x = (uint16)(UI_CAMERA_VIEW_X + (((uint16)boundary_col * UI_CAMERA_VIEW_W) + (LCDW / 2)) / LCDW);
        ips200_draw_point(x, y, RGB565_CYAN);

        boundary_col = ImageDeal[row].RightBoundary;
        if(boundary_col < 0)
        {
            boundary_col = 0;
        }
        else if(boundary_col >= LCDW)
        {
            boundary_col = LCDW - 1;
        }
        x = (uint16)(UI_CAMERA_VIEW_X + (((uint16)boundary_col * UI_CAMERA_VIEW_W) + (LCDW / 2)) / LCDW);
        ips200_draw_point(x, y, RGB565_CYAN);
    }
}

/* 画相机预览 */
static void ui_draw_camera_preview(void)
{
    if(!image_is_ready() || !image_is_result_ready())
    {
        ips200_set_color(RGB565_WHITE, RGB565_BLACK);
        ips200_show_string(UI_CAMERA_VIEW_X, UI_CAMERA_VIEW_Y, "not ready");
        return;
    }

    if(UI_CAMERA_PREVIEW_RAW == ui_camera_preview_mode)
    {
        ips200_show_gray_image(UI_CAMERA_VIEW_X,
                               UI_CAMERA_VIEW_Y,
                               image_get_raw_buffer(),
                               LCDW,
                               LCDH,
                               UI_CAMERA_VIEW_W,
                               UI_CAMERA_VIEW_H,
                               0);
    }
    else
    {
        ips200_show_gray_image(UI_CAMERA_VIEW_X,
                               UI_CAMERA_VIEW_Y,
                               image_get_binary_buffer(),
                               LCDW,
                               LCDH,
                               UI_CAMERA_VIEW_W,
                               UI_CAMERA_VIEW_H,
                               1);
    }

    ui_draw_camera_overlay();
    ui_draw_target_ring_overlay();
}

/* 相机预览信息 */
static void ui_draw_camera_info(void)
{
    uint8 stable_count;
    uint8 shot_latch;

    stable_count = image_get_target_ring_stable_count();
    shot_latch = image_get_target_ring_shot_latch();

    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_show_string(0, UI_CAMERA_INFO_Y, "threshold");
    ips200_show_uint8(80, UI_CAMERA_INFO_Y, ImageStatus.Threshold);
    ips200_show_string(112, UI_CAMERA_INFO_Y, "tg");
    ips200_show_uint8(128, UI_CAMERA_INFO_Y, image_get_target_ring_found());
    ips200_show_string(144, UI_CAMERA_INFO_Y, "st");
    ips200_show_uint8(160, UI_CAMERA_INFO_Y, stable_count);
    ips200_show_string(176, UI_CAMERA_INFO_Y, "sh");
    ips200_show_uint8(192, UI_CAMERA_INFO_Y, shot_latch);
}

/* 画文本行 */
static void ui_draw_text_row(uint8 row, const char *name, uint8 selected)
{
    uint16 y;
    uint16 color;

    y = (uint16)(UI_LIST_Y + (uint16)row * UI_ROW_HEIGHT);
    color = selected ? RGB565_BLUE : RGB565_BLACK;

    ips200_set_color(color, RGB565_WHITE);
    ips200_show_string(0, y, selected ? ">" : " ");
    ips200_show_string(UI_LABEL_X, y, name);
}

/* 画数值行 */
static void ui_draw_value_row(uint8 row, const char *name, int16 value, uint8 selected)
{
    uint16 y;
    uint16 color;

    y = (uint16)(UI_LIST_Y + (uint16)row * UI_ROW_HEIGHT);
    color = selected ? (ui_editing ? RGB565_RED : RGB565_BLUE) : RGB565_BLACK;

    ips200_set_color(color, RGB565_WHITE);
    ips200_show_string(0, y, selected ? (ui_editing ? "*" : ">") : " ");
    ips200_show_string(UI_LABEL_X, y, name);
    ips200_show_int16(UI_VALUE_X, y, value);
}

/* 画文本数值行 */
static void ui_draw_value_row_text(uint8 row, const char *name, const char *text, uint8 selected)
{
    uint16 y;
    uint16 color;

    y = (uint16)(UI_LIST_Y + (uint16)row * UI_ROW_HEIGHT);
    color = selected ? (ui_editing ? RGB565_RED : RGB565_BLUE) : RGB565_BLACK;

    ips200_set_color(color, RGB565_WHITE);
    ips200_show_string(0, y, selected ? (ui_editing ? "*" : ">") : " ");
    ips200_show_string(UI_LABEL_X, y, name);
    ips200_show_string(UI_VALUE_X, y, text);
}

/* 主界面 */
static void ui_draw_root_page(void)
{
    uint8 i;

    ui_draw_title_red("Main");

    for(i = 0; i < UI_ROOT_COUNT; i++)
    {
        ui_draw_text_row(i, ui_root_name[i], (i == ui_root_selected) ? 1 : 0);
    }
}

/* 参数主界面 */
static void ui_draw_param_menu_page(void)
{
    uint8 i;

    ui_draw_title("Param tuning");

    for(i = 0; i < UI_PARAM_COUNT; i++)
    {
        ui_draw_text_row(i, ui_param_name[i], (i == ui_param_selected) ? 1 : 0);
    }
}

/* 相机参数界面 */
static void ui_draw_camera_param_page(void)
{
    int16 step_value;

    ui_draw_title("Camera");
    step_value = flash_get_camera_step((flash_camera_slot_t)ui_camera_selected);
    ui_draw_step_value(step_value);

    ui_draw_value_row(0, ui_camera_name[0], ui_camera_page.exp_time, (0 == ui_camera_selected) ? 1 : 0);
    ui_draw_value_row(1, ui_camera_name[1], ui_camera_page.gain, (1 == ui_camera_selected) ? 1 : 0);
    ui_draw_value_row(2, ui_camera_name[2], ui_camera_page.threshold_offset, (2 == ui_camera_selected) ? 1 : 0);
    ui_draw_value_row(3, ui_camera_name[3], ui_camera_page.fire_row_min, (3 == ui_camera_selected) ? 1 : 0);
    ui_draw_value_row(4, ui_camera_name[4], ui_camera_page.fire_row_max, (4 == ui_camera_selected) ? 1 : 0);
    ui_draw_value_row(5, ui_camera_name[5], ui_camera_page.fire_center_tol, (5 == ui_camera_selected) ? 1 : 0);
}

/* 舵机参数界面 */
static void ui_draw_servo_param_page(void)
{
    char value_text[8];
    char step_text[8];
    int16 step_value;

    ui_draw_title("Servo");
    step_value = flash_get_servo_step((flash_servo_slot_t)ui_servo_selected);
    if((UI_SERVO_ERR2 == ui_servo_selected) || (UI_SERVO_IMU_D == ui_servo_selected))
    {
        ui_format_tenths_text(step_value, step_text);
        ui_draw_step_text(step_text);
    }
    else
    {
        ui_draw_step_value(step_value);
    }

    ui_draw_value_row(0, ui_servo_name[0], ui_servo_page.steer_p, (0 == ui_servo_selected) ? 1 : 0);
    ui_draw_value_row(1, ui_servo_name[1], ui_servo_page.steer_d, (1 == ui_servo_selected) ? 1 : 0);

    ui_format_tenths_text(ui_servo_page.err2_k, value_text);
    ui_draw_value_row_text(2, ui_servo_name[2], value_text, (2 == ui_servo_selected) ? 1 : 0);

    ui_draw_value_row(3, ui_servo_name[3], ui_servo_page.ackerman, (3 == ui_servo_selected) ? 1 : 0);

    ui_format_tenths_text(ui_servo_page.imu_d, value_text);
    ui_draw_value_row_text(4, ui_servo_name[4], value_text, (4 == ui_servo_selected) ? 1 : 0);

    ui_draw_value_row(5, ui_servo_name[5], ui_servo_page.tow_point, (5 == ui_servo_selected) ? 1 : 0);
}

/* 电机参数界面 */
static void ui_draw_motor_param_page(void)
{
    ui_draw_title("Motor");
    ui_draw_step_value(flash_get_motor_step((flash_motor_slot_t)ui_motor_selected));

    ui_draw_value_row(0, ui_motor_name[0], ui_motor_page.target_speed, (0 == ui_motor_selected) ? 1 : 0);
    ui_draw_value_row(1, ui_motor_name[1], ui_motor_page.straight_speed, (1 == ui_motor_selected) ? 1 : 0);
}

/* 方案界面 */
static void ui_draw_plan_select_page(void)
{
    uint8 i;
    uint16 y;
    uint16 color;
    uint8 active_plan;

    active_plan = flash_get_active_plan();

    ui_draw_title("Plan select");

    for(i = 0; i < FLASH_PLAN_COUNT; i++)
    {
        y = (uint16)(UI_LIST_Y + (uint16)i * UI_ROW_HEIGHT);
        color = (i == ui_plan_selected) ? RGB565_BLUE : RGB565_BLACK;

        ips200_set_color(color, RGB565_WHITE);
        ips200_show_string(0, y, (i == ui_plan_selected) ? ">" : " ");
        ips200_show_string(UI_LABEL_X, y, "plan");
        ips200_show_uint8((uint16)(UI_LABEL_X + 40), y, (uint8)(i + 1));

        if(i == active_plan)
        {
            ips200_show_string(UI_VALUE_X, y, "use");
        }
    }
}

/* 相机预览界面 */
static void ui_draw_camera_view_page(void)
{
    if(ui_dirty)
    {
        ui_prepare_camera_view();
    }

    ui_draw_camera_preview();
    ui_draw_camera_info();
}

/* 重画 */
static void ui_render(void)
{
    if(UI_PAGE_CAMERA_VIEW == ui_page)
    {
        ui_draw_camera_view_page();
        return;
    }

    ips200_clear(RGB565_WHITE);

    switch(ui_page)
    {
        case UI_PAGE_ROOT:
            ui_draw_root_page();
            break;
        case UI_PAGE_PARAM_MENU:
            ui_draw_param_menu_page();
            break;
        case UI_PAGE_CAMERA_PARAM:
            ui_draw_camera_param_page();
            break;
        case UI_PAGE_SERVO_PARAM:
            ui_draw_servo_param_page();
            break;
        case UI_PAGE_MOTOR_PARAM:
            ui_draw_motor_param_page();
            break;
        case UI_PAGE_PLAN_SELECT:
            ui_draw_plan_select_page();
            break;
        default:
            break;
    }

    ui_draw_power_percent();
}

/* 进入页面 */
static void ui_enter_page(void)
{
    if(UI_PAGE_ROOT == ui_page)
    {
        switch(ui_root_selected)
        {
            case UI_ROOT_CAMERA_VIEW:
                ui_page = UI_PAGE_CAMERA_VIEW;
                break;
            case UI_ROOT_PARAM_TUNING:
                ui_param_selected = UI_PARAM_SERVO;
                ui_page = UI_PAGE_PARAM_MENU;
                break;
            case UI_ROOT_PLAN_SELECT:
                ui_page = UI_PAGE_PLAN_SELECT;
                ui_plan_selected = flash_get_active_plan();
                break;
            default:
                break;
        }
        ui_dirty = 1;
        return;
    }

    if(UI_PAGE_PARAM_MENU == ui_page)
    {
        if(UI_PARAM_SERVO == ui_param_selected)
        {
            ui_page = UI_PAGE_SERVO_PARAM;
        }
        else if(UI_PARAM_CAMERA == ui_param_selected)
        {
            ui_camera_selected = UI_CAMERA_EXP_TIME;
            ui_page = UI_PAGE_CAMERA_PARAM;
        }
        else if(UI_PARAM_MOTOR == ui_param_selected)
        {
            ui_page = UI_PAGE_MOTOR_PARAM;
        }
        ui_dirty = 1;
        return;
    }

    if(UI_PAGE_PLAN_SELECT == ui_page)
    {
        flash_set_active_plan(ui_plan_selected);
        image_reload_camera_page();
        ui_load_page_value();
        ui_dirty = 1;
        return;
    }

    if((UI_PAGE_CAMERA_PARAM == ui_page) ||
       (UI_PAGE_SERVO_PARAM == ui_page) ||
       (UI_PAGE_MOTOR_PARAM == ui_page))
    {
        if(ui_editing)
        {
            ui_save_current_page();
        }
        else
        {
            ui_backup_page_value();
            ui_editing = 1;
            ui_dirty = 1;
        }
    }
}

/* 返回页面 */
static void ui_back_page(void)
{
    if(ui_editing)
    {
        ui_save_current_page();
        return;
    }

    switch(ui_page)
    {
        case UI_PAGE_CAMERA_VIEW:
        case UI_PAGE_PARAM_MENU:
        case UI_PAGE_PLAN_SELECT:
            ui_page = UI_PAGE_ROOT;
            break;
        case UI_PAGE_CAMERA_PARAM:
        case UI_PAGE_SERVO_PARAM:
        case UI_PAGE_MOTOR_PARAM:
            ui_page = UI_PAGE_PARAM_MENU;
            break;
        default:
            break;
    }

    ui_dirty = 1;
}

/* 按键处理 */
static void ui_handle_event(ui_event_t event)
{
    if(UI_PAGE_CAMERA_VIEW == ui_page)
    {
        switch(event)
        {
            case UI_EVENT_BACK:
                ui_back_page();
                break;
            case UI_EVENT_UP:
            case UI_EVENT_DOWN:
                if(UI_CAMERA_PREVIEW_RAW == ui_camera_preview_mode)
                {
                    ui_camera_preview_mode = UI_CAMERA_PREVIEW_BINARY;
                }
                else
                {
                    ui_camera_preview_mode = UI_CAMERA_PREVIEW_RAW;
                }
                ui_dirty = 1;
                break;
            case UI_EVENT_ENTER:
            case UI_EVENT_NONE:
            default:
                break;
        }
        return;
    }

    switch(event)
    {
        case UI_EVENT_BACK:
            ui_back_page();
            break;
        case UI_EVENT_UP:
            if(ui_editing)
            {
                if(UI_PAGE_CAMERA_PARAM == ui_page)
                {
                    ui_adjust_camera_value(1);
                }
                else if(UI_PAGE_SERVO_PARAM == ui_page)
                {
                    ui_adjust_servo_value(1);
                }
                else if(UI_PAGE_MOTOR_PARAM == ui_page)
                {
                    ui_adjust_motor_value(1);
                }
            }
            else
            {
                ui_move_selected(-1);
            }
            break;
        case UI_EVENT_DOWN:
            if(ui_editing)
            {
                if(UI_PAGE_CAMERA_PARAM == ui_page)
                {
                    ui_adjust_camera_value(-1);
                }
                else if(UI_PAGE_SERVO_PARAM == ui_page)
                {
                    ui_adjust_servo_value(-1);
                }
                else if(UI_PAGE_MOTOR_PARAM == ui_page)
                {
                    ui_adjust_motor_value(-1);
                }
            }
            else
            {
                ui_move_selected(1);
            }
            break;
        case UI_EVENT_ENTER:
            ui_enter_page();
            break;
        case UI_EVENT_NONE:
        default:
            break;
    }
}

/* 初始化 */
void ui_init(void)
{
    /* 初始化按键 */
    ui_key_init();
    /* 初始化屏幕 */
    ui_screen_init();
    /* 读取电量 */
    ui_power_init();
    /* 初始化UI定时器 */
    ui_task_init();
    /* 读取当前页面参数 */
    ui_load_page_value();
    /* 备份当前页面参数 */
    ui_backup_page_value();
    /* 允许UI运行 */
    ui_ready = 1;
    /* 首次进入强制刷新 */
    ui_dirty = 1;
    /* 绘制首帧界面 */
    ui_render();
    ui_dirty = 0;
}

/* UI更新 */
void ui_update(void)
{
    ui_event_t event;
    uint8 key_tick_ready;
    uint8 screen_tick_ready;

    /* 未初始化时先初始化 */
    if(!ui_ready)
    {
        ui_init();
    }

    /* 读定时器标志 */
    interrupt_global_disable();
    key_tick_ready = ui_key_tick_ready;
    screen_tick_ready = ui_screen_tick_ready;
    ui_key_tick_ready = 0;
    ui_screen_tick_ready = 0;
    interrupt_global_enable();

    if(key_tick_ready)
    {
        /* 读按键事件 */
        event = ui_key_update();
        /* 处理页面逻辑 */
        ui_handle_event(event);
    }

    if(screen_tick_ready && (ui_dirty || (UI_PAGE_CAMERA_VIEW == ui_page)))
    {
        /* 有变化时刷新界面 */
        ui_render();
        ui_dirty = 0;
    }
}

/* 相机预览页 */
uint8 ui_is_camera_view(void)
{
    return (UI_PAGE_CAMERA_VIEW == ui_page) ? 1 : 0;
}
