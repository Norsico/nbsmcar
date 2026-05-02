#include "flash.h"
#include "ui.h"

#define UI_TITLE_Y                   (0)                    /* 标题行 */
#define UI_STATUS_Y                  (16)                   /* 状态行 */
#define UI_LIST_Y                    (32)                   /* 列表起始 */
#define UI_ROW_HEIGHT                (16)                   /* 行高 */
#define UI_LABEL_X                   (16)                   /* 标签X */
#define UI_VALUE_X                   (152)                  /* 数值X */

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
    UI_PARAM_CAMERA = 0,                                    /* 相机 */
    UI_PARAM_MOTOR,                                         /* 电机 */
    UI_PARAM_COUNT                                          /* 参数分类数量 */
} ui_param_row_t;

typedef enum
{
    UI_CAMERA_EXP_TIME = 0,                                 /* 曝光 */
    UI_CAMERA_GAIN,                                         /* 增益 */
    UI_CAMERA_COUNT                                         /* 相机参数数量 */
} ui_camera_row_t;

typedef enum
{
    UI_MOTOR_TARGET_SPEED = 0,                              /* 目标速度 */
    UI_MOTOR_COUNT                                          /* 电机参数数量 */
} ui_motor_row_t;

static const char *ui_root_name[UI_ROOT_COUNT] =
{
    "Camera view",
    "Param tuning",
    "Plan select"
};

static const char *ui_param_name[UI_PARAM_COUNT] =
{
    "camera",
    "motor"
};

static const char *ui_camera_name[UI_CAMERA_COUNT] =
{
    "exp time",
    "gain"
};

static const char *ui_motor_name[UI_MOTOR_COUNT] =
{
    "target speed"
};

static uint8 ui_ready = 0;
static uint8 ui_dirty = 1;
static uint8 ui_editing = 0;
static ui_page_t ui_page = UI_PAGE_ROOT;
static uint8 ui_root_selected = 0;
static uint8 ui_param_selected = 0;
static uint8 ui_camera_selected = 0;
static uint8 ui_motor_selected = 0;
static uint8 ui_plan_selected = 0;
static uint8 ui_key_stable_level[4] = {1, 1, 1, 1};
static uint8 ui_key_raw_level[4] = {1, 1, 1, 1};
static uint8 ui_key_debounce_count[4] = {0, 0, 0, 0};
static uint8 ui_key_pressed[4] = {0, 0, 0, 0};
static flash_camera_page_t ui_camera_page;
static flash_motor_page_t ui_motor_page;
static flash_camera_page_t ui_camera_backup;
static flash_motor_page_t ui_motor_backup;

/* 读当前值 */
static void ui_load_page_value(void)
{
    flash_get_camera_page(&ui_camera_page);
    flash_get_motor_page(&ui_motor_page);
    ui_plan_selected = flash_get_active_plan();
}

/* 备份当前值 */
static void ui_backup_page_value(void)
{
    memcpy(&ui_camera_backup, &ui_camera_page, sizeof(ui_camera_backup));
    memcpy(&ui_motor_backup, &ui_motor_page, sizeof(ui_motor_backup));
}

/* 恢复当前值 */
static void ui_restore_page_value(void)
{
    memcpy(&ui_camera_page, &ui_camera_backup, sizeof(ui_camera_page));
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

/* 限幅 */
static int16 ui_limit_value(int16 value, int16 min_value, int16 max_value)
{
    if(value < min_value)
    {
        return min_value;
    }

    if(value > max_value)
    {
        return max_value;
    }

    return value;
}

/* 调相机值 */
static void ui_adjust_camera_value(int8 direction)
{
    int16 min_value;
    int16 max_value;
    int16 step_value;
    int16 value;

    flash_get_camera_range((flash_camera_slot_t)ui_camera_selected, &min_value, &max_value, &step_value);

    if(UI_CAMERA_EXP_TIME == ui_camera_selected)
    {
        value = (int16)(ui_camera_page.exp_time + (direction < 0 ? -step_value : step_value));
        ui_camera_page.exp_time = ui_limit_value(value, min_value, max_value);
    }
    else if(UI_CAMERA_GAIN == ui_camera_selected)
    {
        value = (int16)(ui_camera_page.gain + (direction < 0 ? -step_value : step_value));
        ui_camera_page.gain = ui_limit_value(value, min_value, max_value);
    }

    ui_dirty = 1;
}

/* 调电机值 */
static void ui_adjust_motor_value(int8 direction)
{
    int16 min_value;
    int16 max_value;
    int16 step_value;
    int16 value;

    flash_get_motor_range(FLASH_MOTOR_TARGET_SPEED, &min_value, &max_value, &step_value);

    value = (int16)(ui_motor_page.target_speed + (direction < 0 ? -step_value : step_value));
    ui_motor_page.target_speed = ui_limit_value(value, min_value, max_value);
    ui_dirty = 1;
}

/* 保存相机值 */
static void ui_save_camera_value(void)
{
    flash_set_camera_value(FLASH_CAMERA_EXP_TIME, ui_camera_page.exp_time);
    flash_set_camera_value(FLASH_CAMERA_GAIN, ui_camera_page.gain);
    ui_editing = 0;
    ui_dirty = 1;
}

/* 保存电机值 */
static void ui_save_motor_value(void)
{
    flash_set_motor_value(FLASH_MOTOR_TARGET_SPEED, ui_motor_page.target_speed);
    ui_editing = 0;
    ui_dirty = 1;
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
    ui_draw_title("Camera");

    ui_draw_value_row(0, ui_camera_name[0], ui_camera_page.exp_time, (0 == ui_camera_selected) ? 1 : 0);
    ui_draw_value_row(1, ui_camera_name[1], ui_camera_page.gain, (1 == ui_camera_selected) ? 1 : 0);
}

/* 电机参数界面 */
static void ui_draw_motor_param_page(void)
{
    ui_draw_title("Motor");

    ui_draw_value_row(0, ui_motor_name[0], ui_motor_page.target_speed, 1);
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
    ui_draw_title("Camera view");
    ips200_set_color(RGB565_BLACK, RGB565_WHITE);
    ips200_show_string(0, UI_LIST_Y, "not ready");
}

/* 重画 */
static void ui_render(void)
{
    ips200_clear(RGB565_WHITE);

    switch(ui_page)
    {
        case UI_PAGE_ROOT:
            ui_draw_root_page();
            break;
        case UI_PAGE_CAMERA_VIEW:
            ui_draw_camera_view_page();
            break;
        case UI_PAGE_PARAM_MENU:
            ui_draw_param_menu_page();
            break;
        case UI_PAGE_CAMERA_PARAM:
            ui_draw_camera_param_page();
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
        if(UI_PARAM_CAMERA == ui_param_selected)
        {
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
        ui_load_page_value();
        ui_dirty = 1;
        return;
    }

    if((UI_PAGE_CAMERA_PARAM == ui_page) || (UI_PAGE_MOTOR_PARAM == ui_page))
    {
        if(ui_editing)
        {
            if(UI_PAGE_CAMERA_PARAM == ui_page)
            {
                ui_save_camera_value();
            }
            else
            {
                ui_save_motor_value();
            }
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
        ui_restore_page_value();
        ui_editing = 0;
        ui_dirty = 1;
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
    ui_key_init();
    ui_screen_init();
    ui_load_page_value();
    ui_backup_page_value();
    ui_ready = 1;
    ui_dirty = 1;
    ui_render();
    ui_dirty = 0;
}

void ui_update(void)
{
    ui_event_t event;

    if(!ui_ready)
    {
        ui_init();
    }

    event = ui_key_update();
    ui_handle_event(event);

    if(ui_dirty)
    {
        ui_render();
        ui_dirty = 0;
    }
}
