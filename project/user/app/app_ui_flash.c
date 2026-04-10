/*
 * app_ui_flash.c - UI 参数页数据入口
 */

#include "app_ui_flash.h"

#include "app_line.h"
#include "dev_servo.h"
#include "dev_wheel.h"
#include "system_state.h"

static uint8 g_ui_flash_ready = 0;
static flash_start_page_t g_ui_start_page = {FLASH_START_SPEED_DEFAULT, FLASH_START_ENABLE_DEFAULT, 0};

static void ui_flash_fill_default_steer_pd_page(flash_param_page_t *page)
{
    if(0 == page)
    {
        return;
    }

    page->first_value_tenth = FLASH_STEER_P_DEFAULT_TENTH;
    page->second_value_tenth = FLASH_STEER_D_DEFAULT_TENTH;
}

static uint8 ui_flash_steer_pd_page_is_valid(const flash_param_page_t *page)
{
    if(0 == page)
    {
        return 0;
    }

    if((page->first_value_tenth < FLASH_STEER_P_MIN_TENTH) ||
       (page->first_value_tenth > FLASH_STEER_P_MAX_TENTH))
    {
        return 0;
    }

    if((page->second_value_tenth < FLASH_STEER_D_MIN_TENTH) ||
       (page->second_value_tenth > FLASH_STEER_D_MAX_TENTH))
    {
        return 0;
    }

    /* 旧版本这两个槽位没用过，读到 0/0 时按新默认值迁过去。 */
    if((0 == page->first_value_tenth) && (0 == page->second_value_tenth))
    {
        return 0;
    }

    return 1;
}

static void ui_flash_apply_servo_limit_page(const flash_line_tune_page_t *page)
{
    if(0 == page)
    {
        return;
    }

    car_servo_set_limit(page->servo_min_angle, page->servo_max_angle);
}

static void ui_flash_fill_default_start_page(flash_start_page_t *page)
{
    if(0 == page)
    {
        return;
    }

    page->target_speed = FLASH_START_SPEED_DEFAULT;
    page->enable = FLASH_START_ENABLE_DEFAULT;
    page->reserved = 0;
}

static void ui_flash_normalize_start_page(flash_start_page_t *page)
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

static uint8 ui_flash_start_page_is_valid(const flash_start_page_t *page)
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

static void ui_flash_apply_start_page(const flash_start_page_t *page)
{
    uint8 previous_enable = 0;

    if(0 == page)
    {
        return;
    }

    previous_enable = g_ui_start_page.enable;
    g_ui_start_page.target_speed = page->target_speed;
    g_ui_start_page.enable = page->enable;
    g_ui_start_page.reserved = 0;
    ui_flash_normalize_start_page(&g_ui_start_page);

    /* Start 页不只是存档，还要直接驱动当前运行状态。 */
    if(g_ui_start_page.enable)
    {
        if(!previous_enable)
        {
            car_wheel_control_reset();
        }

        /* 启动页参数生效后，同步后轮目标并让舵机回中。 */
        car_wheel_set_target((float)g_ui_start_page.target_speed);
        car_servo_set_center();

        if(SYS_EMERGENCY != g_system_state)
        {
            g_system_state = SYS_RUNNING;
        }
    }
    else
    {
        /* 关闭启动开关时沿用旧逻辑，清空闭环并退回准备态。 */
        car_wheel_control_reset();
        if(SYS_EMERGENCY != g_system_state)
        {
            g_system_state = SYS_PREPARE;
        }
    }
}

static void ui_flash_ensure_ready(void)
{
    /* 外部直接读写 Start 页前，先保证缓存已经从 flash 恢复。 */
    if(!g_ui_flash_ready)
    {
        ui_flash_init();
    }
}

void ui_flash_init(void)
{
    flash_store_data_t store_data;
    flash_param_page_t param_page;
    flash_line_tune_page_t line_tune_page;
    flash_start_page_t page;

    flash_store_get_data(&store_data);
    param_page = store_data.param_page;
    if(!ui_flash_steer_pd_page_is_valid(&param_page))
    {
        ui_flash_fill_default_steer_pd_page(&store_data.param_page);
        flash_store_set_data(&store_data);
    }
    line_tune_page = store_data.line_tune_page;
    ui_flash_apply_servo_limit_page(&line_tune_page);

    /* 上电先校验启动页，异常值直接拉回默认配置。 */
    flash_store_get_start_page(&page);
    if(!ui_flash_start_page_is_valid(&page))
    {
        ui_flash_fill_default_start_page(&page);
        flash_store_set_start_page(&page);
    }

    ui_flash_apply_start_page(&page);
    g_ui_flash_ready = 1;
}

void ui_flash_get_steer_pd_range(flash_param_slot_t slot, uint16 *min_value, uint16 *max_value, uint16 *step_value)
{
    uint16 min_value_local = 0;
    uint16 max_value_local = 0;
    uint16 step_value_local = 0;

    switch(slot)
    {
        case FLASH_PARAM_SLOT_FIRST:
            min_value_local = FLASH_STEER_P_MIN_TENTH;
            max_value_local = FLASH_STEER_P_MAX_TENTH;
            step_value_local = FLASH_STEER_P_STEP_TENTH;
            break;
        case FLASH_PARAM_SLOT_SECOND:
            min_value_local = FLASH_STEER_D_MIN_TENTH;
            max_value_local = FLASH_STEER_D_MAX_TENTH;
            step_value_local = FLASH_STEER_D_STEP_TENTH;
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

void ui_flash_get_camera_range(flash_camera_slot_t slot, uint16 *min_value, uint16 *max_value, uint16 *step_value)
{
    uint16 min_value_local = 0;
    uint16 max_value_local = 0;
    uint16 step_value_local = 0;

    /* Camera 页显示范围和步进统一从这里出，避免页面层散着写。 */
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

uint16 ui_flash_get_camera_value(flash_camera_slot_t slot)
{
    return flash_store_get_camera_value(slot);
}

uint16 ui_flash_get_steer_pd_value_tenth(flash_param_slot_t slot)
{
    return (uint16)flash_store_get_param_value_tenth(slot);
}

void ui_flash_get_servo_limit_range(uint16 *min_value, uint16 *max_value, uint16 *step_value)
{
    if(0 != min_value)
    {
        *min_value = FLASH_SERVO_LIMIT_ANGLE_MIN;
    }

    if(0 != max_value)
    {
        *max_value = FLASH_SERVO_LIMIT_ANGLE_MAX;
    }

    if(0 != step_value)
    {
        *step_value = FLASH_SERVO_LIMIT_ANGLE_STEP;
    }
}

uint16 ui_flash_get_servo_limit_min_value(void)
{
    flash_line_tune_page_t page;

    flash_store_get_line_tune_page(&page);
    return page.servo_min_angle;
}

uint16 ui_flash_get_servo_limit_max_value(void)
{
    flash_line_tune_page_t page;

    flash_store_get_line_tune_page(&page);
    return page.servo_max_angle;
}

uint8 ui_flash_adjust_camera_value(flash_camera_slot_t slot, int16 delta)
{
    uint16 current_value = 0;
    uint16 min_value = 0;
    uint16 max_value = 0;
    int32 next_value = 0;

    /* 先做 UI 侧限幅，再交给相机应用层下发并落盘。 */
    current_value = flash_store_get_camera_value(slot);
    ui_flash_get_camera_range(slot, &min_value, &max_value, 0);
    next_value = (int32)current_value + (int32)delta;

    if(next_value < min_value)
    {
        next_value = min_value;
    }
    else if(next_value > max_value)
    {
        next_value = max_value;
    }

    if((uint16)next_value == current_value)
    {
        return 0;
    }

    return line_app_set_camera_param_value(slot, (uint16)next_value);
}

uint8 ui_flash_adjust_servo_limit_min_value(int16 delta)
{
    flash_line_tune_page_t page;
    uint16 min_value = 0;
    uint16 max_value = 0;
    int32 next_value = 0;

    flash_store_get_line_tune_page(&page);
    ui_flash_get_servo_limit_range(&min_value, &max_value, 0);
    next_value = (int32)page.servo_min_angle + (int32)delta;
    if(next_value < min_value)
    {
        next_value = min_value;
    }
    else if(next_value > max_value)
    {
        next_value = max_value;
    }
    if((uint16)next_value >= page.servo_max_angle)
    {
        next_value = (int32)page.servo_max_angle - FLASH_SERVO_LIMIT_ANGLE_STEP;
    }
    if(next_value < min_value)
    {
        next_value = min_value;
    }
    if((uint16)next_value == page.servo_min_angle)
    {
        return 0;
    }

    page.servo_min_angle = (uint8)next_value;
    ui_flash_apply_servo_limit_page(&page);
    return flash_store_set_line_tune_page(&page);
}

uint8 ui_flash_adjust_servo_limit_max_value(int16 delta)
{
    flash_line_tune_page_t page;
    uint16 min_value = 0;
    uint16 max_value = 0;
    int32 next_value = 0;

    flash_store_get_line_tune_page(&page);
    ui_flash_get_servo_limit_range(&min_value, &max_value, 0);
    next_value = (int32)page.servo_max_angle + (int32)delta;
    if(next_value < min_value)
    {
        next_value = min_value;
    }
    else if(next_value > max_value)
    {
        next_value = max_value;
    }
    if((uint16)next_value <= page.servo_min_angle)
    {
        next_value = (int32)page.servo_min_angle + FLASH_SERVO_LIMIT_ANGLE_STEP;
    }
    if(next_value > max_value)
    {
        next_value = max_value;
    }
    if((uint16)next_value == page.servo_max_angle)
    {
        return 0;
    }

    page.servo_max_angle = (uint8)next_value;
    ui_flash_apply_servo_limit_page(&page);
    return flash_store_set_line_tune_page(&page);
}

uint8 ui_flash_adjust_steer_pd_value_tenth(flash_param_slot_t slot, int16 delta)
{
    uint16 current_value = 0;
    uint16 min_value = 0;
    uint16 max_value = 0;
    int32 next_value = 0;

    current_value = ui_flash_get_steer_pd_value_tenth(slot);
    ui_flash_get_steer_pd_range(slot, &min_value, &max_value, 0);
    next_value = (int32)current_value + (int32)delta;

    if(next_value < min_value)
    {
        next_value = min_value;
    }
    else if(next_value > max_value)
    {
        next_value = max_value;
    }

    if((uint16)next_value == current_value)
    {
        return 0;
    }

    return line_app_set_steer_pd_value_tenth(slot, (int16)next_value);
}

void ui_flash_get_start_page(flash_start_page_t *page)
{
    ui_flash_ensure_ready();

    if(0 == page)
    {
        return;
    }

    *page = g_ui_start_page;
}

void ui_flash_get_start_speed_range(uint16 *min_value, uint16 *max_value, uint16 *step_value)
{
    if(0 != min_value)
    {
        *min_value = FLASH_START_SPEED_MIN;
    }

    if(0 != max_value)
    {
        *max_value = FLASH_START_SPEED_MAX;
    }

    if(0 != step_value)
    {
        *step_value = FLASH_START_SPEED_STEP;
    }
}

uint8 ui_flash_adjust_start_speed(int16 delta)
{
    flash_start_page_t page;
    int32 next_speed = 0;

    /* Start 页速度改动要同时更新 RAM 缓存、运行态和 flash。 */
    ui_flash_ensure_ready();
    page = g_ui_start_page;
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
        return 0;
    }

    page.target_speed = (uint16)next_speed;
    ui_flash_apply_start_page(&page);
    return flash_store_set_start_page(&g_ui_start_page);
}

uint8 ui_flash_toggle_start_enable(void)
{
    flash_start_page_t page;

    /* 启动开关切换后，沿用同一套应用逻辑同步系统状态。 */
    ui_flash_ensure_ready();
    page = g_ui_start_page;
    page.enable = (uint8)!page.enable;
    ui_flash_apply_start_page(&page);
    return flash_store_set_start_page(&g_ui_start_page);
}

uint8 ui_flash_start_is_enabled(void)
{
    ui_flash_ensure_ready();
    return g_ui_start_page.enable;
}
