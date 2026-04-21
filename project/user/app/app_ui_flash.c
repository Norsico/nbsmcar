/*
 * app_ui_flash.c - UI 参数页数据入口
 */

#include "app_ui_flash.h"

#include "app_line.h"
#include "dev_servo.h"
#include "dev_wheel.h"

static uint8 g_ui_flash_ready = 0;
static flash_start_page_t g_ui_start_page = {0, 0, 0};

static void ui_flash_fill_default_steer_pd_page(flash_param_page_t *page)
{
    if(0 == page)
    {
        return;
    }

    page->first_value = (int16)FlashSteerPConfig.default_value;
    page->second_value = (int16)FlashSteerDConfig.default_value;
    page->third_value = (int16)FlashSteerErr2Config.default_value;
    page->fourth_value = (int16)FlashSteerImuDConfig.default_value;
}

static uint8 ui_flash_steer_pd_page_is_valid(const flash_param_page_t *page)
{
    if(0 == page)
    {
        return 0;
    }

    if((page->first_value < (int16)FlashSteerPConfig.min) ||
       (page->first_value > (int16)FlashSteerPConfig.max))
    {
        return 0;
    }

    if((page->second_value < (int16)FlashSteerDConfig.min) ||
       (page->second_value > (int16)FlashSteerDConfig.max))
    {
        return 0;
    }

    if((page->third_value < (int16)FlashSteerErr2Config.min) ||
       (page->third_value > (int16)FlashSteerErr2Config.max))
    {
        return 0;
    }

    if((page->fourth_value < (int16)FlashSteerImuDConfig.min) ||
       (page->fourth_value > (int16)FlashSteerImuDConfig.max))
    {
        return 0;
    }

    return 1;
}

static void ui_flash_apply_servo_limit_page(const flash_servo_limit_page_t *page)
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

    page->target_speed = FlashStartSpeedConfig.default_value;
    page->enable = (uint8)FlashStartEnableConfig.default_value;
    page->reserved = 0;
}

static void ui_flash_normalize_start_page(flash_start_page_t *page)
{
    if(0 == page)
    {
        return;
    }

    if(page->target_speed > FlashStartSpeedConfig.max)
    {
        page->target_speed = FlashStartSpeedConfig.max;
    }
    page->enable = 0;
    page->reserved = 0;
}

static uint8 ui_flash_start_page_is_valid(const flash_start_page_t *page)
{
    if(0 == page)
    {
        return 0;
    }

    if(page->target_speed < FlashStartSpeedConfig.min ||
       page->target_speed > FlashStartSpeedConfig.max)
    {
        return 0;
    }

    return 1;
}

static void ui_flash_apply_start_page(const flash_start_page_t *page)
{
    if(0 == page)
    {
        return;
    }

    g_ui_start_page.target_speed = page->target_speed;
    g_ui_start_page.enable = 0;
    g_ui_start_page.reserved = 0;
    ui_flash_normalize_start_page(&g_ui_start_page);

    /* Car Speed 页只同步目标速度和舵机中值。 */
    car_wheel_set_target(g_ui_start_page.target_speed);
    car_servo_set_center();
}

static void ui_flash_ensure_ready(void)
{
    /* 读写 Car Speed 页前先完成缓存恢复。 */
    if(!g_ui_flash_ready)
    {
        ui_flash_init();
    }
}

/* 上电恢复 UI 相关参数。 */
void ui_flash_init(void)
{
    flash_store_data_t store_data;
    flash_param_page_t param_page;
    flash_servo_limit_page_t servo_limit_page;
    flash_start_page_t page;

    flash_store_get_data(&store_data);
    param_page = store_data.param_page;
    if(!ui_flash_steer_pd_page_is_valid(&param_page))
    {
        ui_flash_fill_default_steer_pd_page(&store_data.param_page);
        flash_store_set_data(&store_data);
    }
    servo_limit_page = store_data.servo_limit_page;
    ui_flash_apply_servo_limit_page(&servo_limit_page);

    /* 校验 Car Speed 页存档。 */
    flash_store_get_start_page(&page);
    if(!ui_flash_start_page_is_valid(&page))
    {
        ui_flash_fill_default_start_page(&page);
        flash_store_set_start_page(&page);
    }

    ui_flash_apply_start_page(&page);
    g_ui_flash_ready = 1;
}

/* 读取 Steer PD 页面范围。 */
void ui_flash_get_steer_pd_range(flash_param_slot_t slot, uint16 *min_value, uint16 *max_value, uint16 *step_value)
{
    uint16 min_value_local = 0;
    uint16 max_value_local = 0;
    uint16 step_value_local = 0;

    switch(slot)
    {
        case FLASH_PARAM_SLOT_FIRST:
            min_value_local = FlashSteerPConfig.min;
            max_value_local = FlashSteerPConfig.max;
            step_value_local = FlashSteerPConfig.step;
            break;
        case FLASH_PARAM_SLOT_SECOND:
            min_value_local = FlashSteerDConfig.min;
            max_value_local = FlashSteerDConfig.max;
            step_value_local = FlashSteerDConfig.step;
            break;
        case FLASH_PARAM_SLOT_THIRD:
            min_value_local = FlashSteerErr2Config.min;
            max_value_local = FlashSteerErr2Config.max;
            step_value_local = FlashSteerErr2Config.step;
            break;
        case FLASH_PARAM_SLOT_FOURTH:
            min_value_local = FlashSteerImuDConfig.min;
            max_value_local = FlashSteerImuDConfig.max;
            step_value_local = FlashSteerImuDConfig.step;
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

/* 读取 Camera 页面范围。 */
void ui_flash_get_camera_range(flash_camera_slot_t slot, uint16 *min_value, uint16 *max_value, uint16 *step_value)
{
    uint16 min_value_local = 0;
    uint16 max_value_local = 0;
    uint16 step_value_local = 0;

    /* 返回 Camera 页范围和步进。 */
    switch(slot)
    {
        case FLASH_CAMERA_SLOT_EXP_TIME:
            min_value_local = FlashCameraExpTimeConfig.min;
            max_value_local = FlashCameraExpTimeConfig.max;
            step_value_local = FlashCameraExpTimeConfig.step;
            break;
        case FLASH_CAMERA_SLOT_GAIN:
            min_value_local = FlashCameraGainConfig.min;
            max_value_local = FlashCameraGainConfig.max;
            step_value_local = FlashCameraGainConfig.step;
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

uint16 ui_flash_get_steer_pd_value(flash_param_slot_t slot)
{
    return (uint16)flash_store_get_param_value(slot);
}

void ui_flash_get_servo_limit_range(uint16 *min_value, uint16 *max_value, uint16 *step_value)
{
    if(0 != min_value)
    {
        *min_value = FlashServoMinAngleConfig.min;
    }

    if(0 != max_value)
    {
        *max_value = FlashServoMinAngleConfig.max;
    }

    if(0 != step_value)
    {
        *step_value = FlashServoMinAngleConfig.step;
    }
}

uint16 ui_flash_get_servo_limit_min_value(void)
{
    flash_servo_limit_page_t page;

    flash_store_get_servo_limit_page(&page);
    return page.servo_min_angle;
}

uint16 ui_flash_get_servo_limit_max_value(void)
{
    flash_servo_limit_page_t page;

    flash_store_get_servo_limit_page(&page);
    return page.servo_max_angle;
}

/* 调整 Camera 页参数。 */
uint8 ui_flash_adjust_camera_value(flash_camera_slot_t slot, int16 delta)
{
    uint16 current_value = 0;
    uint16 min_value = 0;
    uint16 max_value = 0;
    int32 next_value = 0;

    /* UI 层先完成限幅。 */
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

/* 调整最小舵机限幅。 */
uint8 ui_flash_adjust_servo_limit_min_value(int16 delta)
{
    flash_servo_limit_page_t page;
    uint16 min_value = 0;
    uint16 max_value = 0;
    int32 next_value = 0;

    flash_store_get_servo_limit_page(&page);
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
        next_value = (int32)page.servo_max_angle -
                     (int32)FlashServoMinAngleConfig.step;
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
    return flash_store_set_servo_limit_page(&page);
}

/* 调整最大舵机限幅。 */
uint8 ui_flash_adjust_servo_limit_max_value(int16 delta)
{
    flash_servo_limit_page_t page;
    uint16 min_value = 0;
    uint16 max_value = 0;
    int32 next_value = 0;

    flash_store_get_servo_limit_page(&page);
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
        next_value = (int32)page.servo_min_angle +
                     (int32)FlashServoMaxAngleConfig.step;
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
    return flash_store_set_servo_limit_page(&page);
}

/* 调整 Steer PD 参数。 */
uint8 ui_flash_adjust_steer_pd_value(flash_param_slot_t slot, int16 delta)
{
    uint16 current_value = 0;
    uint16 min_value = 0;
    uint16 max_value = 0;
    int32 next_value = 0;

    current_value = ui_flash_get_steer_pd_value(slot);
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

    return line_app_set_steer_pd_value(slot, (int16)next_value);
}

/* 读取 Car Speed 页缓存。 */
void ui_flash_get_start_page(flash_start_page_t *page)
{
    ui_flash_ensure_ready();

    if(0 == page)
    {
        return;
    }

    *page = g_ui_start_page;
}

/* 读取 Car Speed 页速度范围。 */
void ui_flash_get_start_speed_range(uint16 *min_value, uint16 *max_value, uint16 *step_value)
{
    if(0 != min_value)
    {
        *min_value = FlashStartSpeedConfig.min;
    }

    if(0 != max_value)
    {
        *max_value = FlashStartSpeedConfig.max;
    }

    if(0 != step_value)
    {
        *step_value = FlashStartSpeedConfig.step;
    }
}

/* 调整 Car Speed 页目标速度。 */
uint8 ui_flash_adjust_start_speed(int16 delta)
{
    flash_start_page_t page;
    int32 next_speed = 0;

    /* Car Speed 同时更新缓存和 flash。 */
    ui_flash_ensure_ready();
    page = g_ui_start_page;
    next_speed = (int32)page.target_speed + (int32)delta;

    if(next_speed < (int32)FlashStartSpeedConfig.min)
    {
        next_speed = (int32)FlashStartSpeedConfig.min;
    }
    else if(next_speed > (int32)FlashStartSpeedConfig.max)
    {
        next_speed = (int32)FlashStartSpeedConfig.max;
    }

    if((uint16)next_speed == page.target_speed)
    {
        return 0;
    }

    page.target_speed = (uint16)next_speed;
    ui_flash_apply_start_page(&page);
    return flash_store_set_start_page(&g_ui_start_page);
}
