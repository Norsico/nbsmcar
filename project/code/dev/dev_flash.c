#include "dev_flash.h"

#include "zf_device_mt9v03x.h"
#include "zf_driver_eeprom.h"

#define FLASH_STORE_ADDR                (0x0000)
#define FLASH_STORE_MAGIC               (0x4653)
#define FLASH_STORE_VERSION             (0x0003)
#define FLASH_STORE_LINE_KP_MAX_TENTH   (100)
#define FLASH_STORE_LINE_KD_MAX_TENTH   (100)
/* 先占用用户 EEPROM 的第一个扇区，后面扩参数也从这里接着走。 */

typedef struct
{
    uint16 magic;
    uint16 version;
    flash_store_data_t store_data;
    uint16 checksum;
} flash_store_image_t;

static flash_store_image_t g_flash_store_cache;
static uint8 g_flash_store_ready = 0;

static uint8 flash_store_steer_pd_value_in_range(flash_param_slot_t slot, int16 value_tenth)
{
    switch(slot)
    {
        case FLASH_PARAM_SLOT_FIRST:
            return (value_tenth >= FLASH_STEER_P_MIN_TENTH && value_tenth <= FLASH_STEER_P_MAX_TENTH) ? 1 : 0;
        case FLASH_PARAM_SLOT_SECOND:
            return (value_tenth >= FLASH_STEER_D_MIN_TENTH && value_tenth <= FLASH_STEER_D_MAX_TENTH) ? 1 : 0;
        default:
            return 0;
    }
}

static uint8 flash_store_camera_value_in_range(flash_camera_slot_t slot, uint16 value)
{
    switch(slot)
    {
        case FLASH_CAMERA_SLOT_AUTO_EXP:
            return (value >= FLASH_CAMERA_AUTO_EXP_MIN && value <= FLASH_CAMERA_AUTO_EXP_MAX) ? 1 : 0;
        case FLASH_CAMERA_SLOT_EXP_TIME:
            return (value >= FLASH_CAMERA_EXP_TIME_MIN && value <= FLASH_CAMERA_EXP_TIME_MAX) ? 1 : 0;
        case FLASH_CAMERA_SLOT_GAIN:
            return (value >= FLASH_CAMERA_GAIN_MIN && value <= FLASH_CAMERA_GAIN_MAX) ? 1 : 0;
        default:
            return 0;
    }
}

static uint8 flash_store_start_speed_in_range(uint16 value)
{
    return (value >= FLASH_START_SPEED_MIN && value <= FLASH_START_SPEED_MAX) ? 1 : 0;
}

static uint8 flash_store_start_enable_in_range(uint8 value)
{
    return (value >= FLASH_START_ENABLE_MIN && value <= FLASH_START_ENABLE_MAX) ? 1 : 0;
}

/* 这里先把 Line Tune 页做一层基础校验，防止异常页直接带进 UI。 */
static uint8 flash_store_line_tune_page_is_valid(const flash_line_tune_page_t *page)
{
    if(0 == page)
    {
        return 0;
    }

    if(page->kp_tenth > FLASH_STORE_LINE_KP_MAX_TENTH)
    {
        return 0;
    }

    if(page->kd_tenth > FLASH_STORE_LINE_KD_MAX_TENTH)
    {
        return 0;
    }

    if((page->servo_min_angle < FLASH_SERVO_LIMIT_ANGLE_MIN) ||
       (page->servo_min_angle > FLASH_SERVO_LIMIT_ANGLE_MAX))
    {
        return 0;
    }

    if((page->servo_max_angle < FLASH_SERVO_LIMIT_ANGLE_MIN) ||
       (page->servo_max_angle > FLASH_SERVO_LIMIT_ANGLE_MAX))
    {
        return 0;
    }

    if(page->servo_min_angle >= page->servo_max_angle)
    {
        return 0;
    }

    return 1;
}

/* 简单做个校验和，足够判断这块数据是不是乱了。 */
static uint16 flash_store_calc_checksum(const flash_store_image_t *image)
{
    const uint8 *raw_ptr = 0;
    uint16 checksum = 0;
    uint16 i = 0;

    raw_ptr = (const uint8 *)image;
    for(i = 0; i < (uint16)(sizeof(flash_store_image_t) - sizeof(image->checksum)); i++)
    {
        checksum = (uint16)(checksum + raw_ptr[i]);
    }

    return checksum;
}

/* 默认值先收在一起，后面接正式参数时直接从这里改。 */
static void flash_store_fill_default_data(flash_store_data_t *store_ptr)
{
    memset(store_ptr, 0, sizeof(*store_ptr));
    store_ptr->param_page.first_value_tenth = FLASH_STEER_P_DEFAULT_TENTH;
    store_ptr->param_page.second_value_tenth = FLASH_STEER_D_DEFAULT_TENTH;
    store_ptr->camera_page.auto_exp = MT9V03X_AUTO_EXP_DEF;
    store_ptr->camera_page.exp_time = 75;
    store_ptr->camera_page.gain = 36;
    store_ptr->line_tune_page.kp_tenth = FLASH_LINE_KP_DEFAULT_TENTH;
    store_ptr->line_tune_page.kd_tenth = FLASH_LINE_KD_DEFAULT_TENTH;
    store_ptr->line_tune_page.servo_min_angle = FLASH_LINE_SERVO_MIN_DEFAULT;
    store_ptr->line_tune_page.servo_max_angle = FLASH_LINE_SERVO_MAX_DEFAULT;
    store_ptr->start_page.target_speed = FLASH_START_SPEED_DEFAULT;
    store_ptr->start_page.enable = FLASH_START_ENABLE_DEFAULT;
    store_ptr->start_page.reserved = 0;
}

/* 默认镜像长什么样，也统一收在这里。 */
static void flash_store_fill_default_image(flash_store_image_t *image)
{
    memset(image, 0, sizeof(*image));
    image->magic = FLASH_STORE_MAGIC;
    image->version = FLASH_STORE_VERSION;
    flash_store_fill_default_data(&image->store_data);
    image->checksum = flash_store_calc_checksum(image);
}

/* 现在先检查已经接进来的这一组参数。 */
static uint8 flash_store_data_is_valid(const flash_store_data_t *store_ptr)
{
    if(!flash_store_steer_pd_value_in_range(FLASH_PARAM_SLOT_FIRST, store_ptr->param_page.first_value_tenth))
    {
        return 0;
    }

    if(!flash_store_steer_pd_value_in_range(FLASH_PARAM_SLOT_SECOND, store_ptr->param_page.second_value_tenth))
    {
        return 0;
    }

    if(!flash_store_camera_value_in_range(FLASH_CAMERA_SLOT_AUTO_EXP, store_ptr->camera_page.auto_exp))
    {
        return 0;
    }

    if(!flash_store_camera_value_in_range(FLASH_CAMERA_SLOT_EXP_TIME, store_ptr->camera_page.exp_time))
    {
        return 0;
    }

    if(!flash_store_camera_value_in_range(FLASH_CAMERA_SLOT_GAIN, store_ptr->camera_page.gain))
    {
        return 0;
    }

    if(!flash_store_line_tune_page_is_valid(&store_ptr->line_tune_page))
    {
        return 0;
    }

    if(!flash_store_start_speed_in_range(store_ptr->start_page.target_speed))
    {
        return 0;
    }

    if(!flash_store_start_enable_in_range(store_ptr->start_page.enable))
    {
        return 0;
    }

    return 1;
}

/* 上电读出内容后，先确认头、版本和参数值都还是正常的。 */
static uint8 flash_store_image_is_valid(const flash_store_image_t *image)
{
    if(FLASH_STORE_MAGIC != image->magic)
    {
        return 0;
    }

    if(FLASH_STORE_VERSION != image->version)
    {
        return 0;
    }

    if(!flash_store_data_is_valid(&image->store_data))
    {
        return 0;
    }

    if(flash_store_calc_checksum(image) != image->checksum)
    {
        return 0;
    }

    return 1;
}

/* 每次访问 IAP 前都先把接口打开。 */
static void flash_store_hw_begin(void)
{
    iap_init();
}

/* IAP 用完就关掉，别一直挂着。 */
static void flash_store_hw_end(void)
{
    iap_idle();
}

/* 把 RAM 里的参数镜像整体写回去。 */
static void flash_store_save_cache(void)
{
    g_flash_store_cache.magic = FLASH_STORE_MAGIC;
    g_flash_store_cache.version = FLASH_STORE_VERSION;
    g_flash_store_cache.checksum = flash_store_calc_checksum(&g_flash_store_cache);

    flash_store_hw_begin();
    extern_iap_write_buff(FLASH_STORE_ADDR, (uint8 *)&g_flash_store_cache, sizeof(g_flash_store_cache));
    flash_store_hw_end();
}

/* 上电先读一份缓存，读坏了就按默认值重建。 */
static void flash_store_load_cache(void)
{
    flash_store_image_t image;

    flash_store_hw_begin();
    iap_read_buff(FLASH_STORE_ADDR, (uint8 *)&image, sizeof(image));
    flash_store_hw_end();

    if(flash_store_image_is_valid(&image))
    {
        memcpy(&g_flash_store_cache, &image, sizeof(g_flash_store_cache));
    }
    else
    {
        flash_store_fill_default_image(&g_flash_store_cache);
        flash_store_save_cache();
    }

    g_flash_store_ready = 1;
}

void flash_store_init(void)
{
    flash_store_load_cache();
}

void flash_store_get_data(flash_store_data_t *store_ptr)
{
    if(0 == g_flash_store_ready)
    {
        flash_store_init();
    }

    if(0 == store_ptr)
    {
        return;
    }

    memcpy(store_ptr, &g_flash_store_cache.store_data, sizeof(*store_ptr));
}

uint8 flash_store_set_data(const flash_store_data_t *store_ptr)
{
    if(0 == g_flash_store_ready)
    {
        flash_store_init();
    }

    if(0 == store_ptr)
    {
        return 0;
    }

    if(!flash_store_data_is_valid(store_ptr))
    {
        return 0;
    }

    memcpy(&g_flash_store_cache.store_data, store_ptr, sizeof(g_flash_store_cache.store_data));
    flash_store_save_cache();
    return 1;
}

void flash_store_get_param_page(flash_param_page_t *page)
{
    if(0 == g_flash_store_ready)
    {
        flash_store_init();
    }

    if(0 == page)
    {
        return;
    }

    memcpy(page, &g_flash_store_cache.store_data.param_page, sizeof(*page));
}

int16 flash_store_get_param_value_tenth(flash_param_slot_t slot)
{
    if(0 == g_flash_store_ready)
    {
        flash_store_init();
    }

    switch(slot)
    {
        case FLASH_PARAM_SLOT_FIRST:
            return g_flash_store_cache.store_data.param_page.first_value_tenth;
        case FLASH_PARAM_SLOT_SECOND:
            return g_flash_store_cache.store_data.param_page.second_value_tenth;
        default:
            return 0;
    }
}

uint8 flash_store_set_param_value_tenth(flash_param_slot_t slot, int16 value_tenth)
{
    int16 *target_value = 0;

    if(0 == g_flash_store_ready)
    {
        flash_store_init();
    }

    switch(slot)
    {
        case FLASH_PARAM_SLOT_FIRST:
            if(!flash_store_steer_pd_value_in_range(slot, value_tenth))
            {
                return 0;
            }
            target_value = &g_flash_store_cache.store_data.param_page.first_value_tenth;
            break;
        case FLASH_PARAM_SLOT_SECOND:
            if(!flash_store_steer_pd_value_in_range(slot, value_tenth))
            {
                return 0;
            }
            target_value = &g_flash_store_cache.store_data.param_page.second_value_tenth;
            break;
        default:
            return 0;
    }

    if(value_tenth == *target_value)
    {
        return 1;
    }

    *target_value = value_tenth;
    flash_store_save_cache();
    return 1;
}

void flash_store_get_camera_page(flash_camera_page_t *page)
{
    if(0 == g_flash_store_ready)
    {
        flash_store_init();
    }

    if(0 == page)
    {
        return;
    }

    memcpy(page, &g_flash_store_cache.store_data.camera_page, sizeof(*page));
}

uint8 flash_store_set_camera_page(const flash_camera_page_t *page)
{
    flash_store_data_t new_store_data;

    if(0 == g_flash_store_ready)
    {
        flash_store_init();
    }

    if(0 == page)
    {
        return 0;
    }

    if(!flash_store_camera_value_in_range(FLASH_CAMERA_SLOT_AUTO_EXP, page->auto_exp))
    {
        return 0;
    }

    if(!flash_store_camera_value_in_range(FLASH_CAMERA_SLOT_EXP_TIME, page->exp_time))
    {
        return 0;
    }

    if(!flash_store_camera_value_in_range(FLASH_CAMERA_SLOT_GAIN, page->gain))
    {
        return 0;
    }

    memcpy(&new_store_data, &g_flash_store_cache.store_data, sizeof(new_store_data));
    memcpy(&new_store_data.camera_page, page, sizeof(*page));
    memcpy(&g_flash_store_cache.store_data, &new_store_data, sizeof(g_flash_store_cache.store_data));
    flash_store_save_cache();
    return 1;
}

uint16 flash_store_get_camera_value(flash_camera_slot_t slot)
{
    if(0 == g_flash_store_ready)
    {
        flash_store_init();
    }

    switch(slot)
    {
        case FLASH_CAMERA_SLOT_AUTO_EXP:
            return g_flash_store_cache.store_data.camera_page.auto_exp;
        case FLASH_CAMERA_SLOT_EXP_TIME:
            return g_flash_store_cache.store_data.camera_page.exp_time;
        case FLASH_CAMERA_SLOT_GAIN:
            return g_flash_store_cache.store_data.camera_page.gain;
        default:
            return 0;
    }
}

uint8 flash_store_set_camera_value(flash_camera_slot_t slot, uint16 value)
{
    flash_camera_page_t page;

    if(0 == g_flash_store_ready)
    {
        flash_store_init();
    }

    if(!flash_store_camera_value_in_range(slot, value))
    {
        return 0;
    }

    memcpy(&page, &g_flash_store_cache.store_data.camera_page, sizeof(page));

    switch(slot)
    {
        case FLASH_CAMERA_SLOT_AUTO_EXP:
            if(value == page.auto_exp)
            {
                return 1;
            }
            page.auto_exp = (uint8)value;
            break;
        case FLASH_CAMERA_SLOT_EXP_TIME:
            if(value == page.exp_time)
            {
                return 1;
            }
            page.exp_time = value;
            break;
        case FLASH_CAMERA_SLOT_GAIN:
            if(value == page.gain)
            {
                return 1;
            }
            page.gain = (uint8)value;
            break;
        default:
            return 0;
    }

    return flash_store_set_camera_page(&page);
}

void flash_store_get_line_tune_page(flash_line_tune_page_t *page)
{
    if(0 == g_flash_store_ready)
    {
        flash_store_init();
    }

    if(0 == page)
    {
        return;
    }

    memcpy(page, &g_flash_store_cache.store_data.line_tune_page, sizeof(*page));
}

/* 巡线调参这一页整体覆盖，保持和 Camera 页一样的“改完立即落盘”语义。 */
uint8 flash_store_set_line_tune_page(const flash_line_tune_page_t *page)
{
    if(0 == g_flash_store_ready)
    {
        flash_store_init();
    }

    if(0 == page)
    {
        return 0;
    }

    if(!flash_store_line_tune_page_is_valid(page))
    {
        return 0;
    }

    memcpy(&g_flash_store_cache.store_data.line_tune_page, page, sizeof(*page));
    flash_store_save_cache();
    return 1;
}

void flash_store_get_start_page(flash_start_page_t *page)
{
    if(0 == g_flash_store_ready)
    {
        flash_store_init();
    }

    if(0 == page)
    {
        return;
    }

    memcpy(page, &g_flash_store_cache.store_data.start_page, sizeof(*page));
}

uint8 flash_store_set_start_page(const flash_start_page_t *page)
{
    if(0 == g_flash_store_ready)
    {
        flash_store_init();
    }

    if(0 == page)
    {
        return 0;
    }

    if(!flash_store_start_speed_in_range(page->target_speed))
    {
        return 0;
    }

    if(!flash_store_start_enable_in_range(page->enable))
    {
        return 0;
    }

    memcpy(&g_flash_store_cache.store_data.start_page, page, sizeof(*page));
    flash_store_save_cache();
    return 1;
}

void flash_store_reset_data(void)
{
    if(0 == g_flash_store_ready)
    {
        flash_store_init();
    }

    flash_store_fill_default_data(&g_flash_store_cache.store_data);
    flash_store_save_cache();
}
