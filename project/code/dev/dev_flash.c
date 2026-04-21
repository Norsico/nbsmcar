#include "dev_flash.h"

#include "zf_device_mt9v03x.h"
#include "zf_driver_eeprom.h"

#define FLASH_STORE_ADDR                (0x0000)
#define FLASH_STORE_MAGIC               (0x4653)
#define FLASH_STORE_VERSION             (0x0006)
/* 掉电参数写在用户 EEPROM 第一个扇区。 */

typedef struct
{
    uint16 magic;
    uint16 version;
    flash_store_data_t store_data;
    uint16 checksum;
} flash_store_image_t;

static flash_store_image_t g_flash_store_cache;
static uint8 g_flash_store_ready = 0;

/* 校验 Steer PD 存档范围。 */
static uint8 flash_store_steer_pd_value_in_range(flash_param_slot_t slot, int16 value)
{
    switch(slot)
    {
        case FLASH_PARAM_SLOT_FIRST:
            return (value >= (int16)FlashSteerPConfig.min &&
                    value <= (int16)FlashSteerPConfig.max) ? 1 : 0;
        case FLASH_PARAM_SLOT_SECOND:
            return (value >= (int16)FlashSteerDConfig.min &&
                    value <= (int16)FlashSteerDConfig.max) ? 1 : 0;
        case FLASH_PARAM_SLOT_THIRD:
            return (value >= (int16)FlashSteerErr2Config.min &&
                    value <= (int16)FlashSteerErr2Config.max) ? 1 : 0;
        case FLASH_PARAM_SLOT_FOURTH:
            return (value >= (int16)FlashSteerImuDConfig.min &&
                    value <= (int16)FlashSteerImuDConfig.max) ? 1 : 0;
        default:
            return 0;
    }
}

static uint8 flash_store_camera_value_in_range(flash_camera_slot_t slot, uint16 value)
{
    switch(slot)
    {
        case FLASH_CAMERA_SLOT_AUTO_EXP:
            return (value >= FlashCameraAutoExpConfig.min &&
                    value <= FlashCameraAutoExpConfig.max) ? 1 : 0;
        case FLASH_CAMERA_SLOT_EXP_TIME:
            return (value >= FlashCameraExpTimeConfig.min &&
                    value <= FlashCameraExpTimeConfig.max) ? 1 : 0;
        case FLASH_CAMERA_SLOT_GAIN:
            return (value >= FlashCameraGainConfig.min &&
                    value <= FlashCameraGainConfig.max) ? 1 : 0;
        default:
            return 0;
    }
}

static uint8 flash_store_start_speed_in_range(uint16 value)
{
    return (value >= FlashStartSpeedConfig.min &&
            value <= FlashStartSpeedConfig.max) ? 1 : 0;
}

static uint8 flash_store_start_enable_in_range(uint8 value)
{
    return (value >= FlashStartEnableConfig.min &&
            value <= FlashStartEnableConfig.max) ? 1 : 0;
}

/* 校验舵机限幅页。 */
static uint8 flash_store_servo_limit_page_is_valid(const flash_servo_limit_page_t *page)
{
    if(0 == page)
    {
        return 0;
    }

    if((page->servo_min_angle < FlashServoMinAngleConfig.min) ||
       (page->servo_min_angle > FlashServoMinAngleConfig.max))
    {
        return 0;
    }

    if((page->servo_max_angle < FlashServoMaxAngleConfig.min) ||
       (page->servo_max_angle > FlashServoMaxAngleConfig.max))
    {
        return 0;
    }

    if(page->servo_min_angle >= page->servo_max_angle)
    {
        return 0;
    }

    return 1;
}

/* 计算存档校验和。 */
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

/* 填充默认参数。 */
static void flash_store_fill_default_data(flash_store_data_t *store_ptr)
{
    memset(store_ptr, 0, sizeof(*store_ptr));
    store_ptr->param_page.first_value = (int16)FlashSteerPConfig.default_value;
    store_ptr->param_page.second_value = (int16)FlashSteerDConfig.default_value;
    store_ptr->param_page.third_value = (int16)FlashSteerErr2Config.default_value;
    store_ptr->param_page.fourth_value = (int16)FlashSteerImuDConfig.default_value;
    store_ptr->camera_page.auto_exp = (uint8)FlashCameraAutoExpConfig.default_value;
    store_ptr->camera_page.exp_time = FlashCameraExpTimeConfig.default_value;
    store_ptr->camera_page.gain = (uint8)FlashCameraGainConfig.default_value;
    store_ptr->servo_limit_page.servo_min_angle = (uint8)FlashServoMinAngleConfig.default_value;
    store_ptr->servo_limit_page.servo_max_angle = (uint8)FlashServoMaxAngleConfig.default_value;
    store_ptr->start_page.target_speed = FlashStartSpeedConfig.default_value;
    store_ptr->start_page.enable = (uint8)FlashStartEnableConfig.default_value;
    store_ptr->start_page.reserved = 0;
}

/* 生成默认存档镜像。 */
static void flash_store_fill_default_image(flash_store_image_t *image)
{
    memset(image, 0, sizeof(*image));
    image->magic = FLASH_STORE_MAGIC;
    image->version = FLASH_STORE_VERSION;
    flash_store_fill_default_data(&image->store_data);
    image->checksum = flash_store_calc_checksum(image);
}

/* 校验掉电参数内容。 */
static uint8 flash_store_data_is_valid(const flash_store_data_t *store_ptr)
{
    if(!flash_store_steer_pd_value_in_range(FLASH_PARAM_SLOT_FIRST, store_ptr->param_page.first_value))
    {
        return 0;
    }

    if(!flash_store_steer_pd_value_in_range(FLASH_PARAM_SLOT_SECOND, store_ptr->param_page.second_value))
    {
        return 0;
    }

    if(!flash_store_steer_pd_value_in_range(FLASH_PARAM_SLOT_THIRD, store_ptr->param_page.third_value))
    {
        return 0;
    }

    if(!flash_store_steer_pd_value_in_range(FLASH_PARAM_SLOT_FOURTH, store_ptr->param_page.fourth_value))
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

    if(!flash_store_servo_limit_page_is_valid(&store_ptr->servo_limit_page))
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

/* 校验整块掉电镜像。 */
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

/* 打开 IAP 接口。 */
static void flash_store_hw_begin(void)
{
    iap_init();
}

/* 关闭 IAP 接口。 */
static void flash_store_hw_end(void)
{
    iap_idle();
}

/* 回写掉电缓存。 */
static void flash_store_save_cache(void)
{
    g_flash_store_cache.magic = FLASH_STORE_MAGIC;
    g_flash_store_cache.version = FLASH_STORE_VERSION;
    g_flash_store_cache.checksum = flash_store_calc_checksum(&g_flash_store_cache);

    flash_store_hw_begin();
    extern_iap_write_buff(FLASH_STORE_ADDR, (uint8 *)&g_flash_store_cache, sizeof(g_flash_store_cache));
    flash_store_hw_end();
}

/* 载入掉电缓存，异常时重建默认值。 */
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

/* 初始化掉电参数缓存。 */
void flash_store_init(void)
{
    flash_store_load_cache();
}

/* 读取整块掉电参数。 */
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

/* 整体覆盖掉电参数。 */
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

/* 读取 Steer PD 页面。 */
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

int16 flash_store_get_param_value(flash_param_slot_t slot)
{
    if(0 == g_flash_store_ready)
    {
        flash_store_init();
    }

    switch(slot)
    {
        case FLASH_PARAM_SLOT_FIRST:
            return g_flash_store_cache.store_data.param_page.first_value;
        case FLASH_PARAM_SLOT_SECOND:
            return g_flash_store_cache.store_data.param_page.second_value;
        case FLASH_PARAM_SLOT_THIRD:
            return g_flash_store_cache.store_data.param_page.third_value;
        case FLASH_PARAM_SLOT_FOURTH:
            return g_flash_store_cache.store_data.param_page.fourth_value;
        default:
            return 0;
    }
}

/* 修改单个 Steer PD 参数并落盘。 */
uint8 flash_store_set_param_value(flash_param_slot_t slot, int16 value)
{
    int16 *target_value = 0;

    if(0 == g_flash_store_ready)
    {
        flash_store_init();
    }

    switch(slot)
    {
        case FLASH_PARAM_SLOT_FIRST:
            if(!flash_store_steer_pd_value_in_range(slot, value))
            {
                return 0;
            }
            target_value = &g_flash_store_cache.store_data.param_page.first_value;
            break;
        case FLASH_PARAM_SLOT_SECOND:
            if(!flash_store_steer_pd_value_in_range(slot, value))
            {
                return 0;
            }
            target_value = &g_flash_store_cache.store_data.param_page.second_value;
            break;
        case FLASH_PARAM_SLOT_THIRD:
            if(!flash_store_steer_pd_value_in_range(slot, value))
            {
                return 0;
            }
            target_value = &g_flash_store_cache.store_data.param_page.third_value;
            break;
        case FLASH_PARAM_SLOT_FOURTH:
            if(!flash_store_steer_pd_value_in_range(slot, value))
            {
                return 0;
            }
            target_value = &g_flash_store_cache.store_data.param_page.fourth_value;
            break;
        default:
            return 0;
    }

    if(value == *target_value)
    {
        return 1;
    }

    *target_value = value;
    flash_store_save_cache();
    return 1;
}

/* 读取摄像头参数页。 */
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

/* 整体覆盖摄像头参数页。 */
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

/* 读取舵机限幅页。 */
void flash_store_get_servo_limit_page(flash_servo_limit_page_t *page)
{
    if(0 == g_flash_store_ready)
    {
        flash_store_init();
    }

    if(0 == page)
    {
        return;
    }

    memcpy(page, &g_flash_store_cache.store_data.servo_limit_page, sizeof(*page));
}

/* 整体覆盖舵机限幅页。 */
uint8 flash_store_set_servo_limit_page(const flash_servo_limit_page_t *page)
{
    if(0 == g_flash_store_ready)
    {
        flash_store_init();
    }

    if(0 == page)
    {
        return 0;
    }

    if(!flash_store_servo_limit_page_is_valid(page))
    {
        return 0;
    }

    memcpy(&g_flash_store_cache.store_data.servo_limit_page, page, sizeof(*page));
    flash_store_save_cache();
    return 1;
}

/* 读取 Start 页。 */
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

/* 整体覆盖 Start 页。 */
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

/* 恢复默认掉电参数。 */
void flash_store_reset_data(void)
{
    if(0 == g_flash_store_ready)
    {
        flash_store_init();
    }

    flash_store_fill_default_data(&g_flash_store_cache.store_data);
    flash_store_save_cache();
}
