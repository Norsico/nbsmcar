#include "flash.h"
#include "motor.h"
#include "servo.h"

typedef struct
{
    uint16 magic;                                               /* 标记 */
    uint16 version;                                             /* 版本 */
    uint8 active_plan;                                          /* 当前方案 */
    uint8 reserve;                                              /* 保留 */
    flash_plan_t plan[FLASH_PLAN_COUNT];                        /* 方案表 */
    uint16 checksum;                                            /* 校验 */
} flash_store_image_t;

static flash_store_image_t flash_store_image;
static uint8 flash_ready = 0;

static const flash_value_config_t flash_camera_config[FLASH_CAMERA_COUNT] =
{
    {FLASH_CAMERA_EXP_TIME_STEP},
    {FLASH_CAMERA_GAIN_STEP}
};

static const flash_value_config_t flash_servo_config[FLASH_SERVO_COUNT] =
{
    {FLASH_SERVO_P_STEP},
    {FLASH_SERVO_D_STEP},
    {FLASH_SERVO_ERR2_STEP},
    {FLASH_SERVO_ACKERMAN_STEP},
    {FLASH_SERVO_IMU_D_STEP},
    {FLASH_SERVO_TOW_POINT_STEP},
    {FLASH_SERVO_MIN_ANGLE_STEP},
    {FLASH_SERVO_MAX_ANGLE_STEP}
};

static const flash_value_config_t flash_motor_config[FLASH_MOTOR_COUNT] =
{
    {FLASH_MOTOR_TARGET_STEP}
};

/* 方案索引 */
static uint8 flash_plan_is_valid(uint8 plan_index)
{
    return (plan_index < FLASH_PLAN_COUNT) ? 1 : 0;
}

/* 相机限值 */
int16 flash_limit_camera_value(flash_camera_slot_t slot, int16 value)
{
    switch(slot)
    {
        case FLASH_CAMERA_EXP_TIME:
            return (value < FLASH_CAMERA_EXP_TIME_MIN) ? FLASH_CAMERA_EXP_TIME_MIN : value;
        case FLASH_CAMERA_GAIN:
            if(value < FLASH_CAMERA_GAIN_MIN)
            {
                return FLASH_CAMERA_GAIN_MIN;
            }
            if(value > FLASH_CAMERA_GAIN_MAX)
            {
                return FLASH_CAMERA_GAIN_MAX;
            }
            return value;
        default:
            return 0;
    }
}

/* 舵机限值 */
int16 flash_limit_servo_value(flash_servo_slot_t slot, int16 value)
{
    switch(slot)
    {
        case FLASH_SERVO_P:
            if(value < FLASH_SERVO_P_MIN)
            {
                return FLASH_SERVO_P_MIN;
            }
            if(value > FLASH_SERVO_P_MAX)
            {
                return FLASH_SERVO_P_MAX;
            }
            return value;
        case FLASH_SERVO_D:
            if(value < FLASH_SERVO_D_MIN)
            {
                return FLASH_SERVO_D_MIN;
            }
            if(value > FLASH_SERVO_D_MAX)
            {
                return FLASH_SERVO_D_MAX;
            }
            return value;
        case FLASH_SERVO_ERR2:
            if(value < FLASH_SERVO_ERR2_MIN)
            {
                return FLASH_SERVO_ERR2_MIN;
            }
            if(value > FLASH_SERVO_ERR2_MAX)
            {
                return FLASH_SERVO_ERR2_MAX;
            }
            return value;
        case FLASH_SERVO_ACKERMAN:
            if(value < FLASH_SERVO_ACKERMAN_MIN)
            {
                return FLASH_SERVO_ACKERMAN_MIN;
            }
            if(value > FLASH_SERVO_ACKERMAN_MAX)
            {
                return FLASH_SERVO_ACKERMAN_MAX;
            }
            return value;
        case FLASH_SERVO_IMU_D:
            if(value < FLASH_SERVO_IMU_D_MIN)
            {
                return FLASH_SERVO_IMU_D_MIN;
            }
            if(value > FLASH_SERVO_IMU_D_MAX)
            {
                return FLASH_SERVO_IMU_D_MAX;
            }
            return value;
        case FLASH_SERVO_TOW_POINT:
            if(value < FLASH_SERVO_TOW_POINT_MIN)
            {
                return FLASH_SERVO_TOW_POINT_MIN;
            }
            if(value > FLASH_SERVO_TOW_POINT_MAX)
            {
                return FLASH_SERVO_TOW_POINT_MAX;
            }
            return value;
        case FLASH_SERVO_MIN_ANGLE:
            if(value < FLASH_SERVO_MIN_ANGLE_MIN)
            {
                return FLASH_SERVO_MIN_ANGLE_MIN;
            }
            if(value > FLASH_SERVO_MIN_ANGLE_MAX)
            {
                return FLASH_SERVO_MIN_ANGLE_MAX;
            }
            return value;
        case FLASH_SERVO_MAX_ANGLE:
            if(value < FLASH_SERVO_MAX_ANGLE_MIN)
            {
                return FLASH_SERVO_MAX_ANGLE_MIN;
            }
            if(value > FLASH_SERVO_MAX_ANGLE_MAX)
            {
                return FLASH_SERVO_MAX_ANGLE_MAX;
            }
            return value;
        default:
            return 0;
    }
}

/* 电机限值 */
int16 flash_limit_motor_value(flash_motor_slot_t slot, int16 value)
{
    switch(slot)
    {
        case FLASH_MOTOR_TARGET_SPEED:
            return (value < 0) ? 0 : value;
        default:
            return 0;
    }
}

/* 相机参数校验 */
static uint8 flash_camera_value_is_valid(flash_camera_slot_t slot, int16 value)
{
    if(slot >= FLASH_CAMERA_COUNT)
    {
        return 0;
    }

    return (value == flash_limit_camera_value(slot, value)) ? 1 : 0;
}

/* 舵机参数校验 */
static uint8 flash_servo_value_is_valid(flash_servo_slot_t slot, int16 value)
{
    if(slot >= FLASH_SERVO_COUNT)
    {
        return 0;
    }

    return (value == flash_limit_servo_value(slot, value)) ? 1 : 0;
}

/* 电机参数校验 */
static uint8 flash_motor_value_is_valid(flash_motor_slot_t slot, int16 value)
{
    if(slot >= FLASH_MOTOR_COUNT)
    {
        return 0;
    }

    return (value == flash_limit_motor_value(slot, value)) ? 1 : 0;
}

/* 相机存档校验 */
static uint8 flash_camera_store_value_is_valid(flash_camera_slot_t slot, int16 value)
{
    switch(slot)
    {
        case FLASH_CAMERA_EXP_TIME:
            return (value >= 0) ? 1 : 0;
        case FLASH_CAMERA_GAIN:
            if(value < FLASH_CAMERA_GAIN_MIN)
            {
                return 0;
            }
            if(value > FLASH_CAMERA_GAIN_MAX)
            {
                return 0;
            }
            return 1;
        default:
            return 0;
    }
}

/* 相机页校验 */
static uint8 flash_camera_page_is_valid(const flash_camera_page_t *page)
{
    if(0 == page)
    {
        return 0;
    }

    if(!flash_camera_store_value_is_valid(FLASH_CAMERA_EXP_TIME, page->exp_time))
    {
        return 0;
    }

    if(!flash_camera_store_value_is_valid(FLASH_CAMERA_GAIN, page->gain))
    {
        return 0;
    }

    return 1;
}

/* 舵机页校验 */
static uint8 flash_servo_page_is_valid(const flash_servo_page_t *page)
{
    if(0 == page)
    {
        return 0;
    }

    if(!flash_servo_value_is_valid(FLASH_SERVO_P, page->steer_p))
    {
        return 0;
    }
    if(!flash_servo_value_is_valid(FLASH_SERVO_D, page->steer_d))
    {
        return 0;
    }
    if(!flash_servo_value_is_valid(FLASH_SERVO_ERR2, page->err2_k))
    {
        return 0;
    }
    if(!flash_servo_value_is_valid(FLASH_SERVO_ACKERMAN, page->ackerman))
    {
        return 0;
    }
    if(!flash_servo_value_is_valid(FLASH_SERVO_IMU_D, page->imu_d))
    {
        return 0;
    }
    if(!flash_servo_value_is_valid(FLASH_SERVO_TOW_POINT, page->tow_point))
    {
        return 0;
    }
    if(!flash_servo_value_is_valid(FLASH_SERVO_MIN_ANGLE, page->servo_min_angle))
    {
        return 0;
    }
    if(!flash_servo_value_is_valid(FLASH_SERVO_MAX_ANGLE, page->servo_max_angle))
    {
        return 0;
    }
    if(page->servo_min_angle >= page->servo_max_angle)
    {
        return 0;
    }

    return 1;
}

/* 电机页校验 */
static uint8 flash_motor_page_is_valid(const flash_motor_page_t *page)
{
    if(0 == page)
    {
        return 0;
    }

    if(!flash_motor_value_is_valid(FLASH_MOTOR_TARGET_SPEED, page->target_speed))
    {
        return 0;
    }

    return 1;
}

/* 方案校验 */
static uint8 flash_plan_data_is_valid(const flash_plan_t *plan)
{
    if(0 == plan)
    {
        return 0;
    }

    if(!flash_camera_page_is_valid(&plan->camera_page))
    {
        return 0;
    }
    if(!flash_servo_page_is_valid(&plan->servo_page))
    {
        return 0;
    }
    if(!flash_motor_page_is_valid(&plan->motor_page))
    {
        return 0;
    }

    return 1;
}

/* 校验和 */
static uint16 flash_calc_checksum(const flash_store_image_t *image)
{
    uint8 *data_ptr;
    uint16 checksum;
    uint16 i;

    data_ptr = (uint8 *)image;
    checksum = 0;

    for(i = 0; i < (uint16)(sizeof(flash_store_image_t) - sizeof(image->checksum)); i++)
    {
        checksum = (uint16)(checksum + data_ptr[i]);
    }

    return checksum;
}

/* 相机页修正 */
static uint8 flash_normalize_camera_page(flash_camera_page_t *page)
{
    uint8 changed;

    changed = 0;

    if(0 == page)
    {
        return 0;
    }

    if(page->exp_time < FLASH_CAMERA_EXP_TIME_MIN)
    {
        page->exp_time = FLASH_CAMERA_EXP_TIME_DEFAULT;
        changed = 1;
    }
    if(page->gain < FLASH_CAMERA_GAIN_MIN || page->gain > FLASH_CAMERA_GAIN_MAX)
    {
        page->gain = FLASH_CAMERA_GAIN_DEFAULT;
        changed = 1;
    }

    return changed;
}

/* 舵机页修正 */
static uint8 flash_normalize_servo_page(flash_servo_page_t *page)
{
    uint8 changed;

    changed = 0;

    if(0 == page)
    {
        return 0;
    }

    if(page->steer_p != flash_limit_servo_value(FLASH_SERVO_P, page->steer_p))
    {
        page->steer_p = FLASH_SERVO_P_DEFAULT;
        changed = 1;
    }
    if(page->steer_d != flash_limit_servo_value(FLASH_SERVO_D, page->steer_d))
    {
        page->steer_d = FLASH_SERVO_D_DEFAULT;
        changed = 1;
    }
    if(page->err2_k != flash_limit_servo_value(FLASH_SERVO_ERR2, page->err2_k))
    {
        page->err2_k = FLASH_SERVO_ERR2_DEFAULT;
        changed = 1;
    }
    if(page->ackerman != flash_limit_servo_value(FLASH_SERVO_ACKERMAN, page->ackerman))
    {
        page->ackerman = FLASH_SERVO_ACKERMAN_DEFAULT;
        changed = 1;
    }
    if(page->imu_d != flash_limit_servo_value(FLASH_SERVO_IMU_D, page->imu_d))
    {
        page->imu_d = FLASH_SERVO_IMU_D_DEFAULT;
        changed = 1;
    }
    if(page->tow_point != flash_limit_servo_value(FLASH_SERVO_TOW_POINT, page->tow_point))
    {
        page->tow_point = FLASH_SERVO_TOW_POINT_DEFAULT;
        changed = 1;
    }
    if(page->servo_min_angle != flash_limit_servo_value(FLASH_SERVO_MIN_ANGLE, page->servo_min_angle))
    {
        page->servo_min_angle = FLASH_SERVO_MIN_ANGLE_DEFAULT;
        changed = 1;
    }
    if(page->servo_max_angle != flash_limit_servo_value(FLASH_SERVO_MAX_ANGLE, page->servo_max_angle))
    {
        page->servo_max_angle = FLASH_SERVO_MAX_ANGLE_DEFAULT;
        changed = 1;
    }
    if(page->servo_min_angle >= page->servo_max_angle)
    {
        page->servo_min_angle = FLASH_SERVO_MIN_ANGLE_DEFAULT;
        page->servo_max_angle = FLASH_SERVO_MAX_ANGLE_DEFAULT;
        changed = 1;
    }

    return changed;
}

/* 电机页修正 */
static uint8 flash_normalize_motor_page(flash_motor_page_t *page)
{
    if(0 == page)
    {
        return 0;
    }

    if(page->target_speed < 0)
    {
        page->target_speed = 0;
        return 1;
    }

    return 0;
}

/* 方案修正 */
static uint8 flash_normalize_plan(flash_plan_t *plan)
{
    uint8 changed;

    changed = 0;

    if(0 == plan)
    {
        return 0;
    }

    if(flash_normalize_camera_page(&plan->camera_page))
    {
        changed = 1;
    }
    if(flash_normalize_servo_page(&plan->servo_page))
    {
        changed = 1;
    }
    if(flash_normalize_motor_page(&plan->motor_page))
    {
        changed = 1;
    }

    return changed;
}

/* 方案1默认值 */
static void flash_fill_plan0(flash_plan_t *plan)
{
    plan->camera_page.exp_time = FLASH_CAMERA_EXP_TIME_DEFAULT;
    plan->camera_page.gain = FLASH_CAMERA_GAIN_DEFAULT;

    plan->servo_page.steer_p = FLASH_SERVO_P_DEFAULT;
    plan->servo_page.steer_d = FLASH_SERVO_D_DEFAULT;
    plan->servo_page.err2_k = FLASH_SERVO_ERR2_DEFAULT;
    plan->servo_page.ackerman = FLASH_SERVO_ACKERMAN_DEFAULT;
    plan->servo_page.imu_d = FLASH_SERVO_IMU_D_DEFAULT;
    plan->servo_page.tow_point = FLASH_SERVO_TOW_POINT_DEFAULT;
    plan->servo_page.servo_min_angle = FLASH_SERVO_MIN_ANGLE_DEFAULT;
    plan->servo_page.servo_max_angle = FLASH_SERVO_MAX_ANGLE_DEFAULT;

    plan->motor_page.target_speed = 0;
}

/* 方案2默认值 */
static void flash_fill_plan1(flash_plan_t *plan)
{
    flash_fill_plan0(plan);
    plan->motor_page.target_speed = 100;
}

/* 默认镜像 */
static void flash_fill_default_image(flash_store_image_t *image)
{
    memset(image, 0, sizeof(*image));
    image->magic = FLASH_STORE_MAGIC;
    image->version = FLASH_STORE_VERSION;
    image->active_plan = 0;
    flash_fill_plan0(&image->plan[0]);
    flash_fill_plan1(&image->plan[1]);
    image->checksum = flash_calc_checksum(image);
}

/* 镜像校验 */
static uint8 flash_image_is_valid(const flash_store_image_t *image)
{
    uint8 i;

    if(0 == image)
    {
        return 0;
    }
    if(FLASH_STORE_MAGIC != image->magic)
    {
        return 0;
    }
    if(FLASH_STORE_VERSION != image->version)
    {
        return 0;
    }
    if(!flash_plan_is_valid(image->active_plan))
    {
        return 0;
    }

    for(i = 0; i < FLASH_PLAN_COUNT; i++)
    {
        if(!flash_plan_data_is_valid(&image->plan[i]))
        {
            return 0;
        }
    }

    if(flash_calc_checksum(image) != image->checksum)
    {
        return 0;
    }

    return 1;
}

/* 写flash */
static void flash_save(void)
{
    flash_store_image.magic = FLASH_STORE_MAGIC;
    flash_store_image.version = FLASH_STORE_VERSION;
    flash_store_image.checksum = flash_calc_checksum(&flash_store_image);

    iap_init();
    extern_iap_write_buff(FLASH_STORE_ADDR, (uint8 *)&flash_store_image, sizeof(flash_store_image));
    iap_idle();
}

/* 应用当前方案 */
static void flash_apply_current_plan(void)
{
    flash_plan_t *plan;

    plan = &flash_store_image.plan[flash_store_image.active_plan];

    servo_set_pid(plan->servo_page.steer_p,
                  plan->servo_page.steer_d,
                  plan->servo_page.err2_k,
                  plan->servo_page.imu_d);
    servo_set_tow_point(plan->servo_page.tow_point);
    servo_set_limit(plan->servo_page.servo_min_angle, plan->servo_page.servo_max_angle);
    motor_set_target(plan->motor_page.target_speed, plan->motor_page.target_speed);
}

/* 读flash */
static void flash_load(void)
{
    flash_store_image_t image;
    uint8 i;
    uint8 changed;

    iap_init();
    iap_read_buff(FLASH_STORE_ADDR, (uint8 *)&image, sizeof(image));
    iap_idle();

    if(flash_image_is_valid(&image))
    {
        memcpy(&flash_store_image, &image, sizeof(flash_store_image));
        changed = 0;
        for(i = 0; i < FLASH_PLAN_COUNT; i++)
        {
            if(flash_normalize_plan(&flash_store_image.plan[i]))
            {
                changed = 1;
            }
        }
        if(changed)
        {
            flash_save();
        }
    }
    else
    {
        flash_fill_default_image(&flash_store_image);
        flash_save();
    }

    flash_apply_current_plan();
    flash_ready = 1;
}

/* 读准备 */
static void flash_ensure_ready(void)
{
    if(!flash_ready)
    {
        flash_load();
    }
}

/* 当前方案 */
static flash_plan_t *flash_get_current_plan(void)
{
    return &flash_store_image.plan[flash_store_image.active_plan];
}

void flash_init(void)
{
    flash_load();
}

uint8 flash_get_active_plan(void)
{
    flash_ensure_ready();
    return flash_store_image.active_plan;
}

uint8 flash_set_active_plan(uint8 plan_index)
{
    flash_ensure_ready();

    if(!flash_plan_is_valid(plan_index))
    {
        return 0;
    }
    if(plan_index == flash_store_image.active_plan)
    {
        return 1;
    }

    flash_store_image.active_plan = plan_index;
    flash_apply_current_plan();
    flash_save();
    return 1;
}

void flash_get_camera_page(flash_camera_page_t *page)
{
    flash_ensure_ready();

    if(0 == page)
    {
        return;
    }

    memcpy(page, &flash_get_current_plan()->camera_page, sizeof(*page));
}

void flash_get_servo_page(flash_servo_page_t *page)
{
    flash_ensure_ready();

    if(0 == page)
    {
        return;
    }

    memcpy(page, &flash_get_current_plan()->servo_page, sizeof(*page));
}

void flash_get_motor_page(flash_motor_page_t *page)
{
    flash_ensure_ready();

    if(0 == page)
    {
        return;
    }

    memcpy(page, &flash_get_current_plan()->motor_page, sizeof(*page));
}

int16 flash_get_motor_value(flash_motor_slot_t slot)
{
    flash_ensure_ready();

    switch(slot)
    {
        case FLASH_MOTOR_TARGET_SPEED:
            return flash_get_current_plan()->motor_page.target_speed;
        default:
            return 0;
    }
}

uint8 flash_set_camera_value(flash_camera_slot_t slot, int16 value)
{
    flash_ensure_ready();

    if(!flash_camera_value_is_valid(slot, value))
    {
        return 0;
    }

    switch(slot)
    {
        case FLASH_CAMERA_EXP_TIME:
            flash_get_current_plan()->camera_page.exp_time = value;
            break;
        case FLASH_CAMERA_GAIN:
            flash_get_current_plan()->camera_page.gain = value;
            break;
        default:
            return 0;
    }

    flash_save();
    return 1;
}

uint8 flash_set_servo_page(const flash_servo_page_t *page)
{
    flash_ensure_ready();

    if(!flash_servo_page_is_valid(page))
    {
        return 0;
    }

    memcpy(&flash_get_current_plan()->servo_page, page, sizeof(*page));
    flash_apply_current_plan();
    flash_save();
    return 1;
}

uint8 flash_set_motor_value(flash_motor_slot_t slot, int16 value)
{
    flash_ensure_ready();

    if(!flash_motor_value_is_valid(slot, value))
    {
        return 0;
    }

    switch(slot)
    {
        case FLASH_MOTOR_TARGET_SPEED:
            flash_get_current_plan()->motor_page.target_speed = value;
            break;
        default:
            return 0;
    }

    flash_apply_current_plan();
    flash_save();
    return 1;
}

int16 flash_get_camera_step(flash_camera_slot_t slot)
{
    if(slot >= FLASH_CAMERA_COUNT)
    {
        return 0;
    }

    return flash_camera_config[slot].step;
}

int16 flash_get_servo_step(flash_servo_slot_t slot)
{
    if(slot >= FLASH_SERVO_COUNT)
    {
        return 0;
    }

    return flash_servo_config[slot].step;
}

int16 flash_get_motor_step(flash_motor_slot_t slot)
{
    if(slot >= FLASH_MOTOR_COUNT)
    {
        return 0;
    }

    return flash_motor_config[slot].step;
}
