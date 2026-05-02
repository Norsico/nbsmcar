#ifndef _FLASH_H_
#define _FLASH_H_

#include "zf_common_headfile.h"

#define FLASH_STORE_ADDR             (0x0000)              /* flash地址 */
#define FLASH_STORE_MAGIC            (0x5346)              /* flash标记 */
#define FLASH_STORE_VERSION          (0x0001)              /* flash版本 */
#define FLASH_PLAN_COUNT             (2)                   /* 方案数量 */

typedef struct
{
    int16 step;                                             /* 步进 */
} flash_value_config_t;

typedef enum
{
    FLASH_CAMERA_EXP_TIME = 0,                              /* 曝光 */
    FLASH_CAMERA_GAIN,                                      /* 增益 */
    FLASH_CAMERA_COUNT                                      /* 相机参数数量 */
} flash_camera_slot_t;

typedef enum
{
    FLASH_MOTOR_TARGET_SPEED = 0,                           /* 目标速度 */
    FLASH_MOTOR_COUNT                                       /* 电机参数数量 */
} flash_motor_slot_t;

typedef struct
{
    int16 exp_time;                                         /* 曝光 */
    int16 gain;                                             /* 增益 */
} flash_camera_page_t;

typedef struct
{
    int16 target_speed;                                     /* 目标速度 */
} flash_motor_page_t;

typedef struct
{
    flash_camera_page_t camera_page;                        /* 相机页 */
    flash_motor_page_t motor_page;                          /* 电机页 */
} flash_plan_t;

void flash_init(void);
uint8 flash_get_active_plan(void);
uint8 flash_set_active_plan(uint8 plan_index);
void flash_get_camera_page(flash_camera_page_t *page);
void flash_get_motor_page(flash_motor_page_t *page);
int16 flash_get_camera_value(flash_camera_slot_t slot);
int16 flash_get_motor_value(flash_motor_slot_t slot);
uint8 flash_set_camera_value(flash_camera_slot_t slot, int16 value);
uint8 flash_set_motor_value(flash_motor_slot_t slot, int16 value);
int16 flash_limit_camera_value(flash_camera_slot_t slot, int16 value);
int16 flash_limit_motor_value(flash_motor_slot_t slot, int16 value);
int16 flash_get_camera_step(flash_camera_slot_t slot);
int16 flash_get_motor_step(flash_motor_slot_t slot);
void flash_reset(void);

#endif
