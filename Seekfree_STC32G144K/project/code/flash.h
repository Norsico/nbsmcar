#ifndef _FLASH_H_
#define _FLASH_H_

#include "zf_common_headfile.h"

#define FLASH_STORE_ADDR                 (0x0000)              /* flash地址 */
#define FLASH_STORE_MAGIC                (0x5346)              /* flash标记 */
#define FLASH_STORE_VERSION              (0x0002)              /* flash版本 */
#define FLASH_PLAN_COUNT                 (2)                   /* 方案数量 */

#define FLASH_CAMERA_EXP_TIME_MIN        (1)                   /* 曝光下限 */
#define FLASH_CAMERA_EXP_TIME_DEFAULT    (110)                 /* 曝光默认值 */
#define FLASH_CAMERA_EXP_TIME_STEP       (10)                  /* 曝光步进 */

#define FLASH_CAMERA_GAIN_MIN            (16)                  /* 增益下限 */
#define FLASH_CAMERA_GAIN_MAX            (64)                  /* 增益上限 */
#define FLASH_CAMERA_GAIN_DEFAULT        (36)                  /* 增益默认值 */
#define FLASH_CAMERA_GAIN_STEP           (1)                   /* 增益步进 */

#define FLASH_SERVO_P_MIN                (0)                   /* 舵机p下限 */
#define FLASH_SERVO_P_MAX                (600)                 /* 舵机p上限 */
#define FLASH_SERVO_P_DEFAULT            (28)                  /* 舵机p默认值 */
#define FLASH_SERVO_P_STEP               (2)                   /* 舵机p步进 */

#define FLASH_SERVO_D_MIN                (0)                   /* 舵机d下限 */
#define FLASH_SERVO_D_MAX                (800)                 /* 舵机d上限 */
#define FLASH_SERVO_D_DEFAULT            (30)                  /* 舵机d默认值 */
#define FLASH_SERVO_D_STEP               (2)                   /* 舵机d步进 */

#define FLASH_SERVO_ERR2_MIN             (0)                   /* 二次误差下限 */
#define FLASH_SERVO_ERR2_MAX             (200)                 /* 二次误差上限 */
#define FLASH_SERVO_ERR2_DEFAULT         (8)                   /* 二次误差默认值 */
#define FLASH_SERVO_ERR2_STEP            (1)                   /* 二次误差步进 */

#define FLASH_SERVO_ACKERMAN_MIN         (0)                   /* 阿克曼下限 */
#define FLASH_SERVO_ACKERMAN_MAX         (32767)               /* 阿克曼上限 */
#define FLASH_SERVO_ACKERMAN_DEFAULT     (1105)                /* 阿克曼默认值 */
#define FLASH_SERVO_ACKERMAN_STEP        (10)                  /* 阿克曼步进 */

#define FLASH_SERVO_IMU_D_MIN            (0)                   /* 陀螺仪d下限 */
#define FLASH_SERVO_IMU_D_MAX            (100)                 /* 陀螺仪d上限 */
#define FLASH_SERVO_IMU_D_DEFAULT        (4)                   /* 陀螺仪d默认值 */
#define FLASH_SERVO_IMU_D_STEP           (1)                   /* 陀螺仪d步进 */

#define FLASH_SERVO_TOW_POINT_MIN        (1)                   /* 前瞻下限 */
#define FLASH_SERVO_TOW_POINT_MAX        (49)                  /* 前瞻上限 */
#define FLASH_SERVO_TOW_POINT_DEFAULT    (32)                  /* 前瞻默认值 */
#define FLASH_SERVO_TOW_POINT_STEP       (1)                   /* 前瞻步进 */

#define FLASH_SERVO_MIN_ANGLE_MIN        (50)                  /* 左限幅下限 */
#define FLASH_SERVO_MIN_ANGLE_MAX        (120)                 /* 左限幅上限 */
#define FLASH_SERVO_MIN_ANGLE_DEFAULT    (72)                  /* 左限幅默认值 */
#define FLASH_SERVO_MIN_ANGLE_STEP       (2)                   /* 左限幅步进 */

#define FLASH_SERVO_MAX_ANGLE_MIN        (50)                  /* 右限幅下限 */
#define FLASH_SERVO_MAX_ANGLE_MAX        (120)                 /* 右限幅上限 */
#define FLASH_SERVO_MAX_ANGLE_DEFAULT    (108)                 /* 右限幅默认值 */
#define FLASH_SERVO_MAX_ANGLE_STEP       (2)                   /* 右限幅步进 */

#define FLASH_MOTOR_TARGET_STEP          (10)                  /* 目标步进 */

typedef struct
{
    int16 step;                                                 /* 步进 */
} flash_value_config_t;

typedef enum
{
    FLASH_CAMERA_EXP_TIME = 0,                                  /* 曝光 */
    FLASH_CAMERA_GAIN,                                          /* 增益 */
    FLASH_CAMERA_COUNT                                          /* 相机参数数量 */
} flash_camera_slot_t;

typedef enum
{
    FLASH_SERVO_P = 0,                                          /* 舵机p */
    FLASH_SERVO_D,                                              /* 舵机d */
    FLASH_SERVO_ERR2,                                           /* 二次误差 */
    FLASH_SERVO_ACKERMAN,                                       /* 阿克曼 */
    FLASH_SERVO_IMU_D,                                          /* 陀螺仪d */
    FLASH_SERVO_TOW_POINT,                                      /* 前瞻 */
    FLASH_SERVO_MIN_ANGLE,                                      /* 左限幅 */
    FLASH_SERVO_MAX_ANGLE,                                      /* 右限幅 */
    FLASH_SERVO_COUNT                                           /* 舵机参数数量 */
} flash_servo_slot_t;

typedef enum
{
    FLASH_MOTOR_TARGET_SPEED = 0,                               /* 目标速度 */
    FLASH_MOTOR_COUNT                                           /* 电机参数数量 */
} flash_motor_slot_t;

typedef struct
{
    int16 exp_time;                                             /* 曝光 */
    int16 gain;                                                 /* 增益 */
} flash_camera_page_t;

typedef struct
{
    int16 steer_p;                                              /* 舵机p */
    int16 steer_d;                                              /* 舵机d */
    int16 err2_k;                                               /* 二次误差 */
    int16 ackerman;                                             /* 阿克曼 */
    int16 imu_d;                                                /* 陀螺仪d */
    int16 tow_point;                                            /* 前瞻 */
    int16 servo_min_angle;                                      /* 左限幅 */
    int16 servo_max_angle;                                      /* 右限幅 */
} flash_servo_page_t;

typedef struct
{
    int16 target_speed;                                         /* 目标速度 */
} flash_motor_page_t;

typedef struct
{
    flash_camera_page_t camera_page;                            /* 相机页 */
    flash_servo_page_t servo_page;                              /* 舵机页 */
    flash_motor_page_t motor_page;                              /* 电机页 */
} flash_plan_t;

void flash_init(void);
uint8 flash_get_active_plan(void);
uint8 flash_set_active_plan(uint8 plan_index);
void flash_get_camera_page(flash_camera_page_t *page);
void flash_get_servo_page(flash_servo_page_t *page);
void flash_get_motor_page(flash_motor_page_t *page);
int16 flash_get_motor_value(flash_motor_slot_t slot);
uint8 flash_set_camera_value(flash_camera_slot_t slot, int16 value);
uint8 flash_set_servo_page(const flash_servo_page_t *page);
uint8 flash_set_motor_value(flash_motor_slot_t slot, int16 value);
int16 flash_limit_camera_value(flash_camera_slot_t slot, int16 value);
int16 flash_limit_servo_value(flash_servo_slot_t slot, int16 value);
int16 flash_limit_motor_value(flash_motor_slot_t slot, int16 value);
int16 flash_get_camera_step(flash_camera_slot_t slot);
int16 flash_get_servo_step(flash_servo_slot_t slot);
int16 flash_get_motor_step(flash_motor_slot_t slot);

#endif
