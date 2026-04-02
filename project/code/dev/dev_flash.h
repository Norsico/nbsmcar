#ifndef _DEV_FLASH_H_
#define _DEV_FLASH_H_

#include "zf_common_typedef.h"

/* 当前参数页先拿两项数值做最小验证，数值统一按 0.1 来存。 */
#define FLASH_PARAM_VALUE_MIN_TENTH     (0)
#define FLASH_PARAM_VALUE_MAX_TENTH     (999)
#define FLASH_PARAM_VALUE_STEP_TENTH    (1)

/* 摄像头参数页按摄像头原生整数值来存，下面这些范围就是屏幕调参时实际允许调的范围。 */
#define FLASH_CAMERA_AUTO_EXP_MIN       (0)     /* 自动曝光下限：0 表示关闭自动曝光。 */
#define FLASH_CAMERA_AUTO_EXP_MAX       (63)    /* 自动曝光上限：逐飞库注释给的有效范围是 0-63。 */
#define FLASH_CAMERA_AUTO_EXP_STEP      (1)     /* 自动曝光步进：屏幕上每次调整加减 1。 */
#define FLASH_CAMERA_EXP_TIME_MIN       (1)     /* 曝光时间下限：工程侧收口为 1，避免出现 0 这种无意义配置。 */
#define FLASH_CAMERA_EXP_TIME_MAX       (300)   /* 曝光时间上限：底层库没写明确数值上限，这里按默认值 50、示例值 100 和赛道调参需求，先把 UI 可调范围收口到 300。 */
#define FLASH_CAMERA_EXP_TIME_STEP      (1)     /* 曝光时间步进：屏幕上每次调整加减 1。 */
#define FLASH_CAMERA_GAIN_MIN           (16)    /* 图像增益下限：逐飞库注释给的有效范围起点。 */
#define FLASH_CAMERA_GAIN_MAX           (64)    /* 图像增益上限：逐飞库注释给的有效范围终点。 */
#define FLASH_CAMERA_GAIN_STEP          (1)     /* 图像增益步进：屏幕上每次调整加减 1。 */
#define FLASH_LINE_KP_DEFAULT_TENTH     (12)    /* 巡线 KP 默认值 1.2，单位统一按 0.1 保存。 */
#define FLASH_LINE_KD_DEFAULT_TENTH     (10)    /* 巡线 KD 默认值 1.0，单位统一按 0.1 保存。 */
#define FLASH_LINE_NEAR_ROW_DEFAULT     (3)     /* 近点默认看车头附近第 3 行偏移。 */
#define FLASH_LINE_FAR_ROW_DEFAULT      (15)    /* 远点默认比近点再往上提前 15 行。 */
#define FLASH_LINE_NEAR_WEIGHT_DEFAULT  (3)     /* 近点默认权重 3，优先稳住车头。 */
#define FLASH_LINE_FAR_WEIGHT_DEFAULT   (2)     /* 远点默认权重 2，给一点提前量。 */
#define FLASH_LINE_SERVO_MIN_DEFAULT    (80)    /* 舵机默认左限幅。 */
#define FLASH_LINE_SERVO_MAX_DEFAULT    (110)   /* 舵机默认右限幅。 */
#define FLASH_START_SPEED_MIN           (0)     /* 启动页后轮目标速度下限，0 表示静止。 */
#define FLASH_START_SPEED_MAX           (200)   /* 启动页后轮目标速度上限，先按 0-200 收口。 */
#define FLASH_START_SPEED_STEP          (1)     /* 启动页后轮目标速度步进。 */
#define FLASH_START_ENABLE_MIN          (0)     /* 启动开关下限，0 表示关闭。 */
#define FLASH_START_ENABLE_MAX          (1)     /* 启动开关上限，1 表示开启。 */
#define FLASH_START_ENABLE_STEP         (1)     /* 启动开关步进。 */
#define FLASH_START_SPEED_DEFAULT       (0)     /* 上电默认不主动给后轮速度。 */
#define FLASH_START_ENABLE_DEFAULT      (0)     /* 上电默认不自动起跑。 */

/* 当前参数页里这两行，后面真要扩参数就继续往后加。 */
typedef enum
{
    FLASH_PARAM_SLOT_FIRST = 0,
    FLASH_PARAM_SLOT_SECOND,
    FLASH_PARAM_SLOT_COUNT
} flash_param_slot_t;

typedef enum
{
    FLASH_CAMERA_SLOT_AUTO_EXP = 0,  /* 自动曝光参数。 */
    FLASH_CAMERA_SLOT_EXP_TIME,      /* 曝光时间参数。 */
    FLASH_CAMERA_SLOT_GAIN,          /* 图像增益参数。 */
    FLASH_CAMERA_SLOT_COUNT
} flash_camera_slot_t;

/* 这是当前参数页在用的那一组参数。 */
typedef struct
{
    int16 first_value_tenth;
    int16 second_value_tenth;
} flash_param_page_t;

typedef struct
{
    uint8 auto_exp;   /* 自动曝光配置值。 */
    uint16 exp_time;  /* 曝光时间配置值。 */
    uint8 gain;       /* 图像增益配置值。 */
} flash_camera_page_t;

typedef struct
{
    uint8 kp_tenth;         /* 巡线 KP，单位 0.1。 */
    uint8 kd_tenth;         /* 巡线 KD，单位 0.1。 */
    uint8 near_row_offset;  /* 近点参考行，相对有效底部的偏移量。 */
    uint8 far_row_offset;   /* 远点参考行，相对有效底部的偏移量。 */
    uint8 near_weight;      /* 近点权重。 */
    uint8 far_weight;       /* 远点权重。 */
    uint8 servo_min_angle;  /* 舵机最小角限制。 */
    uint8 servo_max_angle;  /* 舵机最大角限制。 */
} flash_line_tune_page_t;

typedef struct
{
    uint16 target_speed;    /* 后轮闭环目标速度，左右轮先共用同一个值。 */
    uint8 enable;           /* 启动开关：0 关闭，1 开启。 */
    uint8 reserved;         /* 预留字节，后面扩状态位时继续往这里加。 */
} flash_start_page_t;

/* 掉电参数先按一个总结构来管，后面别的模块直接往这里扩。 */
typedef struct
{
    flash_param_page_t param_page;
    flash_camera_page_t camera_page;
    flash_line_tune_page_t line_tune_page;
    flash_start_page_t start_page;
} flash_store_data_t;

/* 上电初始化，负责把 EEPROM 里的内容读到 RAM。 */
void flash_store_init(void);
/* 读取整个掉电参数结构。 */
void flash_store_get_data(flash_store_data_t *store_ptr);
/* 整体覆盖掉电参数结构，适合后面多个模块一起改完再存。 */
uint8 flash_store_set_data(const flash_store_data_t *store_ptr);
/* 读取当前参数页这一组参数。 */
void flash_store_get_param_page(flash_param_page_t *page);
/* 读取参数页里某一项，返回值单位是 0.1。 */
int16 flash_store_get_param_value_tenth(flash_param_slot_t slot);
/* 修改参数页里某一项，值变了就立刻落盘。 */
uint8 flash_store_set_param_value_tenth(flash_param_slot_t slot, int16 value_tenth);
/* 读取摄像头参数页这一组参数。 */
void flash_store_get_camera_page(flash_camera_page_t *page);
/* 整体覆盖摄像头参数页。 */
uint8 flash_store_set_camera_page(const flash_camera_page_t *page);
/* 读取摄像头参数页里某一项。 */
uint16 flash_store_get_camera_value(flash_camera_slot_t slot);
/* 修改摄像头参数页里某一项，值变了就立刻落盘。 */
uint8 flash_store_set_camera_value(flash_camera_slot_t slot, uint16 value);
/* 读取巡线调参页这一组参数。 */
void flash_store_get_line_tune_page(flash_line_tune_page_t *page);
/* 整体覆盖巡线调参页。 */
uint8 flash_store_set_line_tune_page(const flash_line_tune_page_t *page);
/* 读取启动页这一组参数。 */
void flash_store_get_start_page(flash_start_page_t *page);
/* 整体覆盖启动页。 */
uint8 flash_store_set_start_page(const flash_start_page_t *page);
/* 恢复默认参数并写回。 */
void flash_store_reset_data(void);

#endif
