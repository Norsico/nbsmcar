#ifndef _DEV_FLASH_H_
#define _DEV_FLASH_H_

#include "ackerman.h"
#include "zf_common_typedef.h"
#include "zf_device_mt9v03x.h"

// 参数结构体
typedef struct
{
    uint16 min;            /* 下限。 */
    uint16 max;            /* 上限。 */
    uint16 step;           /* 步进。 */
    uint16 default_value;  /* 默认值。 */
} flash_value_config_t;


/****************************** Steer PD 页 ******************************/
// 舵机P参数
static const flash_value_config_t FlashSteerPConfig =
{
    0,   /* min */
    600,  /* max */
    2,   /* step */
    28       /* default */
};
// 舵机D参数
static const flash_value_config_t FlashSteerDConfig =
{
    0,   /* min */
    800,  /* max */
    2,   /* step */
    30       /* default */
};
// 舵机二次误差系数，单位 0.1，取值 3 对应一号常用量级 0.3
static const flash_value_config_t FlashSteerErr2Config =
{
    0,   /* min */
    200,   /* max */
    1,   /* step */
    8    /* default */
};
// 舵机陀螺仪抑制系数，单位 0.1，直接作用于原始 gyro z
static const flash_value_config_t FlashSteerImuDConfig =
{
    0,   /* min */
    100,  /* max */
    1,   /* step */
    4    /* default */
};
// 阿克曼差速缩放因子，直接按原始整数调节
static const flash_value_config_t FlashAckermanKConfig =
{
    0,   /* min */
    32767,  /* max */
    10,  /* step */
    1105 /* default */
};
// 普通赛道基础前瞻，直接按图像行号调节
static const flash_value_config_t FlashTowPointConfig =
{
    1,   /* min */
    49,  /* max */
    1,   /* step */
    32   /* default */
};

/****************************** Camera 页 ******************************/
// 自动曝光参数
static const flash_value_config_t FlashCameraAutoExpConfig =
{
    0,   /* min */
    63,  /* max */
    1,   /* step */
    0   /* default */
};

// 曝光时间参数
static const flash_value_config_t FlashCameraExpTimeConfig =
{
    1,    /* min */
    300,  /* max */
    10,   /* step */
    110       /* default */
};

// 增益参数
static const flash_value_config_t FlashCameraGainConfig =
{
    16,  /* min */
    64,  /* max */
    1,   /* step */
    36       /* default */
};

/****************************** Servo Limit 页 ******************************/
// 舵机最小限幅参数
static const flash_value_config_t FlashServoMinAngleConfig =
{
    50,   /* min */
    120,  /* max */
    2,    /* step */
    72    /* default */
};

// 舵机最大限幅参数
static const flash_value_config_t FlashServoMaxAngleConfig =
{
    50,   /* min */
    120,  /* max */
    2,    /* step */
    108   /* default */
};

/****************************** Car Speed 页 ******************************/
// 目标速度参数
static const flash_value_config_t FlashStartSpeedConfig =
{
    0,     /* min */
    4000,  /* max */
    10,    /* step */
    100       /* default */
};

// 启动开关参数
static const flash_value_config_t FlashStartEnableConfig =
{
    0,  /* min */
    1,  /* max */
    1,  /* step */
    0      /* default */
};


typedef enum
{
    FLASH_PARAM_SLOT_FIRST = 0,     /* Steer P。 */
    FLASH_PARAM_SLOT_SECOND,        /* Steer D。 */
    FLASH_PARAM_SLOT_THIRD,         /* 二次误差系数。 */
    FLASH_PARAM_SLOT_FOURTH,        /* 陀螺仪抑制系数。 */
    FLASH_PARAM_SLOT_FIFTH,         /* 阿克曼差速缩放因子。 */
    FLASH_PARAM_SLOT_SIXTH,         /* 基础前瞻。 */
    FLASH_PARAM_SLOT_COUNT
} flash_param_slot_t;

typedef enum
{
    FLASH_CAMERA_SLOT_AUTO_EXP = 0,  /* 自动曝光参数。 */
    FLASH_CAMERA_SLOT_EXP_TIME,      /* 曝光时间参数。 */
    FLASH_CAMERA_SLOT_GAIN,          /* 图像增益参数。 */
    FLASH_CAMERA_SLOT_COUNT
} flash_camera_slot_t;

/* Param Config 里的 Steer PD 页。 */
typedef struct
{
    int16 first_value;        /* Steer P。 */
    int16 second_value;       /* Steer D。 */
    int16 third_value;        /* 舵机二次误差系数，单位 0.1。 */
    int16 fourth_value;       /* 陀螺仪抑制系数，单位 0.1。 */
    int16 fifth_value;        /* 阿克曼差速缩放因子，直接存原始整数。 */
    int16 sixth_value;        /* 普通赛道基础前瞻。 */
} flash_param_page_t;

typedef struct
{
    uint8 auto_exp;   /* 自动曝光配置值。 */
    uint16 exp_time;  /* 曝光时间配置值。 */
    uint8 gain;       /* 图像增益配置值。 */
} flash_camera_page_t;

/* 当前这组 flash 只保留舵机限幅，不再保留废弃的 line kp/kd。 */
typedef struct
{
    uint8 servo_min_angle;  /* 舵机最小角限制。 */
    uint8 servo_max_angle;  /* 舵机最大角限制。 */
} flash_servo_limit_page_t;

typedef struct
{
    uint16 target_speed;    /* 后轮闭环目标速度，左右轮先共用同一个值。 */
    uint8 enable;           /* 启动开关：0 关闭，1 开启。 */
    uint8 reserved;         /* 预留字节。 */
} flash_start_page_t;

/* 掉电参数总结构。 */
typedef struct
{
    flash_param_page_t param_page;
    flash_camera_page_t camera_page;
    flash_servo_limit_page_t servo_limit_page;
    flash_start_page_t start_page;
} flash_store_data_t;

/* 上电初始化，负责把 EEPROM 里的内容读到 RAM。 */
void flash_store_init(void);
/* 读取整个掉电参数结构。 */
void flash_store_get_data(flash_store_data_t *store_ptr);
/* 整体覆盖掉电参数结构。 */
uint8 flash_store_set_data(const flash_store_data_t *store_ptr);
/* 读取当前参数页这一组参数。 */
void flash_store_get_param_page(flash_param_page_t *page);
/* 读取参数页里某一项。 */
int16 flash_store_get_param_value(flash_param_slot_t slot);
/* 修改参数页里某一项，值变了就立刻落盘。 */
uint8 flash_store_set_param_value(flash_param_slot_t slot, int16 value);
/* 读取摄像头参数页这一组参数。 */
void flash_store_get_camera_page(flash_camera_page_t *page);
/* 整体覆盖摄像头参数页。 */
uint8 flash_store_set_camera_page(const flash_camera_page_t *page);
/* 读取摄像头参数页里某一项。 */
uint16 flash_store_get_camera_value(flash_camera_slot_t slot);
/* 修改摄像头参数页里某一项，值变了就立刻落盘。 */
uint8 flash_store_set_camera_value(flash_camera_slot_t slot, uint16 value);
/* 读取舵机限幅页这一组参数。 */
void flash_store_get_servo_limit_page(flash_servo_limit_page_t *page);
/* 整体覆盖舵机限幅页。 */
uint8 flash_store_set_servo_limit_page(const flash_servo_limit_page_t *page);
/* 读取启动页这一组参数。 */
void flash_store_get_start_page(flash_start_page_t *page);
/* 整体覆盖启动页。 */
uint8 flash_store_set_start_page(const flash_start_page_t *page);
/* 恢复默认参数并写回。 */
void flash_store_reset_data(void);

#endif
