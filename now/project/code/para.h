#ifndef __PARA_H__
#define __PARA_H__

#include "headfile.h"

/*软件参数，修改设置*/
#define SERVO_KP                   (0)
#define SERVO_KD                   (0)
#define SERVO_ERR2_K               (0)
#define SERVO_IMU_D                (0)
#define SERVO_ACKERMAN             (1300)
#define SERVO_POINT                (20)

#define MOTOR_TARGET_SPEED         (0)
#define MOTOR_STRAIGHT_SPEED       (0)
#define MOTOR_LEFT_KP              (58)
#define MOTOR_LEFT_KI              (5)
#define MOTOR_RIGHT_KP             (50)
#define MOTOR_RIGHT_KI             (5)
#define FAN_DUTY                   (0) 

#define CAMERA_LASER_ROW           (50)

/*硬件配置，修改设置*/
#define MOTOR_CTRL_PERIOD_MS       (5)
#define MOTOR_PWM_FREQ             (17000)
#define MOTOR_DUTY_LIMIT           (9000)

#define SERVO_PWM_FREQ             (300)
#define SERVO_ANGLE_MIN            (7500)
#define SERVO_ANGLE_CENTER         (9000)
#define SERVO_ANGLE_MAX            (10500)

#define FAN_PWM_FREQ               (300)

#define CAMERA_EXPOSURE            (120)
#define CAMERA_GAIN                (20)
#define CAMERA_INIT_RETRY          (5)
#define CAMERA_INIT_DELAY_MS       (100)

#define BATTERY_EMPTY              (114)
#define BATTERY_FULL               (126)
#define BATTERY_SAMPLE_COUNT       (3)

#define BUZZER_SHORT_MS            (80)

#define IRQ_PRIORITY_MOTOR         (2)
#define IRQ_PRIORITY_NORMAL        (0)

/*硬件，引脚定义，不用动*/
#define LED_DEBUG                  (IO_P52)                 // 核心板LED

#define BUZZER                     (IO_P96)                 // 蜂鸣器

#define BATTERY_ADC                (ADC1_CH0_P10)           // 电池电压采样

#define LASER_LEFT_2               (IO_P95)                 // 5个激光
#define LASER_LEFT_1               (IO_P94)
#define LASER_CENTER               (IO_P92)
#define LASER_RIGHT_1              (IO_P93)
#define LASER_RIGHT_2              (IO_P91)

#define SWITCH_MODE1               (IO_PB0)                 // 拨码开关
#define SWITCH_MODE2               (IO_PB1)

#define KEY_BACK                   (IO_PB2)                 // 4个按钮
#define KEY_UP                     (IO_PB3)
#define KEY_DOWN                   (IO_PB4)
#define KEY_ENTER                  (IO_P32)

#define SERVO_PWM                  (PWME_CH3P_PA4)          // 舵机

#define FAN_LEFT_PWM               (PWMF_CH1_PA1)           // 负压风扇
#define FAN_RIGHT_PWM              (PWMF_CH2_PA3)

#define MOTOR_RIGHT_DIR            (IO_P75)                 // 左右电机
#define MOTOR_RIGHT_PWM            (PWMB_CH1_P74)
#define MOTOR_LEFT_DIR             (IO_P77)
#define MOTOR_LEFT_PWM             (PWMB_CH3_P76)

#define ENCODER_LEFT               (PWMC_ENCODER)           // 编码器
#define ENCODER_LEFT_CHA           (PWMC_ENCODER_CH1P_P40)
#define ENCODER_LEFT_CHB           (PWMC_ENCODER_CH2P_P42)
#define ENCODER_RIGHT              (PWMA_ENCODER)
#define ENCODER_RIGHT_CHA          (PWMA_ENCODER_CH1P_P60)
#define ENCODER_RIGHT_CHB          (PWMA_ENCODER_CH2P_P62)

#define IPS_SPI                    (IPS200_SPI)             // 屏幕
#define IPS_SCL                    (IPS200_SCL_PIN)
#define IPS_SDA                    (IPS200_SDA_PIN)
#define IPS_RST                    (IPS200_RST_PIN)
#define IPS_DC                     (IPS200_DC_PIN)
#define IPS_CS                     (IPS200_CS_PIN)
#define IPS_BLK                    (IPS200_BLK_PIN)

#define IMU_SPC                    (IMU660RA_SPC_PIN)				// 陀螺仪
#define IMU_SDI                    (IMU660RA_SDI_PIN)
#define IMU_SDO                    (IMU660RA_SDO_PIN)
#define IMU_CS                     (IMU660RA_CS_PIN)

#define CAMERA_SCL                 (MT9V03X_COF_IIC_SCL)   // 摄像头
#define CAMERA_SDA                 (MT9V03X_COF_IIC_SDA)
#define CAMERA_D0                  (MT9V03X_D0_PIN)
#define CAMERA_D1                  (MT9V03X_D1_PIN)
#define CAMERA_D2                  (MT9V03X_D2_PIN)
#define CAMERA_D3                  (MT9V03X_D3_PIN)
#define CAMERA_D4                  (MT9V03X_D4_PIN)
#define CAMERA_D5                  (MT9V03X_D5_PIN)
#define CAMERA_D6                  (MT9V03X_D6_PIN)
#define CAMERA_D7                  (MT9V03X_D7_PIN)
#define CAMERA_VSY                 (FIFO_VSY_PIN)
#define CAMERA_RCK                 (FIFO_RCK_PIN)
#define CAMERA_WEN                 (FIFO_WE_PIN)
#define CAMERA_WRST                (FIFO_WRST_PIN)
#define CAMERA_OE                  (FIFO_OE_PIN)
#define CAMERA_RRST                (FIFO_RRST_PIN)

typedef struct
{
    int16 kp;
    int16 kd;
    int16 err2_k;
    int16 imu_d;
	  int16 ackerman;
    int16 tow_point;
} servo_para;

typedef struct
{
    int16 target_speed;
    int16 straight_speed;
    int16 left_kp;
    int16 left_ki;
    int16 right_kp;
    int16 right_ki;
    int16 fan_duty;
} motor_para;

typedef struct
{
    uint16 exposure;
    uint8 gain;
    uint8 laser_row;
} camera_para;

typedef struct
{
    servo_para servo;
    motor_para motor;
    camera_para camera;
} car_para;

extern car_para SmartCar;

void para_init(void);

#endif