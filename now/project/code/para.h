#ifndef __PARA_H__
#define __PARA_H__

#include "headfile.h"

/* Servo */
// 右转
#define SERVO_KP                   (144)
#define SERVO_KD                   (124)
#define SERVO_ERR2_K               (40)
#define SERVO_IMU_D                (20)
#define SERVO_ACKERMAN             (1300)
#define SERVO_POINT                (22)
// 左转
#define SERVO_LEFT_KP              (128)
#define SERVO_LEFT_KD              (148)
#define SERVO_LEFT_ERR2_K          (40)
#define SERVO_LEFT_IMU_D           (64)
#define SERVO_LEFT_ACKERMAN        (1200)
#define SERVO_LEFT_POINT           (22)
// 直道
#define SERVO_ST_KP                (100)
#define SERVO_ST_KD                (116)
#define SERVO_ST_ERR2_K            (36)
#define SERVO_ST_IMU_D             (20)
#define SERVO_ST_ACKERMAN          (1000)
#define SERVO_ST_POINT             (18)
// 环岛
#define SERVO_IN_RING_POINT        (25)
#define SERVO_OUT_RING_POINT       (24)

/* Motor */
#define MOTOR_LEFT_KP              (55)
#define MOTOR_LEFT_KI              (6)
#define MOTOR_RIGHT_KP             (55)
#define MOTOR_RIGHT_KI             (6)
#define MOTOR_TARGET_SPEED         (230)
#define MOTOR_STRAIGHT_SPEED       (230)
#define MOTOR_RING_SPEED           (220)
#define MOTOR_RAMP_SPEED           (130)
#define FAN_DUTY                   (70)
#define FAN_STRAIGHT_DUTY          (70)
#define FAN_ENABLE                 (1)

/* Camera */
#define CAMERA_EXPOSURE            (100)
#define CAMERA_GAIN                (30)
#define CAMERA_THRESHOLD_OFFSET    (11)
#define CAMERA_THRESHOLD_TRI_DELTA (15)

/* Laser */
#define CAMERA_LASER_TEST          (0)
#define CAMERA_LASER_FIRE_US       (4500)
#define CAMERA_LASER_UI_TEST_COL   (39)

// 弯道激光
#define CAMERA_LASER_LEFT3_COL     (10)
#define CAMERA_LASER_LEFT2_COL     (19)
#define CAMERA_LASER_LEFT1_COL     (29)
#define CAMERA_LASER_CENTER_COL    (39)
#define CAMERA_LASER_RIGHT1_COL    (49)
#define CAMERA_LASER_RIGHT2_COL    (58)
#define CAMERA_LASER_RIGHT3_COL    (69)
#define CAMERA_LASER_ROW1          (58)
#define CAMERA_LASER_ROW2          (57)
#define CAMERA_LASER_ROW3          (56)
#define CAMERA_LASER_OK_NUM        (1)
#define CAMERA_LASER_FIRE_INTERVAL (5)
// 直道激光
#define CAMERA_LASER_ST_LEFT3_COL  (10)
#define CAMERA_LASER_ST_LEFT2_COL  (19)
#define CAMERA_LASER_ST_LEFT1_COL  (29)
#define CAMERA_LASER_ST_CENTER_COL (39)
#define CAMERA_LASER_ST_RIGHT1_COL (49)
#define CAMERA_LASER_ST_RIGHT2_COL (58)
#define CAMERA_LASER_ST_RIGHT3_COL (69)
#define CAMERA_LASER_ST_ROW1       (56)
#define CAMERA_LASER_ST_ROW2       (55)
#define CAMERA_LASER_ST_ROW3       (54)
#define CAMERA_LASER_ST_OK_NUM     (1)
#define CAMERA_LASER_ST_FIRE_INTERVAL (5)

/* Other */
// 圈数
#define OTHER_LAP_COUNT            (1)
#define OTHER_BLIND_BOX_SPEED      (150)
#define OTHER_BLIND_BOX_SLOW_SPEED (70)
#define OTHER_BOX_LASER_ROW1       (56)
#define OTHER_BOX_LASER_ROW2       (55)
#define OTHER_BOX_LASER_ROW3       (54)
#define OTHER_BOX_LASER_OK_NUM     (1)
#define OTHER_BOX_LASER_GAP        (10)

/* 硬件配置 */
#define MOTOR_CTRL_PERIOD_MS       (5)
#define MOTOR_STRAIGHT_DELAY_MS    (2000)  /* 电机控制启动后延迟启用直道 */
#define MOTOR_PWM_FREQ             (17000)
#define MOTOR_DUTY_LIMIT           (9000)

#define SERVO_PWM_FREQ             (300)
#define SERVO_ANGLE_MIN            (7300-400)
#define SERVO_ANGLE_CENTER         (9000-400)
#define SERVO_ANGLE_MAX            (10700-400)

#define FAN_PWM_FREQ               (300)
#define FAN_ESC_DUTY_MIN           (3000)
#define FAN_ESC_DUTY_MAX           (6000)
#define FAN_ESC_DUTY_STEP          (30)

#define CAMERA_INIT_RETRY          (5)
#define CAMERA_INIT_DELAY_MS       (100)

#define BATTERY_EMPTY              (114)
#define BATTERY_FULL               (126)
#define BATTERY_SAMPLE_COUNT       (3)

#define BUZZER_SHORT_MS            (80)
#define BUZZER_STOP_ALARM_ON_MS    (300)
#define BUZZER_STOP_ALARM_OFF_MS   (1000)

#define IRQ_PRIORITY_MOTOR         (2)
#define IRQ_PRIORITY_NORMAL        (0)

/*硬件，引脚定义，不用动*/
#define LED_DEBUG                  (IO_P52)                 // 核心板LED

#define BUZZER                     (IO_PA1)                 // Buzzer 蜂鸣器

#define BATTERY_ADC                (ADC1_CH0_P10)           // 电池电压采样

#define LASER_LEFT_3               (IO_P93)                 // Laser 1 and 6 swapped: left to right 9.3 9.6 9.5 9.4 9.2 9.7 9.1
#define LASER_LEFT_2               (IO_P96)
#define LASER_LEFT_1               (IO_P95)
#define LASER_CENTER               (IO_P94)
#define LASER_RIGHT_1              (IO_P92)
#define LASER_RIGHT_2              (IO_P97)
#define LASER_RIGHT_3              (IO_P91)

#define SWITCH_MODE1               (IO_PB0)                 // 拨码开关
#define SWITCH_MODE2               (IO_PB1)

#define KEY_BACK                   (IO_PB2)                 // 4个按钮
#define KEY_UP                     (IO_PB3)
#define KEY_DOWN                   (IO_PB4)
#define KEY_ENTER                  (IO_P32)

#define SERVO_PWM                  (PWME_CH3P_PA4)          // 舵机

#define FAN_PWM                    (PWMF_CH2_PA3)           // Brushless fan 无刷

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
    int16 left_kp;
    int16 left_kd;
    int16 left_err2_k;
    int16 left_imu_d;
    int16 left_ackerman;
    int16 left_tow_point;
    int16 st_kp;
    int16 st_kd;
    int16 st_err2_k;
    int16 st_imu_d;
    int16 st_ackerman;
    int16 st_tow_point;
    int16 in_ring_point;
    int16 out_ring_point;
} servo_para;

typedef struct
{
    int16 target_speed;
    int16 straight_speed;
    int16 ring_speed;
    int16 ramp_speed;
    int16 left_kp;
    int16 left_ki;
    int16 right_kp;
    int16 right_ki;
    int16 fan_duty;
    int16 fan_straight_duty;
    uint8 fan_en;
} motor_para;

typedef struct
{
    uint16 exposure;
    uint8 gain;
    uint8 threshold_offset;
    uint8 threshold_tri_delta;
    uint8 laser_test;
    uint16 laser_fire_us;
    uint8 laser_fire_interval;
    uint8 laser_left3_col;
    uint8 laser_left2_col;
    uint8 laser_left1_col;
    uint8 laser_center_col;
    uint8 laser_right1_col;
    uint8 laser_right2_col;
    uint8 laser_right3_col;
    uint8 laser_row1;
    uint8 laser_row2;
    uint8 laser_row3;
    uint8 laser_ok_num;
    uint8 laser_st_fire_interval;
    uint8 laser_st_left3_col;
    uint8 laser_st_left2_col;
    uint8 laser_st_left1_col;
    uint8 laser_st_center_col;
    uint8 laser_st_right1_col;
    uint8 laser_st_right2_col;
    uint8 laser_st_right3_col;
    uint8 laser_st_row1;
    uint8 laser_st_row2;
    uint8 laser_st_row3;
    uint8 laser_st_ok_num;
    uint8 laser_ui_test_col;
} camera_para;

typedef struct
{
    uint8 lap_count;
    int16 blind_box_speed;
    int16 blind_box_slow_speed;
    uint8 box_laser_row1;
    uint8 box_laser_row2;
    uint8 box_laser_row3;
    uint8 box_laser_ok_num;
    uint8 box_laser_gap;
} other_para;
	
typedef struct
{
    servo_para servo;
    motor_para motor;
    camera_para camera;
    other_para other;
} car_para;

extern car_para SmartCar;

void para_init(void);

#endif
