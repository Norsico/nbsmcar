#ifndef _SERVO_H_
#define _SERVO_H_

#include "zf_common_headfile.h"

#define SERVO_CTRL_PIT                 (TIM0_PIT)             /* 舵机定时器TIM0 */
#define SERVO_CTRL_PERIOD_MS           (10)                   /* 舵机定时器周期 */
#define SERVO_CTRL_PRIORITY            (2)                    /* 舵机定时器优先级 */

#define SERVO_PWM_PIN                  (PWME_CH3P_PA4)        /* 舵机PWM */
#define SERVO_PWM_FREQ                 (300)                  /* 舵机频率 */

#define SERVO_ANGLE_MIN                (7500)                 /* 舵机最小角 */
#define SERVO_ANGLE_CENTER             (9000)                 /* 舵机中角 */
#define SERVO_ANGLE_MAX                (10500)                /* 舵机最大角 */
#define SERVO_MECHANICAL_TRIM          (320)                  /* 舵机机械零偏，最终输出减 2.00°。 */

#define SERVO_IMU_INIT_RETRY_MAX       (5)                    /* 陀螺仪重试次数 */
#define SERVO_IMU_INIT_DELAY_MS        (100)                  /* 陀螺仪重试间隔 */
#define SERVO_IMU_CALIBRATE_SAMPLES    (100)                  /* 陀螺仪校准次数 */

#define SERVO_ACKERMAN_MAX_ANGLE       (3000)                 /* 阿克曼最大转角 */
#define SERVO_ACKERMAN_DEFAULT         (1285)                 /* 阿克曼默认值 */

void servo_init(void);
void servo_update(void);
void servo_set_center(void);
void servo_set_pid(int16 steer_p, int16 steer_d, int16 err2_k, int16 imu_d);
void servo_set_tow_point(int16 tow_point);
void servo_set_ackerman(int16 ackerman_value);
void servo_calc_motor_target(int16 speed, int16 *left_speed, int16 *right_speed);
int32 servo_get_diff_scale(void);
int16 servo_get_speed_delta(int16 speed);

#endif
