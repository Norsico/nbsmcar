#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "zf_common_typedef.h"
#include "zf_driver_gpio.h"
#include "zf_driver_pwm.h"
#include "zf_driver_encoder.h"
#include "zf_driver_pit.h"

#define MOTOR_CTRL_PIT                (TIM1_PIT)              /* 电机定时器TIM1 */
#define MOTOR_CTRL_PERIOD_MS          (5)                     /* 电机定时器周期 */
#define MOTOR_PWM_FREQ                (17000)                 /* 电机频率 */

#define RIGHT_MOTOR_DIR_PIN           (IO_P75)                /* 右电机方向 */
#define RIGHT_MOTOR_PWM_PIN           (PWMB_CH1_P74)          /* 右电机PWM */
#define LEFT_MOTOR_DIR_PIN            (IO_P77)                /* 左电机方向 */
#define LEFT_MOTOR_PWM_PIN            (PWMB_CH3_P76)          /* 左电机PWM */

#define MOTOR_ENCODER_LEFT            (PWMC_ENCODER)          /* 左编码器口 */
#define MOTOR_ENCODER_LEFT_CHA        (PWMC_ENCODER_CH1P_P40) /* 左编码器A */
#define MOTOR_ENCODER_LEFT_CHB        (PWMC_ENCODER_CH2P_P42) /* 左编码器B */

#define MOTOR_ENCODER_RIGHT           (PWMA_ENCODER)          /* 右编码器口 */
#define MOTOR_ENCODER_RIGHT_CHA       (PWMA_ENCODER_CH1P_P60) /* 右编码器A */
#define MOTOR_ENCODER_RIGHT_CHB       (PWMA_ENCODER_CH2P_P62) /* 右编码器B */

typedef struct
{
    volatile int16 target_left;      /* 左目标 */
    volatile int16 target_right;     /* 右目标 */
    volatile int16 count_left;       /* 左计数 */
    volatile int16 count_right;      /* 右计数 */
    volatile int8 pwm_left;          /* 左输出 */
    volatile int8 pwm_right;         /* 右输出 */
} motor_data_t;

extern motor_data_t motor_data;

void motor_init(void);
void motor_stop(void);
void motor_set_target(int16 left_target, int16 right_target);
void motor_set_pid_left(int16 kp, int16 ki);
void motor_set_pid_right(int16 kp, int16 ki);
void motor_get_pid_left(int16 *kp, int16 *ki);
void motor_get_pid_right(int16 *kp, int16 *ki);
void motor_update(void);

#endif
