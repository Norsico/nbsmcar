/*
 * **************************************************************************
 * ********************                                  ********************
 * ********************      COPYRIGHT INFORMATION       ********************
 * ********************                                  ********************
 * **************************************************************************
 *                                                                          *
 *                                   _oo8oo_                                *
 *                                  o8888888o                               *
 *                                  88" . "88                               *
 *                                  (| -_- |)                               *
 *                                  0\  =  /0                               *
 *                                ___/'==='\___                             *
 *                              .' \\|     |// '.                           *
 *                             / \\|||  :  |||// \                          *
 *                            / _||||| -:- |||||_ \                         *
 *                           |   | \\\  -  /// |   |                        *
 *                           | \_|  ''\---/''  |_/ |                        *
 *                           \  .-\__  '-'  __/-.  /                        *
 *                         ___'. .'  /--.--\  '. .'___                      *
 *                      ."" '<  '.___\_<|>_/___.'  >' "".                   *
 *                     | | :  `- \`.:`\ _ /`:.`/ -`  : | |                  *
 *                     \  \ `-.   \_ __\ /__ _/   .-` /  /                  *
 *                 =====`-.____`.___ \_____/ ___.`____.-`=====              *
 *                                   `=---=`                                *
 * **************************************************************************
 * ********************                                  ********************
 * ********************      	佛祖保佑 永远无BUG		  ********************
 * ********************                                  ********************
 * **************************************************************************
 */
#include "headfile.h"


void main(void)
{
    clock_init(SYSTEM_CLOCK_114M); 				// 时钟配置及系统初始化<务必保留>

    debug_init();
    para_init();
    flash_init();
		servo_init();      // IMU初始化（SPI通信）
		state_init();      // 状态初始化
		buzzer_init();
    motor_init();      // 仅初始化GPIO、PWM、编码器（不启动中断）
    image_init();      // 摄像头初始化（DMA + 中断）

    system_delay_ms(100);  // 等待所有外设稳定

    motor_start_control(); // 最后启动电机中断 + 风扇爬坡

    if(CarMode == CAR_MODE_UI)
    {
        ui_init();
    }
    while(1)
    {
        switch(CarMode)
        {
            case CAR_MODE_UI:
            {
                buzzer_stop_alarm_enable(0);
                if(ui_is_debug())
                {
                    image_update();
                    servo_update();
                }
                else
                {
                    image_update_laser_test();
                    pwm_set_duty(SERVO_PWM, SERVO_ANGLE_CENTER/3+1500);
                }
                ui_update();
            } break;

            case CAR_MODE_RUN:
            {
                if(BlindBoxPhase != BLIND_BOX_STOP)
                {
                    buzzer_stop_alarm_enable(0);
                    image_update();
                }

                if(BlindBoxPhase == BLIND_BOX_STOP)
                {
                    buzzer_stop_alarm_enable(1);
                    servo_update_motor_target();
                    pwm_set_duty(SERVO_PWM, SERVO_ANGLE_CENTER/3+1500);
                    pwm_set_duty(FAN_PWM, 3000);
                    break;
                }

                servo_update();
                motor_update_fan();  /* 动态调整风扇 */
            } break;

            case CAR_MODE_STOP:
            {
				buzzer_stop_alarm_enable(1);
				pwm_set_duty(SERVO_PWM, SERVO_ANGLE_CENTER/3+1500);
                motor_output(0, 0);
                pwm_set_duty(FAN_PWM, 3000);
            } break;
        }
    }
}


