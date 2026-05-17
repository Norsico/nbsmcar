#include "zf_common_headfile.h"
#include "flash.h"
#include "image.h"
#include "motor.h"
#include "servo.h"
#include "state.h"
#include "ui.h"
#include "wifi.h"

static uint8 main_get_neg_pressure_duty(void)
{
    int16 duty;

    duty = flash_get_motor_value(FLASH_MOTOR_NEG_PRESSURE_DUTY);
    duty = flash_limit_motor_value(FLASH_MOTOR_NEG_PRESSURE_DUTY, duty);

    return (uint8)duty;
}

static void main_apply_run_neg_pressure(void)
{
    uint8 duty;

    duty = main_get_neg_pressure_duty();
    if(0U == duty)
    {
        bldc_motor_stop();
        return;
    }

    bldc_motor_set_duty(duty, duty);
}

void main(void)
{
    clock_init(SYSTEM_CLOCK_96M);            // 时钟配置及系统初始化<务必保留>
    debug_init();                            // 调试串口信息初始化

    /********** 状态判断 *********/
    state_init();

    /********** 模块初始化 *********/
    motor_init();
    bldc_motor_init();
    servo_init();

    /********** flash初始化 *********/
    flash_init();
    image_init();

    if(STATE_UI == state_get_mode())
    {
        ui_init();
    }
    else if(STATE_RUN == state_get_mode())
    {
        bldc_motor_bootstrap_run(main_get_neg_pressure_duty());
    }

    while(1)
    {
        image_buzzer_update();
        image_laser_update();

        switch(state_get_mode())
        {
            /* UI状态 */
            case STATE_UI:
            {
                bldc_motor_stop();
                image_update();
                ui_update();
                if(ui_is_camera_view())
                {
                    servo_update();
                }
                else
                {
                    servo_set_center();
                }
                break;
            }
            
            /* WiFi状态 */
            case STATE_WIFI:
            {
                bldc_motor_stop();
                motor_update();
                wifi_update();
                break;
            }
            
            /* Run状态 */
            case STATE_RUN:
            {
                main_apply_run_neg_pressure();
                image_update();
                if(STATE_RUN != state_get_mode())
                {
                    bldc_motor_stop();
                    servo_set_center();
                    motor_update();
                    break;
                }
                servo_update();
                motor_update();
                break;
            }

            /* 零速闭环刹停状态 */
            case STATE_BRAKE_STOP:
            {
                bldc_motor_stop();
                servo_set_center();
                motor_update();
                break;
            }

            /* Stop状态 */
            case STATE_STOP:
            default:
            {
                bldc_motor_stop();
                // 停止电机
                motor_stop();
                servo_set_center();
                break;
            }
        }
    }
}
