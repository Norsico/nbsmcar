#include "zf_common_headfile.h"
#include "flash.h"
#include "image.h"
#include "motor.h"
#include "servo.h"
#include "state.h"
#include "ui.h"
#include "wifi.h"

void main(void)
{
    clock_init(SYSTEM_CLOCK_96M);            // 时钟配置及系统初始化<务必保留>
    debug_init();                            // 调试串口信息初始化
    gpio_init(IO_P52, GPO, GPIO_HIGH, GPO_PUSH_PULL);  // 测帧率的灯

    /********** 状态判断 *********/
    state_init();

    /********** 模块初始化 *********/
    motor_init();
    servo_init();

    /********** flash初始化 *********/
    flash_init();
    image_init();

    if(STATE_UI == state_get_mode())
    {
        ui_init();
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
                motor_update();
                wifi_update();
                break;
            }
            
            /* Run状态 */
            case STATE_RUN:
            {
                image_update();
                servo_update();
                motor_update();
                break;
            }

            /* Stop状态 */
            case STATE_STOP:
            default:
            {
                // 停止电机
                motor_stop();
                servo_set_center();
                break;
            }
        }
    }
}
