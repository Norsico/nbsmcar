#include "zf_common_headfile.h"
#include "flash.h"
#include "motor.h"
#include "power.h"
#include "state.h"
#include "ui.h"
#include "wifi.h"

void main(void)
{
    clock_init(SYSTEM_CLOCK_96M);            // 时钟配置及系统初始化<务必保留>
    debug_init();                            // 调试串口信息初始化

    /********** 状态判断 *********/
    state_init();

    /********** 模块初始化 *********/
    motor_init();
    power_init();

    /********** flash初始化 *********/
    flash_init();

    if(STATE_UI == state_get_mode())
    {
        ui_init();
    }

    while(1)
    {
        switch(state_get_mode())
        {
            /* UI状态 */
            case STATE_UI:
            {
                ui_update();
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
                motor_update();
                break;
            }

            /* Stop状态 */
            case STATE_STOP:
            default:
            {
                
                // 停止电机
                motor_set_target(0, 0);
                break;
            }
        }
    }
}
