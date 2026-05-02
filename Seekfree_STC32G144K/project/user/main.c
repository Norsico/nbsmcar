#include "zf_common_headfile.h"
#include "motor.h"
#include "state.h"

void main(void)
{
    clock_init(SYSTEM_CLOCK_96M);            // 时钟配置及系统初始化<务必保留>
    debug_init();                            // 调试串口信息初始化

    /********** 状态初始化 *********/
    state_init();

    /********** 模块初始化 *********/
    motor_init();

    /********** 参数初始化 **********/
    motor_set_target(0, 0);                  // 停止后轮

    while(1)
    {
        switch(state_get_mode())
        {
            /* UI状态 */
            case STATE_UI:
            {

                break;
            }
            
            /* WiFi状态 */
            case STATE_WIFI:
            {

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
