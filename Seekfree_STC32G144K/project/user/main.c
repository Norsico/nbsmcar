#include "zf_common_headfile.h"
#include "motor.h"

void main(void)
{
    clock_init(SYSTEM_CLOCK_96M);            // 时钟配置及系统初始化<务必保留>
    debug_init();                            // 调试串口信息初始化

    motor_init();
    motor_set_target(0, 0);                  // 上电停轮

    while(1)
    {
        motor_update();
    }
}
