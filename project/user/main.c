
#include "zf_common_headfile.h"
#include "car_init.h"
#include "car_motor.h"

// **************************** 代码区域 ****************************

void main(void)
{
    clock_init(SYSTEM_CLOCK_96M); 				// 时钟配置及系统初始化<务必保留>
    debug_init();                       		// 调试串口信息初始化

    car_init(0, NULL, NULL, NULL);

    car_motor_stop_all();
    car_motor_set_speed(RIGHT_MOTOR, 20);            // 右后轮：往前走，PWM=20%

    while(1)
    {

        system_delay_ms(200);
    }
}
// **************************** 代码区域 ****************************
