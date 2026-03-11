#include "zf_common_headfile.h"
#include "car_init.h"
#include "car_servo.h"

#define CAR_SERVO_TEST_ANGLE_COUNT    (4)

static const uint8 car_servo_test_sequence[CAR_SERVO_TEST_ANGLE_COUNT] =
{
    CAR_SERVO_CENTER_ANGLE,
    CAR_SERVO_MAX_ANGLE,
    CAR_SERVO_CENTER_ANGLE,
    CAR_SERVO_MIN_ANGLE
};

void main(void)
{
    uint8 servo_angle_index = 0;

    clock_init(SYSTEM_CLOCK_96M); 				// 时钟配置及系统初始化<务必保留>
    debug_init();                       		// 调试串口信息初始化

    car_init(0, NULL, NULL, NULL);

    while(1)
    {
        car_servo_set_angle(car_servo_test_sequence[servo_angle_index]);
        system_delay_ms(1000);

        servo_angle_index ++;
        if(servo_angle_index >= CAR_SERVO_TEST_ANGLE_COUNT)
        {
            servo_angle_index = 0;
        }
    }
}
// **************************** 代码区域 ****************************
