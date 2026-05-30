#include "headfile.h"


void main(void)
{
    clock_init(SYSTEM_CLOCK_96M); 				// 时钟配置及系统初始化<务必保留>
    debug_init();
    para_init();
    flash_init();
	state_init();
	buzzer_init();
    motor_init();
	servo_init();
    if(CarMode != CAR_MODE_STOP)
    {
        image_init();
    }

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
                image_update();
                if(ui_is_debug())
                {
                    servo_update();
                }
                else
                {
                    pwm_set_duty(SERVO_PWM, SERVO_ANGLE_CENTER/3+1500);
                }
                ui_update();
            } break;

            case CAR_MODE_RUN:
            {
                image_update();
                servo_update();
            } break;

            case CAR_MODE_STOP:
            {
								pwm_set_duty(SERVO_PWM, SERVO_ANGLE_CENTER/3+1500);
                motor_output(0, 0);
                fan_set_duty(0);
            } break;
        }
    }
}


