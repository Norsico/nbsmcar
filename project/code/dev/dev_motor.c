#include "dev_motor.h"

// 无刷电机速度控制

// 无刷电机PWM初始化
/* 初始化两路无刷输出。 */
void bldc_motor_init(void){
	pwm_init(BLDC_MOTOR_1,BLDC_MOTOR_FREQ,0);
	pwm_init(BLDC_MOTOR_2,BLDC_MOTOR_FREQ,0);
}
/* 百分比转电调占空比。 */
static uint32 transform_per_to_duty(uint8 percentage){
	uint32 min_duty = (1 * BLDC_MOTOR_FREQ * 10); // 1000ms/50Hz = 20ms ; 1/20*100=占空比 ; 占空比*100=duty
	uint32 max_duty = (2 * BLDC_MOTOR_FREQ * 10);
	uint32 step_duty = (1 * BLDC_MOTOR_FREQ * 10 / 100); // 映射 1ms 间隔对应 100% ，0.01ms - 1%
	if(percentage >100){
		return max_duty; //直接返回最大值
	}
	return min_duty + percentage * step_duty;
}
/* 同时设置两路无刷占空比。 */
void bldc_motor_set_duty(uint8 left_speed,uint8 right_speed){
	uint32 left = 0;
	uint32 right = 0;
	if(left_speed >100 || right_speed > 100)
		return; // 超过之间不响应
	left = transform_per_to_duty(left_speed);
	right = transform_per_to_duty(right_speed);
	pwm_set_duty(BLDC_MOTOR_1,left);
	pwm_set_duty(BLDC_MOTOR_2,right);
}
// 停止电机
/* 停止无刷电机。 */
void bldc_motor_stop(void){
	bldc_motor_set_duty(0,0);
}
