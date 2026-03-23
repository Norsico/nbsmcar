#include "dev_motor.h"

// 无刷电机速度控制

// 无刷电机PWM初始化
void bldc_motor_init(void){
	pwm_init(BLDC_MOTOR_1,BLDC_MOTOR_FREQ,0);
	pwm_init(BLDC_MOTOR_2,BLDC_MOTOR_FREQ,0);
}
/**
 * @brief 计算速度转换函数
 * @note 电调高电平范围1ms~2ms，对应0%~100%
 * @param percentage 输出功率大小
 * @return 返回经计算的 duty 数值
*/
static uint32 transform_per_to_duty(uint8 percentage){
	uint32 min_duty = (1 * BLDC_MOTOR_FREQ * 10); // 1000ms/50Hz = 20ms ; 1/20*100=占空比 ; 占空比*100=duty
	uint32 max_duty = (2 * BLDC_MOTOR_FREQ * 10);
	uint32 step_duty = (1 * BLDC_MOTOR_FREQ * 10 / 100); // 映射 1ms 间隔对应 100% ，0.01ms - 1%
	if(percentage >100){
		return max_duty; //直接返回最大值
	}
	return min_duty + percentage * step_duty;	
}	
/**
 * @brief 同时设置两个电机的占空比
 * @param left_speed 左电机占空比（百分比）
 * @param right_speed 右电机占空比（百分比）
*/
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
void bldc_motor_stop(void){
	bldc_motor_set_duty(0,0);
}
	