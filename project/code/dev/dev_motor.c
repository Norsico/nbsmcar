#include "dev_motor.h"
#include "system_state.h"

// 无刷电机速度控制

static uint8 g_bldc_target_left = 0;
static uint8 g_bldc_target_right = 0;
static uint8 g_bldc_output_left = 0;
static uint8 g_bldc_output_right = 0;
static uint8 g_bldc_softstart_active = 0;
static uint8 g_bldc_softstart_step = 0;
static uint32 g_bldc_softstart_tick = 0;

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

/* 直接把当前风扇目标写到电调。 */
static void bldc_motor_apply_output(uint8 left_speed, uint8 right_speed)
{
	uint32 left = 0;
	uint32 right = 0;

	left = transform_per_to_duty(left_speed);
	right = transform_per_to_duty(right_speed);
	pwm_set_duty(BLDC_MOTOR_1,left);
	pwm_set_duty(BLDC_MOTOR_2,right);
}

/* 发车时只做一次先慢后快的起转曲线。 */
static void bldc_motor_update_softstart(void)
{
	const uint8 softstart_steps = 12;
	const uint32 softstart_period_ms = 20;
	const uint16 softstart_den = (uint16)(softstart_steps * softstart_steps);
	uint32 now = g_system_ticks;
	uint16 softstart_num = 0;
	uint8 left_speed = 0;
	uint8 right_speed = 0;

	if(!g_bldc_softstart_active)
	{
		return;
	}

	if((now - g_bldc_softstart_tick) < softstart_period_ms)
	{
		return;
	}

	g_bldc_softstart_tick = now;
	if(g_bldc_softstart_step < softstart_steps)
	{
		g_bldc_softstart_step++;
	}

	softstart_num = (uint16)(g_bldc_softstart_step * g_bldc_softstart_step);
	left_speed = (uint8)(((uint16)g_bldc_target_left * softstart_num) / softstart_den);
	right_speed = (uint8)(((uint16)g_bldc_target_right * softstart_num) / softstart_den);

	g_bldc_output_left = left_speed;
	g_bldc_output_right = right_speed;
	bldc_motor_apply_output(g_bldc_output_left, g_bldc_output_right);

	if(g_bldc_softstart_step >= softstart_steps)
	{
		g_bldc_softstart_active = 0;
		g_bldc_output_left = g_bldc_target_left;
		g_bldc_output_right = g_bldc_target_right;
		bldc_motor_apply_output(g_bldc_output_left, g_bldc_output_right);
	}
}

// 无刷电机PWM初始化
/* 初始化两路无刷输出。 */
void bldc_motor_init(void){
	g_bldc_target_left = 0;
	g_bldc_target_right = 0;
	g_bldc_output_left = 0;
	g_bldc_output_right = 0;
	g_bldc_softstart_active = 0;
	g_bldc_softstart_step = 0;
	g_bldc_softstart_tick = 0;
	pwm_init(BLDC_MOTOR_1,BLDC_MOTOR_FREQ,transform_per_to_duty(0));
	pwm_init(BLDC_MOTOR_2,BLDC_MOTOR_FREQ,transform_per_to_duty(0));
}
/* 同时设置两路无刷占空比。 */
void bldc_motor_set_duty(uint8 left_speed,uint8 right_speed){
	if(left_speed >100 || right_speed > 100)
		return; // 超过之间不响应

	g_bldc_target_left = left_speed;
	g_bldc_target_right = right_speed;

	if((0 == left_speed) && (0 == right_speed))
	{
		g_bldc_softstart_active = 0;
		g_bldc_softstart_step = 0;
		g_bldc_output_left = 0;
		g_bldc_output_right = 0;
		bldc_motor_apply_output(0, 0);
		return;
	}

	if(g_bldc_softstart_active)
	{
		bldc_motor_update_softstart();
		return;
	}

	if((0 == g_bldc_output_left) && (0 == g_bldc_output_right))
	{
		g_bldc_softstart_active = 1;
		g_bldc_softstart_step = 0;
		g_bldc_softstart_tick = g_system_ticks;
		bldc_motor_apply_output(0, 0);
		return;
	}

	/* 已经转起来后，目标变化直接跟随，不再重新软启动。 */
	g_bldc_output_left = left_speed;
	g_bldc_output_right = right_speed;
	bldc_motor_apply_output(g_bldc_output_left, g_bldc_output_right);
}

/* 直接写入目标占空比，用于上电前的阻塞预启动。 */
void bldc_motor_set_duty_direct(uint8 left_speed, uint8 right_speed)
{
	if(left_speed > 100 || right_speed > 100)
	{
		return;
	}

	g_bldc_target_left = left_speed;
	g_bldc_target_right = right_speed;
	g_bldc_output_left = left_speed;
	g_bldc_output_right = right_speed;
	g_bldc_softstart_active = 0;
	g_bldc_softstart_step = 0;
	g_bldc_softstart_tick = g_system_ticks;
	bldc_motor_apply_output(g_bldc_output_left, g_bldc_output_right);
}
// 停止电机
/* 停止无刷电机。 */
void bldc_motor_stop(void){
	bldc_motor_set_duty(0,0);
}

/* 软启动完成后，后轮才允许跟随发车。 */
uint8 bldc_motor_is_ready(void)
{
	if(g_bldc_softstart_active)
	{
		return 0;
	}

	return 1;
}
