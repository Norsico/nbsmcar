#include "dev_wheel.h"
#include "dev_encoder.h"

#define MOTOR_PWM_MAX 9900
#define WHELL_PWM_FREQ (17000)

/**************电机控制PWM*******************/
// 电机速度限幅函数，-100~100
static int8 car_wheel_limit_speed(int8 speed_percent)
{
    if(speed_percent > 100)
    {
        return 100;
    }

    if(speed_percent < -100)
    {
        return -100;
    }

    return speed_percent;
}
// 单独设置速度
void car_wheel_set_speed(car_wheel_index_enum wheel, int8 speed_percent)
{
		uint32 duty = 0;
		int8 speed = car_wheel_limit_speed(speed_percent);
	
		if(wheel == RIGHT_MOTOR){
			// 右电机
			if(speed>0){
				gpio_set_level(RIGHT_MOTOR_DIR_PIN,GPIO_LOW);
				duty = (uint32)(speed) * (PWM_DUTY_MAX / 100);
			}
			else{
				gpio_set_level(RIGHT_MOTOR_DIR_PIN,GPIO_HIGH);
				duty = (uint32)(-speed) * (PWM_DUTY_MAX / 100);				
			}
			pwm_set_duty(RIGHT_MOTOR_PWM_PIN, duty);
		}
		else if (wheel == LEFT_MOTOR){
			// 左电机
			if(speed>0){
				gpio_set_level(LEFT_MOTOR_DIR_PIN,GPIO_HIGH);
				duty = (uint32)(speed) * (PWM_DUTY_MAX / 100);
			}
			else{
				gpio_set_level(LEFT_MOTOR_DIR_PIN,GPIO_LOW);
				duty = (uint32)(-speed) * (PWM_DUTY_MAX / 100);				
			}
			pwm_set_duty(LEFT_MOTOR_PWM_PIN, duty);			
		}
}

// 设置电机占空比
void car_wheel_set_dual(int8 right_wheel_speed_percent, int8 left_wheel_speed_percent)
{
    car_wheel_set_speed(RIGHT_MOTOR, right_wheel_speed_percent);
    car_wheel_set_speed(LEFT_MOTOR, left_wheel_speed_percent);
}


// 全部停止
void car_wheel_stop_all(void)
{
    pwm_set_duty(RIGHT_MOTOR_PWM_PIN, 0);
    pwm_set_duty(LEFT_MOTOR_PWM_PIN, 0);
}

static void car_wheel_pid_reset_single(pid_control_t *pid)
{
	  pid->current = 0.0f;
    pid->error = 0.0f;
    pid->prev_error = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
}

/* 启停切换时把编码器和 PID 状态一起清掉，避免旧输出残留导致瞬时抽动。 */
void car_wheel_control_reset(void)
{
	  wheel_pid_left.target = 0.0f;
    wheel_pid_right.target = 0.0f;
    car_wheel_pid_reset_single(&wheel_pid_left);
    car_wheel_pid_reset_single(&wheel_pid_right);
    encoder_clear();
		car_wheel_stop_all();
}


// 初始化函数
void car_wheel_init(void)
{
    gpio_init(RIGHT_MOTOR_DIR_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(RIGHT_MOTOR_PWM_PIN, WHELL_PWM_FREQ, 0);

    gpio_init(LEFT_MOTOR_DIR_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(LEFT_MOTOR_PWM_PIN, WHELL_PWM_FREQ, 0);

    car_wheel_stop_all();
}

/**************电机控制PID****************/

pid_control_t wheel_pid_left;
pid_control_t wheel_pid_right;

static void wheel_pid_init(pid_control_t* pid){
	// 参数结构体设置
	pid->param.kp = 7.0f;
	pid->param.ki = 1.16f;
	pid->param.kd = 0.0f;
	pid->param.max_out = 9900.0; // 电机控制百分比
	pid->param.min_out = -9900.0;
	pid->param.deadband = 0.0f; // 死区用于消除误差极小时的抖动，电机不设死区
	// 控制器结构体
	pid->target = 0;
	pid->current = 0;
	pid->error = 0;
	pid->prev_error = 0;
	pid->integral = 0;
	pid->output = 0;
	pid->dt = 0.01f; // 0.01s(10ms)
}

void car_wheel_pid_init(void){
	// 左右电机的pid结构体初始化
	wheel_pid_init(&wheel_pid_left);
	wheel_pid_init(&wheel_pid_right);
}

void car_wheel_set_target(float left_speed, float right_speed)
{
	wheel_pid_left.target = left_speed;
	wheel_pid_right.target = right_speed;
}
// 将PID计算的PWM写入电机-左轮
static void car_wheel_update_left(void)
{
	int16 pwm = wheel_pid_left.output;
	int8 speed_percent = 0;
	
	if(pwm>0){
		//正转
		speed_percent = (int8)(pwm*100.0f/MOTOR_PWM_MAX);
		
		// 二次限幅，可不用
		if(speed_percent >40) speed_percent = 40;
		// 设置速度
	}
	else{
		//反转
		speed_percent = (int8)(pwm*100.0f/MOTOR_PWM_MAX);
		if(speed_percent<-40) speed_percent = -40;
	}
	car_wheel_set_speed(LEFT_MOTOR,speed_percent);
}
// 将PID计算的PWM写入电机-右轮
static void car_wheel_update_right(void)
{
	int16 pwm = wheel_pid_right.output;
	int8 speed_percent = 0;
	
	if(pwm>0){
		//正转
		speed_percent = (int8)(pwm*100.0f/MOTOR_PWM_MAX);
		
		// 二次限幅，可不用
		if(speed_percent >40) speed_percent = 40;
		// 设置速度
	}
	else{
		//反转
		speed_percent = (int8)(pwm*100.0f/MOTOR_PWM_MAX);
		if(speed_percent<-40) speed_percent = -40;
	}
	car_wheel_set_speed(RIGHT_MOTOR,speed_percent);
}
static void car_wheel_limit_output_to_target_direction(pid_control_t *pid)
{
	  if(pid->target > 0.0f)
    {
        if(pid->output < 0.0f)
        {
            pid->output = 0.0f;
        }
    }
    else if(pid->target < 0.0f)
    {
        if(pid->output > 0.0f)
        {
            pid->output = 0.0f;
        }
    }
    else
    {
        pid->output = 0.0f;
    }
}
// 车轮速度更新函数
void car_wheel_update(void)
{
	// 获取当前编码器速度（编码器未移植）
	int16 current_left = encoder_get_left();
	int16 current_right = encoder_get_right();
	// 增量式pid计算
	pid_incremental_pi(&wheel_pid_left,current_left,wheel_pid_left.target);
	pid_incremental_pi(&wheel_pid_right,current_right,wheel_pid_right.target);
	/* 前进目标下不允许直接打反转，先把瞬时反向输出钳掉。 */
  car_wheel_limit_output_to_target_direction(&wheel_pid_left);
  car_wheel_limit_output_to_target_direction(&wheel_pid_right);
	// 电机控制
	car_wheel_update_left();
	car_wheel_update_right();
}
