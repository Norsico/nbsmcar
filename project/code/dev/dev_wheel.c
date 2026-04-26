#include "dev_wheel.h"
#include "ackerman.h"
#include "dev_encoder.h"
#include "dev_flash.h"
#include "dev_servo.h"
#include "search_line.h"

#define MOTOR_PWM_MAX 99
#define WHELL_PWM_FREQ (17000)
#define CAR_WHEEL_OUTPUT_LIMIT_PERCENT (90)      /* 电机输出百分比限幅。 */
#define CAR_WHEEL_ACKERMAN_CENTER_ANGLE  CAR_SERVO_CENTER_ANGLE /* 阿克曼角度按左负右正换算。 */
static int16 car_wheel_run_request_speed = 0.0f;  /* Start 页速度当前只保留作运行开关。 */
int16 car_wheel_target_speed = 0.0f;  // 整车目标速度

int16 ref_left_target = 0.0f;
int16 ref_right_target = 0.0f;

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
/* 设置单个后轮输出。 */
void car_wheel_set_speed(car_wheel_index_enum wheel, int8 speed_percent)
{
        // pwm_set_duty 中 duty 为 uint32 类型
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
/* 同时设置左右后轮输出。 */
void car_wheel_set_dual(int8 right_wheel_speed_percent, int8 left_wheel_speed_percent)
{
    car_wheel_set_speed(RIGHT_MOTOR, right_wheel_speed_percent);
    car_wheel_set_speed(LEFT_MOTOR, left_wheel_speed_percent);
}

// 停止全部
/* 停止左右后轮。 */
void car_wheel_stop_all(void)
{
    car_wheel_set_dual(0, 0);
}

// 初始化函数
/* 初始化后轮驱动。 */
void car_wheel_init(void)
{
    gpio_init(RIGHT_MOTOR_DIR_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(RIGHT_MOTOR_PWM_PIN, WHELL_PWM_FREQ, 0);

    gpio_init(LEFT_MOTOR_DIR_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(LEFT_MOTOR_PWM_PIN, WHELL_PWM_FREQ, 0);

    car_wheel_stop_all();
}

/**************电机控制PID****************/

// pid初始化时kp、ki、kd参数扩大1000倍，注意不要超过32767
pid_control_t wheel_pid_left;
pid_control_t wheel_pid_right;

static void wheel_pid_init_left(void)
{
    /* 左轮 PID 初始化。 */
    pid_param_init(&wheel_pid_left, 19100, 420, 0, 9900, -9900);
    pid_init(&wheel_pid_left);
}

static void wheel_pid_init_right(void)
{
    /* 右轮 PID 初始化。 */
    pid_param_init(&wheel_pid_right, 18900, 420, 0, 9900, -9900);
    pid_init(&wheel_pid_right);
}

/* 初始化左右轮 PID。 */
void car_wheel_pid_init(void){
	// 左右电机的pid结构体初始化
	wheel_pid_init_left();
	wheel_pid_init_right();
}

/* 更新左右轮目标值。 */
static void car_wheel_apply_target(int16 left_speed, int16 right_speed)
{
    wheel_pid_left.target = left_speed;
    wheel_pid_right.target = right_speed;
}

/* 设置整车基础速度。 */
void car_wheel_set_target(int16 speed)
{
    if(speed > (int16)FlashStartSpeedConfig.max)
    {
        speed = FlashStartSpeedConfig.max;
    }
    if(speed < 0)
    {
        speed = 0;
    }

    car_wheel_run_request_speed = speed;
    if(speed <= 0)
    {
        car_wheel_target_speed = 0;
    }
}

/* 清空单个 PID 累积状态。 */
static void car_wheel_pid_reset_single(pid_control_t *pid)
{
    pid->current = 0;
    pid->error = 0;
    pid->prev_error = 0;
    pid->integral = 0;
    pid->output = 0;
}

// 进入紧急情况情况，清空PID参数
/* 清空后轮闭环状态并停轮。 */
void car_wheel_control_reset(void)
{
    car_wheel_run_request_speed = 0;
    car_wheel_target_speed = 0;
    ref_left_target = 0;
    ref_right_target = 0;
    car_wheel_apply_target(0, 0);
    car_wheel_pid_reset_single(&wheel_pid_left);
    car_wheel_pid_reset_single(&wheel_pid_right);
    encoder_clear();
    car_wheel_stop_all();
}

/* 紧急状态下按当前轮速给反向小占空比，尽快把后轮拖停。 */
void car_wheel_emergency_brake(void)
{
    const int16 emergency_release_speed = 12;   /* 编码器绝对值低于这个门槛后，认为车轮已基本停住。 */
    const int8 emergency_brake_percent = 35;    /* 反拖刹车占空比，先用中等力度避免明显倒窜。 */
    int16 current_left = 0;
    int16 current_right = 0;
    int8 left_output = 0;
    int8 right_output = 0;

    car_wheel_run_request_speed = 0;
    car_wheel_target_speed = 0;
    ref_left_target = 0;
    ref_right_target = 0;
    car_wheel_apply_target(0, 0);
    car_wheel_pid_reset_single(&wheel_pid_left);
    car_wheel_pid_reset_single(&wheel_pid_right);

    current_left = encoder_get_left();
    current_right = encoder_get_right();

    if(current_left > emergency_release_speed)
    {
        left_output = -emergency_brake_percent;
    }
    else if(current_left < -emergency_release_speed)
    {
        left_output = emergency_brake_percent;
    }

    if(current_right > emergency_release_speed)
    {
        right_output = -emergency_brake_percent;
    }
    else if(current_right < -emergency_release_speed)
    {
        right_output = emergency_brake_percent;
    }

    car_wheel_set_speed(LEFT_MOTOR, left_output);
    car_wheel_set_speed(RIGHT_MOTOR, right_output);
}

/* 无刷未就绪时保持后轮待转，不清 Start 速度目标。 */
void car_wheel_hold(void)
{
    car_wheel_target_speed = 0;
    ref_left_target = 0;
    ref_right_target = 0;
    car_wheel_apply_target(0, 0);
    car_wheel_pid_reset_single(&wheel_pid_left);
    car_wheel_pid_reset_single(&wheel_pid_right);
    encoder_clear();
    car_wheel_stop_all();
}

/* 按国一直道/非直道口径切当前整车目标速度。 */
static uint16 car_wheel_get_runtime_target_speed(void)
{
    if(car_wheel_run_request_speed <= 0)
    {
        car_wheel_target_speed = 0;
        return 0;
    }

    car_wheel_target_speed = SearchLine_GetSpeedGoal();
    return car_wheel_target_speed;
}

/* 按当前舵机角解算阿克曼左右轮目标。 */
static void car_wheel_update_reference_target(void)
{
    uint16 base_speed = 0;
    uint16 steer_angle = 0;

    base_speed = car_wheel_get_runtime_target_speed();
    // 扩大1000倍用于阿克曼输入，实际最大应该为20000
    steer_angle = (CAR_WHEEL_ACKERMAN_CENTER_ANGLE - car_servo_get_current_angle());

    ackerman_calc_wheel_speeds(base_speed, steer_angle);
    ref_left_target = ackerman_get_left_speed();
    ref_right_target = ackerman_get_right_speed();

}

/* 将阿克曼结果写入左右轮目标。 */
static void car_wheel_update_target_from_vehicle(void)
{
    if(car_wheel_target_speed <= 0)
    {
        car_wheel_apply_target(0, 0);
        return;
    }

    /* 后轮目标按当前舵机角解算。 */
    car_wheel_apply_target(ref_left_target, ref_right_target);
}

// 将PID计算的PWM写入电机-左轮
/* 输出左轮 PWM。 */
static void car_wheel_update_left(void)
{
	int16 pwm = wheel_pid_left.output;
	int8 speed_percent = 0;
	
		if(pwm>0){
			//正转
			speed_percent = (pwm/MOTOR_PWM_MAX);
			
			// 二次限幅
			if(speed_percent > CAR_WHEEL_OUTPUT_LIMIT_PERCENT) speed_percent = CAR_WHEEL_OUTPUT_LIMIT_PERCENT;
			// 设置速度
		}
		else{
			//反转
			speed_percent = (pwm/MOTOR_PWM_MAX);
			if(speed_percent < -CAR_WHEEL_OUTPUT_LIMIT_PERCENT) speed_percent = -CAR_WHEEL_OUTPUT_LIMIT_PERCENT;
		}
		car_wheel_set_speed(LEFT_MOTOR,speed_percent);
}
// 将PID计算的PWM写入电机-右轮
/* 输出右轮 PWM。 */
static void car_wheel_update_right(void)
{
	int16 pwm = wheel_pid_right.output;
	int8 speed_percent = 0;
	
		if(pwm>0){
			//正转
			speed_percent = (pwm/MOTOR_PWM_MAX);
			
			// 二次限幅
			if(speed_percent > CAR_WHEEL_OUTPUT_LIMIT_PERCENT) speed_percent = CAR_WHEEL_OUTPUT_LIMIT_PERCENT;
			// 设置速度
		}
		else{
			//反转
			speed_percent = (pwm/MOTOR_PWM_MAX);
			if(speed_percent < -CAR_WHEEL_OUTPUT_LIMIT_PERCENT) speed_percent = -CAR_WHEEL_OUTPUT_LIMIT_PERCENT;
		}
		
		car_wheel_set_speed(RIGHT_MOTOR,speed_percent);
}
// 车轮速度更新函数
/* 按当前编码器结果更新后轮闭环。 */
void car_wheel_update(void)
{
	// 获取当前编码器速度（编码器未移植）
	int16 current_left = encoder_get_left();
	int16 current_right = encoder_get_right();

    /* 当前后轮目标只按目标速度和舵机角解算阿克曼差速。 */
    car_wheel_update_reference_target();

    car_wheel_update_target_from_vehicle();

	// 增量式pid计算
	pid_incremental_pi(&wheel_pid_left,current_left,wheel_pid_left.target);
	pid_incremental_pi(&wheel_pid_right,current_right,wheel_pid_right.target);
	// 电机控制
	car_wheel_update_right();
	car_wheel_update_left();
	
}
