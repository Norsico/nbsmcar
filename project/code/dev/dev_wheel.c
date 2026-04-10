#include "dev_wheel.h"
#include "ackerman.h"
#include "dev_encoder.h"
#include "dev_flash.h"
#include "dev_servo.h"

#define MOTOR_PWM_MAX 9900
#define WHELL_PWM_FREQ (17000)
#define CAR_WHEEL_ENCODER_FILTER_ALPHA (0.08f)   /* 编码器滤波系数，对齐国一参考 `Speed_Encoder_l_/r_`。 */
#define CAR_WHEEL_STRAIGHT_SPEED_ADD  (20.0f)    /* 直道预览加速量，当前仅用于 `speed_goal_eff` 观察值；按 Start 页步进 10 先上抬两格。 */
#define CAR_WHEEL_TARGET_SPEED_MAX    ((float)FLASH_START_SPEED_MAX)   /* 对齐 Start 页速度上限，当前随 `FLASH_START_SPEED_MAX` 放宽到 400。 */
#define CAR_WHEEL_OUTPUT_LIMIT_PERCENT (75)      /* 最终下发到电机的百分比速度限幅，范围 0~100；当前按用户要求放宽到 75。 */
#define CAR_WHEEL_DIFF_MIDDLE_LINE    (39)       /* 对齐参考 `Det_True - 39` 的差速分界。 */
#define CAR_WHEEL_DIFF_RATIO          (6.3f)     /* 对齐参考 `Dif_spd_rat` 默认量级。 */
#define CAR_WHEEL_DIFF_ERR_S          (0.0f)     /* 对齐参考 `err_s` 默认值，当前先不加被动差速补偿。 */
#define CAR_WHEEL_DIFF_DUTY_MAX       (20.0f)    /* 对齐参考差速前的单侧丢线计数限幅。 */
#define CAR_WHEEL_DIFF_SPEED_SCALE    (80.0f)    /* 对齐参考 `AcquareSpeed` 的比例换算。 */
#define CAR_WHEEL_LONG_TURNING_ARC    (0.0067425f) /* 对齐参考 `LTA_Long_Turning_Arc`。 */
static float g_car_wheel_speed_target = 0.0f;
static uint8 g_car_wheel_straight_acc = 0;
static uint8 g_car_wheel_det_true = CAR_WHEEL_DIFF_MIDDLE_LINE;
static uint8 g_car_wheel_left_line = 0;
static uint8 g_car_wheel_right_line = 0;

int16 enc_l_f = 0;
int16 enc_r_f = 0;
float speed_goal_eff = 0.0f;
float ref_left_target = 0.0f;
float ref_right_target = 0.0f;

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

// 停止全部
void car_wheel_stop_all(void)
{
    car_wheel_set_dual(0, 0);
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

static void car_wheel_apply_target(float left_speed, float right_speed)
{
    wheel_pid_left.target = left_speed;
    wheel_pid_right.target = right_speed;
}

void car_wheel_set_target(float speed)
{
    g_car_wheel_speed_target = speed;
    speed_goal_eff = speed;
}

void car_wheel_set_straight_acc(uint8 straight_acc)
{
    g_car_wheel_straight_acc = straight_acc;
}

void car_wheel_set_line_observation(uint8 det_true, uint8 left_line, uint8 right_line)
{
    g_car_wheel_det_true = det_true;
    g_car_wheel_left_line = left_line;
    g_car_wheel_right_line = right_line;
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
    g_car_wheel_speed_target = 0.0f;
    g_car_wheel_straight_acc = 0;
    g_car_wheel_det_true = CAR_WHEEL_DIFF_MIDDLE_LINE;
    g_car_wheel_left_line = 0;
    g_car_wheel_right_line = 0;
    enc_l_f = 0;
    enc_r_f = 0;
    speed_goal_eff = 0.0f;
    ref_left_target = 0.0f;
    ref_right_target = 0.0f;
    car_wheel_apply_target(0.0f, 0.0f);
    car_wheel_pid_reset_single(&wheel_pid_left);
    car_wheel_pid_reset_single(&wheel_pid_right);
    encoder_clear();
    car_wheel_stop_all();
}

static void car_wheel_update_reference_diff_preview(int16 current_left, int16 current_right)
{
    float err = 0.0f;
    float steer_turn_duty = 0.0f;
    float steer_duty = 0.0f;
    float acquare_speed = 0.0f;
    float steer_turn_angle = 0.0f;
    float error_speed = 0.0f;
    int16 pulse_differential = 0;

    err = speed_goal_eff - ((float)current_left + (float)current_right) / 2.0f;
    if(g_car_wheel_det_true > CAR_WHEEL_DIFF_MIDDLE_LINE)
    {
        steer_turn_duty = (float)g_car_wheel_right_line;
    }
    else
    {
        steer_turn_duty = (float)g_car_wheel_left_line;
    }

    steer_duty = steer_turn_duty;
    if(steer_duty < 0.0f)
    {
        steer_duty = -steer_duty;
    }
    if(steer_duty > CAR_WHEEL_DIFF_DUTY_MAX)
    {
        steer_duty = CAR_WHEEL_DIFF_DUTY_MAX;
    }

    acquare_speed = (((float)enc_l_f + (float)enc_r_f) / 2.0f) * CAR_WHEEL_DIFF_SPEED_SCALE;
    steer_turn_angle = CAR_WHEEL_DIFF_RATIO * steer_duty;
    error_speed = CAR_WHEEL_LONG_TURNING_ARC * steer_turn_angle * acquare_speed;
    if(error_speed >= 0.0f)
    {
        pulse_differential = (int16)(error_speed / CAR_WHEEL_DIFF_SPEED_SCALE + 0.5f);
    }
    else
    {
        pulse_differential = (int16)(error_speed / CAR_WHEEL_DIFF_SPEED_SCALE - 0.5f);
    }

    ref_left_target = speed_goal_eff + err * CAR_WHEEL_DIFF_ERR_S;
    ref_right_target = speed_goal_eff + err * CAR_WHEEL_DIFF_ERR_S;
    if(g_car_wheel_det_true > CAR_WHEEL_DIFF_MIDDLE_LINE)
    {
        ref_right_target -= (float)pulse_differential;
    }
    else if(g_car_wheel_det_true < CAR_WHEEL_DIFF_MIDDLE_LINE)
    {
        ref_left_target -= (float)pulse_differential;
    }

    if(ref_left_target < 0.0f)
    {
        ref_left_target = 0.0f;
    }
    if(ref_right_target < 0.0f)
    {
        ref_right_target = 0.0f;
    }
}

static void car_wheel_update_reference_preview(int16 current_left, int16 current_right)
{
    float left_filtered = 0.0f;
    float right_filtered = 0.0f;
    float effective_speed = 0.0f;

    left_filtered = CAR_WHEEL_ENCODER_FILTER_ALPHA * (float)current_left +
                    (1.0f - CAR_WHEEL_ENCODER_FILTER_ALPHA) * (float)enc_l_f;
    right_filtered = CAR_WHEEL_ENCODER_FILTER_ALPHA * (float)current_right +
                     (1.0f - CAR_WHEEL_ENCODER_FILTER_ALPHA) * (float)enc_r_f;
    enc_l_f = (int16)left_filtered;
    enc_r_f = (int16)right_filtered;

    effective_speed = g_car_wheel_speed_target;
    if(g_car_wheel_straight_acc && (g_car_wheel_speed_target > 0.0f))
    {
        effective_speed += CAR_WHEEL_STRAIGHT_SPEED_ADD;
    }

    if(effective_speed > CAR_WHEEL_TARGET_SPEED_MAX)
    {
        effective_speed = CAR_WHEEL_TARGET_SPEED_MAX;
    }
    if(effective_speed < 0.0f)
    {
        effective_speed = 0.0f;
    }

    speed_goal_eff = effective_speed;
    car_wheel_update_reference_diff_preview(current_left, current_right);
}

/* 当前后轮目标直接切到阶段 7 的参考差速结果，不再保留左右同目标。 */
static void car_wheel_update_target_from_vehicle(void)
{
    if(g_car_wheel_speed_target <= 0.0f)
    {
        car_wheel_apply_target(0.0f, 0.0f);
        return;
    }

    /* 普通赛道阶段 8：后轮目标改用参考差速预览值。 */
    car_wheel_apply_target(ref_left_target, ref_right_target);
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
// 将PID计算的PWM写入电机-左轮
static void car_wheel_update_left(void)
{
	int16 pwm = wheel_pid_left.output;
	int8 speed_percent = 0;
	
		if(pwm>0){
			//正转
			speed_percent = (int8)(pwm*100.0f/MOTOR_PWM_MAX);
			
			// 二次限幅，可不用
			if(speed_percent > CAR_WHEEL_OUTPUT_LIMIT_PERCENT) speed_percent = CAR_WHEEL_OUTPUT_LIMIT_PERCENT;
			// 设置速度
		}
		else{
			//反转
			speed_percent = (int8)(pwm*100.0f/MOTOR_PWM_MAX);
			if(speed_percent < -CAR_WHEEL_OUTPUT_LIMIT_PERCENT) speed_percent = -CAR_WHEEL_OUTPUT_LIMIT_PERCENT;
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
			if(speed_percent > CAR_WHEEL_OUTPUT_LIMIT_PERCENT) speed_percent = CAR_WHEEL_OUTPUT_LIMIT_PERCENT;
			// 设置速度
		}
		else{
			//反转
			speed_percent = (int8)(pwm*100.0f/MOTOR_PWM_MAX);
			if(speed_percent < -CAR_WHEEL_OUTPUT_LIMIT_PERCENT) speed_percent = -CAR_WHEEL_OUTPUT_LIMIT_PERCENT;
		}
		car_wheel_set_speed(RIGHT_MOTOR,speed_percent);
}
// 车轮速度更新函数
void car_wheel_update(void)
{
	// 获取当前编码器速度（编码器未移植）
	int16 current_left = encoder_get_left();
	int16 current_right = encoder_get_right();

    /* 先把国一电机侧的观测前置量补出来，后轮目标仍沿用当前链路。 */
    car_wheel_update_reference_preview(current_left, current_right);

    car_wheel_update_target_from_vehicle();

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
