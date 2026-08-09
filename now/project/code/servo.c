#include "headfile.h"

static int16 ServoAngle = SERVO_ANGLE_CENTER;  // 舵机当前角度
static int16 ServoLastError = 0;               // 上一时刻误差
static uint16 ServoLastSequence = 0;

// 限幅
static int16 servo_limit(int16 angle)
{
    if(angle < SERVO_ANGLE_MIN)
    {
        return SERVO_ANGLE_MIN;
    }
    else if(angle > SERVO_ANGLE_MAX)
    {
        return SERVO_ANGLE_MAX;
    }
    else return angle;
}


// 差速控制
void servo_update_motor_target(void)
{
    int16 speed;
    int16 steer_angle;
    int16 tan_value;
    int16 speed_delta;
    int16 ackerman;
    int32 diff_scale;

    /* 停车状态优先，其余状态沿用原有速度选择。 */
    if(BlindBoxPhase == BLIND_BOX_STOP)
    {
        speed = 0;
    }
    else if(Image.is_ramp)
    {
        speed = SmartCar.motor.ramp_speed;
    }
    else if(Image.ring != 0)
    {
        speed = SmartCar.motor.ring_speed;
    }
    else if(Image.param_st)
    {
        speed = SmartCar.motor.straight_speed;
    }
    else
    {
        speed = SmartCar.motor.target_speed;
    }

    steer_angle = (int16)(SERVO_ANGLE_CENTER - ServoAngle);
    tan_value = (int16)(((int32)steer_angle * 175) / 1000);
    if((Image.param_st != 0) && (Image.ring == 0) && (Image.is_ramp == 0))
    {
        ackerman = SmartCar.servo.st_ackerman;
    }
    else if(Image.error < 0)
    {
        ackerman = SmartCar.servo.left_ackerman;
    }
    else
    {
        ackerman = SmartCar.servo.ackerman;
    }
    diff_scale = ((int32)ackerman * tan_value) / 100;
    speed_delta = (int16)(((int32)speed * diff_scale) / 10000);

    if(speed_delta <= 0)
    {
        Motor.target_left = speed + speed_delta;
		Motor.target_right = speed;
    }
    else if(speed_delta > 0)
    {
        Motor.target_right = speed- speed_delta ;
		Motor.target_left = speed;
    }
}

void servo_init(void)
{
    ServoAngle = SERVO_ANGLE_CENTER;
    ServoLastError = 0;
    ServoLastSequence = 0;

    pwm_init(SERVO_PWM, SERVO_PWM_FREQ, SERVO_ANGLE_CENTER/3+1500);
	while(1){
		if(imu660ra_init())
			printf("\r\nIMU660RA init error.");      // IMU660RA 初始化失败
      else
        break;
	}
}

void servo_update(void)
{
    int16 error;
    int16 error_d;
    int16 control;
    int16 kp;
    int16 kd;
    int16 err2_k;
    int16 imu_d;

    if((Image.ready == 0) || (Image.result_ready == 0))
    {
        return;
    }
    if(Image.sequence == ServoLastSequence)
    {
        return;
    }
    ServoLastSequence = Image.sequence;

    error = Image.error;
    error_d = error - ServoLastError;

    if((Image.param_st != 0) && (Image.ring == 0) && (Image.is_ramp == 0))
    {
        kp = SmartCar.servo.st_kp;
        kd = SmartCar.servo.st_kd;
        err2_k = SmartCar.servo.st_err2_k;
        imu_d = SmartCar.servo.st_imu_d;
    }
    else if(error < 0)
    {
        kp = SmartCar.servo.left_kp;
        kd = SmartCar.servo.left_kd;
        err2_k = SmartCar.servo.left_err2_k;
        imu_d = SmartCar.servo.left_imu_d;
    }
    else
    {
        kp = SmartCar.servo.kp;
        kd = SmartCar.servo.kd;
        err2_k = SmartCar.servo.err2_k;
        imu_d = SmartCar.servo.imu_d;
    }

    	imu660ra_get_gyro();
    if(error >= 0)
    {
        control = (int16)(kp * error + kd * error_d
							+ (int32)err2_k * error * error / 10
							- (int32)imu_d * imu660ra_gyro_z / 100);
    }
    else
    {
        control = (int16)(kp * error + kd * error_d
							- (int32)err2_k * error * error / 10
							- (int32)imu_d * imu660ra_gyro_z / 100);
    }
    ServoLastError = error;

	ServoAngle = servo_limit(SERVO_ANGLE_CENTER - control);
    pwm_set_duty(SERVO_PWM, ServoAngle/3+1500);

    servo_update_motor_target();
}
