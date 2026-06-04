#include "headfile.h"

car_para SmartCar;

void para_init(void)
{
    SmartCar.servo.kp = SERVO_KP;
    SmartCar.servo.kd = SERVO_KD;
    SmartCar.servo.err2_k = SERVO_ERR2_K;
    SmartCar.servo.imu_d = SERVO_IMU_D;
    SmartCar.servo.ackerman = SERVO_ACKERMAN;
    SmartCar.servo.tow_point = SERVO_POINT;

    SmartCar.motor.target_speed = MOTOR_TARGET_SPEED;
    SmartCar.motor.straight_speed = MOTOR_STRAIGHT_SPEED;
    SmartCar.motor.left_kp = MOTOR_LEFT_KP;
    SmartCar.motor.left_ki = MOTOR_LEFT_KI;
    SmartCar.motor.right_kp = MOTOR_RIGHT_KP;
    SmartCar.motor.right_ki = MOTOR_RIGHT_KI;
	
    SmartCar.motor.fan_duty = FAN_DUTY;

    SmartCar.camera.exposure = CAMERA_EXPOSURE;
    SmartCar.camera.gain = CAMERA_GAIN;
    SmartCar.camera.threshold_offset = CAMERA_THRESHOLD_OFFSET;
    SmartCar.camera.laser_row = CAMERA_LASER_ROW;
}
