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
    SmartCar.servo.in_ring_point = SERVO_IN_RING_POINT;
    SmartCar.servo.out_ring_point = SERVO_OUT_RING_POINT;

    SmartCar.motor.target_speed = MOTOR_TARGET_SPEED;
    SmartCar.motor.straight_speed = MOTOR_STRAIGHT_SPEED;
    SmartCar.motor.ring_speed = MOTOR_RING_SPEED;
    SmartCar.motor.left_kp = MOTOR_LEFT_KP;
    SmartCar.motor.left_ki = MOTOR_LEFT_KI;
    SmartCar.motor.right_kp = MOTOR_RIGHT_KP;
    SmartCar.motor.right_ki = MOTOR_RIGHT_KI;

    SmartCar.motor.fan_duty = FAN_DUTY;
    SmartCar.motor.fan_straight_duty = FAN_STRAIGHT_DUTY;
    SmartCar.motor.fan_en = FAN_ENABLE;

    SmartCar.camera.exposure = CAMERA_EXPOSURE;
    SmartCar.camera.gain = CAMERA_GAIN;
    SmartCar.camera.threshold_offset = CAMERA_THRESHOLD_OFFSET;
    SmartCar.camera.laser_row = CAMERA_LASER_ROW;
    SmartCar.camera.target_gap = CAMERA_TARGET_GAP;
    SmartCar.camera.laser_test = CAMERA_LASER_TEST;
    SmartCar.camera.laser_fire_us = CAMERA_LASER_FIRE_US;
    SmartCar.camera.laser_left2_col = CAMERA_LASER_LEFT2_COL;
    SmartCar.camera.laser_left1_col = CAMERA_LASER_LEFT1_COL;
    SmartCar.camera.laser_center_col = CAMERA_LASER_CENTER_COL;
    SmartCar.camera.laser_right1_col = CAMERA_LASER_RIGHT1_COL;
    SmartCar.camera.laser_right2_col = CAMERA_LASER_RIGHT2_COL;
    SmartCar.camera.laser_ui_test_col = CAMERA_LASER_UI_TEST_COL;
}
