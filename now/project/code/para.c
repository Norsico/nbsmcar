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
    SmartCar.servo.st_kp = SERVO_ST_KP;
    SmartCar.servo.st_kd = SERVO_ST_KD;
    SmartCar.servo.st_err2_k = SERVO_ST_ERR2_K;
    SmartCar.servo.st_imu_d = SERVO_ST_IMU_D;
    SmartCar.servo.st_ackerman = SERVO_ST_ACKERMAN;
    SmartCar.servo.st_tow_point = SERVO_ST_POINT;
    SmartCar.servo.in_ring_point = SERVO_IN_RING_POINT;
    SmartCar.servo.out_ring_point = SERVO_OUT_RING_POINT;

    SmartCar.motor.target_speed = MOTOR_TARGET_SPEED;
    SmartCar.motor.straight_speed = MOTOR_STRAIGHT_SPEED;
    SmartCar.motor.ring_speed = MOTOR_RING_SPEED;
    SmartCar.motor.ramp_speed = MOTOR_RAMP_SPEED;
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
    SmartCar.camera.threshold_tri_delta = CAMERA_THRESHOLD_TRI_DELTA;
    SmartCar.camera.laser_test = CAMERA_LASER_TEST;
    SmartCar.camera.laser_fire_us = CAMERA_LASER_FIRE_US;
    SmartCar.camera.laser_interval = CAMERA_LASER_INTERVAL;
    SmartCar.camera.laser_fire_interval = CAMERA_LASER_FIRE_INTERVAL;
    SmartCar.camera.laser_left3_col = CAMERA_LASER_LEFT3_COL;
    SmartCar.camera.laser_left2_col = CAMERA_LASER_LEFT2_COL;
    SmartCar.camera.laser_left1_col = CAMERA_LASER_LEFT1_COL;
    SmartCar.camera.laser_center_col = CAMERA_LASER_CENTER_COL;
    SmartCar.camera.laser_right1_col = CAMERA_LASER_RIGHT1_COL;
    SmartCar.camera.laser_right2_col = CAMERA_LASER_RIGHT2_COL;
    SmartCar.camera.laser_right3_col = CAMERA_LASER_RIGHT3_COL;
    SmartCar.camera.laser_row1 = CAMERA_LASER_ROW1;
    SmartCar.camera.laser_row2 = CAMERA_LASER_ROW2;
    SmartCar.camera.laser_row3 = CAMERA_LASER_ROW3;
    SmartCar.camera.laser_ok_num = CAMERA_LASER_OK_NUM;
    SmartCar.camera.laser_st_interval = CAMERA_LASER_ST_INTERVAL;
    SmartCar.camera.laser_st_fire_interval = CAMERA_LASER_ST_FIRE_INTERVAL;
    SmartCar.camera.laser_st_left3_col = CAMERA_LASER_ST_LEFT3_COL;
    SmartCar.camera.laser_st_left2_col = CAMERA_LASER_ST_LEFT2_COL;
    SmartCar.camera.laser_st_left1_col = CAMERA_LASER_ST_LEFT1_COL;
    SmartCar.camera.laser_st_center_col = CAMERA_LASER_ST_CENTER_COL;
    SmartCar.camera.laser_st_right1_col = CAMERA_LASER_ST_RIGHT1_COL;
    SmartCar.camera.laser_st_right2_col = CAMERA_LASER_ST_RIGHT2_COL;
    SmartCar.camera.laser_st_right3_col = CAMERA_LASER_ST_RIGHT3_COL;
    SmartCar.camera.laser_st_row1 = CAMERA_LASER_ST_ROW1;
    SmartCar.camera.laser_st_row2 = CAMERA_LASER_ST_ROW2;
    SmartCar.camera.laser_st_row3 = CAMERA_LASER_ST_ROW3;
    SmartCar.camera.laser_st_ok_num = CAMERA_LASER_ST_OK_NUM;
    SmartCar.camera.laser_ui_test_col = CAMERA_LASER_UI_TEST_COL;

    SmartCar.other.lap_count = OTHER_LAP_COUNT;
}
