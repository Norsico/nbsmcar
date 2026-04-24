/*
 * AttitudeSolve.h
 *
 *  Created on: 2024Äê1ÔÂ9ÈÕ
 *      Author: 15958
 */

#ifndef CODE_ATTITUDESOLVE_H_
#define CODE_ATTITUDESOLVE_H_

#include "zf_device_imu660ra.h"
#include "zf_common_function.h"
#include "zf_driver_delay.h"

#define Balance_Angle   balanceAngle
#define GyroOffsetCaculate 1

typedef struct
{
    float gyroOffsetX;
    float gyroOffsetY;
    float gyroOffsetZ;
} GyroOffsetTypeDef;

typedef struct
{
    float gyroX;
    float gyroY;
    float gyroZ;
    float accX;
    float accY;
    float accZ;
} IMUDataTypeDef;

typedef struct
{
    float q0;
    float q1;
    float q2;
    float q3;
} QuaterTypedef;

typedef struct
{
    float pitch;    //¸©Ñö½Ç
    float roll;     //Æ«º½½Ç
    float yaw;      //·­¹ö½Ç
} EulerTypedef;

extern EulerTypedef g_eulerAngle;
extern IMUDataTypeDef g_imuData;
extern GyroOffsetTypeDef m_gyroOffset;

extern float balanceAngle;

void IMU_get_values(void);
void IMU_get_euler(void);
void IMU_gyro_offset_init(void);

#endif /* CODE_ATTITUDESOLVE_H_ */



