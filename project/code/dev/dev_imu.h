/*
 * dev_imu.h - 陀螺仪驱动模块
 *
 * 用于获取 IMU660RA 陀螺仪数据
 * 注意: 直接使用逐飞库函数，无需重新封装
 */

#ifndef __DEV_IMU_H__
#define __DEV_IMU_H__

/************ 头文件 ************/
#include "zf_common_typedef.h"
#include "zf_device_imu660ra.h"

/************ 逐飞库提供的函数和变量 ************/
// 初始化
// imu660ra_init();

// 读取数据
// imu660ra_get_acc();    // 读取加速度
// imu660ra_get_gyro();  // 读取角速度

// 原始值变量
// imu660ra_gyro_x, imu660ra_gyro_y, imu660ra_gyro_z  // 角速度
// imu660ra_acc_x, imu660ra_acc_y, imu660ra_acc_z      // 加速度

// 转换函数 (原始值 -> 实际值)
// imu660ra_gyro_transition(value)  // 返回 °/s
// imu660ra_acc_transition(value)    // 返回 g

/************ 接口函数 ************/
// 陀螺仪模块初始化（单次尝试）
#define imu_init()    imu660ra_init()

// 陀螺仪模块初始化（带重试）
extern uint8 imu_init_with_retry(void);

// 更新陀螺仪数据（读取传感器）
#define imu_update()  do{ imu660ra_get_acc(); imu660ra_get_gyro(); }while(0)

// 获取角速度原始值
#define imu_get_gyro_x()   imu660ra_gyro_x
#define imu_get_gyro_y()   imu660ra_gyro_y
#define imu_get_gyro_z()   imu660ra_gyro_z

// 获取加速度原始值
#define imu_get_acc_x()   imu660ra_acc_x
#define imu_get_acc_y()   imu660ra_acc_y
#define imu_get_acc_z()   imu660ra_acc_z

// 获取角速度转换值 (单位: °/s)
#define imu_get_gyro_x_f()   imu660ra_gyro_transition(imu660ra_gyro_x)
#define imu_get_gyro_y_f()   imu660ra_gyro_transition(imu660ra_gyro_y)
#define imu_get_gyro_z_f()   imu660ra_gyro_transition(imu660ra_gyro_z)

// 获取加速度转换值 (单位: g)
#define imu_get_acc_x_f()   imu660ra_acc_transition(imu660ra_acc_x)
#define imu_get_acc_y_f()   imu660ra_acc_transition(imu660ra_acc_y)
#define imu_get_acc_z_f()   imu660ra_acc_transition(imu660ra_acc_z)

/************ 角度积分算法 ************/
// 更新角度积分（需在固定周期调用，推荐 10ms）
extern void imu_angle_update(void);

/************ 零偏校准算法 ************/
// 执行零偏校准（需要在静止状态下调用）
extern void imu_calibrate(uint16 samples);

// 检查是否已校准
extern uint8 imu_is_calibrated(void);

// 手动设置零偏
extern void imu_set_offset(int16 offset_x, int16 offset_y, int16 offset_z);

// 获取零偏值
extern int16 imu_get_offset_x(void);
extern int16 imu_get_offset_y(void);
extern int16 imu_get_offset_z(void);

#endif /* __DEV_IMU_H__ */
