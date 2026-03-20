/*
 * dev_imu.c - 陀螺仪算法实现
 *
 * 包含：角度积分、零偏校准
 */

#include "dev_imu.h"
#include "zf_common_headfile.h"

/************ 角度积分状态变量 ************/
// 累计角度（单位：度）
static float imu_yaw;        // 偏航角 (Z轴)
static float imu_pitch;     // 俯仰角 (X轴)
static float imu_roll;      // 横滚角 (Y轴)

/************ 零偏校准变量 ************/
static int16 gyro_offset_x;  // 陀螺仪X轴零偏
static int16 gyro_offset_y;  // 陀螺仪Y轴零偏
static int16 gyro_offset_z;  // 陀螺仪Z轴零偏
static uint8 imu_calibrated; // 校准标志

/************ 算法参数 ************/
// 采样周期（秒）
static float imu_dt;         // 默认值在 imu_set_dt 中设置

/************ 角度积分函数 ************/
/**
 * @brief 更新角度积分（需在固定周期调用）
 * @param dt 采样周期（秒），如 0.01 表示 10ms
 */
void imu_angle_update(float dt)
{
    int16 gyro_x, gyro_y, gyro_z;
    float gyro_x_f, gyro_y_f, gyro_z_f;

    // 获取去除零偏后的角速度
    gyro_x = imu_get_gyro_x() - gyro_offset_x;
    gyro_y = imu_get_gyro_y() - gyro_offset_y;
    gyro_z = imu_get_gyro_z() - gyro_offset_z;

    // 转换为角度/秒
    gyro_x_f = imu660ra_gyro_transition(gyro_x);
    gyro_y_f = imu660ra_gyro_transition(gyro_y);
    gyro_z_f = imu660ra_gyro_transition(gyro_z);

    // 积分计算角度
    imu_yaw   = imu_yaw + gyro_z_f * dt;
    imu_pitch = imu_pitch + gyro_x_f * dt;
    imu_roll  = imu_roll + gyro_y_f * dt;
}

/**
 * @brief 重置角度积分
 */
void imu_angle_reset(void)
{
    imu_yaw = 0.0f;
    imu_pitch = 0.0f;
    imu_roll = 0.0f;
}

/**
 * @brief 设置采样周期
 * @param dt 采样周期（秒）
 */
void imu_set_dt(float dt)
{
    imu_dt = dt;
}

/************ 零偏校准函数 ************/
/**
 * @brief 执行零偏校准（需要在静止状态下调用）
 * @param samples 采样次数（建议 100-200）
 */
void imu_calibrate(uint16 samples)
{
    int32 sum_x, sum_y, sum_z;
    uint16 i;

    sum_x = 0;
    sum_y = 0;
    sum_z = 0;

    // 连续采样多次求平均（减少延时以避免USB超时）
    for (i = 0; i < samples; i++)
    {
        imu_update();
        sum_x = sum_x + imu_get_gyro_x();
        sum_y = sum_y + imu_get_gyro_y();
        sum_z = sum_z + imu_get_gyro_z();
        system_delay_ms(5);  // 5ms 采样间隔（减少总延时）
    }

    // 计算零偏
    gyro_offset_x = (int16)(sum_x / samples);
    gyro_offset_y = (int16)(sum_y / samples);
    gyro_offset_z = (int16)(sum_z / samples);

    imu_calibrated = 1;

    // 校准后重置角度
    imu_angle_reset();
}

/**
 * @brief 检查是否已校准
 * @return 1 已校准，0 未校准
 */
uint8 imu_is_calibrated(void)
{
    return imu_calibrated;
}

/**
 * @brief 手动设置零偏
 */
void imu_set_offset(int16 offset_x, int16 offset_y, int16 offset_z)
{
    gyro_offset_x = offset_x;
    gyro_offset_y = offset_y;
    gyro_offset_z = offset_z;
    imu_calibrated = 1;
}

/**
 * @brief 获取X轴零偏
 */
int16 imu_get_offset_x(void)
{
    return gyro_offset_x;
}

/**
 * @brief 获取Y轴零偏
 */
int16 imu_get_offset_y(void)
{
    return gyro_offset_y;
}

/**
 * @brief 获取Z轴零偏
 */
int16 imu_get_offset_z(void)
{
    return gyro_offset_z;
}

/************ 获取角度值 ************/
float imu_get_yaw(void)
{
    return imu_yaw;
}

float imu_get_pitch(void)
{
    return imu_pitch;
}

float imu_get_roll(void)
{
    return imu_roll;
}
