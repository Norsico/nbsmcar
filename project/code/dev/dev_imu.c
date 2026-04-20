/*
 * dev_imu.c - 陀螺仪算法实现
 *
 * 包含：角度积分、零偏校准
 */

#include "dev_imu.h"
#include "zf_common_headfile.h"

/************ 零偏校准变量 ************/
static int16 gyro_offset_x;  // 陀螺仪X轴零偏
static int16 gyro_offset_y;  // 陀螺仪Y轴零偏
static int16 gyro_offset_z;  // 陀螺仪Z轴零偏
static uint8 imu_calibrated; // 校准标志


/************ 初始化重试参数 ************/
#define IMU_INIT_RETRY_MAX    5       // 最大重试次数
#define IMU_INIT_RETRY_DELAY  100     // 重试间隔 (ms)

/************ 初始化函数 ************/
/**
 * @brief IMU初始化（带重试机制）
 * @return 0 初始化成功，1 初始化失败
 *
 * 功能说明：
 *   由于IMU可能在上电时未就绪，添加重试机制提高初始化成功率
 *
 * 重试策略：
 *   - 最多重试5次
 *   - 每次间隔100ms
 *   - 任意一次成功即返回成功
 */
uint8 imu_init_with_retry(void)
{
    uint8 retry = 0;

    while(retry < IMU_INIT_RETRY_MAX)
    {
        if(imu_init() == 0)
        {
            // 初始化成功
            return 0;
        }

        // 初始化失败，延时后重试
        retry++;
        if(retry < IMU_INIT_RETRY_MAX)
        {
            system_delay_ms(IMU_INIT_RETRY_DELAY);
        }
    }

    // 重试次数用尽，初始化失败
    return 1;
}

/************ 角度积分函数 ************/
/**
 * @brief 更新角度积分（需在固定周期调用）
 */
void imu_angle_update(void)
{
    int16 gyro_x, gyro_y, gyro_z;

    // 获取去除零偏后的角速度
    gyro_x = imu_get_gyro_x() - gyro_offset_x;
    gyro_y = imu_get_gyro_y() - gyro_offset_y;
    gyro_z = imu_get_gyro_z() - gyro_offset_z;

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
