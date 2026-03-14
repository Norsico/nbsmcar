/*********************************************************************************************************************
* 在线调参模块实现
*
* 功能说明：封装逐飞助手调参功能，提供简单的接口
*
* 修改记录
* 日期              备注
* 2026-03-13        创建调参模块
********************************************************************************************************************/

#include "tuning_param.h"

// 参数变化回调函数指针数组
static void (*param_callback[TUNING_PARAM_CHANNEL_MAX])(float) = {NULL};

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     更新调参参数
// 参数说明     void
// 返回参数     void
// 使用示例     tuning_param_update();
//-------------------------------------------------------------------------------------------------------------------
void tuning_param_update(void)
{
    uint8 channel = 0;

    // 解析上位机发送的参数
    seekfree_assistant_data_analysis();

    // 检查每个通道是否有参数更新
    for(channel = 0; channel < TUNING_PARAM_CHANNEL_MAX; channel++)
    {
        // 如果参数有更新，且设置了回调函数
        if(seekfree_assistant_parameter_update_flag[channel] && param_callback[channel])
        {
            // 调用回调函数
            param_callback[channel](seekfree_assistant_parameter[channel]);
            // 清除更新标志
            seekfree_assistant_parameter_update_flag[channel] = 0;
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取指定通道参数值
// 参数说明     channel: 通道号（0-7）
// 返回参数     float: 参数值
// 使用示例     float kp = tuning_param_get(0);
//-------------------------------------------------------------------------------------------------------------------
float tuning_param_get(uint8 channel)
{
    if(channel >= TUNING_PARAM_CHANNEL_MAX)
    {
        return 0.0f;
    }

    return seekfree_assistant_parameter[channel];
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置参数变化回调函数
// 参数说明     channel: 通道号（0-7）
//              callback: 回调函数指针
// 返回参数     void
// 使用示例     tuning_param_set_callback(0, motor_pid_set_kp);
//-------------------------------------------------------------------------------------------------------------------
void tuning_param_set_callback(uint8 channel, void (*callback)(float))
{
    if(channel >= TUNING_PARAM_CHANNEL_MAX)
    {
        return;
    }

    param_callback[channel] = callback;
}