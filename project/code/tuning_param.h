/*********************************************************************************************************************
* 在线调参模块
*
* 功能说明：封装逐飞助手调参功能，提供简单的接口
*
* 修改记录
* 日期              备注
* 2026-03-13        创建调参模块
********************************************************************************************************************/

#ifndef _TUNING_PARAM_H_
#define _TUNING_PARAM_H_

#include "zf_common_headfile.h"

// 调参通道数量（0-7，最多8个通道）
#define TUNING_PARAM_CHANNEL_MAX    (8)

// 函数声明
void tuning_param_update(void);                        // 更新调参参数（在主循环调用）
float tuning_param_get(uint8 channel);                 // 获取指定通道参数值
void tuning_param_set_callback(uint8 channel, void (*callback)(float));  // 设置参数变化回调

#endif