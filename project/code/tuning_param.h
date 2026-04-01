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
void tuning_param_boot_init(void);                     // 启动阶段初始化调参配置
uint8 tuning_param_start_transport(void);             // 按配置启动WiFi连接
void tuning_param_task(void);                         // 调参总任务（接收+发送）
void tuning_param_update(void);                        // 更新调参参数（在主循环调用）
float tuning_param_get(uint8 channel);                 // 获取指定通道参数值
void tuning_param_set_callback(uint8 channel, void (*callback)(float));  // 设置参数变化回调
uint8 tuning_param_is_active(void);                    // 当前是否进入调参模式
uint8 tuning_param_should_disable_display(void);       // 调参时是否关闭IPS显示
uint8 tuning_param_should_skip_line_init(void);        // 调参时是否跳过巡线初始化
uint8 tuning_param_should_pause_line_app(void);        // 调参时是否暂停巡线处理

#endif
