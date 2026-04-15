/*********************************************************************************************************************
 * WiFi 遥测模块
 *
 * 功能说明：封装 WiFi 连接和示波器上行能力，不再接收上位机调参写入
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
void tuning_param_boot_init(void);                     // 启动阶段初始化 WiFi 遥测配置
uint8 tuning_param_start_transport(void);             // 按配置启动WiFi连接
void tuning_param_task(void);                         // WiFi 总任务（连接+发送）
void tuning_param_update(void);                       // 兼容旧接口，当前不处理参数下行
float tuning_param_get(uint8 channel);                // 兼容旧接口，当前固定返回 0
void tuning_param_set_callback(uint8 channel, void (*callback)(float));  // 兼容旧接口，当前不注册回调
uint8 tuning_param_is_active(void);                   // 当前是否进入 WiFi 模式
uint8 tuning_param_should_disable_display(void);      // WiFi 模式时是否关闭IPS显示
uint8 tuning_param_should_skip_line_init(void);       // WiFi 模式时是否跳过巡线初始化
uint8 tuning_param_should_pause_line_app(void);       // WiFi 模式时是否暂停巡线处理

#endif
