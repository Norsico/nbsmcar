#ifndef _DEV_ENCODER_H_
#define _DEV_ENCODER_H_

#include "zf_common_typedef.h"

// 编码器数据结构
typedef struct {
    int16 left;     // 左轮编码器计数值
    int16 right;    // 右轮编码器计数值
} encoder_data_t;

// 全局编码器数据
extern encoder_data_t encoder_data;

// 函数声明
void encoder_init(void);                    // 编码器初始化
void encoder_update(uint8 sample_count);    // 更新编码器数据，sample_count 表示累计了多少个采样周期
int16 encoder_get_left(void);               // 获取左轮编码器数据
int16 encoder_get_right(void);              // 获取右轮编码器数据
void encoder_clear(void);                   // 清空编码器计数


#endif
