#include "dev_encoder.h"
#include "zf_driver_encoder.h"

// 左轮编码器定义 (对应物理编码器2，正走时为正数)
#define ENCODER_LEFT                   	(PWMC_ENCODER)              // 左轮编码器接口
#define ENCODER_LEFT_CHA   		        (PWMC_ENCODER_CH1P_P40)     // PULSE 对应的引脚
#define ENCODER_LEFT_CHB       	        (PWMC_ENCODER_CH2P_P42)     // DIR 对应的引脚

// 右轮编码器定义 (对应物理编码器1，需要取反使正走时为正数)
#define ENCODER_RIGHT                 	(PWMA_ENCODER)              // 右轮编码器接口
#define ENCODER_RIGHT_CHA            	(PWMA_ENCODER_CH1P_P60)     // PULSE 对应的引脚
#define ENCODER_RIGHT_CHB               (PWMA_ENCODER_CH2P_P62)     // DIR 对应的引脚

// 全局编码器数据
encoder_data_t encoder_data = {0, 0};

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     编码器初始化
// 参数说明     void
// 返回参数     void
// 使用示例     encoder_init();
//-------------------------------------------------------------------------------------------------------------------
void encoder_init(void)
{
    encoder_quad_init(ENCODER_LEFT, ENCODER_LEFT_CHA, ENCODER_LEFT_CHB);     // 初始化左轮编码器
    encoder_quad_init(ENCODER_RIGHT, ENCODER_RIGHT_CHA, ENCODER_RIGHT_CHB);   // 初始化右轮编码器
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     更新编码器数据（在定时器中断中调用）
// 参数说明     void
// 返回参数     void
// 使用示例     encoder_update();
//-------------------------------------------------------------------------------------------------------------------
void encoder_update(void)
{
    int16 left_raw = 0;
    int16 right_raw = 0;

    left_raw = encoder_get_count(ENCODER_LEFT);
    right_raw = - encoder_get_count(ENCODER_RIGHT);

    if((0 == encoder_data.left) && (0 == encoder_data.right))
    {

        encoder_data.left = left_raw;
        encoder_data.right = right_raw;

        
    }
    else
    {

        /* 编码器一阶滤波。 */
        encoder_data.left = (int16)(((int32)left_raw * 8 + (int32)encoder_data.left * 92) / 100);
        encoder_data.right = (int16)(((int32)right_raw * 8 + (int32)encoder_data.right * 92) / 100);
    }
    // printf("left_raw:%d, right_raw:%d\n", encoder_data.left, encoder_data.right);

    encoder_clear_count(ENCODER_LEFT);                                       // 清空左轮编码器计数
    encoder_clear_count(ENCODER_RIGHT);                                      // 清空右轮编码器计数
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取左轮编码器数据
// 参数说明     void
// 返回参数     int16           左轮编码器计数值
// 使用示例     int16 left_count = encoder_get_left();
//-------------------------------------------------------------------------------------------------------------------
int16 encoder_get_left(void)
{
    return encoder_data.left;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取右轮编码器数据
// 参数说明     void
// 返回参数     int16           右轮编码器计数值
// 使用示例     int16 right_count = encoder_get_right();
//-------------------------------------------------------------------------------------------------------------------
int16 encoder_get_right(void)
{
    return encoder_data.right;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     清空编码器计数
// 参数说明     void
// 返回参数     void
// 使用示例     encoder_clear();
//-------------------------------------------------------------------------------------------------------------------
void encoder_clear(void)
{
    encoder_clear_count(ENCODER_LEFT);                                       // 清空左轮编码器计数
    encoder_clear_count(ENCODER_RIGHT);                                      // 清空右轮编码器计数
    encoder_data.left = 0;
    encoder_data.right = 0;
}
