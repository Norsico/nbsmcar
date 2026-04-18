/*********************************************************************************************************************
* TC264 Opensourec Library 即（TC264 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 TC264 开源库的一部分
*
* TC264 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          cpu0_main
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          ADS v1.8.0
* 适用平台          TC264D
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2022-09-15       pudding            first version
********************************************************************************************************************/
#include "zf_common_headfile.h"
#include "C_H.h"
#include "tft180.h"
#include "steer.h"
#include "Fuzzy.h"
#include "motor.h"
#include "key.h"
#pragma section all "cpu0_dsram"
// 将本语句与#pragma section all restore语句之间的全局变量都放在CPU0的RAM中

// **************************** 代码区域 ****************************
int core0_main(void)
{
    clock_init();                   // 获取时钟频率<务必保留>
    debug_init();                   // 初始化默认调试串口
    // 此处编写用户代码 例如外设初始化代码等
//    uint8 buff;

    Data_Settings();   // 参数给定
    //gpioinit();      // 按键初始化
    tft180_init();     // 初始化屏幕
    Steer_init();      // 舵机初始化
    mt9v03x_init ();   // 摄像头初始化
    Speed_decision();  /// pidinitall();      //pid参数初始化
    MOTOR_init();      // 电机初始化
    Encoder_init();    // 编码器初始化
    Start_gpio_init(); // 软启动开跑初始化
    mt9v03x_set_exposure_time(300);  // 曝光时间300
    pit_init(CCU61_CH0,5000);        // 5ms中断 电机控制
    pit_init(CCU60_CH1,10000);     // 20ms中断 舵机控制
    // 此处编写用户代码 例如外设初始化代码等
    cpu_wait_event_ready();          // 等待所有核心初始化完毕
    while (TRUE)
    {
        // 此处编写需要循环执行的代码
           if(mt9v03x_finish_flag)
            {
              ImageProcess();
            }
           key_Start_run();
            tft180();
        // 此处编写需要循环执行的代码
    }
}

//#define PWM_CH1             ATOM0_CH4_P02_4 //左轮正       PWM2
//#define PWM_CH2             ATOM0_CH5_P02_5 //左轮反
//#define PWM_CH3             ATOM0_CH6_P02_6 //右轮正
//#define PWM_CH4             ATOM0_CH7_P02_7 //右轮反       PWM1
#pragma section all restore
// **************************** 代码区域 ****************************

// *************************** 例程常见问题说明 ***************************
// 遇到问题时请按照以下问题检查列表检查
// 问题1：PWM_CHx 对应的通道长时间无信号、电压变化
//      查看程序是否正常烧录，是否下载报错，确认正常按下复位按键
//      查看端口是否被外部拉低或者拉高，使用万用表的蜂鸣器档测量是否与GND或者VCC短接
