#include <display.h>
#include <image.h>
#include "zf_common_headfile.h"
#include "steer.h"
#include "Fuzzy.h"
#include "motor.h"
#pragma section all "cpu0_dsram"


extern uint32 exposure_time;     //曝光时间


//       本代码仅供学习参考，禁止售卖
//       最爱bin口味   QQ：1971402825
int core0_main(void)
{
    clock_init();                   // 获取时钟频率<务必保留>
    debug_init();                   // 初始化默认调试串口

        //存数据
//        flash_set_buffer_speed();     //存速度数据         开启之前记得给数据赋值
//        flash_set_buffer_image();     //存图像数据
//        flash_set_buffer_rings();      //存元素数据

       //取数据
       flash_get_buffer_speed();
       flash_get_buffer_image();
       flash_get_buffer_rings();

       Data_Settings();      //参数设置
       gpio_read_set();      //读取拨码开关状态
//       gpio_init(P33_10,GPO, 1, GPO_PUSH_PULL);
//       gpio_init(P33_10, GPO, 0, GPO_OPEN_DTAIN);
//       gpio_set_level(P33_10,0);
//       gpio_toggle_level(P33_10);
       Speed_decision();       //电机PID设置
       ips114_set_dir(2);     //定义屏幕方向
       ips114_init();        //屏幕初始化
       Steer_init();         //舵机初始化
       MOTOR_init();         //电机初始化
       Encoder_init();       //编码器初始化
       mt9v03x_init ();      //摄像头初始化
//       icm20602_init();      //陀螺仪初始化
//       dl1a_init();          //tof初始化     开了屏幕会卡，我也不知道为什么

       mt9v03x_set_exposure_time_set(exposure_time); // 曝光时间设置完毕之后需要重启

//       pwm_init(ATOM0_CH5_P02_5,50,700);//负压风扇1初始化
//       pwm_init(ATOM0_CH4_P02_4,50,700);//负压风扇2初始化


       gpio_init(P22_0,GPI,0,GPI_PULL_DOWN);  //按键1初始化
       gpio_init(P22_1,GPI,0,GPI_PULL_DOWN);  //按键2初始化
       gpio_init(P22_2,GPI,0,GPI_PULL_DOWN);  //按键3初始化
       gpio_init(P22_3,GPI,0,GPI_PULL_DOWN);  //按键4初始化



       low_pass_filter_init();   //滤波代码初始化

      pit_init(CCU61_CH0,  5000);   //5ms中断    电机控制
      pit_init(CCU60_CH1, 10000);   //10ms中断   舵机控制
      pit_init(CCU61_CH1, 20000);   //20ms中断   按键状态检测及陀螺仪数据


    // 此处编写用户代码 例如外设初始化代码等
    cpu_wait_event_ready();         // 等待所有核心初始化完毕
     //  interrupt_global_enable(0);     // 打开全局中断

    while (TRUE)
    {
//        pwm_set_duty(ATOM1_CH5_P02_5,600);   //负压风扇1输出 ATOM0_CH5_P02_5
//        pwm_set_duty(ATOM1_CH6_P02_6,600);   //负压风扇2输出 ATOM1_CH6_P02_6


    }
}

#pragma section all restore
// **************************** 代码区域 ****************************

