#include "zf_common_headfile.h"
#include "car_init.h"


// **************************** 代码区域 ****************************

void main(void)
{
    clock_init(SYSTEM_CLOCK_96M); 				// 时钟配置及系统初始化<务必保留>
    debug_init();                       		// 调试串口信息初始化

    
    seekfree_assistant_wifi_init("QQ", "1234567890xia", "192.168.43.236");

    seekfree_assistant_oscilloscope_data.channel_num = 1;
    seekfree_assistant_oscilloscope_data.dat[0] = 0.0f;

    printf("\r\n assistant param[1] -> oscilloscope channel[0] \r\n");

    // 此处编写用户代码 例如外设初始化代码等
    while(1)
    {
        seekfree_assistant_data_analysis();

        if(seekfree_assistant_parameter_update_flag[0])
        {
            seekfree_assistant_oscilloscope_data.dat[0] = seekfree_assistant_parameter[0];
            seekfree_assistant_parameter_update_flag[0] = 0;
        }

        seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);

        system_delay_ms(20);

        // 此处编写需要循环执行的代码
    }
}
// **************************** 代码区域 ****************************


