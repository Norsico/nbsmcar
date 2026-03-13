#include "zf_common_headfile.h"
#include "SearchLine.h"

#define LED1                    (IO_P52 )

void main(void)
{
    clock_init(SYSTEM_CLOCK_96M); 				// 时钟配置及系统初始化<务必保留>
    debug_init();                       		// 调试串口信息初始化
	gpio_init(LED1, GPO, 0, GPO_PUSH_PULL);

    ips200_init();
    ips200_show_string(0, 0, "mt9v03x init.");
    while(1)
    {
        system_delay_ms(100);
        if(mt9v03x_init())
        {
            ips200_show_string(0, 16, "mt9v03x reinit.");
        }
        else
        {
            break;
        }
		gpio_toggle_level(LED1);
    }
    ips200_show_string(0, 16, "init success.");

    while(1)
    {
        if(mt9v03x_finish_flag)
        {
            uint8 row, col;

            Get_Reference_Point();
            Search_Reference_Col();
            Search_line();
            SearchLine_Update_Center();
            ips200_show_gray_image(0, 0, mt9v03x_image[0], CAMERA_RAW_W, CAMERA_RAW_H, CAMERA_VALID_W, CAMERA_RAW_H, 0);

            // 画出参考列的垂直线（绿色）
            for(row = 0; row < CAMERA_RAW_H; row++)
            {
                ips200_draw_point(Reference_Col, row, RGB565_GREEN);
            }

            // 画出远距离数组的可视化（蓝色点）
            for(col = 10; col < CAMERA_VALID_W - 10; col += 5)  // 每5列画一个点
            {
                if(Remote_Distance[col] < CAMERA_RAW_H)
                {
                    ips200_draw_point(col, Remote_Distance[col], RGB565_BLUE);
                }
            }

            // 画出中心线（红色虚线）- 每隔2行画一个点
            for(row = 0; row < CAMERA_RAW_H; row += 2)
            {
                ips200_draw_point(Center_Line[row], row, RGB565_RED);
            }

            // 画出左边界线（黄色虚线）- 每隔2行画一个点
            for(row = 0; row < CAMERA_RAW_H; row += 2)
            {
                ips200_draw_point(Left_Edge_Line[row], row, RGB565_YELLOW);
            }

            // 画出右边界线（青色虚线）- 每隔2行画一个点
            for(row = 0; row < CAMERA_RAW_H; row += 2)
            {
                ips200_draw_point(Right_Edge_Line[row], row, RGB565_CYAN);
            }

            ips200_show_string(0, 128, "L00:");
            ips200_show_uint8(32, 128, mt9v03x_image[0][0]);
            ips200_show_string(64, 128, "R00:");
            ips200_show_uint8(96, 128, mt9v03x_image[0][CAMERA_LAST_VALID_COL]);
            ips200_show_string(0, 144, "REF:");
            ips200_show_uint8(32, 144, Reference_Point);
            ips200_show_string(64, 144, "MIN:");
            ips200_show_uint8(96, 144, White_Min_Point);
            ips200_show_string(128, 144, "MAX:");
            ips200_show_uint8(160, 144, White_Max_Point);
            ips200_show_string(0, 160, "RCOL:");
            ips200_show_uint8(40, 160, Reference_Col);
            ips200_show_string(80, 160, "RD:");
            ips200_show_uint8(104, 160, Remote_Distance[Reference_Col]);
            ips200_show_string(128, 160, "CTR:");
            ips200_show_uint8(160, 160, Center_Line[CAMERA_RAW_H - 1]);
            mt9v03x_finish_flag = 0;
        }
    }
}
