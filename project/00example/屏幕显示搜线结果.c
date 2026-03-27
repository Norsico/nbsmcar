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
            uint8 row;
            uint8 bottom_row = SEARCH_VALID_BOTTOM_ROW;
            uint8 reference_distance = 0;
            uint8 left_bottom_col = 0;
            uint8 right_bottom_col = 0;
            uint8 center_bottom_col = 0;

            Get_Reference_Point();
            Search_Reference_Col();
            Search_line();
            SearchLine_Update_Center();
            // 联合查看最长白列、左右边界和中线
            ips200_show_gray_image(0, 0, mt9v03x_image[0], CAMERA_RAW_W, CAMERA_RAW_H, CAMERA_VALID_W, CAMERA_RAW_H, 0);

            reference_distance = Remote_Distance[Reference_Col];
            left_bottom_col = Left_Edge_Line[bottom_row];
            right_bottom_col = Right_Edge_Line[bottom_row];
            center_bottom_col = Center_Line[bottom_row];

            // 最长白列对应的参考列（绿色）
            for(row = 0; row < CAMERA_RAW_H; row++)
            {
                ips200_draw_point(Reference_Col, row, RGB565_GREEN);
            }

            // 左边界（黄色）
            for(row = SEARCH_VALID_TOP_ROW; row <= SEARCH_VALID_BOTTOM_ROW; row++)
            {
                if(Left_Edge_Line[row] < CAMERA_VALID_W)
                {
                    ips200_draw_point(Left_Edge_Line[row], row, RGB565_YELLOW);
                }
            }

            // 右边界（青色）
            for(row = SEARCH_VALID_TOP_ROW; row <= SEARCH_VALID_BOTTOM_ROW; row++)
            {
                if(Right_Edge_Line[row] < CAMERA_VALID_W)
                {
                    ips200_draw_point(Right_Edge_Line[row], row, RGB565_CYAN);
                }
            }

            // 中线 = (左边界 + 右边界) / 2（红色）
            for(row = SEARCH_VALID_TOP_ROW; row <= SEARCH_VALID_BOTTOM_ROW; row++)
            {
                if(Center_Line[row] < CAMERA_VALID_W)
                {
                    ips200_draw_point(Center_Line[row], row, RGB565_RED);
                }
            }

            // 参考列对应的远距离落点仍然保留，辅助判断搜索起点是否可靠（蓝色）
            ips200_draw_point(Reference_Col, reference_distance, RGB565_BLUE);
            if(reference_distance > 0)
            {
                ips200_draw_point(Reference_Col, reference_distance - 1, RGB565_BLUE);
            }
            if(reference_distance + 1 < CAMERA_RAW_H)
            {
                ips200_draw_point(Reference_Col, reference_distance + 1, RGB565_BLUE);
            }

            // 底部中线位置打红色十字，方便先看最近处是否居中
            ips200_draw_point(center_bottom_col, bottom_row, RGB565_RED);
            if(bottom_row > 0)
            {
                ips200_draw_point(center_bottom_col, bottom_row - 1, RGB565_RED);
            }
            if(bottom_row + 1 < CAMERA_RAW_H)
            {
                ips200_draw_point(center_bottom_col, bottom_row + 1, RGB565_RED);
            }
            if(center_bottom_col > 0)
            {
                ips200_draw_point(center_bottom_col - 1, bottom_row, RGB565_RED);
            }
            if(center_bottom_col + 1 < CAMERA_VALID_W)
            {
                ips200_draw_point(center_bottom_col + 1, bottom_row, RGB565_RED);
            }

            ips200_show_string(0, 144, "REF:");
            ips200_show_uint8(32, 144, Reference_Point);
            ips200_show_string(64, 144, "MIN:");
            ips200_show_uint8(96, 144, White_Min_Point);
            ips200_show_string(128, 144, "MAX:");
            ips200_show_uint8(160, 144, White_Max_Point);
            ips200_show_string(0, 160, "RC:");
            ips200_show_uint8(24, 160, Reference_Col);
            ips200_show_string(56, 160, "RY:");
            ips200_show_uint8(80, 160, reference_distance);
            ips200_show_string(112, 160, "CT:");
            ips200_show_uint8(136, 160, center_bottom_col);
            ips200_show_string(0, 176, "LB:");
            ips200_show_uint8(24, 176, left_bottom_col);
            ips200_show_string(56, 176, "RB:");
            ips200_show_uint8(80, 176, right_bottom_col);
            ips200_show_string(112, 176, "ROW:");
            ips200_show_uint8(144, 176, bottom_row);
            mt9v03x_finish_flag = 0;
        }
    }
}
