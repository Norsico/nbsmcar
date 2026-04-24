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
* 开发环境          ADS v1.9.4
* 适用平台          TC264D
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2022-09-15       pudding            first version
********************************************************************************************************************/
#include "zf_common_headfile.h"

#include "small_driver_uart_control.h"

#pragma section all "cpu0_dsram"
// 将本语句与#pragma section all restore语句之间的全局变量都放在CPU0的RAM中

// *************************** 例程硬件连接说明 ***************************
// 使用逐飞科技 tc264 V2.6主板 按照下述方式进行接线
//      模块引脚    单片机引脚
//      RX          查看 small_driver_uart_control.h 中 SMALL_DRIVER_TX  宏定义 默认 P15_7
//      TX          查看 small_driver_uart_control.h 中 SMALL_DRIVER_RX  宏定义 默认 P15_6
//      GND         GND


// *************************** 例程测试说明 ***************************
// 1.核心板烧录完成本例程 主板电池供电 连接 CYT2BL3 FOC 双驱
// 2.如果初次使用 请先点击双驱上的MODE按键 以矫正零点位置 矫正时 电机会发出音乐
// 3.可以在逐飞助手上位机上看到如下串口信息：
//      left speed:xxxx, right speed:xxxx
// 如果发现现象与说明严重不符 请参照本文件最下方 例程常见问题说明 进行排查

// **************************** 代码区域 ****************************


#define MAX_DUTY            (30)                                                // 最大 测试 占空比



//以下是WiFi模块参数
#define INCLUDE_BOUNDARY_TYPE   4
#define WIFI_STATE              0             //1为开启WiFi模块，0为关闭WiFi模块

#define WIFI_SSID_TEST          "dafeifei"
#define WIFI_PASSWORD_TEST      "123456789"                  // 如果需要连接的WIFI 没有密码则需要将 这里 替换为 NULL
#define TCP_TARGET_IP           "192.168.137.1"             // 连接目标的 IP
#define TCP_TARGET_PORT         "8086"                      // 连接目标的端口
#define WIFI_LOCAL_PORT         "6666"                      // 本机的端口 0：随机  可设置范围2048-65535  默认 6666
// 边界的点数量远大于图像高度，便于保存回弯的情况
#define BOUNDARY_NUM            (MT9V03X_H * 3 / 2)

// 只有X边界
uint8 xy_x1_boundary[BOUNDARY_NUM], xy_x2_boundary[BOUNDARY_NUM], xy_x3_boundary[BOUNDARY_NUM];

// 只有Y边界
uint8 xy_y1_boundary[BOUNDARY_NUM], xy_y2_boundary[BOUNDARY_NUM], xy_y3_boundary[BOUNDARY_NUM];

// X Y边界都是单独指定的
uint8 x1_boundary[MT9V03X_H], x2_boundary[MT9V03X_H], x3_boundary[MT9V03X_H];
uint8 y1_boundary[MT9V03X_W], y2_boundary[MT9V03X_W], y3_boundary[MT9V03X_W];

// 图像备份数组，在发送前将图像备份再进行发送，这样可以避免图像出现撕裂的问题
uint8 image_copy[MT9V03X_H][MT9V03X_W];
//以上是WiFi模块参数



int8 duty = 0;
bool dir = true;
extern int16 cnt1;
extern LadderMovePoint L_Move; //左右两点爬梯算法左点
extern LadderMovePoint R_Move; //左右两点爬梯算法右点
extern Dispose_Image DI;       //图像处理结构体声明
int hangkuan[10];
extern uint8 L_Y3;
extern uint8 L_X3;
extern uint8 L_Y1;
extern uint8 L_X1;
int core0_main(void)
{
#if(1 == INCLUDE_BOUNDARY_TYPE || 2 == INCLUDE_BOUNDARY_TYPE || 4 == INCLUDE_BOUNDARY_TYPE)
    int32 i = 0;
#elif(3 == INCLUDE_BOUNDARY_TYPE)
    int32 j = 0;
#endif
    clock_init();                   // 获取时钟频率<务必保留>
    debug_init();                   // 初始化默认调试串口
    // 此处编写用户代码 例如外设初始化代码等

//以下是WiFi模块代码，需要时再开启
    if(WIFI_STATE)
    {
        while(wifi_spi_init(WIFI_SSID_TEST, WIFI_PASSWORD_TEST))
        {
            printf("\r\n connect wifi failed. \r\n");
            system_delay_ms(100);                                                   // 初始化失败 等待 100ms
        }

        printf("\r\n module version:%s",wifi_spi_version);                          // 模块固件版本
        printf("\r\n module mac    :%s",wifi_spi_mac_addr);                         // 模块 MAC 信息
        printf("\r\n module ip     :%s",wifi_spi_ip_addr_port);                     // 模块 IP 地址

        // zf_device_wifi_spi.h 文件内的宏定义可以更改模块连接(建立) WIFI 之后，是否自动连接 TCP 服务器、创建 UDP 连接
        if(1 != WIFI_SPI_AUTO_CONNECT)                                              // 如果没有开启自动连接 就需要手动连接目标 IP
        {
            while(wifi_spi_socket_connect(                                          // 向指定目标 IP 的端口建立 TCP 连接
                "TCP",                                                              // 指定使用TCP方式通讯
                TCP_TARGET_IP,                                                      // 指定远端的IP地址，填写上位机的IP地址
                TCP_TARGET_PORT,                                                    // 指定远端的端口号，填写上位机的端口号，通常上位机默认是8080
                WIFI_LOCAL_PORT))                                                   // 指定本机的端口号
            {
                // 如果一直建立失败 考虑一下是不是没有接硬件复位
                printf("\r\n Connect TCP Servers error, try again.");
                system_delay_ms(100);                                               // 建立连接失败 等待 100ms
            }
        }

        // 推荐先初始化摄像头，后初始化逐飞助手
    }
//以上是WiFi模块代码

    imu660ra_init();                     //初始化
//    IMU_gyro_offset_init();          //去零漂
    pit_init(CCU60_CH0, 2000);    //开启一个2ms定时中断
    small_driver_uart_init();		// 初始化驱动通讯功能
    gpio_init (P33_10,GPO, 0,GPO_PUSH_PULL);
    servo_init();
      EKF_Init( );
      flash_init();
      ips200_init(IPS200_TYPE_SPI);
//总钻风初始化
      ips200_show_string(0, 0, "mt9v03x init.");
      menu_dis_init();
        speed_base_init();

      while(1)
      {
          if(mt9v03x_init())
              ips200_show_string(0, 80, "mt9v03x reinit.");
          else
              break;
          system_delay_ms(500);                                                   // 短延时快速闪灯表示异常
      }

      seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);
//以下是WiFi模块代码
      // 如果要发送图像信息，则务必调用seekfree_assistant_camera_information_config函数进行必要的参数设置
      // 如果需要发送边线则还需调用seekfree_assistant_camera_boundary_config函数设置边线的信息
    if(WIFI_STATE)
    {
      #if(0 == INCLUDE_BOUNDARY_TYPE)
          // 发送总钻风图像信息(仅包含原始图像信息)
          seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, Pixle_hb[0], LCDW, LCDH);


      #elif(1 == INCLUDE_BOUNDARY_TYPE)
          // 发送总钻风图像信息(并且包含三条边界信息，边界信息只含有横轴坐标，纵轴坐标由图像高度得到，意味着每个边界在一行中只会有一个点)
          // 对边界数组写入数据
          for(i = 0; i < MT9V03X_H; i++)
          {
              x1_boundary[i] = 70 - (70 - 20) * i / MT9V03X_H;
              x2_boundary[i] = MT9V03X_W / 2;
              x3_boundary[i] = 118 + (168 - 118) * i / MT9V03X_H;
          }
          seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, image_copy[0], MT9V03X_W, MT9V03X_H);
          seekfree_assistant_camera_boundary_config(X_BOUNDARY, MT9V03X_H, x1_boundary, x2_boundary, x3_boundary, NULL, NULL ,NULL);


      #elif(2 == INCLUDE_BOUNDARY_TYPE)
          // 发送总钻风图像信息(并且包含三条边界信息，边界信息只含有纵轴坐标，横轴坐标由图像宽度得到，意味着每个边界在一列中只会有一个点)
          // 通常很少有这样的使用需求
          // 对边界数组写入数据
          for(i = 0; i < MT9V03X_W; i++)
          {
              y1_boundary[i] = i * MT9V03X_H / MT9V03X_W;
              y2_boundary[i] = MT9V03X_H / 2;
              y3_boundary[i] = (MT9V03X_W - i) * MT9V03X_H / MT9V03X_W;
          }
          seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, image_copy[0], MT9V03X_W, MT9V03X_H);
          seekfree_assistant_camera_boundary_config(Y_BOUNDARY, MT9V03X_W, NULL, NULL ,NULL, y1_boundary, y2_boundary, y3_boundary);


      #elif(3 == INCLUDE_BOUNDARY_TYPE)
          // 发送总钻风图像信息(并且包含三条边界信息，边界信息含有横纵轴坐标)
          // 这样的方式可以实现对于有回弯的边界显示
          j = 0;
          for(i = MT9V03X_H - 1; i >= MT9V03X_H / 2; i--)
          {
              // 直线部分
              xy_x1_boundary[j] = 34;
              xy_y1_boundary[j] = i;

              xy_x2_boundary[j] = 47;
              xy_y2_boundary[j] = i;

              xy_x3_boundary[j] = 60;
              xy_y3_boundary[j] = i;
              j++;
          }

          for(i = MT9V03X_H / 2 - 1; i >= 0; i--)
          {
              // 直线连接弯道部分
              xy_x1_boundary[j] = 34 + (MT9V03X_H / 2 - i) * (MT9V03X_W / 2 - 34) / (MT9V03X_H / 2);
              xy_y1_boundary[j] = i;

              xy_x2_boundary[j] = 47 + (MT9V03X_H / 2 - i) * (MT9V03X_W / 2 - 47) / (MT9V03X_H / 2);
              xy_y2_boundary[j] = 15 + i * 3 / 4;

              xy_x3_boundary[j] = 60 + (MT9V03X_H / 2 - i) * (MT9V03X_W / 2 - 60) / (MT9V03X_H / 2);
              xy_y3_boundary[j] = 30 + i / 2;
              j++;
          }

          for(i = 0; i < MT9V03X_H / 2; i++)
          {
              // 回弯部分
              xy_x1_boundary[j] = MT9V03X_W / 2 + i * (138 - MT9V03X_W / 2) / (MT9V03X_H / 2);
              xy_y1_boundary[j] = i;

              xy_x2_boundary[j] = MT9V03X_W / 2 + i * (133 - MT9V03X_W / 2) / (MT9V03X_H / 2);
              xy_y2_boundary[j] = 15 + i * 3 / 4;

              xy_x3_boundary[j] = MT9V03X_W / 2 + i * (128 - MT9V03X_W / 2) / (MT9V03X_H / 2);
              xy_y3_boundary[j] = 30 + i / 2;
              j++;
          }
          seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, image_copy[0], MT9V03X_W, MT9V03X_H);
          seekfree_assistant_camera_boundary_config(XY_BOUNDARY, BOUNDARY_NUM, xy_x1_boundary, xy_x2_boundary, xy_x3_boundary, xy_y1_boundary, xy_y2_boundary, xy_y3_boundary);


      #elif(4 == INCLUDE_BOUNDARY_TYPE)
          // 发送总钻风图像信息(并且包含三条边界信息，边界信息只含有横轴坐标，纵轴坐标由图像高度得到，意味着每个边界在一行中只会有一个点)
          // 对边界数组写入数据
          for(i = 0; i <  DI.ui8_DisposeScopeDown; i++)
          {
              x1_boundary[i] =DI.ui8_LPoint[i];
              x2_boundary[i] = (DI.ui8_LPoint[i] + DI.ui8_RPoint[i]) / 2;
              x3_boundary[i] = DI.ui8_RPoint[i];
          }
          seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, Pixle_hb[0], LCDW, LCDH);
          seekfree_assistant_camera_boundary_config(X_BOUNDARY, LCDH, x1_boundary, x2_boundary, x3_boundary, NULL, NULL ,NULL);


      #endif
    }
//以上是WiFi模块代码
      ips200_show_string(0, 16, "init success.");
      InitDisposeImageData();       //图像参数初始化
//      at24c02_write_bytes(0,data_buff,255);
      var_load();
    // 此处编写用户代码 例如外设初始化代码等
    cpu_wait_event_ready();         // 等待所有核心初始化完毕

    while (TRUE)
    {
        // 此处编写需要循环执行的代码
//             ips200_show_float(0, 0,euler_angle.roll,5,3);

//             if(mt9v03x_finish_flag)
 //            {

/*
        ips200_show_string(0,164, "poserror");
        ips200_show_int (200,164,poserror,3);
        ips200_show_string(0,180, "Distance");
        ips200_show_int (200,180,Distance,3);
        ips200_show_string(0,196, "S_Left");
        ips200_show_int (200,196,S_Left,3);
        ips200_show_string(0,212, "34");
        ips200_show_int (200,212,street_len[34],3);
        ips200_show_string(0,228, "-5--10");
        ips200_show_int (200,228,DI.ui8_LPoint[60-street_len[34]-5]-DI.ui8_LPoint[60-street_len[34]-10],3);
        ips200_show_string(0,244, "47-58");
        ips200_show_int (200,244,DI.ui8_LPoint[47]-DI.ui8_LPoint[58],3);
        ips200_show_string(0,260, "48-52");
        ips200_show_int (200,260,DI.ui8_LPoint[48]-DI.ui8_LPoint[52],3);
        ips200_show_string(0,276, "56-35");
        ips200_show_int (200,276,DI.ui8_RPoint[56]-DI.ui8_RPoint[35],3);
        ips200_show_string(0,288, "-5-5");
        ips200_show_int (200,288,DI.ui8_LPoint[60-street_len[34]+5]-DI.ui8_LPoint[60-street_len[34]-5],3);
*/

/*
        ips200_show_string(0,164, "2");
        ips200_show_int (200,164,DI.ui8_ScanLineToR[2]-DI.ui8_ScanLineToL[2],3);
        ips200_show_string(0,180, "4");
        ips200_show_int (200,180,DI.ui8_ScanLineToR[4]-DI.ui8_ScanLineToL[4],3);
        ips200_show_string(0,196, "5");
        ips200_show_int (200,196,DI.ui8_ScanLineToR[5]-DI.ui8_ScanLineToL[5],3);
        ips200_show_string(0,212, "3");
        ips200_show_int (200,212,DI.ui8_ScanLineToR[3]-DI.ui8_ScanLineToL[3],3);
        ips200_show_string(0,228, "6");
        ips200_show_int (200,228,DI.ui8_ScanLineToR[6]-DI.ui8_ScanLineToL[6],3);
        ips200_show_string(0,244, "8");
        ips200_show_int (200,244,DI.ui8_ScanLineToR[8]-DI.ui8_ScanLineToL[8],3);
        ips200_show_string(0,260, "7");
        ips200_show_int (200,260,DI.ui8_ScanLineToR[7]-DI.ui8_ScanLineToL[7],3);
*/

/*
        ips200_show_string(0,228, "-12--5");
        ips200_show_int (200,228,DI.ui8_RPoint[60-street_len[48]-12]-DI.ui8_RPoint[60-street_len[48]-5],3);
        ips200_show_string(0,244, "55-50");
        ips200_show_int (200,244,DI.ui8_RPoint[55]-DI.ui8_RPoint[50],3);
        ips200_show_string(0,260, "50-45");
        ips200_show_int (200,260,DI.ui8_RPoint[50]-DI.ui8_RPoint[45],3);
        ips200_show_string(0,276, "55");
        ips200_show_int (200,276,DI.ui8_LPoint[30]-DI.ui8_LPoint[55],3);
        ips200_show_string(0,288, "-5-5");
        ips200_show_int (200,288,DI.ui8_RPoint[60-street_len[48]-5]-DI.ui8_RPoint[60-street_len[48]+5],3);
*/

/*
        ips200_show_string(0,276, "S_Right");
        ips200_show_string(0,288, "S_Left");
        ips200_show_int (100, 276,S_Right, 5);
        ips200_show_int (100, 288,S_Left, 5);
        ips200_show_string(0,260, "k_r");
        ips200_show_string(0,244, "k_l");

        ips200_show_float (100, 260, k_r,3, 3);
        ips200_show_float (100, 244, k_l,3, 3);DI.ui8_ScanLineToR[3]-DI.ui8_ScanLineToL[3]
*/
/*
        ips200_show_string (0, 260,"R[3]-L[3]");
        ips200_show_int (100, 260,DI.ui8_ScanLineToR[3]-DI.ui8_ScanLineToL[3], 5);
*/
/*
                  ips200_show_string(0,100, "S_Right");
                  ips200_show_string(0,116, "S_Left");
                  ips200_show_string(0,132, "k_r");
                  ips200_show_string(0,148, "k_l");
                  ips200_show_int (100, 100,S_Right, 5);
                  ips200_show_int (100, 116,S_Left, 5);
                  ips200_show_float (100, 132, k_r,3, 3);
                  ips200_show_float (100, 148, k_l,3, 3);
                  ips200_show_string(0,164, "Vistable_scale");
                  ips200_show_int (200,164,Vistable_scale,3);
                  ips200_show_string(0,180, "SumAngle");
                  ips200_show_int (200,180,SumAngle,3);
                  ips200_show_string(0,196, "S_Left");
                  ips200_show_int (200,196,hangkuan60[19],3);
                  ips200_show_string(0,212, "ui8_ZebraTimes");
                  ips200_show_int (200,212,ui8_ZebraTimes,3);
                  ips200_show_string(0,244, "speed");
                  ips200_show_float (100,244,(motor_value.receive_left_speed_data-motor_value.receive_right_speed_data)/2,5,5);

                  ips200_show_int (100,228, fuzhu_out,3);
 //                 ips200_show_int (100, 244, street_len[35],3);
                  ips200_show_int (100, 260, SPEED,3);
                  ips200_show_int (100,276,DI.ui8_RPoint[DI.ui8_ScanLineY[9]],3);
*/

                      for (uint8 Y = 0; Y <= DI.ui8_DisposeScopeDown; Y++)
                          {
                              Pixle_hb[Y][DI.ui8_LPoint[Y]] = 0;                          //左边界显示黑
                              Pixle_hb[Y][DI.ui8_RPoint[Y]] = 0;                          //右边界显示黑
                              Pixle_hb[Y][(DI.ui8_LPoint[Y] + DI.ui8_RPoint[Y]) / 2] = 0; //中线显示黑
        //                      hangkuan60[Y]=DI.ui8_RPoint[Y]-DI.ui8_LPoint[Y];
                          }
                          for (uint8 Y = 0; Y <= 9; Y++)
                          {
                            for (uint8 X = DI.ui8_LPoint[DI.ui8_ScanLineY[Y]]; X <= DI.ui8_RPoint[DI.ui8_ScanLineY[Y]]; X++)
                            {
                                Pixle_hb[DI.ui8_ScanLineY[Y]][X] = 0;
                                hangkuan[Y]=DI.ui8_RPoint[DI.ui8_ScanLineY[Y]]-DI.ui8_LPoint[DI.ui8_ScanLineY[Y]];

                            }
                            Pixle_hb[Y][DI.ui8_ScanLineToR[Y]] = 0;
                            Pixle_hb[Y][DI.ui8_ScanLineToL[Y]] = 0;
                          }
        //                  errorout=(DI.ui8_LPoint[56] + DI.ui8_RPoint[56]) / 2-40;
                          ips200_show_gray_image (0, 0, Pixle_hb[0], LCDW, LCDH, LCDW, LCDH, 0);
                          menu_display();

                  if(WIFI_STATE)
                  {
                      for(i = 0; i <  DI.ui8_DisposeScopeDown; i++)
                      {
                          x1_boundary[i] =DI.ui8_LPoint[i];
                          x2_boundary[i] = (DI.ui8_LPoint[i] + DI.ui8_RPoint[i]) / 2;
                          x3_boundary[i] = DI.ui8_RPoint[i];
                      }
                  }
 //                 BeeOn;
//                 ips200_displayimage03x((const uint8 *)Pixle_hb, LCDW, LCDH);                       // 显示原始图像

 //                ips200_show_gray_image(0, 0, (const uint8 *)mt9v03x_image, MT9V03X_W, MT9V03X_H, 240, 180, 64);     // 显示二值化图像
 //                mt9v03x_finish_flag = 0;
//             }
 //       ips200_displayimage03x((const uint8 *)mt9v03x_image, MT9V03X_W, MT9V03X_H);
/*
        small_driver_set_duty(duty * (PWM_DUTY_MAX / 100), -duty * (PWM_DUTY_MAX / 100));   // 计算占空比输出

*/
//        printf("left speed:%d, right speed:%d\r\n", motor_value.receive_left_speed_data, motor_value.receive_right_speed_data);
//        printf("speed:%d,%d\r\n", output_l,motor_value.receive_left_speed_data);
//        printf("hangkuan:%d\r\n",hangkuan60[4]);//59:40     54:36   49:34     44:32      39:27      34:24      29:22
                                                 //24:19     19:17   14:13     09:11     04:11
        if(WIFI_STATE)
        {
            seekfree_assistant_camera_send();
        }

//        system_delay_ms(50);
//        printf(" g_eulerAngle.roll:%.2f,%d\r\n",inte_l,output_l);
//        pwm_set_duty(SERVO_MOTOR_PWM, (uint32)SERVO_MOTOR_DUTY(70));
//        ALL_SERVO_ANGLE(20,20,20,20);
        // 此处编写需要循环执行的代码
    }
}

#pragma section all restore
// **************************** 代码区域 ****************************
// *************************** 例程常见问题说明 ***************************
// 遇到问题时请按照以下问题检查列表检查
// 问题1：串口没有数据
//      查看逐飞助手上位机打开的是否是正确的串口，检查打开的 COM 口是否对应的是调试下载器或者 USB-TTL 模块的 COM 口
//      如果是使用逐飞科技 英飞凌TriCore 调试下载器连接，那么检查下载器线是否松动，检查核心板串口跳线是否已经焊接，串口跳线查看核心板原理图即可找到
//      如果是使用 USB-TTL 模块连接，那么检查连线是否正常是否松动，模块 TX 是否连接的核心板的 RX，模块 RX 是否连接的核心板的 TX
// 问题2：串口数据乱码
//      查看逐飞助手上位机设置的波特率是否与程序设置一致，程序中 zf_common_debug.h 文件中 DEBUG_UART_BAUDRATE 宏定义为 debug uart 使用的串口波特率
// 问题3：无刷电机无反应
//      检查Rx信号引脚是否接对，信号线是否松动
// 问题4：无刷电机转动但转速显示无速度
//      检查Tx信号引脚是否接对，信号线是否松动
