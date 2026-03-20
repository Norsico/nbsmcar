/*********************************************************************************************************************
* STC32G144K Opensourec Library
* Copyright (c) 2025 SEEKFREE
*
********************************************************************************************************************/
#include "zf_common_headfile.h"
#include "system_state.h"
#include "app_display.h"
#include "app_key.h"
#include "dev_imu.h"
#include "dev_wifi.h"

/************ 宏定义 ************/
#define TICKS_MS 1         // 系统tick 1ms
#define KEY_SCAN_PERIOD 20 // 按键扫描 50Hz
#define IMU_PERIOD 10      // 陀螺仪读取 100Hz
#define DISPLAY_PERIOD 100 // 显示刷新 10Hz
#define WIFI_PERIOD 50     // WiFi任务 20Hz
#define LED_DEBUG (IO_P52)			// 调试LED

#define SERVO_FREQ               (50)                                                // 控制频率为50HZ，最高支持300HZ
#define PWM_1              (PWMF_CH1_PA1)
#define PWM_2              (PWMF_CH2_PA3)
#define PWM_3              (PWMF_CH3_PA5)
#define PWM_4              (PWMF_CH4_PA7)
/************ WDT 宏定义 ************/
// 直接进行寄存器操作
#define WDT_PRESCALER  0x07       // PS=7, ~1049ms@96MHz
#define wdt_enable()   (WDT_CONTR = 0x20 | 0x10 | WDT_PRESCALER)  // EN+CLR+PS
#define wdt_feed()     (WDT_CONTR |= 0x10)   // CLR_WDT=1

/************* 全局变量 ****************/
// 系统参数
volatile system_state_t g_system_state = SYS_INIT;
vuint32 g_system_ticks = 0;
// 各任务计数器
vuint32 g_key_ticks =0;
vuint32 g_imu_ticks = 0;
vuint32 g_display_ticks = 0;
vuint32 g_wifi_ticks = 0;
// 任务执行标志位
vuint8 g_flag_key = 0;
vuint8 g_flag_imu = 0;
vuint8 g_flag_display = 0;
vuint8 g_flag_wifi = 0;

// 其他变量
uint8 imu_retry = 0;
const uint8 IMU_MAX_RETRY = 50; // 最多尝试 50 次
const uint8 IMU_CALIBRATE_SAMPLES = 50; // 零偏校准采样次数
/******** 外部函数 **********/
//	extern void key_scan_task();
//	extern void camera_task();
//	extern void motor_task();
//	extern void imu_task();

/************** 定时器回调函数 **********/
void system_tick_handler(void)
{
	g_system_ticks++;

	// 按键扫描 20ms
	if(g_system_ticks - g_key_ticks >= KEY_SCAN_PERIOD){
		g_key_ticks = g_system_ticks;
		g_flag_key = 1;
	}

	// 陀螺仪读取 10ms
	if(g_system_ticks - g_imu_ticks >= IMU_PERIOD){
		g_imu_ticks = g_system_ticks;
		g_flag_imu = 1;
	}

	// 显示 100ms
	if(g_system_ticks - g_display_ticks >= DISPLAY_PERIOD){
		g_display_ticks = g_system_ticks;
		g_flag_display = 1;
	}

	// WiFi任务 50ms
	if(g_system_ticks - g_wifi_ticks >= WIFI_PERIOD){
		g_wifi_ticks = g_system_ticks;
		g_flag_wifi = 1;
	}
}

void main(void)
{
	uint8 i;  // C251 语法：循环变量必须在函数开头声明

	// 系统初始化
	clock_init(SYSTEM_CLOCK_96M);
	debug_init();
	gpio_init(LED_DEBUG, GPO, GPIO_HIGH, GPO_PUSH_PULL);
	// ----- 安全模式 ------
	// 检查看门狗复位标志
	if(WDT_FLAG)
	{
		WDT_FLAG = 0;
		while(1)
		{
			gpio_toggle_level(LED_DEBUG);
			system_delay_ms(100);
		}
	}

	// ----- 正常启动 ------

	// 用户初始化代码
	/*
	while(1)
		{
        if(imu_init())
            printf("\r\nIMU660RA init error.");      // IMU660RA 初始化失败
        else
            break;
        gpio_toggle_level(LED_DEBUG);                     // 翻转 LED 引脚输出电平 控制 LED 亮灭 初始化出错这个灯会闪的很慢
    }
	// while初始化时间可能较长
		display_init(); // 显示屏初始化
	
		// 测试代码
    // 设置 IMU 采样周期 (10ms = 0.01s)
    imu_set_dt(IMU_PERIOD * 0.001f);

    // 执行零偏校准 (需要在静止状态下)
    // 注意：校准过程避免USB大量数据输出，防止CDC枚举异常
    imu_calibrate(IMU_CALIBRATE_SAMPLES);

    // 校准完成后输出状态
		// 由于typec口使用的是软件仿真的串口，和程序强相关，调试信息无法很好地显示
    //printf("\r\nIMU calibrate OK! offset: x=%d, y=%d, z=%d",
    //       imu_get_offset_x(), imu_get_offset_y(), imu_get_offset_z());

	*/
	// WiFi初始化
/*
	if(wifi_init())
	{
		// WiFi初始化失败 - 闪烁3次 (快闪)
		printf("\r\n[Error] WiFi init failed! LED blink 3 times.");
		for(i = 0; i < 3; i++)
		{
			gpio_toggle_level(LED_DEBUG);
			system_delay_ms(200);
		}
		g_system_state = SYS_EMERGENCY;
	}
	else
	{
		// 检查IP
		if(wifi_spi_ip_addr_port[0] == '\0' || wifi_spi_ip_addr_port[0] == '0')
		{
			// 无IP - 慢闪3次
			printf("\r\n[Error] No IP address! LED blink 3 times (slow).");
			for(i = 0; i < 3; i++)
			{
				gpio_toggle_level(LED_DEBUG);
				system_delay_ms(500);
			}
			g_system_state = SYS_EMERGENCY;
		}
		else
		{
			// 连接 TCP 服务器（电脑上的逐飞助手）
			printf("\r\n[WiFi] Connecting to TCP server: 192.168.43.144:8086...");
			if(wifi_spi_socket_connect("TCP", "192.168.43.144", "8086", "8086"))
			{
				printf("\r\n[WiFi] TCP connection FAILED!");
				// TCP失败 - 快闪5次
				printf("\r\n[Error] TCP failed! LED blink 3 times (fast).");
				for(i = 0; i < 5; i++)
				{
					gpio_toggle_level(LED_DEBUG);
					system_delay_ms(100);
				}
				g_system_state = SYS_EMERGENCY;
			}
			else
			{
				printf("\r\n[WiFi] TCP connected!");
				gpio_set_level(LED_DEBUG, GPIO_LOW);  // LED常亮表示成功
			}
		}
	}
*/
	/* 按键和激光笔初始化 */
	key_init();
	// 初始化完成
	pit_ms_init(TIM2_PIT, TICKS_MS, system_tick_handler); // 启动定时器2进行时间片调度
	// 如果是初始化状态
	if(g_system_state == SYS_INIT)
	{
		g_system_state = SYS_PREPARE;
	}

	while(1)
	{


		switch(g_system_state){
			case SYS_PREPARE:
				// 陀螺仪数据 100Hz
				if(g_flag_imu){
					g_flag_imu = 0;
					//imu_update();                 // 读取原始数据
					//imu_angle_update(IMU_PERIOD * 0.001f);  // 更新角度积分

					// 调试输出 - 原始数据
					// printf("\r\nAcc: x=%5d, y=%5d, z=%5d", imu_get_acc_x(), imu_get_acc_y(), imu_get_acc_z());
					// printf("\r\nGyro: x=%5d, y=%5d, z=%5d", imu_get_gyro_x(), imu_get_gyro_y(), imu_get_gyro_z());

					// 调试输出 - 累计角度
					// printf("\r\nYaw: %7.2f  Pitch: %7.2f  Roll: %7.2f", imu_get_yaw(), imu_get_pitch(), imu_get_roll());
				}
				// 按键扫描 50Hz
				if(g_flag_key){
					g_flag_key = 0;
					key_scan();       // 按键扫描
					laser_process(); // 激光笔控制
				}
				// 显示信息 10Hz
				if(g_flag_display){
					g_flag_display = 0;
				}
				// WiFi任务 20Hz
				if(g_flag_wifi){
					g_flag_wifi = 0;
				}
				//wifi_task();
				//wifi_send_oscilloscope(0.1,0.2,0.3,0.4);
				break;
			case SYS_RUNNING:
				break;
			case SYS_STOPED:
				break;
			case SYS_EMERGENCY:
				break;
			default:
				break;
		}
	}
}
