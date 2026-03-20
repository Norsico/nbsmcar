/*********************************************************************************************************************
* STC32G144 Opensource Library - WiFi SPI Device Driver
* Copyright (c) 2025 SEEKFREE
*
********************************************************************************************************************/
#include "dev_wifi.h"
#include "zf_driver_gpio.h"

/************ 全局变量 ************/
static uint8 wifi_initialized = 0;

/************ 初始化 ************/
uint8 wifi_init(void)
{
    uint8 retry = 0;
    uint8 ret;

    printf("\r\n[WiFi] ========================================");
    printf("\r\n[WiFi] WiFi Driver Starting...");
    printf("\r\n[WiFi] SSID: %s", WIFI_SSID);
    printf("\r\n[WiFi] AUTO_CONNECT: %d", WIFI_SPI_AUTO_CONNECT);
    printf("\r\n[WiFi] TARGET_IP: %s", WIFI_SPI_TARGET_IP);
    printf("\r\n[WiFi] TARGET_PORT: %s", WIFI_SPI_TARGET_PORT);
    printf("\r\n[WiFi] ----------------------------------------");

    // 循环尝试初始化
    while(retry < 5)
    {
        retry++;
        printf("\r\n[WiFi] Init attempt %d/5...", retry);

        // 测试 INT 引脚状态
        printf("\r\n[WiFi] INT pin (P1.5) level: %d", gpio_get_level(WIFI_SPI_INT_PIN));
        printf("\r\n[WiFi] Calling wifi_spi_init()...");

        // 调用官方初始化函数
        ret = wifi_spi_init(WIFI_SSID, WIFI_PASSWORD);

        if(ret == 0)
        {
            // 成功
            printf("\r\n[WiFi] ========================================");
            printf("\r\n[WiFi] WiFi Init SUCCESS!");
            printf("\r\n[WiFi] Version: %s", wifi_spi_version);
            printf("\r\n[WiFi] MAC: %s", wifi_spi_mac_addr);
            printf("\r\n[WiFi] IP:Port = %s", wifi_spi_ip_addr_port);
            printf("\r\n[WiFi] ========================================\r\n");

            // 初始化Seekfree Assistant
            seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);
            wifi_initialized = 1;
            return 0;
        }
        else
        {
            // 失败，显示错误码
            printf("\r\n[WiFi] Init FAILED! ret=%d (0x%02X)", ret, ret);
            printf("\r\n[WiFi] Error breakdown:");
            if(ret == 1) printf("\r\n  ret=1: SPI通信失败 (Get version failed)");
            if(ret == 2) printf("\r\n  ret=2: MAC获取失败");
            if(ret == 4) printf("\r\n  ret=4: WiFi连接失败");
            if(ret == 8) printf("\r\n  ret=8: Socket连接失败");
        }

        if(retry < 5)
        {
            printf("\r\n[WiFi] Waiting 2s before retry...\r\n");
            system_delay_ms(2000);
        }
    }

    printf("\r\n[WiFi] ERROR: All init attempts failed!");
    printf("\r\n[WiFi] Please check:");
    printf("\r\n  1. Hardware connection");
    printf("\r\n  2. SPI pin definitions in zf_device_wifi_spi.h");
    printf("\r\n  3. Try re-powering the WiFi module\r\n");
    return 1;
}

/************ WiFi任务 ************/
void wifi_task(void)
{
    if(!wifi_initialized)
        return;

    // 处理上位机指令
    seekfree_assistant_data_analysis();
}

/************ 发送示波器数据 ************/
void wifi_send_oscilloscope(float ch1, float ch2, float ch3, float ch4)
{
    if(!wifi_initialized)
        return;

    seekfree_assistant_oscilloscope_data.channel_num = 4;
    seekfree_assistant_oscilloscope_data.dat[0] = ch1;
    seekfree_assistant_oscilloscope_data.dat[1] = ch2;
    seekfree_assistant_oscilloscope_data.dat[2] = ch3;
    seekfree_assistant_oscilloscope_data.dat[3] = ch4;

    seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);
}

/************ 参数处理 ************/
uint8 wifi_param_updated(uint8 index)
{
    if(index >= SEEKFREE_ASSISTANT_SET_PARAMETR_COUNT)
        return 0;

    if(seekfree_assistant_parameter_update_flag[index])
    {
        seekfree_assistant_parameter_update_flag[index] = 0;
        return 1;
    }
    return 0;
}

float wifi_get_param(uint8 index)
{
    if(index >= SEEKFREE_ASSISTANT_SET_PARAMETR_COUNT)
        return 0.0f;
    return seekfree_assistant_parameter[index];
}
