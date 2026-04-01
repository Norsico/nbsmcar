/*********************************************************************************************************************
* STC32G144 Opensource Library - WiFi SPI Device Driver
* Copyright (c) 2025 SEEKFREE
*
********************************************************************************************************************/
#include "dev_wifi.h"
#include "zf_driver_gpio.h"

/************ 静态变量 ************/
static uint8 wifi_initialized = 0;

uint8 wifi_is_initialized(void)
{
    return wifi_initialized;
}

/************ 初始化 ************/
uint8 wifi_init(const char* wifi_ssid,
                const char* wifi_password,
                const char* target_ip,
                const char* target_port,
                const char* local_port)
{
    const char* WIFI_SSID = (wifi_ssid && wifi_ssid[0]) ? wifi_ssid : "";
    const char* WIFI_PASSWORD = (wifi_password && wifi_password[0]) ? wifi_password : 0;
    const char* TARGET_IP = (target_ip && target_ip[0]) ? target_ip : WIFI_SPI_TARGET_IP;
    const char* TARGET_PORT = (target_port && target_port[0]) ? target_port : WIFI_SPI_TARGET_PORT;
    const char* LOCAL_PORT = (local_port && local_port[0]) ? local_port : WIFI_SPI_LOCAL_PORT;
    uint8 retry = 0;
    uint8 ret = 0;

    if(wifi_initialized)
    {
        return 0;
    }

    printf("\r\n[WiFi] ========================================");
    printf("\r\n[WiFi] WiFi Driver Starting...");
    printf("\r\n[WiFi] SSID: %s", WIFI_SSID);
    printf("\r\n[WiFi] AUTO_CONNECT: %d", WIFI_SPI_AUTO_CONNECT);
    printf("\r\n[WiFi] TARGET_IP: %s", TARGET_IP);
    printf("\r\n[WiFi] TARGET_PORT: %s", TARGET_PORT);
    printf("\r\n[WiFi] LOCAL_PORT: %s", LOCAL_PORT);
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
        ret = wifi_spi_init((char *)WIFI_SSID, (char *)WIFI_PASSWORD);

        if(ret == 0)
        {
            // 成功
            printf("\r\n[WiFi] ========================================");
            printf("\r\n[WiFi] WiFi Init SUCCESS!");
            printf("\r\n[WiFi] Version: %s", wifi_spi_version);
            printf("\r\n[WiFi] MAC: %s", wifi_spi_mac_addr);
            printf("\r\n[WiFi] IP:Port = %s", wifi_spi_ip_addr_port);
            printf("\r\n[WiFi] ========================================\r\n");

            // socket连接
            if(0 == WIFI_SPI_AUTO_CONNECT)
            {
                while(wifi_spi_socket_connect("TCP", (char *)TARGET_IP, (char *)TARGET_PORT, (char *)LOCAL_PORT))
                {
                    system_delay_ms(100);
                }
            }

            // 初始化Seekfree Assistant
            seekfree_assistant_init();
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
            printf("\r\n[WiFi] Waiting 100ms before retry...\r\n");
            system_delay_ms(100);
        }
    } 
    return 1;
}
