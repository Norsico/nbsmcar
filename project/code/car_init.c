#include "car_init.h"
#include "car_motor.h"
#include "car_servo.h"
#include "encoder.h"
#include "motor_pid.h"

// WiFi 默认配置
#define DEFAULT_WIFI_SSID       "QQ"
#define DEFAULT_WIFI_PASSWORD   "1234567890xia"
#define DEFAULT_TARGET_IP       "192.168.43.236"

static const char *car_target_ip_or_default(const char *target_ip)
{
    if((NULL != target_ip) && ('\0' != target_ip[0]))
    {
        return target_ip;
    }

    return DEFAULT_TARGET_IP;
}

void car_init(uint8 enable_wifi, const char *wifi_ssid, const char *wifi_password, const char *target_ip)
{
    const char *connect_ip = NULL;
    const char *use_ssid = NULL;
    const char *use_password = NULL;

    // 初始化车辆基础模块
    car_servo_init();
    car_motor_init();
    encoder_init();
    motor_pid_init();  // 初始化电机PID控制器

    if(!enable_wifi)
    {
        return;
    }

    // 使用默认 WiFi 参数（如果未提供）
    use_ssid = (wifi_ssid && wifi_ssid[0]) ? wifi_ssid : DEFAULT_WIFI_SSID;
    use_password = (wifi_password && wifi_password[0]) ? wifi_password : DEFAULT_WIFI_PASSWORD;
    connect_ip = car_target_ip_or_default(target_ip);

    while(wifi_spi_init(use_ssid, use_password))
    {
        printf("\r\n connect wifi failed. \r\n");
        system_delay_ms(100);
    }

    printf("\r\n module version:%s \r\n", wifi_spi_version);
    printf("\r\n module mac    :%s \r\n", wifi_spi_mac_addr);
    printf("\r\n module ip     :%s \r\n", wifi_spi_ip_addr_port);

    if(0 == WIFI_SPI_AUTO_CONNECT)
    {
        while(wifi_spi_socket_connect("TCP", connect_ip, WIFI_SPI_TARGET_PORT, WIFI_SPI_LOCAL_PORT))
        {
            printf("\r\n Connect TCP Servers error, try again. \r\n");
            system_delay_ms(100);
        }
    }

    seekfree_assistant_init();
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);
}
