#include "car_init.h"

static const char *car_target_ip_or_default(const char *target_ip)
{
    if((NULL != target_ip) && ('\0' != target_ip[0]))
    {
        return target_ip;
    }

    return WIFI_SPI_TARGET_IP;
}

void seekfree_assistant_wifi_init(const char *wifi_ssid, const char *wifi_password, const char *target_ip)
{
    const char *connect_ip = car_target_ip_or_default(target_ip);

    while(wifi_spi_init(wifi_ssid, wifi_password))
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
