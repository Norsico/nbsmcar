#ifndef __SMARTCAR_HEADFILE_H__
#define __SMARTCAR_HEADFILE_H__

#include "intrins.h"

#include "stc32g144k246.h"

/* Common / 通用基础 */
#include "zf_common_typedef.h"
#include "zf_common_clock.h"
#include "zf_common_interrupt.h"
#include "zf_common_font.h"
#include "zf_common_function.h"
#include "zf_common_debug.h"

/* Driver / 单片机外设驱动 */
#include "zf_driver_uart.h"         /* UART / 串口 */
#include "zf_driver_gpio.h"
#include "zf_driver_adc.h"
#include "zf_driver_spi.h"
#include "zf_driver_timer.h"
#include "zf_driver_pwm.h"
#include "zf_driver_exti.h"
#include "zf_driver_delay.h"
#include "zf_driver_eeprom.h"
#include "zf_driver_pit.h"
#include "zf_driver_encoder.h"
#include "zf_driver_soft_iic.h"
#include "zf_driver_usb_cdc.h"      /* USB CDC / USB 虚拟串口 */

/* Device / 外设模块 */
#include "zf_device_config.h"
#include "zf_device_type.h"
#include "zf_device_mt9v03x.h"
#include "zf_device_mt9v03x_dma.h"
#include "zf_device_imu660ra.h"
#include "zf_device_ips200.h"

//#include "zf_device_imu660rb.h"    /* IMU660RB / 陀螺仪，当前未用 */
//#include "zf_device_imu660rc.h"    /* IMU660RC / 陀螺仪，当前未用 */
//#include "zf_device_imu963ra.h"    /* IMU963RA / 陀螺仪，当前未用 */
//#include "zf_device_tft180.h"      /* TFT180 / 屏幕，当前未用 */
//#include "zf_device_ips114.h"      /* IPS114 / 屏幕，当前未用 */
//#include "zf_device_ips200pro.h"   /* IPS200PRO / 屏幕，当前未用 */
//#include "zf_device_dl1a.h"        /* DL1A / 测距模块，当前未用 */
//#include "zf_device_dl1b.h"        /* DL1B / 测距模块，当前未用 */
//#include "zf_device_ble6a20.h"     /* BLE6A20 / 蓝牙模块，当前未用 */
//#include "zf_device_wireless_uart.h" /* Wireless UART / 无线串口，当前未用 */
//#include "zf_device_wifi_spi.h"    /* WIFI SPI / WiFi 模块，当前未用 */

/* Component / 组件 */
//#include "seekfree_assistant.h"    /* SeekFree Assistant / 逐飞助手，当前未用 */
//#include "seekfree_assistant_interface.h"

#include "para.h"
#include "flash.h"
#include "state.h"
#include "motor.h"
#include "buzzer.h"
#include "servo.h"
#include "image.h"
#include "ui.h"
#include "direction_gate.h"

#endif
