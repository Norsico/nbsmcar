#include "zf_common_headfile.h"
#include "motor.h"
#include "wifi.h"

typedef enum
{
    WIFI_STAGE_MODULE = 0,                                      /* 模块初始化 */
    WIFI_STAGE_SOCKET,                                          /* 建链 */
    WIFI_STAGE_READY                                            /* 运行 */
} wifi_stage_t;

static wifi_stage_t wifi_stage = WIFI_STAGE_MODULE;
static uint8 wifi_assistant_ready = 0;
static uint8 wifi_task_ready = 0;
static volatile uint8 wifi_tick_ready = 0;

/* WiFi定时器 */
static void wifi_pit_handler(void)
{
    wifi_tick_ready = 1;
}

/* WiFi定时器初始化 */
static void wifi_task_init(void)
{
    if(wifi_task_ready)
    {
        return;
    }

    pit_ms_init(WIFI_TASK_PIT, WIFI_TASK_PERIOD_MS, wifi_pit_handler);
    wifi_task_ready = 1;
}

/* 同步调参缓存 */
static void wifi_sync_parameter_buffer(void)
{
#if WIFI_PARAM_ENABLE
    int16 kp;
    int16 ki;

    seekfree_assistant_parameter[0] = (float)motor_data.target_left;

    motor_get_pid_left(&kp, &ki);
    seekfree_assistant_parameter[1] = (float)kp;
    seekfree_assistant_parameter[2] = (float)ki;

    motor_get_pid_right(&kp, &ki);
    seekfree_assistant_parameter[3] = (float)kp;
    seekfree_assistant_parameter[4] = (float)ki;
#endif
}

/* 逐飞助手初始化 */
static void wifi_assistant_init(void)
{
    if(wifi_assistant_ready)
    {
        return;
    }

    seekfree_assistant_init();
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);
    wifi_sync_parameter_buffer();
    wifi_assistant_ready = 1;
}

/* WiFi模块初始化 */
static uint8 wifi_module_init(void)
{
    if(0 != wifi_spi_init(WIFI_NAME, WIFI_PASSWORD))
    {
        return 0;
    }

    wifi_stage = WIFI_STAGE_SOCKET;
    return 1;
}

/* WiFi建链 */
static uint8 wifi_socket_init(void)
{
    if(0 != wifi_spi_socket_connect("TCP", WIFI_TARGET_IP, WIFI_SPI_TARGET_PORT, WIFI_SPI_LOCAL_PORT))
    {
        return 0;
    }

    wifi_assistant_init();
    wifi_stage = WIFI_STAGE_READY;
    return 1;
}

/* 电机参数下行 */
#if WIFI_PARAM_ENABLE
static void wifi_update_parameter(void)
{
    int16 left_kp;
    int16 left_ki;
    int16 right_kp;
    int16 right_ki;
    int16 target_speed;
    uint8 update_left;
    uint8 update_right;

    update_left = 0;
    update_right = 0;

    if(seekfree_assistant_parameter_update_flag[0])
    {
        seekfree_assistant_parameter_update_flag[0] = 0;
        target_speed = (int16)seekfree_assistant_parameter[0];
        motor_set_target(target_speed, target_speed);
    }

    if(seekfree_assistant_parameter_update_flag[1])
    {
        seekfree_assistant_parameter_update_flag[1] = 0;
        motor_get_pid_left(&left_kp, &left_ki);
        left_kp = (int16)seekfree_assistant_parameter[1];
        update_left = 1;
    }

    if(seekfree_assistant_parameter_update_flag[2])
    {
        seekfree_assistant_parameter_update_flag[2] = 0;
        if(!update_left)
        {
            motor_get_pid_left(&left_kp, &left_ki);
        }
        left_ki = (int16)seekfree_assistant_parameter[2];
        update_left = 1;
    }

    if(seekfree_assistant_parameter_update_flag[3])
    {
        seekfree_assistant_parameter_update_flag[3] = 0;
        motor_get_pid_right(&right_kp, &right_ki);
        right_kp = (int16)seekfree_assistant_parameter[3];
        update_right = 1;
    }

    if(seekfree_assistant_parameter_update_flag[4])
    {
        seekfree_assistant_parameter_update_flag[4] = 0;
        if(!update_right)
        {
            motor_get_pid_right(&right_kp, &right_ki);
        }
        right_ki = (int16)seekfree_assistant_parameter[4];
        update_right = 1;
    }

    if(update_left)
    {
        motor_set_pid_left(left_kp, left_ki);
    }

    if(update_right)
    {
        motor_set_pid_right(right_kp, right_ki);
    }

    wifi_sync_parameter_buffer();
}
#endif

/* 示波器上行 */
static void wifi_send_oscilloscope(void)
{
    seekfree_assistant_oscilloscope_data.dat[0] = (float)motor_data.target_left;
    seekfree_assistant_oscilloscope_data.dat[1] = (float)motor_data.count_left;
    seekfree_assistant_oscilloscope_data.dat[2] = (float)motor_data.count_right;
    seekfree_assistant_oscilloscope_data.channel_num = WIFI_OSC_CHANNEL_NUM;
    seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);
}

/* WiFi任务 */
void wifi_update(void)
{
    uint8 tick_ready;

    wifi_task_init();

    interrupt_global_disable();
    tick_ready = wifi_tick_ready;
    wifi_tick_ready = 0;
    interrupt_global_enable();

    if(!tick_ready)
    {
        return;
    }

    if(WIFI_STAGE_MODULE == wifi_stage)
    {
        if(!wifi_module_init())
        {
            return;
        }
    }

    if(WIFI_STAGE_SOCKET == wifi_stage)
    {
        if(!wifi_socket_init())
        {
            return;
        }
    }

#if WIFI_PARAM_ENABLE
    /* 收下行 */
    seekfree_assistant_data_analysis();

    /* 写参数 */
    wifi_update_parameter();
#endif

    /* 发示波器 */
    wifi_send_oscilloscope();
}
