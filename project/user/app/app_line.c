#include "app_line.h"

#include "SearchLine.h"
#include "dev_other.h"
#include "dev_servo.h"

static uint8 line_camera_ready = 0;

static uint8 line_app_apply_camera_page(const flash_camera_page_t *page)
{
    int16 config[MT9V03X_CONFIG_FINISH][2];

    if(0 == page)
    {
        return 0;
    }

    memset(config, 0, sizeof(config));
    config[0][0] = MT9V03X_INIT;
    config[0][1] = 0;
    config[1][0] = MT9V03X_AUTO_EXP;
    config[1][1] = page->auto_exp;
    config[2][0] = MT9V03X_EXP_TIME;
    config[2][1] = page->exp_time;
    config[3][0] = MT9V03X_FPS;
    config[3][1] = MT9V03X_FPS_DEF;
    config[4][0] = MT9V03X_SET_COL;
    config[4][1] = MT9V03X_W;
    config[5][0] = MT9V03X_SET_ROW;
    config[5][1] = MT9V03X_H;
    config[6][0] = MT9V03X_LR_OFFSET;
    config[6][1] = MT9V03X_LR_OFFSET_DEF;
    config[7][0] = MT9V03X_UD_OFFSET;
    config[7][1] = MT9V03X_UD_OFFSET_DEF;
    config[8][0] = MT9V03X_GAIN;
    config[8][1] = page->gain;

    return (0 == mt9v03x_set_config(config)) ? 1 : 0;
}

static void line_app_apply_camera_page_from_flash(void)
{
    flash_camera_page_t page;

    flash_store_get_camera_page(&page);
    line_app_apply_camera_page(&page);
}

static void line_app_apply_steer_pd_page_from_flash(void)
{
    flash_param_page_t page;

    flash_store_get_param_page(&page);
    SearchLine_SetSteerPdTenth((uint16)page.first_value_tenth, (uint16)page.second_value_tenth);
}

#if IPS_ENABLE
void line_app_render_frame(void)
{
    if(!line_camera_ready)
    {
        return;
    }

    if(!switch_ui_enabled())
    {
        return;
    }

    /* 显示压缩二值图。 */
    SearchLine_DrawBinaryPreview();
}
#endif

static uint8 line_app_handle_frame(void)
{
    uint8 raw_threshold = 0;

    if(!line_camera_ready)
    {
        return 0;
    }

    if(!mt9v03x_finish_flag)
    {
        return 0;
    }

    SearchLine_Process();
    raw_threshold = SearchLine_GetRawOtsuThreshold();
    if((SYS_RUNNING == g_system_state) && (raw_threshold < 18))
    {
        /* 抓车后当前帧阈值过低，直接锁死到急停态。 */
        system_error = 1;
        g_system_state = SYS_EMERGENCY;
    }
    if(SYS_EMERGENCY != g_system_state)
    {
        /* 当前阶段先只把参考舵机命令接到前轮，后轮目标仍沿用现有链路。 */
        car_servo_set_angle(SearchLine_GetSteerCommand());
    }
    mt9v03x_finish_flag = 0;
    return 1;
}

void line_app_init(void)
{
    car_servo_set_center();

#if IPS_ENABLE
    if(switch_ui_enabled())
    {
        ips200_clear(RGB565_WHITE);
        ips200_show_string(0, 0, "mt9v03x init.");
    }
#endif

    while(mt9v03x_init())
    {
#if IPS_ENABLE
        if(switch_ui_enabled())
        {
            ips200_show_string(0, 16, "mt9v03x reinit.");
        }
#endif
        system_delay_ms(100);
    }

    line_app_apply_camera_page_from_flash();
    line_app_apply_steer_pd_page_from_flash();
    line_camera_ready = 1;

#if IPS_ENABLE
    if(switch_ui_enabled())
    {
        ips200_show_string(0, 16, "init success.");
    }
#endif
}

uint8 line_app_process_frame(void)
{
    return line_app_handle_frame();
}

uint8 line_app_set_camera_param_value(flash_camera_slot_t slot, uint16 value)
{
    flash_camera_page_t page;
    uint8 apply_ok = 0;

    if(!line_camera_ready)
    {
        return 0;
    }

    flash_store_get_camera_page(&page);

    switch(slot)
    {
        case FLASH_CAMERA_SLOT_EXP_TIME:
            page.exp_time = value;
            apply_ok = (0 == mt9v03x_set_exposure_time(value)) ? 1 : 0;
            break;
        case FLASH_CAMERA_SLOT_GAIN:
            page.gain = (uint8)value;
            apply_ok = line_app_apply_camera_page(&page);
            break;
        default:
            return 0;
    }

    if(!apply_ok)
    {
        return 0;
    }

    return flash_store_set_camera_page(&page);
}

uint8 line_app_set_steer_pd_value_tenth(flash_param_slot_t slot, int16 value_tenth)
{
    flash_param_page_t page;

    flash_store_get_param_page(&page);
    switch(slot)
    {
        case FLASH_PARAM_SLOT_FIRST:
            page.first_value_tenth = value_tenth;
            break;
        case FLASH_PARAM_SLOT_SECOND:
            page.second_value_tenth = value_tenth;
            break;
        default:
            return 0;
    }

    if(!flash_store_set_param_value_tenth(slot, value_tenth))
    {
        return 0;
    }

    SearchLine_SetSteerPdTenth((uint16)page.first_value_tenth, (uint16)page.second_value_tenth);
    return 1;
}
