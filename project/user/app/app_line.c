#include "app_line.h"

#include "search_line.h"
#include "steer.h"
#include "dev_other.h"
#include "dev_servo.h"
#include "dev_wheel.h"

static uint8 line_camera_ready = 0; // 摄像头是否初始化完成
static uint8 line_result_ready = 0;
static line_app_preview_mode_t g_line_preview_mode = LINE_APP_PREVIEW_BINARY;
static uint8 g_line_last_straight_acc = 0;

/* 下发整页相机参数。 */
static uint8 line_app_apply_camera_page(const flash_camera_page_t *page)
{
    short int config[MT9V03X_CONFIG_FINISH][2];

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

    return (0 == mt9v03x_sccb_set_config(config)) ? 1 : 0;
}

/* 从 flash 恢复相机参数。 */
static void line_app_apply_camera_page_from_flash(void)
{
    flash_camera_page_t page;

    flash_store_get_camera_page(&page);
    line_app_apply_camera_page(&page);
}

/* 从 flash 恢复舵机 PD 参数。 */
static void line_app_apply_steer_pd_page_from_flash(void)
{
    flash_param_page_t page;

    flash_store_get_param_page(&page);
    Steer_Set_Pd((uint16)page.first_value,
                 (uint16)page.second_value,
                 (uint16)page.third_value);
}

#if IPS_ENABLE
/* 绘制相机页预览。 */
void line_app_render_frame(void)
{
    if(!line_camera_ready)
    {
        // 摄像头未初始化
        return;
    }

    if(!switch_ui_enabled())
    {
        return;
    }

    /* 相机页只切显示模式。 */
    if(LINE_APP_PREVIEW_RAW == g_line_preview_mode)
    {
        SearchLine_DrawRawPreview();
    }
    else
    {
        SearchLine_DrawBinaryPreview();
    }
}
#endif

/* 切换相机页预览模式。 */
void line_app_set_preview_mode(line_app_preview_mode_t mode)
{
    if(LINE_APP_PREVIEW_RAW == mode)
    {
        g_line_preview_mode = LINE_APP_PREVIEW_RAW;
    }
    else
    {
        g_line_preview_mode = LINE_APP_PREVIEW_BINARY;
    }
}

/* 读取当前预览模式。 */
line_app_preview_mode_t line_app_get_preview_mode(void)
{
    return g_line_preview_mode;
}

/* 处理一帧图像。 */
static uint8 line_app_handle_frame(void)
{
    uint8 raw_otsu_threshold = 0;
    uint8 straight_acc = 0;

    if(!line_camera_ready)
    {
        // 摄像头未初始化
        return 0;
    }

    if(!mt9v03x_finish_flag)
    {
        return 0;
    }

    // 处理图像
    SearchLine_Process();
    raw_otsu_threshold = SearchLine_GetRawOtsuThreshold();
    if(raw_otsu_threshold < 25U)
    {
        /* 工作阈值继续锁下限，抓车急停单独看原始阈值，避免黑场时还继续出力。 */
        g_line_last_straight_acc = 0;
        line_result_ready = 0;
        g_system_state = SYS_EMERGENCY;
        return 1;
    }

    straight_acc = SearchLine_GetStraightAcc();
    if(SYS_RUNNING == g_system_state)
    {
        if((0U == g_line_last_straight_acc) && (0U != straight_acc))
        {
            // buzzer_long();
        }
        g_line_last_straight_acc = straight_acc;
    }
    else
    {
        g_line_last_straight_acc = 0;
    }

    line_result_ready = 1;
    
    return 1;
}

/* 按独立节拍执行前轮 PD。 */
void line_app_process_steer(void)
{
    if(!line_camera_ready || !line_result_ready)
    {
        return;
    }

    SteerPID_Realize(SearchLine_GetDetTrue() - ImageSensorMid);
}

/* 初始化巡线应用。 */
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
    Steer_init();
    line_app_apply_steer_pd_page_from_flash();
    line_camera_ready = 1; // 摄像头初始化OK
    line_result_ready = 0;
    g_line_last_straight_acc = 0;

#if IPS_ENABLE
    if(switch_ui_enabled())
    {
        ips200_show_string(0, 16, "init success.");
    }
#endif
}

/* 处理一帧巡线图像。 */
uint8 line_app_process_frame(void)
{
    return line_app_handle_frame();
}

/* 修改单个相机参数并落盘。 */
uint8 line_app_set_camera_param_value(flash_camera_slot_t slot, uint16 value)
{
    flash_camera_page_t page;
    uint8 apply_ok = 0;

    if(!line_camera_ready)
    {
        // 摄像头未初始化
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

/* 修改舵机 PD 参数并同步到搜线模块。 */
uint8 line_app_set_steer_pd_value(flash_param_slot_t slot, int16 value)
{
    flash_param_page_t page;

    flash_store_get_param_page(&page);
    switch(slot)
    {
        case FLASH_PARAM_SLOT_FIRST:
            page.first_value = value;
            break;
        case FLASH_PARAM_SLOT_SECOND:
            page.second_value = value;
            break;
        case FLASH_PARAM_SLOT_THIRD:
            page.third_value = value;
            break;
        default:
            return 0;
    }

    if(!flash_store_set_param_value(slot, value))
    {
        return 0;
    }

    Steer_Set_Pd((uint16)page.first_value,
                 (uint16)page.second_value,
                 (uint16)page.third_value);
    return 1;
}
