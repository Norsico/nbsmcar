#include "app_line.h"

#include "pid_control.h"
#include "SearchLine.h"
#include "dev_flash.h"
#include "dev_other.h"
#include "dev_servo.h"
#include "system_state.h"

#define LINE_TRACK_ERROR_DEADBAND   (1)
#define LINE_TRACK_DEFAULT_KP       (1.20f)
#define LINE_TRACK_DEFAULT_KI       (0.00f)
#define LINE_TRACK_DEFAULT_KD       (1.0f)
#define LINE_TRACK_INTEGRAL_LIMIT   (200.0f)
#define LINE_TRACK_STEER_SIGN       (-1.0f)
#define LINE_TRACK_LOST_MIN_FOUND_ROWS  (12)   /* 工程侧推定阈值：有效中心行太少时，这一帧不参与转向更新。 */
#define LINE_TUNE_KP_MIN_TENTH          (0)    /* 工程侧推定范围：当前默认 1.2，屏幕上先开放到 0.0-10.0。 */
#define LINE_TUNE_KP_MAX_TENTH          (100)
#define LINE_TUNE_KP_STEP_TENTH         (1)
#define LINE_TUNE_KD_MIN_TENTH          (0)    /* 工程侧推定范围：当前默认 1.0，屏幕上先开放到 0.0-10.0。 */
#define LINE_TUNE_KD_MAX_TENTH          (100)
#define LINE_TUNE_KD_STEP_TENTH         (1)
#define LINE_TUNE_NEAR_ROW_MIN          (0)    /* 近点行从有效图像底部开始计偏移，0 表示紧贴底部。 */
#define LINE_TUNE_NEAR_ROW_MAX          (100)  /* 有效搜索区约 102 行，这里先给到 100，给 far 行留出至少 1 行间隔。 */
#define LINE_TUNE_NEAR_ROW_STEP         (1)
#define LINE_TUNE_FAR_ROW_MIN           (1)    /* 远点必须在近点之上，最小值从 1 开始。 */
#define LINE_TUNE_FAR_ROW_MAX           (101)  /* 有效搜索区约 102 行，上限按整块有效区来给。 */
#define LINE_TUNE_FAR_ROW_STEP          (1)
#define LINE_TUNE_NEAR_WEIGHT_MIN       (1)    /* 权重至少为 1，避免某一路被完全关掉。 */
#define LINE_TUNE_NEAR_WEIGHT_MAX       (10)   /* 工程侧先收口到 10，够覆盖常用近远点占比。 */
#define LINE_TUNE_NEAR_WEIGHT_STEP      (1)
#define LINE_TUNE_FAR_WEIGHT_MIN        (1)
#define LINE_TUNE_FAR_WEIGHT_MAX        (10)
#define LINE_TUNE_FAR_WEIGHT_STEP       (1)
#define LINE_TUNE_SERVO_MIN_MIN         (CAR_SERVO_TUNE_MIN_ANGLE)          /* 以当前舵机中心 90 度为基准，左限幅只开放到 60-89。 */
#define LINE_TUNE_SERVO_MIN_MAX         (CAR_SERVO_CENTER_ANGLE - 1)
#define LINE_TUNE_SERVO_MIN_STEP        (1)
#define LINE_TUNE_SERVO_MAX_MIN         (CAR_SERVO_CENTER_ANGLE + 1)        /* 右限幅只开放到 91-120，避免调参时把机构顶死。 */
#define LINE_TUNE_SERVO_MAX_MAX         (CAR_SERVO_TUNE_MAX_ANGLE)
#define LINE_TUNE_SERVO_MAX_STEP        (1)

typedef struct
{
    int16 near_error;
    int16 far_error;
    uint8 near_center;
    uint8 far_center;
    uint8 track_center;
    uint8 servo_angle;
    uint8 control_ready;
    uint8 lost_frame_count;
} line_track_info_t;

static uint8 line_camera_ready = 0;
static pid_control_t g_line_servo_pid;
static line_track_info_t g_line_track_info = {0};
static flash_line_tune_page_t g_line_tune_page_cache = {0};
static uint8 g_line_tune_page_ready = 0;
static uint8 g_line_tune_page_dirty = 0;
static uint8 line_app_limit_angle(float angle);

/* 这组 UI 参数都是无符号整数，统一用这个小工具做基础限幅。 */
static uint16 line_app_limit_uint16(uint16 value, uint16 min_value, uint16 max_value)
{
    if(value < min_value)
    {
        return min_value;
    }

    if(value > max_value)
    {
        return max_value;
    }

    return value;
}

/* 给 UI 提供每个巡线调参项的 min/max/step，避免显示层再重复维护一份。 */
void line_app_get_tune_range(line_tune_slot_t slot, uint16 *min_value, uint16 *max_value, uint16 *step_value)
{
    uint16 min_value_local = 0;
    uint16 max_value_local = 0;
    uint16 step_value_local = 0;

    switch(slot)
    {
        case LINE_TUNE_SLOT_KP:
            min_value_local = LINE_TUNE_KP_MIN_TENTH;
            max_value_local = LINE_TUNE_KP_MAX_TENTH;
            step_value_local = LINE_TUNE_KP_STEP_TENTH;
            break;
        case LINE_TUNE_SLOT_KD:
            min_value_local = LINE_TUNE_KD_MIN_TENTH;
            max_value_local = LINE_TUNE_KD_MAX_TENTH;
            step_value_local = LINE_TUNE_KD_STEP_TENTH;
            break;
        case LINE_TUNE_SLOT_NEAR_ROW:
            min_value_local = LINE_TUNE_NEAR_ROW_MIN;
            max_value_local = LINE_TUNE_NEAR_ROW_MAX;
            step_value_local = LINE_TUNE_NEAR_ROW_STEP;
            break;
        case LINE_TUNE_SLOT_FAR_ROW:
            min_value_local = LINE_TUNE_FAR_ROW_MIN;
            max_value_local = LINE_TUNE_FAR_ROW_MAX;
            step_value_local = LINE_TUNE_FAR_ROW_STEP;
            break;
        case LINE_TUNE_SLOT_NEAR_WEIGHT:
            min_value_local = LINE_TUNE_NEAR_WEIGHT_MIN;
            max_value_local = LINE_TUNE_NEAR_WEIGHT_MAX;
            step_value_local = LINE_TUNE_NEAR_WEIGHT_STEP;
            break;
        case LINE_TUNE_SLOT_FAR_WEIGHT:
            min_value_local = LINE_TUNE_FAR_WEIGHT_MIN;
            max_value_local = LINE_TUNE_FAR_WEIGHT_MAX;
            step_value_local = LINE_TUNE_FAR_WEIGHT_STEP;
            break;
        case LINE_TUNE_SLOT_SERVO_MIN:
            min_value_local = LINE_TUNE_SERVO_MIN_MIN;
            max_value_local = LINE_TUNE_SERVO_MIN_MAX;
            step_value_local = LINE_TUNE_SERVO_MIN_STEP;
            break;
        case LINE_TUNE_SLOT_SERVO_MAX:
            min_value_local = LINE_TUNE_SERVO_MAX_MIN;
            max_value_local = LINE_TUNE_SERVO_MAX_MAX;
            step_value_local = LINE_TUNE_SERVO_MAX_STEP;
            break;
        default:
            break;
    }

    if(0 != min_value)
    {
        *min_value = min_value_local;
    }

    if(0 != max_value)
    {
        *max_value = max_value_local;
    }

    if(0 != step_value)
    {
        *step_value = step_value_local;
    }
}

/* 用户改某一项后，统一把成对约束修正回来，避免近远点或舵机限幅互相打架。 */
static void line_app_normalize_tune_page(flash_line_tune_page_t *page)
{
    if(0 == page)
    {
        return;
    }

    page->kp_tenth = (uint8)line_app_limit_uint16(page->kp_tenth, LINE_TUNE_KP_MIN_TENTH, LINE_TUNE_KP_MAX_TENTH);
    page->kd_tenth = (uint8)line_app_limit_uint16(page->kd_tenth, LINE_TUNE_KD_MIN_TENTH, LINE_TUNE_KD_MAX_TENTH);
    page->near_row_offset = (uint8)line_app_limit_uint16(page->near_row_offset, LINE_TUNE_NEAR_ROW_MIN, LINE_TUNE_NEAR_ROW_MAX);
    page->far_row_offset = (uint8)line_app_limit_uint16(page->far_row_offset, LINE_TUNE_FAR_ROW_MIN, LINE_TUNE_FAR_ROW_MAX);
    page->near_weight = (uint8)line_app_limit_uint16(page->near_weight, LINE_TUNE_NEAR_WEIGHT_MIN, LINE_TUNE_NEAR_WEIGHT_MAX);
    page->far_weight = (uint8)line_app_limit_uint16(page->far_weight, LINE_TUNE_FAR_WEIGHT_MIN, LINE_TUNE_FAR_WEIGHT_MAX);
    page->servo_min_angle = (uint8)line_app_limit_uint16(page->servo_min_angle, LINE_TUNE_SERVO_MIN_MIN, LINE_TUNE_SERVO_MIN_MAX);
    page->servo_max_angle = (uint8)line_app_limit_uint16(page->servo_max_angle, LINE_TUNE_SERVO_MAX_MIN, LINE_TUNE_SERVO_MAX_MAX);

    if(page->near_row_offset >= page->far_row_offset)
    {
        if(page->far_row_offset < LINE_TUNE_FAR_ROW_MAX)
        {
            page->far_row_offset = (uint8)(page->near_row_offset + 1);
        }
        else if(page->near_row_offset > LINE_TUNE_NEAR_ROW_MIN)
        {
            page->near_row_offset = (uint8)(page->far_row_offset - 1);
        }
    }

    if(page->servo_min_angle >= page->servo_max_angle)
    {
        if(page->servo_max_angle > LINE_TUNE_SERVO_MAX_MIN)
        {
            page->servo_min_angle = (uint8)(page->servo_max_angle - 1);
        }
        else
        {
            page->servo_min_angle = FLASH_LINE_SERVO_MIN_DEFAULT;
            page->servo_max_angle = FLASH_LINE_SERVO_MAX_DEFAULT;
        }
    }
}

/* 巡线调参页在 app 层维护一份运行时缓存，显示、控制和延迟保存都走这一份。 */
static void line_app_cache_tune_page(const flash_line_tune_page_t *page)
{
    if(0 == page)
    {
        return;
    }

    memcpy(&g_line_tune_page_cache, page, sizeof(g_line_tune_page_cache));
    g_line_tune_page_ready = 1;
}

static void line_app_load_tune_page(flash_line_tune_page_t *page)
{
    if(0 == page)
    {
        return;
    }

    if(g_line_tune_page_ready)
    {
        memcpy(page, &g_line_tune_page_cache, sizeof(*page));
        return;
    }

    flash_store_get_line_tune_page(page);
    line_app_normalize_tune_page(page);
    line_app_cache_tune_page(page);
    g_line_tune_page_dirty = 0;
}

/* 把巡线调参真正下发到运行时控制：PID 参数和舵机限幅在这里更新。 */
static uint8 line_app_apply_tune_page(const flash_line_tune_page_t *page)
{
    uint8 current_angle = 0;

    if(0 == page)
    {
        return 0;
    }

    g_line_servo_pid.param.kp = (float)page->kp_tenth / 10.0f;
    g_line_servo_pid.param.ki = LINE_TRACK_DEFAULT_KI;
    g_line_servo_pid.param.kd = (float)page->kd_tenth / 10.0f;
    car_servo_set_limit(page->servo_min_angle, page->servo_max_angle);
    current_angle = g_line_track_info.servo_angle;
    if(0 == current_angle)
    {
        current_angle = CAR_SERVO_CENTER_ANGLE;
    }

    g_line_track_info.servo_angle = line_app_limit_angle(current_angle);
    car_servo_set_angle(g_line_track_info.servo_angle);
    return 1;
}

/* 上电先从 flash 恢复巡线调参，后续调参时只改缓存，离页再统一保存。 */
static void line_app_apply_tune_page_from_flash(void)
{
    flash_line_tune_page_t page;

    line_app_load_tune_page(&page);
    line_app_apply_tune_page(&page);
    line_app_cache_tune_page(&page);
    g_line_tune_page_dirty = 0;
}

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

static uint8 line_app_limit_angle(float angle)
{
    if(angle < car_servo_get_min_angle())
    {
        return car_servo_get_min_angle();
    }

    if(angle > car_servo_get_max_angle())
    {
        return car_servo_get_max_angle();
    }

    return (uint8)(angle + 0.5f);
}

static uint8 line_app_limit_row(int16 row)
{
    if(row < SEARCH_VALID_TOP_ROW)
    {
        return SEARCH_VALID_TOP_ROW;
    }

    if(row > SEARCH_VALID_BOTTOM_ROW)
    {
        return SEARCH_VALID_BOTTOM_ROW;
    }

    return (uint8)row;
}

/* near/far 参考行都没找到，或者整帧可用中心线太少时，认为这帧不可靠。 */
static uint8 line_app_frame_track_is_valid(void)
{
    uint8 near_row = 0;
    uint8 far_row = 0;
    uint8 found_count = 0;

    near_row = line_app_limit_row((int16)SEARCH_VALID_BOTTOM_ROW - g_line_tune_page_cache.near_row_offset);
    far_row = line_app_limit_row((int16)SEARCH_VALID_BOTTOM_ROW - g_line_tune_page_cache.far_row_offset);
    found_count = SearchLine_GetCenterFoundRowCount();

    if(found_count < LINE_TRACK_LOST_MIN_FOUND_ROWS)
    {
        return 0;
    }

    if(!SearchLine_IsCenterFound(near_row))
    {
        return 0;
    }

    if(!SearchLine_IsCenterFound(far_row))
    {
        return 0;
    }

    return 1;
}

static void line_app_calc_preview_center(void)
{
    int16 image_center = (int16)(Search_Image_W / 2);
    int16 near_row = 0;
    int16 far_row = 0;
    int16 preview_error = 0;
    int16 weight_sum = 0;

    /* 近点看车头附近，先保证当前这一步不冲出边界。 */
    near_row = SEARCH_VALID_BOTTOM_ROW - g_line_tune_page_cache.near_row_offset;
    /* 远点给一点提前量，让舵机不要总是等压到线边才开始转。 */
    far_row = SEARCH_VALID_BOTTOM_ROW - g_line_tune_page_cache.far_row_offset;

    g_line_track_info.near_center = Center_Line[line_app_limit_row(near_row)];
    g_line_track_info.far_center = Center_Line[line_app_limit_row(far_row)];
    g_line_track_info.near_error = (int16)g_line_track_info.near_center - image_center;
    g_line_track_info.far_error = (int16)g_line_track_info.far_center - image_center;

    weight_sum = g_line_tune_page_cache.near_weight + g_line_tune_page_cache.far_weight;
    preview_error = (int16)((g_line_track_info.near_error * g_line_tune_page_cache.near_weight +
                             g_line_track_info.far_error * g_line_tune_page_cache.far_weight) / weight_sum);
    g_line_track_info.track_center = (uint8)(image_center + preview_error);
    g_line_servo_pid.error = (float)preview_error;
}

static void line_app_update_control(void)
{
    float servo_delta = 0.0f;
    uint8 frame_valid = 0;

    frame_valid = line_app_frame_track_is_valid();
    if(!frame_valid)
    {
        if(g_line_track_info.lost_frame_count < 255)
        {
            g_line_track_info.lost_frame_count++;
        }

        /* 丢线期间沿用上一拍舵角继续走，只在重新找回线时再恢复闭环更新。 */
        g_line_track_info.control_ready = 0;
        g_line_servo_pid.integral = 0.0f;
        car_servo_set_angle(g_line_track_info.servo_angle);
        return;
    }

    g_line_track_info.lost_frame_count = 0;
    line_app_calc_preview_center();

    if(g_line_servo_pid.error > -LINE_TRACK_ERROR_DEADBAND && g_line_servo_pid.error < LINE_TRACK_ERROR_DEADBAND)
    {
        g_line_servo_pid.error = 0.0f;
    }

    if(!g_line_track_info.control_ready)
    {
        g_line_servo_pid.prev_error = g_line_servo_pid.error;
        g_line_servo_pid.integral = 0.0f;
        g_line_track_info.control_ready = 1;
    }

    g_line_servo_pid.integral += g_line_servo_pid.error;
    if(g_line_servo_pid.integral > LINE_TRACK_INTEGRAL_LIMIT)
    {
        g_line_servo_pid.integral = LINE_TRACK_INTEGRAL_LIMIT;
    }
    else if(g_line_servo_pid.integral < -LINE_TRACK_INTEGRAL_LIMIT)
    {
        g_line_servo_pid.integral = -LINE_TRACK_INTEGRAL_LIMIT;
    }

    servo_delta = LINE_TRACK_STEER_SIGN *
                  (g_line_servo_pid.param.kp * g_line_servo_pid.error +
                   g_line_servo_pid.param.ki * g_line_servo_pid.integral +
                   g_line_servo_pid.param.kd * (g_line_servo_pid.error - g_line_servo_pid.prev_error));

    g_line_track_info.servo_angle = line_app_limit_angle((float)CAR_SERVO_CENTER_ANGLE + servo_delta);
    car_servo_set_angle(g_line_track_info.servo_angle);
    g_line_servo_pid.prev_error = g_line_servo_pid.error;
}

#if IPS_ENABLE
/* 参考行只画稀疏点，尽量别把底图遮太狠。 */
static void line_app_draw_preview_row(uint8 row, uint16 color)
{
    uint16 col = 0;

    for(col = 0; col < CAMERA_VALID_W; col += 4)
    {
        ips200_draw_point(col, row, color);
    }
}

/* 在近点/远点中心位置补一个十字，调 preview 参数时更容易对照。 */
static void line_app_draw_preview_cross(uint8 col, uint8 row, uint16 color)
{
    if(col >= CAMERA_VALID_W)
    {
        return;
    }

    ips200_draw_point(col, row, color);
    if(row > 0)
    {
        ips200_draw_point(col, row - 1, color);
    }
    if((uint16)row + 1 < CAMERA_RAW_H)
    {
        ips200_draw_point(col, row + 1, color);
    }
    if(col > 0)
    {
        ips200_draw_point(col - 1, row, color);
    }
    if((uint16)col + 1 < CAMERA_VALID_W)
    {
        ips200_draw_point(col + 1, row, color);
    }
}

/* 预览页叠加当前预瞄配置，让 near/far row 和 center 一眼能看出来。 */
static void line_app_draw_tuning_overlay(void)
{
    uint8 near_row = 0;
    uint8 far_row = 0;

    near_row = line_app_limit_row((int16)SEARCH_VALID_BOTTOM_ROW - g_line_tune_page_cache.near_row_offset);
    far_row = line_app_limit_row((int16)SEARCH_VALID_BOTTOM_ROW - g_line_tune_page_cache.far_row_offset);

    line_app_draw_preview_row(near_row, RGB565_GREEN);
    line_app_draw_preview_row(far_row, RGB565_MAGENTA);
    line_app_draw_preview_cross(g_line_track_info.near_center, near_row, RGB565_GREEN);
    line_app_draw_preview_cross(g_line_track_info.far_center, far_row, RGB565_MAGENTA);
}

void line_app_render_frame(void)
{
    if(!line_camera_ready)
    {
        return;
    }

    if(!g_ips_enable)
    {
        return;
    }

    ips200_show_gray_image(0, 0, mt9v03x_image[0], CAMERA_RAW_W, CAMERA_RAW_H, CAMERA_VALID_W, CAMERA_RAW_H, 0);
    SearchLine_DrawOverlay();
    line_app_draw_tuning_overlay();
}
#endif

static uint8 line_app_handle_frame(void)
{
    if(!line_camera_ready)
    {
        return 0;
    }

    if(!mt9v03x_finish_flag)
    {
        return 0;
    }

    SearchLine_Process();
    line_app_update_control();

    mt9v03x_finish_flag = 0;
    return 1;
}

void line_app_ctrl_init(void)
{
    pid_param_init(&g_line_servo_pid, LINE_TRACK_DEFAULT_KP, LINE_TRACK_DEFAULT_KI, LINE_TRACK_DEFAULT_KD, 0.0f, 0.0f);
    pid_init(&g_line_servo_pid);
    g_line_track_info.near_error = 0;
    g_line_track_info.far_error = 0;
    g_line_track_info.track_center = (uint8)(Search_Image_W / 2);
    g_line_track_info.near_center = (uint8)(Search_Image_W / 2);
    g_line_track_info.far_center = (uint8)(Search_Image_W / 2);
    g_line_track_info.servo_angle = CAR_SERVO_CENTER_ANGLE;
    g_line_track_info.control_ready = 0;
    g_line_track_info.lost_frame_count = 0;
    car_servo_set_center();
}

void line_app_camera_init(void)
{
#if IPS_ENABLE
    if(g_ips_enable)
    {
        ips200_clear(RGB565_WHITE);
        ips200_show_string(0, 0, "mt9v03x init.");
    }
#endif

    while(mt9v03x_init())
    {
#if IPS_ENABLE
        if(g_ips_enable)
        {
            ips200_show_string(0, 16, "mt9v03x reinit.");
        }
#endif
        system_delay_ms(100);
    }

    line_app_apply_camera_page_from_flash();
    line_app_apply_tune_page_from_flash();

    line_camera_ready = 1;

#if IPS_ENABLE
    if(g_ips_enable)
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

uint16 line_app_get_tune_value(line_tune_slot_t slot)
{
    flash_line_tune_page_t page;

    /* UI 读取统一走 app 层缓存，保证显示值和当前运行时配置一致。 */
    line_app_load_tune_page(&page);

    switch(slot)
    {
        case LINE_TUNE_SLOT_KP:
            return page.kp_tenth;
        case LINE_TUNE_SLOT_KD:
            return page.kd_tenth;
        case LINE_TUNE_SLOT_NEAR_ROW:
            return page.near_row_offset;
        case LINE_TUNE_SLOT_FAR_ROW:
            return page.far_row_offset;
        case LINE_TUNE_SLOT_NEAR_WEIGHT:
            return page.near_weight;
        case LINE_TUNE_SLOT_FAR_WEIGHT:
            return page.far_weight;
        case LINE_TUNE_SLOT_SERVO_MIN:
            return page.servo_min_angle;
        case LINE_TUNE_SLOT_SERVO_MAX:
            return page.servo_max_angle;
        default:
            return 0;
    }
}

/* 巡线调参统一走“读运行时页 -> 改一项 -> 归一化 -> 运行时生效”这一条链。 */
uint8 line_app_set_tune_value(line_tune_slot_t slot, uint16 value)
{
    flash_line_tune_page_t page;

    if(!line_camera_ready)
    {
        return 0;
    }

    line_app_load_tune_page(&page);

    switch(slot)
    {
        case LINE_TUNE_SLOT_KP:
            page.kp_tenth = (uint8)value;
            break;
        case LINE_TUNE_SLOT_KD:
            page.kd_tenth = (uint8)value;
            break;
        case LINE_TUNE_SLOT_NEAR_ROW:
            page.near_row_offset = (uint8)value;
            break;
        case LINE_TUNE_SLOT_FAR_ROW:
            page.far_row_offset = (uint8)value;
            break;
        case LINE_TUNE_SLOT_NEAR_WEIGHT:
            page.near_weight = (uint8)value;
            break;
        case LINE_TUNE_SLOT_FAR_WEIGHT:
            page.far_weight = (uint8)value;
            break;
        case LINE_TUNE_SLOT_SERVO_MIN:
            page.servo_min_angle = (uint8)value;
            break;
        case LINE_TUNE_SLOT_SERVO_MAX:
            page.servo_max_angle = (uint8)value;
            break;
        default:
            return 0;
    }

    /* near/far row、servo min/max 这类互相约束的参数都在这里一起修正。 */
    line_app_normalize_tune_page(&page);

    if(!line_app_apply_tune_page(&page))
    {
        return 0;
    }

    line_app_cache_tune_page(&page);
    g_line_tune_page_dirty = 1;
    return 1;
}

/* Line Tune 在离开子页时再统一落盘，避免长按调参时每一步都擦写 flash。 */
uint8 line_app_save_tune_page(void)
{
    flash_line_tune_page_t page;

    if(!line_camera_ready)
    {
        return 0;
    }

    if(!g_line_tune_page_ready || !g_line_tune_page_dirty)
    {
        return 1;
    }

    memcpy(&page, &g_line_tune_page_cache, sizeof(page));
    line_app_normalize_tune_page(&page);
    if(!flash_store_set_line_tune_page(&page))
    {
        return 0;
    }

    line_app_cache_tune_page(&page);
    g_line_tune_page_dirty = 0;
    return 1;
}