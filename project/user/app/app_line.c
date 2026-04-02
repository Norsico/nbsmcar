#include "app_line.h"

#include "SearchLine.h"
#include "dev_flash.h"
#include "dev_other.h"
#include "dev_servo.h"
#include "system_state.h"

#define LINE_TRACK_NEAR_ROW_OFFSET  (3)
#define LINE_TRACK_FAR_ROW_OFFSET   (15)
#define LINE_TRACK_NEAR_WEIGHT      (3)
#define LINE_TRACK_FAR_WEIGHT       (2)
#define LINE_TRACK_ERROR_DEADBAND   (1)
#define LINE_TRACK_DEFAULT_KP       (1.20f)
#define LINE_TRACK_DEFAULT_KI       (0.00f)
#define LINE_TRACK_DEFAULT_KD       (1.0f)
#define LINE_TRACK_INTEGRAL_LIMIT   (200.0f)
#define LINE_TRACK_STEER_SIGN       (-1.0f)
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
    float kp;
    float ki;
    float kd;
    float integral;
    int16 near_error;
    int16 far_error;
    int16 error;
    int16 last_error;
    uint8 near_center;
    uint8 far_center;
    uint8 track_center;
    uint8 servo_angle;
    uint8 near_row_offset;
    uint8 far_row_offset;
    uint8 near_weight;
    uint8 far_weight;
    uint8 control_ready;
} line_track_ctrl_t;

static uint8 line_camera_ready = 0;
static line_track_ctrl_t line_track_ctrl = {0};
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

/* 当 flash 里的巡线调参还没建立时，先用这一组工程默认值起步。 */
static void line_app_fill_default_tune_page(flash_line_tune_page_t *page)
{
    if(0 == page)
    {
        return;
    }

    page->kp_tenth = FLASH_LINE_KP_DEFAULT_TENTH;
    page->kd_tenth = FLASH_LINE_KD_DEFAULT_TENTH;
    page->near_row_offset = FLASH_LINE_NEAR_ROW_DEFAULT;
    page->far_row_offset = FLASH_LINE_FAR_ROW_DEFAULT;
    page->near_weight = FLASH_LINE_NEAR_WEIGHT_DEFAULT;
    page->far_weight = FLASH_LINE_FAR_WEIGHT_DEFAULT;
    page->servo_min_angle = FLASH_LINE_SERVO_MIN_DEFAULT;
    page->servo_max_angle = FLASH_LINE_SERVO_MAX_DEFAULT;
}

/* 给 UI 提供每个巡线调参项的 min/max/step，避免显示层再重复维护一份。 */
void line_app_get_tune_range(line_tune_slot_t slot, uint16 *min_value, uint16 *max_value, uint16 *step_value)
{
    if(0 == min_value || 0 == max_value || 0 == step_value)
    {
        return;
    }

    switch(slot)
    {
        case LINE_TUNE_SLOT_KP:
            *min_value = LINE_TUNE_KP_MIN_TENTH;
            *max_value = LINE_TUNE_KP_MAX_TENTH;
            *step_value = LINE_TUNE_KP_STEP_TENTH;
            break;
        case LINE_TUNE_SLOT_KD:
            *min_value = LINE_TUNE_KD_MIN_TENTH;
            *max_value = LINE_TUNE_KD_MAX_TENTH;
            *step_value = LINE_TUNE_KD_STEP_TENTH;
            break;
        case LINE_TUNE_SLOT_NEAR_ROW:
            *min_value = LINE_TUNE_NEAR_ROW_MIN;
            *max_value = LINE_TUNE_NEAR_ROW_MAX;
            *step_value = LINE_TUNE_NEAR_ROW_STEP;
            break;
        case LINE_TUNE_SLOT_FAR_ROW:
            *min_value = LINE_TUNE_FAR_ROW_MIN;
            *max_value = LINE_TUNE_FAR_ROW_MAX;
            *step_value = LINE_TUNE_FAR_ROW_STEP;
            break;
        case LINE_TUNE_SLOT_NEAR_WEIGHT:
            *min_value = LINE_TUNE_NEAR_WEIGHT_MIN;
            *max_value = LINE_TUNE_NEAR_WEIGHT_MAX;
            *step_value = LINE_TUNE_NEAR_WEIGHT_STEP;
            break;
        case LINE_TUNE_SLOT_FAR_WEIGHT:
            *min_value = LINE_TUNE_FAR_WEIGHT_MIN;
            *max_value = LINE_TUNE_FAR_WEIGHT_MAX;
            *step_value = LINE_TUNE_FAR_WEIGHT_STEP;
            break;
        case LINE_TUNE_SLOT_SERVO_MIN:
            *min_value = LINE_TUNE_SERVO_MIN_MIN;
            *max_value = LINE_TUNE_SERVO_MIN_MAX;
            *step_value = LINE_TUNE_SERVO_MIN_STEP;
            break;
        case LINE_TUNE_SLOT_SERVO_MAX:
            *min_value = LINE_TUNE_SERVO_MAX_MIN;
            *max_value = LINE_TUNE_SERVO_MAX_MAX;
            *step_value = LINE_TUNE_SERVO_MAX_STEP;
            break;
        default:
            *min_value = 0;
            *max_value = 0;
            *step_value = 0;
            break;
    }
}

/* 只做单项范围检查，不处理 near/far、min/max 这种成对关系。 */
static uint8 line_app_tune_value_in_range(line_tune_slot_t slot, uint16 value)
{
    uint16 min_value = 0;
    uint16 max_value = 0;
    uint16 step_value = 0;

    line_app_get_tune_range(slot, &min_value, &max_value, &step_value);
    if(0 == step_value)
    {
        return 0;
    }

    return (value >= min_value && value <= max_value) ? 1 : 0;
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

/* flash 读出来的整页参数先过一遍合法性，异常时直接回默认配置。 */
static uint8 line_app_tune_page_is_valid(const flash_line_tune_page_t *page)
{
    if(0 == page)
    {
        return 0;
    }

    if(!line_app_tune_value_in_range(LINE_TUNE_SLOT_KP, page->kp_tenth))
    {
        return 0;
    }

    if(!line_app_tune_value_in_range(LINE_TUNE_SLOT_KD, page->kd_tenth))
    {
        return 0;
    }

    if(!line_app_tune_value_in_range(LINE_TUNE_SLOT_NEAR_ROW, page->near_row_offset))
    {
        return 0;
    }

    if(!line_app_tune_value_in_range(LINE_TUNE_SLOT_FAR_ROW, page->far_row_offset))
    {
        return 0;
    }

    if(!line_app_tune_value_in_range(LINE_TUNE_SLOT_NEAR_WEIGHT, page->near_weight))
    {
        return 0;
    }

    if(!line_app_tune_value_in_range(LINE_TUNE_SLOT_FAR_WEIGHT, page->far_weight))
    {
        return 0;
    }

    if(!line_app_tune_value_in_range(LINE_TUNE_SLOT_SERVO_MIN, page->servo_min_angle))
    {
        return 0;
    }

    if(!line_app_tune_value_in_range(LINE_TUNE_SLOT_SERVO_MAX, page->servo_max_angle))
    {
        return 0;
    }

    if(page->near_row_offset >= page->far_row_offset)
    {
        return 0;
    }

    if(page->servo_min_angle >= page->servo_max_angle)
    {
        return 0;
    }

    return 1;
}

/* 把巡线调参页真正下发到运行时控制：PID、预瞄点、权重和舵机限幅都在这里生效。 */
static uint8 line_app_apply_tune_page(const flash_line_tune_page_t *page)
{
    uint8 current_angle = 0;

    if(0 == page)
    {
        return 0;
    }

    line_track_ctrl.kp = (float)page->kp_tenth / 10.0f;
    line_track_ctrl.ki = LINE_TRACK_DEFAULT_KI;
    line_track_ctrl.kd = (float)page->kd_tenth / 10.0f;
    line_track_ctrl.near_row_offset = page->near_row_offset;
    line_track_ctrl.far_row_offset = page->far_row_offset;
    line_track_ctrl.near_weight = page->near_weight;
    line_track_ctrl.far_weight = page->far_weight;
    car_servo_set_limit(page->servo_min_angle, page->servo_max_angle);
    current_angle = line_track_ctrl.servo_angle;
    if(0 == current_angle)
    {
        current_angle = CAR_SERVO_CENTER_ANGLE;
    }

    line_track_ctrl.servo_angle = line_app_limit_angle(current_angle);
    car_servo_set_angle(line_track_ctrl.servo_angle);
    return 1;
}

/* 上电先从 flash 恢复巡线调参；读坏了就重建默认页并写回。 */
static void line_app_apply_tune_page_from_flash(void)
{
    flash_line_tune_page_t page;

    flash_store_get_line_tune_page(&page);
    if(!line_app_tune_page_is_valid(&page))
    {
        line_app_fill_default_tune_page(&page);
        flash_store_set_line_tune_page(&page);
    }

    line_app_apply_tune_page(&page);
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

static void line_app_calc_preview_center(void)
{
    int16 image_center = (int16)(Search_Image_W / 2);
    int16 near_row = 0;
    int16 far_row = 0;
    int16 preview_error = 0;
    int16 weight_sum = 0;

    /* 近点看车头附近，先保证当前这一步不冲出边界。 */
    near_row = SEARCH_VALID_BOTTOM_ROW - line_track_ctrl.near_row_offset;
    /* 远点给一点提前量，让舵机不要总是等压到线边才开始转。 */
    far_row = SEARCH_VALID_BOTTOM_ROW - line_track_ctrl.far_row_offset;

    line_track_ctrl.near_center = Center_Line[line_app_limit_row(near_row)];
    line_track_ctrl.far_center = Center_Line[line_app_limit_row(far_row)];
    line_track_ctrl.near_error = (int16)line_track_ctrl.near_center - image_center;
    line_track_ctrl.far_error = (int16)line_track_ctrl.far_center - image_center;

    weight_sum = line_track_ctrl.near_weight + line_track_ctrl.far_weight;
    if(weight_sum <= 0)
    {
        line_track_ctrl.track_center = (uint8)image_center;
        line_track_ctrl.error = 0;
        return;
    }

    preview_error = (int16)((line_track_ctrl.near_error * line_track_ctrl.near_weight +
                             line_track_ctrl.far_error * line_track_ctrl.far_weight) / weight_sum);
    line_track_ctrl.track_center = (uint8)(image_center + preview_error);
    line_track_ctrl.error = preview_error;
}

static void line_app_update_control(void)
{
    float servo_delta = 0.0f;

    line_app_calc_preview_center();

    if(line_track_ctrl.error > -LINE_TRACK_ERROR_DEADBAND && line_track_ctrl.error < LINE_TRACK_ERROR_DEADBAND)
    {
        line_track_ctrl.error = 0;
    }

    if(!line_track_ctrl.control_ready)
    {
        line_track_ctrl.last_error = line_track_ctrl.error;
        line_track_ctrl.integral = 0.0f;
        line_track_ctrl.control_ready = 1;
    }

    line_track_ctrl.integral += (float)line_track_ctrl.error;
    if(line_track_ctrl.integral > LINE_TRACK_INTEGRAL_LIMIT)
    {
        line_track_ctrl.integral = LINE_TRACK_INTEGRAL_LIMIT;
    }
    else if(line_track_ctrl.integral < -LINE_TRACK_INTEGRAL_LIMIT)
    {
        line_track_ctrl.integral = -LINE_TRACK_INTEGRAL_LIMIT;
    }

    servo_delta = LINE_TRACK_STEER_SIGN *
                  (line_track_ctrl.kp * (float)line_track_ctrl.error +
                   line_track_ctrl.ki * line_track_ctrl.integral +
                   line_track_ctrl.kd * (float)(line_track_ctrl.error - line_track_ctrl.last_error));

    line_track_ctrl.servo_angle = line_app_limit_angle((float)CAR_SERVO_CENTER_ANGLE + servo_delta);
    car_servo_set_angle(line_track_ctrl.servo_angle);
    line_track_ctrl.last_error = line_track_ctrl.error;
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

    near_row = line_app_limit_row((int16)SEARCH_VALID_BOTTOM_ROW - line_track_ctrl.near_row_offset);
    far_row = line_app_limit_row((int16)SEARCH_VALID_BOTTOM_ROW - line_track_ctrl.far_row_offset);

    line_app_draw_preview_row(near_row, RGB565_GREEN);
    line_app_draw_preview_row(far_row, RGB565_MAGENTA);
    line_app_draw_preview_cross(line_track_ctrl.near_center, near_row, RGB565_GREEN);
    line_app_draw_preview_cross(line_track_ctrl.far_center, far_row, RGB565_MAGENTA);
}

static void line_app_draw_result(void)
{
    if(!switch_ui_enabled())
    {
        return;
    }

    ips200_show_gray_image(0, 0, mt9v03x_image[0], CAMERA_RAW_W, CAMERA_RAW_H, CAMERA_VALID_W, CAMERA_RAW_H, 0);
    SearchLine_DrawOverlay();
    line_app_draw_tuning_overlay();
}
#endif

static uint8 line_app_handle_frame(uint8 update_control)
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
    if(update_control)
    {
        line_app_update_control();
    }

#if IPS_ENABLE
    line_app_draw_result();
#endif

    mt9v03x_finish_flag = 0;
    return 1;
}

void line_app_init(void)
{
    line_track_ctrl.kp = LINE_TRACK_DEFAULT_KP;
    line_track_ctrl.ki = LINE_TRACK_DEFAULT_KI;
    line_track_ctrl.kd = LINE_TRACK_DEFAULT_KD;
    line_track_ctrl.integral = 0.0f;
    line_track_ctrl.near_error = 0;
    line_track_ctrl.far_error = 0;
    line_track_ctrl.track_center = (uint8)(Search_Image_W / 2);
    line_track_ctrl.near_center = (uint8)(Search_Image_W / 2);
    line_track_ctrl.far_center = (uint8)(Search_Image_W / 2);
    line_track_ctrl.servo_angle = CAR_SERVO_CENTER_ANGLE;
    line_track_ctrl.near_row_offset = FLASH_LINE_NEAR_ROW_DEFAULT;
    line_track_ctrl.far_row_offset = FLASH_LINE_FAR_ROW_DEFAULT;
    line_track_ctrl.near_weight = FLASH_LINE_NEAR_WEIGHT_DEFAULT;
    line_track_ctrl.far_weight = FLASH_LINE_FAR_WEIGHT_DEFAULT;
    line_track_ctrl.error = 0;
    line_track_ctrl.last_error = 0;
    line_track_ctrl.control_ready = 0;
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
    line_app_apply_tune_page_from_flash();

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
    return line_app_handle_frame(1);
}

uint8 line_app_preview_frame(void)
{
    return line_app_handle_frame(0);
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
        case FLASH_CAMERA_SLOT_AUTO_EXP:
            page.auto_exp = (uint8)value;
            apply_ok = line_app_apply_camera_page(&page);
            break;
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

    /* UI 读取时直接以 flash 缓存为准，保证显示值和掉电值一致。 */
    flash_store_get_line_tune_page(&page);
    if(!line_app_tune_page_is_valid(&page))
    {
        line_app_fill_default_tune_page(&page);
    }

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

/* 巡线调参统一走“读整页 -> 改一项 -> 归一化 -> 运行时生效 -> flash 落盘”这一条链。 */
uint8 line_app_set_tune_value(line_tune_slot_t slot, uint16 value)
{
    flash_line_tune_page_t page;

    if(!line_camera_ready)
    {
        return 0;
    }

    flash_store_get_line_tune_page(&page);
    if(!line_app_tune_page_is_valid(&page))
    {
        line_app_fill_default_tune_page(&page);
    }

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
    if(!line_app_tune_page_is_valid(&page))
    {
        return 0;
    }

    if(!line_app_apply_tune_page(&page))
    {
        return 0;
    }

    return flash_store_set_line_tune_page(&page);
}

void line_app_set_pd(float kp, float kd)
{
    line_track_ctrl.kp = kp;
    line_track_ctrl.ki = 0.0f;
    line_track_ctrl.kd = kd;
    line_track_ctrl.integral = 0.0f;
}

void line_app_set_pid(float kp, float ki, float kd)
{
    line_track_ctrl.kp = kp;
    line_track_ctrl.ki = ki;
    line_track_ctrl.kd = kd;
    line_track_ctrl.integral = 0.0f;
}

int16 line_app_get_error(void)
{
    return line_track_ctrl.error;
}

uint8 line_app_get_servo_angle(void)
{
    return line_track_ctrl.servo_angle;
}

uint8 line_app_get_track_center(void)
{
    return line_track_ctrl.track_center;
}
