#include "app_line.h"

#include "SearchLine.h"
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
    uint8 control_ready;
} line_track_ctrl_t;

static uint8 line_camera_ready = 0;
static line_track_ctrl_t line_track_ctrl = {0};

static uint8 line_app_limit_angle(float angle)
{
    if(angle < CAR_SERVO_MIN_ANGLE)
    {
        return CAR_SERVO_MIN_ANGLE;
    }

    if(angle > CAR_SERVO_MAX_ANGLE)
    {
        return CAR_SERVO_MAX_ANGLE;
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

    /* ���㿴��ͷ�������ȱ�֤��ǰ��һ��������߽硣 */
    near_row = SEARCH_VALID_BOTTOM_ROW - LINE_TRACK_NEAR_ROW_OFFSET;
    /* Զ���һ����ǰ�����ö����Ҫ���ǵ�ѹ���߲߱ſ�ʼת�� */
    far_row = SEARCH_VALID_BOTTOM_ROW - LINE_TRACK_FAR_ROW_OFFSET;

    line_track_ctrl.near_center = Center_Line[line_app_limit_row(near_row)];
    line_track_ctrl.far_center = Center_Line[line_app_limit_row(far_row)];
    line_track_ctrl.near_error = (int16)line_track_ctrl.near_center - image_center;
    line_track_ctrl.far_error = (int16)line_track_ctrl.far_center - image_center;

    weight_sum = LINE_TRACK_NEAR_WEIGHT + LINE_TRACK_FAR_WEIGHT;
    if(weight_sum <= 0)
    {
        line_track_ctrl.track_center = (uint8)image_center;
        line_track_ctrl.error = 0;
        return;
    }

    preview_error = (int16)((line_track_ctrl.near_error * LINE_TRACK_NEAR_WEIGHT +
                             line_track_ctrl.far_error * LINE_TRACK_FAR_WEIGHT) / weight_sum);
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
static void line_app_draw_result(void)
{
    if(!g_ips_enable)
    {
        return;
    }

    ips200_show_gray_image(0, 0, mt9v03x_image[0], CAMERA_RAW_W, CAMERA_RAW_H, CAMERA_VALID_W, CAMERA_RAW_H, 0);
    SearchLine_DrawOverlay();
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
    line_track_ctrl.error = 0;
    line_track_ctrl.last_error = 0;
    line_track_ctrl.control_ready = 0;
    car_servo_set_center();

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
    return line_app_handle_frame(1);
}

uint8 line_app_preview_frame(void)
{
    return line_app_handle_frame(0);
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