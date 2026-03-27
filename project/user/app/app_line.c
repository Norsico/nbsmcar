#include "app_line.h"

#include "SearchLine.h"
#include "dev_servo.h"
#include "system_state.h"
#include "pid_control.h"

#define LINE_TRACK_SAMPLE_ROWS      (6)
#define LINE_TRACK_ERROR_DEADBAND   (1)
#define LINE_TRACK_DEFAULT_KP       (0.20f)
#define LINE_TRACK_DEFAULT_KI       (0.00f)
#define LINE_TRACK_DEFAULT_KD       (0.45f)
#define LINE_TRACK_INTEGRAL_LIMIT   (200.0f)
#define LINE_TRACK_STEER_SIGN       (-1.0f)

typedef struct
{
    pid_control_t pid;
    uint8 track_center;
    uint8 servo_angle;
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

static uint8 line_app_calc_track_center(void)
{
    int16 row = 0;
    int16 end_row = 0;
    int16 weight = LINE_TRACK_SAMPLE_ROWS;
    int32 weighted_sum = 0;
    int32 weight_sum = 0;

    end_row = SEARCH_VALID_BOTTOM_ROW - LINE_TRACK_SAMPLE_ROWS + 1;
    if(end_row < SEARCH_VALID_TOP_ROW)
    {
        end_row = SEARCH_VALID_TOP_ROW;
    }

    for(row = SEARCH_VALID_BOTTOM_ROW; row >= end_row; row--)
    {
        weighted_sum += (int32)Center_Line[row] * weight;
        weight_sum += weight;
        weight--;
    }

    if(weight_sum <= 0)
    {
        return (uint8)(Search_Image_W / 2);
    }

    return (uint8)(weighted_sum / weight_sum);
}

static void line_app_update_control(void)
{
    float servo_delta = 0.0f;
    int16 image_center = (int16)(Search_Image_W / 2);

    line_track_ctrl.track_center = line_app_calc_track_center();
    line_track_ctrl.pid.error = (float)((int16)line_track_ctrl.track_center - image_center);

    if(line_track_ctrl.pid.error > -LINE_TRACK_ERROR_DEADBAND && line_track_ctrl.pid.error < LINE_TRACK_ERROR_DEADBAND)
    {
        line_track_ctrl.pid.error = 0.0f;
    }

    line_track_ctrl.pid.integral += line_track_ctrl.pid.error;
    if(line_track_ctrl.pid.integral > LINE_TRACK_INTEGRAL_LIMIT)
    {
        line_track_ctrl.pid.integral = LINE_TRACK_INTEGRAL_LIMIT;
    }
    else if(line_track_ctrl.pid.integral < -LINE_TRACK_INTEGRAL_LIMIT)
    {
        line_track_ctrl.pid.integral = -LINE_TRACK_INTEGRAL_LIMIT;
    }

    servo_delta = LINE_TRACK_STEER_SIGN *
                  (line_track_ctrl.pid.param.kp * line_track_ctrl.pid.error +
                   line_track_ctrl.pid.param.ki * line_track_ctrl.pid.integral +
                   line_track_ctrl.pid.param.kd * (line_track_ctrl.pid.error - line_track_ctrl.pid.prev_error));

    line_track_ctrl.servo_angle = line_app_limit_angle((float)CAR_SERVO_CENTER_ANGLE + servo_delta);
    car_servo_set_angle(line_track_ctrl.servo_angle);
    line_track_ctrl.pid.prev_error = line_track_ctrl.pid.error;
}

#if IPS_ENABLE
static void line_app_draw_control_info(void)
{
    ips200_show_string(0, 160, "ERR:");
    ips200_show_string(64, 160, "ANG:");
    ips200_show_string(128, 160, "CTR:");
    ips200_show_int16(32, 160, (int16)line_track_ctrl.pid.error);
    ips200_show_uint8(96, 160, line_track_ctrl.servo_angle);
    ips200_show_uint8(160, 160, line_track_ctrl.track_center);
}

static void line_app_draw_result(void)
{
    ips200_show_gray_image(0, 0, mt9v03x_image[0], CAMERA_RAW_W, CAMERA_RAW_H, CAMERA_VALID_W, CAMERA_RAW_H, 0);
    SearchLine_DrawOverlay();
    line_app_draw_control_info();
}
#endif

void line_app_init(void)
{
    pid_param_init(&line_track_ctrl.pid, LINE_TRACK_DEFAULT_KP, LINE_TRACK_DEFAULT_KI, LINE_TRACK_DEFAULT_KD, 0.0f, 0.0f);
    pid_init(&line_track_ctrl.pid);
    line_track_ctrl.track_center = (uint8)(Search_Image_W / 2);
    line_track_ctrl.servo_angle = CAR_SERVO_CENTER_ANGLE;
    car_servo_set_center();

#if IPS_ENABLE
    ips200_clear(RGB565_WHITE);
    ips200_show_string(0, 0, "mt9v03x init.");
#endif

    while(mt9v03x_init())
    {
#if IPS_ENABLE
        ips200_show_string(0, 16, "mt9v03x reinit.");
#endif
        system_delay_ms(100);
    }

    line_camera_ready = 1;

#if IPS_ENABLE
    ips200_show_string(0, 16, "init success.");
#endif
}

uint8 line_app_process_frame(void)
{
    if(!line_camera_ready || !mt9v03x_finish_flag)
    {
        return 0;
    }

    SearchLine_Process();
    line_app_update_control();

#if IPS_ENABLE
    if(g_debug_enable && g_ips_enable)
    {
        line_app_draw_result();
    }
#endif

    mt9v03x_finish_flag = 0;
    return 1;
}

void line_app_set_pd(float kp, float kd)
{
    line_track_ctrl.pid.param.kp = kp;
    line_track_ctrl.pid.param.ki = 0.0f;
    line_track_ctrl.pid.param.kd = kd;
    line_track_ctrl.pid.integral = 0.0f;
}

void line_app_set_pid(float kp, float ki, float kd)
{
    line_track_ctrl.pid.param.kp = kp;
    line_track_ctrl.pid.param.ki = ki;
    line_track_ctrl.pid.param.kd = kd;
    line_track_ctrl.pid.integral = 0.0f;
}

int16 line_app_get_error(void)
{
    return (int16)line_track_ctrl.pid.error;
}

uint8 line_app_get_servo_angle(void)
{
    return line_track_ctrl.servo_angle;
}

uint8 line_app_get_track_center(void)
{
    return line_track_ctrl.track_center;
}
