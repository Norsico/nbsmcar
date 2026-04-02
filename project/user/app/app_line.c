#include "app_line.h"

#include "pid_control.h"
#include "SearchLine.h"
#include "dev_display.h"
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
#define LINE_TRACK_SERVO_MAX_OUT    (40.0f)
#define LINE_TRACK_SERVO_MIN_OUT    (-40.0f)

/************ 赛道控制结构体 ************/
typedef struct
{
    int16 near_error;
    int16 far_error;
    uint8 near_center;
    uint8 far_center;
    uint8 track_center;
    uint8 servo_angle;
    uint8 control_ready;
} line_track_info_t;

static uint8 line_camera_ready = 0;
static pid_control_t line_servo_pid;
static line_track_info_t line_track_info = {0};

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

    near_row = SEARCH_VALID_BOTTOM_ROW - LINE_TRACK_NEAR_ROW_OFFSET;
	
    far_row = SEARCH_VALID_BOTTOM_ROW - LINE_TRACK_FAR_ROW_OFFSET;

    line_track_info.near_center = Center_Line[line_app_limit_row(near_row)];
    line_track_info.far_center = Center_Line[line_app_limit_row(far_row)];
    line_track_info.near_error = (int16)line_track_info.near_center - image_center;
    line_track_info.far_error = (int16)line_track_info.far_center - image_center;

    weight_sum = LINE_TRACK_NEAR_WEIGHT + LINE_TRACK_FAR_WEIGHT;
    if(weight_sum <= 0)
    {
        line_track_info.track_center = (uint8)image_center;
        line_servo_pid.error = 0;
        return;
    }

    preview_error = (int16)((line_track_info.near_error * LINE_TRACK_NEAR_WEIGHT +
                             line_track_info.far_error * LINE_TRACK_FAR_WEIGHT) / weight_sum);
    line_track_info.track_center = (uint8)(image_center + preview_error);
    line_servo_pid.error = (float)preview_error;
}

static void line_app_update_control(void)
{
    float servo_delta = 0.0f;

    line_app_calc_preview_center();

    if(line_servo_pid.error > -LINE_TRACK_ERROR_DEADBAND && line_servo_pid.error < LINE_TRACK_ERROR_DEADBAND)
    {
        line_servo_pid.error = 0;
    }

    if(!line_track_info.control_ready)
    {
        line_servo_pid.prev_error = line_servo_pid.error;
        line_servo_pid.integral = 0.0f;
        line_track_info.control_ready = 1;
    }

    line_servo_pid.integral += line_servo_pid.error;
    if(line_servo_pid.integral > LINE_TRACK_INTEGRAL_LIMIT)
    {
        line_servo_pid.integral = LINE_TRACK_INTEGRAL_LIMIT;
    }
    else if(line_servo_pid.integral < -LINE_TRACK_INTEGRAL_LIMIT)
    {
        line_servo_pid.integral = -LINE_TRACK_INTEGRAL_LIMIT;
    }

    servo_delta = LINE_TRACK_STEER_SIGN *
                  (line_servo_pid.param.kp * line_servo_pid.error +
                   line_servo_pid.param.ki * line_servo_pid.integral +
                   line_servo_pid.param.kd * (line_servo_pid.error - line_servo_pid.prev_error));

    line_track_info.servo_angle = line_app_limit_angle((float)CAR_SERVO_CENTER_ANGLE + servo_delta);
    car_servo_set_angle(line_track_info.servo_angle);
    line_servo_pid.prev_error = line_servo_pid.error;
}

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

    mt9v03x_finish_flag = 0;
    return 1;
}

#if IPS_ENABLE
void line_app_render_frame(void)
{
    if(!g_ips_enable)
    {
        return;
    }

    ips200_show_gray_image(0, 0, mt9v03x_image[0], CAMERA_RAW_W, CAMERA_RAW_H, CAMERA_VALID_W, CAMERA_RAW_H, 0);
    SearchLine_DrawOverlay();
}
#endif

void line_app_ctrl_init(void)
{
    pid_param_init(&line_servo_pid, LINE_TRACK_DEFAULT_KP, LINE_TRACK_DEFAULT_KI, LINE_TRACK_DEFAULT_KD,
                   LINE_TRACK_SERVO_MAX_OUT, LINE_TRACK_SERVO_MIN_OUT);
    pid_init(&line_servo_pid);

    line_track_info.near_error = 0;
    line_track_info.far_error = 0;
    line_track_info.track_center = (uint8)(Search_Image_W / 2);
    line_track_info.near_center = (uint8)(Search_Image_W / 2);
    line_track_info.far_center = (uint8)(Search_Image_W / 2);
    line_track_info.servo_angle = CAR_SERVO_CENTER_ANGLE;
    line_track_info.control_ready = 0;
    car_servo_set_center();
}

uint8 line_app_camera_init(void)
{
    while(mt9v03x_init())
    {
        system_delay_ms(100);
    }

    line_camera_ready = 1;
    return 1;
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
    line_servo_pid.param.kp = kp;
    line_servo_pid.param.ki = 0.0f;
    line_servo_pid.param.kd = kd;
    line_servo_pid.integral = 0.0f;
}

void line_app_set_pid(float kp, float ki, float kd)
{
    line_servo_pid.param.kp = kp;
    line_servo_pid.param.ki = ki;
    line_servo_pid.param.kd = kd;
    line_servo_pid.integral = 0.0f;
}

int16 line_app_get_error(void)
{
    return (int16)line_servo_pid.error;
}

uint8 line_app_get_servo_angle(void)
{
    return line_track_info.servo_angle;
}

uint8 line_app_get_track_center(void)
{
    return line_track_info.track_center;
}
