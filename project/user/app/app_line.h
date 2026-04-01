#ifndef _APP_LINE_H_
#define _APP_LINE_H_

#include "zf_common_typedef.h"

void line_app_init(void);
uint8 line_app_process_frame(void);
uint8 line_app_preview_frame(void);
void line_app_set_pid(float kp, float ki, float kd);
void line_app_set_pd(float kp, float kd);
int16 line_app_get_error(void);
uint8 line_app_get_servo_angle(void);
uint8 line_app_get_track_center(void);

#endif