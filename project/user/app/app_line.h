#ifndef _APP_LINE_H_
#define _APP_LINE_H_

#include "dev_flash.h"
#include "system_state.h"
#include "zf_common_typedef.h"

typedef enum
{
    LINE_APP_PREVIEW_BINARY = 0,
    LINE_APP_PREVIEW_RAW
} line_app_preview_mode_t;

void line_app_init(void);
uint8 line_app_process_frame(void);
void line_app_process_steer(void);
uint8 line_app_set_camera_param_value(flash_camera_slot_t slot, uint16 value);
uint8 line_app_set_steer_pd_value(flash_param_slot_t slot, int16 value);
void line_app_set_preview_mode(line_app_preview_mode_t mode);
line_app_preview_mode_t line_app_get_preview_mode(void);

#if IPS_ENABLE
void line_app_render_frame(void);
#endif

#endif
