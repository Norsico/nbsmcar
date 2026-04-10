#ifndef _APP_LINE_H_
#define _APP_LINE_H_

#include "dev_flash.h"
#include "system_state.h"
#include "zf_common_typedef.h"

void line_app_init(void);
uint8 line_app_process_frame(void);
uint8 line_app_set_camera_param_value(flash_camera_slot_t slot, uint16 value);
uint8 line_app_set_steer_pd_value_tenth(flash_param_slot_t slot, int16 value_tenth);

#if IPS_ENABLE
void line_app_render_frame(void);
#endif

#endif
