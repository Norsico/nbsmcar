#ifndef _APP_LINE_H_
#define _APP_LINE_H_

#include "dev_flash.h"
#include "zf_common_typedef.h"

typedef enum
{
    LINE_TUNE_SLOT_KP = 0,
    LINE_TUNE_SLOT_KD,
    LINE_TUNE_SLOT_SERVO_MIN,
    LINE_TUNE_SLOT_SERVO_MAX,
    LINE_TUNE_SLOT_COUNT
} line_tune_slot_t;

void line_app_init(void);
uint8 line_app_process_frame(void);
uint8 line_app_set_camera_param_value(flash_camera_slot_t slot, uint16 value);
void line_app_get_tune_range(line_tune_slot_t slot, uint16 *min_value, uint16 *max_value, uint16 *step_value);
uint16 line_app_get_tune_value(line_tune_slot_t slot);
uint8 line_app_set_tune_value(line_tune_slot_t slot, uint16 value);
uint8 line_app_save_tune_page(void);
#if defined(IPS_ENABLE) && IPS_ENABLE
void line_app_render_frame(void);
#endif

#endif
