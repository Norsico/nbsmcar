#ifndef _STATE_H_
#define _STATE_H_

#include "zf_common_typedef.h"
#include "zf_driver_gpio.h"

#define STATE_SWITCH_UI_PIN          (IO_PB0)                /* UI拨码 */
#define STATE_SWITCH_WIFI_PIN        (IO_PB1)                /* WiFi拨码 */

typedef enum
{
    STATE_UI = 0,                                             /* UI状态 */
    STATE_WIFI,                                               /* WiFi状态 */
    STATE_RUN,                                                /* Run状态 */
    STATE_STOP                                                /* Stop状态 */
} state_mode_t;

typedef struct
{
    uint8 ui_selected;                                        /* UI拨码结果 */
    uint8 wifi_selected;                                      /* WiFi拨码结果 */
    uint8 switch_latched;                                     /* 拨码已锁存 */
    state_mode_t current_mode;                                /* 当前状态 */
} state_data_t;

extern state_data_t state_data;

void state_init(void);
void state_set_mode(state_mode_t mode);
state_mode_t state_get_mode(void);

#endif
