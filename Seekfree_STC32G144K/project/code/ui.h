#ifndef _UI_H_
#define _UI_H_

#include "zf_common_headfile.h"

#define UI_KEY1_PIN                  (IO_PB2)               /* 返回键 */
#define UI_KEY2_PIN                  (IO_PB3)               /* 上键 */
#define UI_KEY3_PIN                  (IO_PB4)               /* 下键 */
#define UI_KEY4_PIN                  (IO_P32)               /* 确认键 */

#define UI_KEY_DEBOUNCE_COUNT        (2)                    /* 按键消抖 */

void ui_init(void);
void ui_update(void);

#endif
