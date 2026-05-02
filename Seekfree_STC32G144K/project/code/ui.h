#ifndef _UI_H_
#define _UI_H_

#include "zf_common_headfile.h"

#define UI_KEY1_PIN                  (IO_PB2)               /* 返回键 */
#define UI_KEY2_PIN                  (IO_PB3)               /* 上键 */
#define UI_KEY3_PIN                  (IO_PB4)               /* 下键 */
#define UI_KEY4_PIN                  (IO_P32)               /* 确认键 */

#define UI_KEY_PIT                   (TIM5_PIT)             /* 按键定时器 */
#define UI_SCREEN_PIT                (TIM6_PIT)             /* 屏幕定时器 */
#define UI_KEY_PERIOD_MS             (10)                   /* 按键周期 */
#define UI_SCREEN_PERIOD_MS          (50)                   /* 屏幕周期 */
#define UI_KEY_PIT_PRIORITY          (0)                    /* 按键优先级 */
#define UI_SCREEN_PIT_PRIORITY       (0)                    /* 屏幕优先级 */

#define UI_KEY_DEBOUNCE_COUNT        (2)                    /* 按键消抖 */

void ui_init(void);
void ui_update(void);

#endif
