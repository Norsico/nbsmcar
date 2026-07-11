#ifndef __BUZZER_H__
#define __BUZZER_H__

#include "headfile.h"

void buzzer_init(void);
void buzzer_short(void);
void buzzer_tick(void);
void buzzer_stop_alarm_enable(uint8 enable);

#endif
