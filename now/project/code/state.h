#ifndef __STATE_H__
#define __STATE_H__

#include "headfile.h"

typedef enum
{
    CAR_MODE_RUN = 0,
    CAR_MODE_UI,
    CAR_MODE_STOP
} car_mode;

typedef enum
{
    BLIND_BOX_OFF = 0,
    BLIND_BOX_DELAY,
    BLIND_BOX_SPEED1,
    BLIND_BOX_SPEED2,
    BLIND_BOX_STOP
} blind_box_phase;

extern volatile car_mode CarMode;
extern volatile blind_box_phase BlindBoxPhase;

void state_init(void);
void state_start_blind_box_delay(void);
void state_tick(void);

#endif
