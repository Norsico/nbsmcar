#ifndef __STATE_H__
#define __STATE_H__

#include "headfile.h"

typedef enum
{
    CAR_MODE_RUN = 0,
    CAR_MODE_UI,
    CAR_MODE_BRAKE_STOP,
    CAR_MODE_STOP
} car_mode;

extern car_mode CarMode;

void state_init(void);

#endif
