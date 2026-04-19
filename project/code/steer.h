#ifndef _STEER_H_
#define _STEER_H_

#include "zf_common_headfile.h"

#define steer_middle 4880
#define steer_right  4100
#define steer_left   5520
#define LimitLeft(Left)    (Left = ((Left > steer_left) ? steer_left : Left))
#define LimitRight(Right)  (Right = ((Right < steer_right) ? steer_right : Right))

extern int S3010_Duty;

void Steer_init(void);
void SteerControl(int duty);
void SteerPID_Realize(float offset);
void Steer_Set_Pd(uint16 p, uint16 d, uint16 err2_tenth);

#endif
