/*
 * speed_prossess.h
 *
 *  Created on: 2025年4月11日
 *      Author: 15958
 */

#ifndef CODE_SPEED_PROSSESS_H_
#define CODE_SPEED_PROSSESS_H_


#include "zf_common_headfile.h"
#include "Platform_Types.h"                  //不加这个头文件没法使用外部声明extern 会导致识别不到
void GetGoalPulse(void);
extern int speed_goal;
#endif /* CODE_SPEED_PROSSESS_H_ */
