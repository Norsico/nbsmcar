#ifndef _SEARCH_LINE_H_
#define _SEARCH_LINE_H_

#include "zf_common_headfile.h"

#define CAMERA_RAW_W                MT9V03X_W
#define CAMERA_RAW_H                MT9V03X_H
#define CAMERA_INVALID_RIGHT_COLS   (1)
#define CAMERA_VALID_W              (CAMERA_RAW_W - CAMERA_INVALID_RIGHT_COLS)
#define CAMERA_LAST_VALID_COL       (CAMERA_VALID_W - 1)

void SearchLine_Process(void);
void SearchLine_DrawBinaryPreview(void);
uint8 SearchLine_GetOtsuThreshold(void);
uint8 SearchLine_GetRawOtsuThreshold(void);
uint8 SearchLine_GetSteerCommand(void);
uint8 SearchLine_GetStraightAcc(void);
uint8 SearchLine_GetDetTrue(void);
uint8 SearchLine_GetLeftLine(void);
uint8 SearchLine_GetRightLine(void);
void SearchLine_SetSteerPdTenth(uint16 p_tenth, uint16 d_tenth);

#endif
