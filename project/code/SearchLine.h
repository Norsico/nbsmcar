#ifndef _SEARCH_LINE_H_
#define _SEARCH_LINE_H_

#include "zf_common_headfile.h"

#define CAMERA_RAW_W                MT9V03X_W
#define CAMERA_RAW_H                MT9V03X_H
#define CAMERA_INVALID_RIGHT_COLS   (1)
#define CAMERA_VALID_W              (CAMERA_RAW_W - CAMERA_INVALID_RIGHT_COLS)
#define CAMERA_LAST_VALID_COL       (CAMERA_VALID_W - 1)

#define Search_Image_W              CAMERA_VALID_W
#define Search_Image_H              CAMERA_RAW_H

#define BLACKPOINT                  (90)
#define WHITEMAXMUL                 (11)
#define WHITEMINMUL                 (7)

#define REFRENCEROW                 (5)
#define SEARCH_COL_STEP             (1)
#define SEARCH_ROW_STEP             (3)
#define SEARCHRANGE                 (20)
#define STOPROW                     (3)
#define CONTRASTOFFSET              (3)
#define SEARCH_IGNORE_TOP_PERCENT   (10)
#define SEARCH_IGNORE_BOTTOM_PERCENT (5)
#define SEARCH_IGNORE_TOP_ROWS      ((Search_Image_H * SEARCH_IGNORE_TOP_PERCENT) / 100)
#define SEARCH_IGNORE_BOTTOM_ROWS   ((Search_Image_H * SEARCH_IGNORE_BOTTOM_PERCENT) / 100)
#define SEARCH_VALID_TOP_ROW        (((SEARCH_IGNORE_TOP_ROWS) > (STOPROW)) ? (SEARCH_IGNORE_TOP_ROWS) : (STOPROW))
#define SEARCH_VALID_BOTTOM_ROW     (Search_Image_H - 1 - SEARCH_IGNORE_BOTTOM_ROWS)

extern uint8 Reference_Point;
extern uint8 Reference_Col;
extern uint8 White_Max_Point;
extern uint8 White_Min_Point;
extern uint8 Reference_Contrast_Ratio;

extern uint8 Remote_Distance[Search_Image_W];
extern uint8 Left_Edge_Line[Search_Image_H];
extern uint8 Right_Edge_Line[Search_Image_H];
extern uint8 Center_Line[Search_Image_H];

void Get_Reference_Point(void);
void Search_Reference_Col(void);
void Search_line(void);
void SearchLine_Update_Center(void);
void SearchLine_Process(void);
void SearchLine_DrawOverlay(void);
uint8 SearchLine_IsCenterFound(uint8 row);
uint8 SearchLine_GetCenterFoundRowCount(void);

#endif