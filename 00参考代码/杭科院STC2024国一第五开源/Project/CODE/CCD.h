#ifndef CCD_H
#define CCD_H

#define compare 0
#define binary 1

#define KEEP_RIGHT_LINE 1
#define KEEP_LEFT_LINE 2

#define KEEP_INR_LINE 3
#define KEEP_INL_LINE 4

#define KEEP_OUTR_LINE 5
#define KEEP_OUTL_LINE 6

#define KEEP_CLOSE_LINE 7
#define KEEP_CROSS_LINE 8

#define KEEP_NORMAL_LINE 0

typedef struct
{
    uint8 cross;
    uint8 lcircle;
    uint8 rcircle;
    uint8 slope;
    uint8 obstacle;
    uint8 zebra;
    uint8 start;
    uint8 all;
}ele;
void elements(void);
void search_midline(uint8 input);
void CCD_Process(uint8 input);
void CCD1_Process(void);
void img_get();
void img1_get();
void ccd_mode(uint8 mode, uint8 line_mode);
extern void Em_Stop(void);
extern uint8 image[128];
extern uint8 image1[128];
extern uint8 image2[128];
extern uint8 fuzzy_image[64];
#endif