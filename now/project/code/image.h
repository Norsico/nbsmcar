#ifndef __IMAGE_H__
#define __IMAGE_H__

#include "headfile.h"

#define IMAGE_W                    (80)
#define IMAGE_H                    (60)
#define IMAGE_MID                  (39)
#define IMAGE_BLACK                (0)
#define IMAGE_WHITE                (1)

typedef struct
{
    uint8 ready;              /* camera init ok */
    uint8 result_ready;       /* this frame can be used by servo */
    uint16 sequence;          /* processed frame counter */
    uint8 threshold;          /* final binary threshold */
    uint16 white_count;       /* white pixel count in ImageBin */
    uint8 tow_row;            /* actual row used as tow point */
    int16 center;             /* weighted center column */
    int16 error;              /* center - IMAGE_MID */
    uint8 valid_count;        /* valid rows from OFFLine to bottom */
    uint8 lost;               /* no reliable track */
    uint8 ring;               /* 0 none, 1 left ring, 2 right ring */
    uint8 ring_step;          /* ring process stage */
    uint8 zebra;              /* zebra line hit in current frame */
    uint8 zebra_count;        /* confirmed zebra hit count */
} image_data;

extern image_data Image;
extern uint8 ImageGray[IMAGE_H][IMAGE_W];
extern uint8 ImageBin[IMAGE_H][IMAGE_W];

void image_init(void);
void image_apply_camera(void);
void image_update(void);
void image_show_debug_overlay(uint16 x, uint16 y, uint16 w, uint16 h);

#endif
