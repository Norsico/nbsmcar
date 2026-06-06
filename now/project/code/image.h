#ifndef __IMAGE_H__
#define __IMAGE_H__

#include "headfile.h"

#define LCDH                       (60)
#define LCDW                       (80)
#define IMAGE_H                    (LCDH)
#define IMAGE_W                    (LCDW)
#define ImageSensorMid             (40)
#define IMAGE_MID                  (ImageSensorMid)
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
    uint8 valid_count;        /* valid rows from visible top to bottom */
    uint8 lost;               /* no reliable track */
    uint8 ring;               /* UI compatibility: element logic disabled */
    uint8 ring_step;          /* UI compatibility: always normal */
    uint8 zebra;              /* UI compatibility: element logic disabled */
    uint8 zebra_count;        /* UI compatibility: element logic disabled */
} image_data;

extern image_data Image;
extern uint8 ImageGray[IMAGE_H][IMAGE_W];
extern uint8 ImageBin[IMAGE_H][IMAGE_W];
extern uint8 Pixle[IMAGE_H][IMAGE_W];

#define Image_Use                  ImageGray

void image_init(void);
void image_apply_camera(void);
void image_update(void);
void image_show_debug_overlay(uint16 x, uint16 y, uint16 w, uint16 h);

#endif
