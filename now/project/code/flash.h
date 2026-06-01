#ifndef __FLASH_H__
#define __FLASH_H__

#include "headfile.h"

typedef struct
{
    int16 exp_time;
    int16 gain;
    int16 threshold_offset;
    int16 laser_row;
} flash_camera_page_t;

void flash_init(void);
uint8 flash_load_para(void);
void flash_save_para(void);
void flash_get_camera_page(flash_camera_page_t *page);

#endif
