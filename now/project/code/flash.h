#ifndef __FLASH_H__
#define __FLASH_H__

#include "headfile.h"

void flash_init(void);
uint8 flash_load_para(void);
void flash_save_para(void);

#endif
