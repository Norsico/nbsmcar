/*
 * key.c
 *
 *  Created on: 2024年3月13日
 *      Author: xiaoming
 */

#include "zf_common_headfile.h"
#include "C_H.h"
void key_Start_run(void)
{
    if(gpio_get_level(P20_7) == 0 && run_flag == 0)
    {
        SystemData.Stop = 0;
        run_flag = 1;
        system_delay_ms(100);
    }
}


