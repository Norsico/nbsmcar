/*
 * flash.h
 *
 *  Created on: 2024年5月18日
 *      Author: Amadeus
 */

#ifndef CODE_FLASH_H_
#define CODE_FLASH_H_


void flash_set_buffer_speed(void);   //存速度数据
void flash_get_buffer_speed(void);   //取速度数据
void flash_set_buffer_image(void);    //存图像数据
void flash_get_buffer_image(void);    //取图像数据
void flash_set_buffer_rings(void);    //存圆环数据
void flash_get_buffer_rings(void);    //存圆环数据
void flash_set_buffer_speed_0();   //上一次方案存
void flash_get_buffer_speed_0();   //上一次方案取
void flash_set_buffer_speed_1();   //方案1存
void flash_get_buffer_speed_1();   //方案1取
void flash_set_buffer_speed_2();   //方案2存
void flash_get_buffer_speed_2();   //方案2取
void flash_set_buffer_speed_3();   //方案3存
void flash_get_buffer_speed_3();   //方案3取
void flash_set_buffer_speed_4();   //方案4存
void flash_get_buffer_speed_4();   //方案4取
void flash_set_buffer_speed_5();   //方案5存
void flash_get_buffer_speed_5();   //方案5取
void s();
void x();
void gpio_read_set();   //拨码开关读取状态

int8 preset_read;


#endif /* CODE_FLASH_H_ */
