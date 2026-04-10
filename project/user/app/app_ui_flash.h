/*
 * app_ui_flash.h - UI 参数页数据入口
 *
 * 负责 UI 页面读取参数范围、同步 flash 和触发运行态生效
 */

#ifndef _APP_UI_FLASH_H_
#define _APP_UI_FLASH_H_

#include "dev_flash.h"

/* 上电恢复 UI 相关参数，并同步 Start 页运行态。 */
void ui_flash_init(void);
/* Camera 页参数读写。 */
void ui_flash_get_camera_range(flash_camera_slot_t slot, uint16 *min_value, uint16 *max_value, uint16 *step_value);
uint16 ui_flash_get_camera_value(flash_camera_slot_t slot);
uint8 ui_flash_adjust_camera_value(flash_camera_slot_t slot, int16 delta);
/* Steer PD 页参数读写。 */
void ui_flash_get_steer_pd_range(flash_param_slot_t slot, uint16 *min_value, uint16 *max_value, uint16 *step_value);
uint16 ui_flash_get_steer_pd_value_tenth(flash_param_slot_t slot);
uint8 ui_flash_adjust_steer_pd_value_tenth(flash_param_slot_t slot, int16 delta);
/* 舵机限幅参数读写。 */
void ui_flash_get_servo_limit_range(uint16 *min_value, uint16 *max_value, uint16 *step_value);
uint16 ui_flash_get_servo_limit_min_value(void);
uint16 ui_flash_get_servo_limit_max_value(void);
uint8 ui_flash_adjust_servo_limit_min_value(int16 delta);
uint8 ui_flash_adjust_servo_limit_max_value(int16 delta);
/* Start 页参数读写。 */
void ui_flash_get_start_page(flash_start_page_t *page);
void ui_flash_get_start_speed_range(uint16 *min_value, uint16 *max_value, uint16 *step_value);
uint8 ui_flash_adjust_start_speed(int16 delta);
uint8 ui_flash_toggle_start_enable(void);
uint8 ui_flash_start_is_enabled(void);

#endif
