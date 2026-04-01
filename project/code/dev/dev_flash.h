#ifndef _DEV_FLASH_H_
#define _DEV_FLASH_H_

#include "zf_common_typedef.h"

/* 当前参数页先拿两项数值做最小验证，数值统一按 0.1 来存。 */
#define FLASH_PARAM_VALUE_MIN_TENTH     (0)
#define FLASH_PARAM_VALUE_MAX_TENTH     (999)
#define FLASH_PARAM_VALUE_STEP_TENTH    (1)

/* 当前参数页里这两行，后面真要扩参数就继续往后加。 */
typedef enum
{
    FLASH_PARAM_SLOT_FIRST = 0,
    FLASH_PARAM_SLOT_SECOND,
    FLASH_PARAM_SLOT_COUNT
} flash_param_slot_t;

/* 这是当前参数页在用的那一组参数。 */
typedef struct
{
    int16 first_value_tenth;
    int16 second_value_tenth;
} flash_param_page_t;

/* 掉电参数先按一个总结构来管，后面别的模块直接往这里扩。 */
typedef struct
{
    flash_param_page_t param_page;
    uint8 reserved[16];
} flash_store_data_t;

/* 上电初始化，负责把 EEPROM 里的内容读到 RAM。 */
void flash_store_init(void);
/* 读取整个掉电参数结构。 */
void flash_store_get_data(flash_store_data_t *store_ptr);
/* 整体覆盖掉电参数结构，适合后面多个模块一起改完再存。 */
uint8 flash_store_set_data(const flash_store_data_t *store_ptr);
/* 读取当前参数页这一组参数。 */
void flash_store_get_param_page(flash_param_page_t *page);
/* 读取参数页里某一项，返回值单位是 0.1。 */
int16 flash_store_get_param_value_tenth(flash_param_slot_t slot);
/* 修改参数页里某一项，值变了就立刻落盘。 */
uint8 flash_store_set_param_value_tenth(flash_param_slot_t slot, int16 value_tenth);
/* 恢复默认参数并写回。 */
void flash_store_reset_data(void);

#endif
