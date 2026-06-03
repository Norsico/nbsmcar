#include "headfile.h"

#define FLASH_PARA_ADDR             (0x0000)
#define FLASH_PARA_MAGIC            (0x5343)
#define FLASH_PARA_VERSION          (1)

typedef struct
{
    uint16 magic;
    uint16 version;
    uint16 size;
    car_para para;
    uint16 checksum;
} flash_para_data;

static uint16 flash_checksum(uint8 *buff, uint16 len)
{
    uint16 sum;

    sum = 0;
    while(len--)
    {
        sum += *buff++;
    }

    return sum;
}

static uint8 flash_same(uint8 *buff1, uint8 *buff2, uint16 len)
{
    while(len--)
    {
        if(*buff1++ != *buff2++)
        {
            return 0;
        }
    }

    return 1;
}

static uint8 flash_para_valid(flash_para_data *store)
{
    if(store->magic != FLASH_PARA_MAGIC)
    {
        return 0;
    }
    if(store->version != FLASH_PARA_VERSION)
    {
        return 0;
    }
    if(store->size != sizeof(car_para))
    {
        return 0;
    }
    if(store->checksum != flash_checksum((uint8 *)&store->para, sizeof(car_para)))
    {
        return 0;
    }

    return 1;
}

void flash_init(void)
{
    iap_init();
    flash_load_para();
}

uint8 flash_load_para(void)
{
    flash_para_data store;

    iap_read_buff(FLASH_PARA_ADDR, (uint8 *)&store, sizeof(store));

    if(flash_para_valid(&store) == 0)
    {
        return 0;
    }

    SmartCar = store.para;
    if((SmartCar.motor.fan_duty < 0) || (SmartCar.motor.fan_duty > 100))
    {
        SmartCar.motor.fan_duty = 0;
    }
    return 1;
}

void flash_save_para(void)
{
    flash_para_data store;
    flash_para_data old_store;

    store.magic = FLASH_PARA_MAGIC;
    store.version = FLASH_PARA_VERSION;
    store.size = sizeof(car_para);
    store.para = SmartCar;
    store.checksum = flash_checksum((uint8 *)&store.para, sizeof(car_para));

    iap_read_buff(FLASH_PARA_ADDR, (uint8 *)&old_store, sizeof(old_store));
    if((flash_para_valid(&old_store) != 0) &&
       (flash_same((uint8 *)&old_store.para, (uint8 *)&store.para, sizeof(car_para)) != 0))
    {
        return;
    }

    iap_erase_page(FLASH_PARA_ADDR);
    iap_write_buff(FLASH_PARA_ADDR, (uint8 *)&store, sizeof(store));
}

void flash_get_camera_page(flash_camera_page_t *page)
{
    if(0 == page)
    {
        return;
    }

    page->exp_time = SmartCar.camera.exposure;
    page->gain = SmartCar.camera.gain;
    page->threshold_offset = SmartCar.camera.threshold_offset;
    page->laser_row = SmartCar.camera.laser_row;
}
