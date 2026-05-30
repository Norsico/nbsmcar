#include "zf_common_headfile.h"
#include "state.h"

typedef struct
{
    uint8 ui_selected;                                        /* UI拨码结果 */
    uint8 wifi_selected;                                      /* WiFi拨码结果 */
    uint8 switch_latched;                                     /* 拨码已锁存 */
    state_mode_t current_mode;                                /* 当前状态 */
} state_data_t;

static state_data_t state_data = {0, 0, 0, STATE_RUN};

/* 拨码初始化 */
static void state_switch_init(void)
{
    gpio_init(STATE_SWITCH_UI_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(STATE_SWITCH_WIFI_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
}

/* 锁存拨码 */
static void state_switch_latch(void)
{
    uint8 ui_pin;
    uint8 wifi_pin;

    /* 拨码往下拨时为高电平。 */
    ui_pin = gpio_get_level(STATE_SWITCH_UI_PIN) ? 1 : 0;
    wifi_pin = gpio_get_level(STATE_SWITCH_WIFI_PIN) ? 1 : 0;

    /* PB0打开时走UI。 */
    state_data.ui_selected = ui_pin;

    /* PB1只在UI关闭时才允许进WiFi。 */
    state_data.wifi_selected = (uint8)(wifi_pin && !state_data.ui_selected);

    state_data.switch_latched = 1;
}

/* 根据拨码决定模式 */
static void state_resolve_mode(void)
{
    if(state_data.ui_selected)
    {
        state_data.current_mode = STATE_UI;
    }
    else if(state_data.wifi_selected)
    {
        state_data.current_mode = STATE_WIFI;
    }
    else
    {
        state_data.current_mode = STATE_RUN;
    }
}

/* 状态初始化 */
void state_init(void)
{
    if(state_data.switch_latched)
    {
        return;
    }

    state_switch_init();
    state_switch_latch();
    state_resolve_mode();
}

/* 切状态 */
void state_set_mode(state_mode_t mode)
{
    state_data.current_mode = mode;
}

/* 读状态 */
state_mode_t state_get_mode(void)
{
    return state_data.current_mode;
}
