#include "headfile.h"

// ==========================================
// 1. 枚举与常量定义
// ==========================================
typedef enum
{
    UI_PAGE_MAIN = 0,
    UI_PAGE_SERVO,
    UI_PAGE_MOTOR,
    UI_PAGE_CAMERA,
    UI_PAGE_DEBUG
} ui_page;

typedef enum
{
    UI_KEY_NONE = 0,
    UI_KEY_BACK_EVENT,
    UI_KEY_UP_EVENT,
    UI_KEY_DOWN_EVENT,
    UI_KEY_ENTER_EVENT
} ui_key_event;

/* IPS200_PORTAIT: x 0-239, y 0-319 */
#define UI_ROW_H                     (16)  //控制每行 y 间距
#define UI_NAME_X                    (16)  //控制名字 x 位置
#define UI_VALUE_X                   (144) //控制数据 x 位置

// ==========================================
// 2. 数据驱动 UI 核心结构体定义 
// ==========================================
// 支持的变量类型
typedef enum {
    VAL_TYPE_INT16,
    VAL_TYPE_UINT16,
    VAL_TYPE_UINT8
} var_type_e;

// 单个参数菜单项描述
typedef struct {
    const char *name;     // UI 上显示的名字
    void *val_ptr;        // 指向真实变量的内存地址
    var_type_e type;      // 变量实际数据类型
    int16 step;           // 每次按键加/减的步长
} ui_param_t;

// 子页面描述
typedef struct {
    const char *title;        // 页面标题
    const ui_param_t *params; // 参数数组指针
    uint8 param_count;        // 参数个数
} ui_menu_t;

// ==========================================
// 3. 菜单数据配置表 (修改参数只改这里)
// ==========================================

// Servo 菜单配置
static const ui_param_t servo_params[] = {
    {"kp",       &SmartCar.servo.kp,            VAL_TYPE_INT16, 2},
    {"kd",       &SmartCar.servo.kd,            VAL_TYPE_INT16, 2},
    {"err2",     &SmartCar.servo.err2_k,        VAL_TYPE_INT16, 1},
    {"imu d",    &SmartCar.servo.imu_d,         VAL_TYPE_INT16, 1},
    {"ackerman", &SmartCar.servo.ackerman,      VAL_TYPE_INT16, 10},
    {"point",    &SmartCar.servo.tow_point,     VAL_TYPE_INT16, 1},
    {"in r pt",  &SmartCar.servo.in_ring_point,  VAL_TYPE_INT16, 1},
    {"out r pt", &SmartCar.servo.out_ring_point, VAL_TYPE_INT16, 1},
};

// Motor 菜单配置
static const ui_param_t motor_params[] = {
	  {"left kp",  &SmartCar.motor.left_kp,        VAL_TYPE_INT16, 1},
    {"left ki",  &SmartCar.motor.left_ki,        VAL_TYPE_INT16, 1},
    {"right kp", &SmartCar.motor.right_kp,       VAL_TYPE_INT16, 1},
    {"right ki", &SmartCar.motor.right_ki,       VAL_TYPE_INT16, 1},
    {"target",   &SmartCar.motor.target_speed,   VAL_TYPE_INT16, 10},
    {"straight", &SmartCar.motor.straight_speed, VAL_TYPE_INT16, 10},
    {"ring",     &SmartCar.motor.ring_speed,     VAL_TYPE_INT16, 10},
    {"fan",      &SmartCar.motor.fan_duty,       VAL_TYPE_INT16, 5},
    {"fan st",   &SmartCar.motor.fan_straight_duty, VAL_TYPE_INT16, 5},

};

// Camera 菜单配置
static const ui_param_t camera_params[] = {
    {"exposure",  &SmartCar.camera.exposure,         VAL_TYPE_INT16, 10},
    {"gain",      &SmartCar.camera.gain,             VAL_TYPE_UINT8,  1},
    {"thr off",   &SmartCar.camera.threshold_offset, VAL_TYPE_UINT8,  1},
    {"laser row", &SmartCar.camera.laser_row,        VAL_TYPE_UINT8,  1},
};

// 页面路由汇总表
static const ui_menu_t menu_pages[] = {
    {"Servo",  servo_params,  (uint8)(sizeof(servo_params) / sizeof(servo_params[0]))},
    {"Motor",  motor_params,  (uint8)(sizeof(motor_params) / sizeof(motor_params[0]))},
    {"Camera", camera_params, (uint8)(sizeof(camera_params) / sizeof(camera_params[0]))},
};

// 主页菜单名字配置
static const char* main_menu_names[] = {"Servo", "Motor", "Camera", "Debug"};
#define MAIN_MENU_COUNT 4

// ==========================================
// 4. 全局状态变量
// ==========================================
static ui_page UiPage = UI_PAGE_MAIN;
static uint8 UiSelect = 0;//参数选择状态
static uint8 UiEdit = 0;//编辑状态
static uint8 UiDebugGray = 0;//显示二值化还是灰度
static uint8 UiPowerPercent;//电池百分比
static uint8 KeyLast[4] = {1, 1, 1, 1};

uint8 ui_is_debug(void)
{
    return (UiPage == UI_PAGE_DEBUG) ? 1 : 0;
}

// ==========================================
// 5. 底层基础功能函数
// ==========================================
static void ui_read_power(void)
{
    uint16 adc_value;
    uint16 power_value;
    uint32 voltage_calc;

    adc_value = adc_mean_filter_convert(BATTERY_ADC, BATTERY_SAMPLE_COUNT);
    voltage_calc = (uint32)adc_value * 363U + 2047U;
    power_value = (uint16)(voltage_calc / 4095U);

    if(power_value <= BATTERY_EMPTY) {
        UiPowerPercent = 0;
    } else if(power_value >= BATTERY_FULL) {
        UiPowerPercent = 100;
    } else {
        UiPowerPercent = (uint8)(((uint16)(power_value - BATTERY_EMPTY) * 100U) / (BATTERY_FULL - BATTERY_EMPTY));
    }
}
static void ui_fill_rect(uint16 x, uint16 y, uint16 w, uint16 h, uint16 color)
{
    uint16 i;
    uint16 j;

    for(i = 0; i < w; i++) {
        for(j = 0; j < h; j++) {
            ips200_draw_point((uint16)(x + i), (uint16)(y + j), color);
        }
    }
}
static void ui_draw_battery(uint16 x, uint16 y, uint8 percent)
{
    uint8 fill_w;
    uint16 color;

    if(percent > 100) percent = 100;

    color = (percent < 10) ? RGB565_RED : RGB565_BLACK;
    fill_w = (uint8)((uint16)percent * 24 / 100);
    if(percent < 10) fill_w = 24;

    ui_fill_rect(x, y, 28, 12, RGB565_WHITE);
    ui_fill_rect((uint16)(x + 28), (uint16)(y + 3), 3, 6, color);

    ui_fill_rect(x, y, 28, 1, color);
    ui_fill_rect(x, (uint16)(y + 11), 28, 1, color);
    ui_fill_rect(x, y, 1, 12, color);
    ui_fill_rect((uint16)(x + 27), y, 1, 12, color);
    ui_fill_rect((uint16)(x + 2), (uint16)(y + 2), fill_w, 8, color);
}

static ui_key_event ui_key_scan(void)
{
    uint8 now[4];
    uint8 i;
    ui_key_event event;

    event = UI_KEY_NONE;
    now[0] = gpio_get_level(KEY_BACK) ? 1 : 0;
    now[1] = gpio_get_level(KEY_UP) ? 1 : 0;
    now[2] = gpio_get_level(KEY_DOWN) ? 1 : 0;
    now[3] = gpio_get_level(KEY_ENTER) ? 1 : 0;

    for(i = 0; i < 4; i++) {
        if((KeyLast[i] == 1) && (now[i] == 0)) {
            event = (ui_key_event)(i + 1);
        }
        KeyLast[i] = now[i];
    }
    return event;
}

// ==========================================
// 6. UI 数据操作抽象层
// ==========================================
static uint8 get_current_page_item_count(void)
{
    if(UiPage == UI_PAGE_MAIN) {
        return (uint8)MAIN_MENU_COUNT;
    }
    if((UiPage >= UI_PAGE_SERVO) && (UiPage <= UI_PAGE_CAMERA)) {
        return menu_pages[UiPage - UI_PAGE_SERVO].param_count;
    }
    return 0;
}

static void ui_change_current_value(int8 dir)
{
    const ui_param_t *p;
    int16 change;

    if((UiPage < UI_PAGE_SERVO) || (UiPage > UI_PAGE_CAMERA)) {
        return;
    }

    p = &menu_pages[UiPage - UI_PAGE_SERVO].params[UiSelect];
    change = (int16)(p->step * dir);

    // 根据数据类型，正确转换指针并加上偏移量
    switch(p->type) {
        case VAL_TYPE_INT16:  *(int16*)p->val_ptr  = (int16)(*(int16*)p->val_ptr + change); break;
        case VAL_TYPE_UINT16: *(uint16*)p->val_ptr = (uint16)((int16)*(uint16*)p->val_ptr + change); break;
        case VAL_TYPE_UINT8:  *(uint8*)p->val_ptr  = (uint8)((int16)*(uint8*)p->val_ptr + change); break;
    }

}

static void ui_show_current_value(const ui_param_t *p, uint16 y) 
{
    int32 val_to_show = 0;
    
    // 取出数值并统一用 int16 显示
    switch (p->type) {
        case VAL_TYPE_INT16:  val_to_show = *(int16*)p->val_ptr; break;
        case VAL_TYPE_UINT16: val_to_show = *(uint16*)p->val_ptr; break;
        case VAL_TYPE_UINT8:  val_to_show = *(uint8*)p->val_ptr; break;
    }
    ips200_show_int16(UI_VALUE_X, y, (int16)val_to_show);
}

static void ui_move(int8 dir)
{
    uint8 max_row = get_current_page_item_count() - 1;
    if(dir < 0) {
        UiSelect = (UiSelect == 0) ? max_row : (UiSelect - 1);
    } else {
        UiSelect = (UiSelect >= max_row) ? 0 : (UiSelect + 1);
    }
}

// ==========================================
// 7. 页面渲染逻辑
// ==========================================
//页面标题
static void ui_show_title(const char *title)
{
    ips200_set_color(RGB565_BLACK, RGB565_WHITE);
    ips200_show_string(0, 0, title);
    if(UiEdit) {
        ips200_show_string(176, 0, "EDIT");
    }
}
//画电池
static void ui_show_power(void)
{
    ui_draw_battery(160, 2, UiPowerPercent);
    if(UiPowerPercent < 10) {
        ips200_set_color(RGB565_RED, RGB565_WHITE);
        ips200_show_string(200, 0, "low");
    } else {
        ips200_set_color(RGB565_BLACK, RGB565_WHITE);
        ips200_show_uint8(200, 0, UiPowerPercent);
        ips200_show_string(224, 0, "%");
    }
}
// 主页面渲染函数
static void ui_show_main(void)
{
    uint8 i;
    uint16 y;

    ui_show_title("Main");
    ui_show_power();

    for(i = 0; i < MAIN_MENU_COUNT; i++) {
        y = (uint16)(48 + i * UI_ROW_H);
        ips200_set_color((i == UiSelect) ? RGB565_WHITE : RGB565_PINK,
                         (i == UiSelect) ? RGB565_PINK : RGB565_WHITE);
        ips200_show_string(0, y, (i == UiSelect) ? ">" : " ");
        ips200_show_string(UI_NAME_X, y, main_menu_names[i]);
    }
}

// 通用的参数页面渲染函数 
static void ui_show_generic_page(ui_page page)
{
    const ui_menu_t *menu;
    const ui_param_t *p;
    uint8 i;
    uint16 y;

    menu = &menu_pages[page - UI_PAGE_SERVO];
    ui_show_title(menu->title);

    for(i = 0; i < menu->param_count; i++) {
        y = (uint16)((i + 1) * UI_ROW_H);
        p = &menu->params[i];

        ips200_set_color((i == UiSelect) ? RGB565_WHITE : RGB565_PINK,
                         (i == UiSelect) ? RGB565_PINK : RGB565_WHITE);

        ips200_show_string(0, y, (i == UiSelect) ? (UiEdit ? "*" : ">") : " ");
        ips200_show_string(UI_NAME_X, y, p->name);
        ui_show_current_value(p, y);
    }
}

//摄像头界面固定内容
static void ui_show_debug(void)
{
    ips200_clear(RGB565_BLACK);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);

    if(UiDebugGray) {
        ips200_show_string(168, 0, "raw");
    } else {
        ips200_show_string(168, 0, "bin");
    }
    if(Image.ready == 0) {
        ips200_show_string(0, 0, "not ready");
    }
}

//图像内容
static void ui_show_camera_image(void)
{
    if(Image.ready == 0) return;

    if(UiDebugGray) {
        ips200_show_gray_image(0,0,ImageGray[0],80,60,160,120,0);
    } else {
        ips200_show_gray_image(0,0,ImageBin[0],80,60,160,120,1);
    }
    image_show_debug_overlay(0,0,160,120);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);

    /* 第1行：二值化阈值 */
    ips200_show_string(0, 124, "Threshold");
    ips200_show_string(88, 124, "   ");
    ips200_show_uint8(88, 124, Image.threshold);

    /* 第2行：左边线拟合误差 */
    ips200_show_string(0, 140, "L err x10");
    ips200_show_string(88, 140, "      ");
    ips200_show_int16(88, 140, Image.straight_left_error_x10);

    /* 第3行：右边线拟合误差 */
    ips200_show_string(0, 156, "R err x10");
    ips200_show_string(88, 156, "      ");
    ips200_show_int16(88, 156, Image.straight_right_error_x10);

    /* 第4行：坡道检测 */
    ips200_show_string(0, 172, "Ramp");
    ips200_show_string(88, 172, Image.is_ramp ? "YES" : "NO ");
    ips200_show_string(144, 172, "Cnt");
    ips200_show_string(184, 172, "  ");
    ips200_show_uint8(184, 172, Image.ramp_count);

    /* 第5行：宽度差（底部-顶部）*/
    ips200_show_string(0, 188, "WB");
    ips200_show_string(32, 188, "   ");
    ips200_show_uint8(32, 188, ImageDeal[55].Wide);
    ips200_show_string(72, 188, "WT");
    ips200_show_string(104, 188, "   ");
    ips200_show_uint8(104, 188, ImageDeal[10].Wide);
    ips200_show_string(144, 188, "Diff");
    ips200_show_string(184, 188, "   ");
    ips200_show_int16(184, 188, ImageDeal[55].Wide - ImageDeal[10].Wide);
}

static void ui_show(void)
{
    if(UiPage == UI_PAGE_DEBUG) {
        ui_show_debug();
        return;
    }

    ips200_clear(RGB565_WHITE);
    if (UiPage == UI_PAGE_MAIN) {
        ui_show_main();
    } 
    else if (UiPage >= UI_PAGE_SERVO && UiPage <= UI_PAGE_CAMERA) {
        ui_show_generic_page(UiPage);
    } 
}

// ==========================================
// 8. 事件处理逻辑
// ==========================================
static void ui_handle_key(ui_key_event event)
{
    if(event == UI_KEY_NONE) return;

    // --- 返回键处理 ---
    if(event == UI_KEY_BACK_EVENT) {
        if(UiEdit) {
            UiEdit = 0;
            flash_save_para();
            if(UiPage == UI_PAGE_CAMERA) image_apply_camera();
        } else if(UiPage != UI_PAGE_MAIN) {
            UiPage = UI_PAGE_MAIN;
            UiSelect = 0;
        }
        return;
    }

    // --- 调试页面特殊按键处理 ---
    if(UiPage == UI_PAGE_DEBUG) {
        if(event == UI_KEY_UP_EVENT || event == UI_KEY_DOWN_EVENT) {
            UiDebugGray = UiDebugGray ? 0 : 1;
        }
        return;
    }

    // --- 回车确认键处理 ---
    if(event == UI_KEY_ENTER_EVENT) {
        if(UiPage == UI_PAGE_MAIN) {
            // 通过主菜单的选择索引直接映射到子页面枚举
            UiPage = (ui_page)(UiSelect + 1); 
            UiSelect = 0;
        } else {
            if(UiEdit) {
                UiEdit = 0;
                flash_save_para();
                if(UiPage == UI_PAGE_CAMERA) image_apply_camera();
            } else {
                UiEdit = 1;
            }
        }
        return;
    }
    // --- 上下键处理 ---
    if((event == UI_KEY_UP_EVENT) || (event == UI_KEY_DOWN_EVENT)) {
        if(UiEdit) {
            // 如果处于编辑状态，直接调用通用的数值修改逻辑
            ui_change_current_value((event == UI_KEY_UP_EVENT) ? 1 : -1);
        } else {
            // 如果处于选择状态，移动光标 
            ui_move((event == UI_KEY_UP_EVENT) ? -1 : 1);
        }
    }
}

// ==========================================
// 9. 外部调用接口
// ==========================================
void ui_init(void)
{
    gpio_init(KEY_BACK, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(KEY_UP, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(KEY_DOWN, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(KEY_ENTER, GPI, GPIO_HIGH, GPI_PULL_UP);

    adc_init(BATTERY_ADC, ADC_12BIT);
    ui_read_power();

    ips200_set_dir(IPS200_PORTAIT);
    ips200_init();
    ips200_set_color(RGB565_BLACK, RGB565_WHITE);
    ips200_clear(RGB565_WHITE);

    UiPage = UI_PAGE_MAIN;
    UiSelect = 0;
    UiEdit = 0;
    ui_show();
}

void ui_update(void)
{
    ui_key_event event;

    event = ui_key_scan();
    if(event != UI_KEY_NONE) {
        ui_handle_key(event);
				ui_show();
    }
    if(UiPage == UI_PAGE_DEBUG) {
        ui_show_camera_image();
    }
    system_delay_ms(10);
}