/*
 * app_display.c - simple two-level menu demo for IPS200
 */

#include "app_display.h"

#include "dev_adc.h"
#include "dev_servo.h"

#define MENU_LINE_HEIGHT        (16)
#define MENU_ITEM_COUNT         (4)
#define MENU_TITLE_Y            (0)
#define MENU_LIST_Y             (16)
#define MENU_DETAIL_Y           (32)
#define MENU_BAT_X              (168)
#define MENU_BAT_Y              (2)
#define MENU_BAT_W              (36)
#define MENU_BAT_H              (10)
#define MENU_BAT_CAP_W          (3)
#define MENU_BAT_TEXT_X         (132)
#define BATTERY_EMPTY_DECI      (114)
#define BATTERY_FULL_DECI       (126)

typedef enum
{
    DISPLAY_PAGE_ROOT = 0,
    DISPLAY_PAGE_CAMERA,
    DISPLAY_PAGE_MENU2,
    DISPLAY_PAGE_MENU3,
    DISPLAY_PAGE_ABOUT
} display_page_t;

typedef struct
{
    const char *title;
    const char *line1;
    const char *line2;
    const char *line3;
    const char *line4;
} display_page_info_t;

static const display_page_info_t g_menu_pages[MENU_ITEM_COUNT] =
{
    {"Camera View", "Live image page", "Show left/right edge", "Show center line", "KEY2 back"},
    {"Menu Slot 2", "Reserved item", "Write your own page", "", ""},
    {"Menu Slot 3", "Reserved item", "Write your own page", "", ""},
    {"About UI",    "Slot 1 is camera", "Other pages reserved", "UI can expand later", "KEY2 back"}
};

static uint8 g_menu_selected = 0;
static display_page_t g_menu_page = DISPLAY_PAGE_ROOT;
static uint8 g_menu_dirty = 1;
static uint8 g_menu_last_battery_percent = 0xFF;

static void display_menu_show_percent(uint16 x, uint16 y, uint8 percent)
{
    char text[4];

    if(percent >= 100)
    {
        text[0] = '1';
        text[1] = '0';
        text[2] = '0';
        text[3] = '\0';
    }
    else if(percent >= 10)
    {
        text[0] = (char)('0' + percent / 10);
        text[1] = (char)('0' + percent % 10);
        text[2] = '\0';
    }
    else
    {
        text[0] = (char)('0' + percent);
        text[1] = '\0';
    }

    ips200_show_string(x, y, "   ");
    ips200_show_string(x, y, text);
}

static void display_menu_draw_rect(uint16 x, uint16 y, uint16 w, uint16 h, uint16 color)
{
    uint16 row = 0;
    uint16 col = 0;

    for(row = 0; row < h; row++)
    {
        for(col = 0; col < w; col++)
        {
            ips200_draw_point(x + col, y + row, color);
        }
    }
}

static void display_menu_draw_rect_frame(uint16 x, uint16 y, uint16 w, uint16 h, uint16 color)
{
    uint16 col = 0;
    uint16 row = 0;

    for(col = 0; col < w; col++)
    {
        ips200_draw_point(x + col, y, color);
        ips200_draw_point(x + col, y + h - 1, color);
    }

    for(row = 0; row < h; row++)
    {
        ips200_draw_point(x, y + row, color);
        ips200_draw_point(x + w - 1, y + row, color);
    }
}

static void display_menu_draw_battery(uint8 force)
{
    uint16 battery = 0;
    uint16 fill_w = 0;
    uint8 percent = 0;

    battery = (uint16)(power_adc_get_voltage() * 10.0f + 0.5f);
    if(battery <= BATTERY_EMPTY_DECI)
    {
        percent = 0;
    }
    else if(battery >= BATTERY_FULL_DECI)
    {
        percent = 100;
    }
    else
    {
        percent = (uint8)(((battery - BATTERY_EMPTY_DECI) * 100U) / (BATTERY_FULL_DECI - BATTERY_EMPTY_DECI));
    }

    if(!force && (percent == g_menu_last_battery_percent))
    {
        return;
    }

    ips200_set_color(RGB565_BLACK, RGB565_WHITE);
    display_menu_draw_rect(MENU_BAT_X, MENU_BAT_Y, MENU_BAT_W + MENU_BAT_CAP_W + 18, MENU_BAT_H + 2, RGB565_WHITE);
    display_menu_draw_rect_frame(MENU_BAT_X, MENU_BAT_Y, MENU_BAT_W, MENU_BAT_H, RGB565_BLACK);
    display_menu_draw_rect(MENU_BAT_X + MENU_BAT_W, MENU_BAT_Y + 3, MENU_BAT_CAP_W, MENU_BAT_H - 6, RGB565_BLACK);

    if(percent > 0)
    {
        fill_w = ((uint16)(MENU_BAT_W - 2) * percent) / 100U;
        if(fill_w > 0)
        {
            display_menu_draw_rect(MENU_BAT_X + 1, MENU_BAT_Y + 1, fill_w, MENU_BAT_H - 2, RGB565_GREEN);
        }
    }

    ips200_show_string(MENU_BAT_TEXT_X, MENU_TITLE_Y, "    ");
    display_menu_show_percent(MENU_BAT_TEXT_X, MENU_TITLE_Y, percent);
    ips200_show_string(MENU_BAT_TEXT_X + 24, MENU_TITLE_Y, "%");
    g_menu_last_battery_percent = percent;
}

static void display_menu_mark_dirty(void)
{
    g_menu_dirty = 1;
}

static void display_menu_draw_root_item(uint8 index)
{
    uint16 y = 0;

    if(index >= MENU_ITEM_COUNT)
    {
        return;
    }

    y = MENU_LIST_Y + (uint16)index * MENU_LINE_HEIGHT;
    if(index == g_menu_selected)
    {
        ips200_set_color(RGB565_WHITE, RGB565_BLUE);
        ips200_show_string(0, y, "> ");
        ips200_show_string(16, y, g_menu_pages[index].title);
    }
    else
    {
        ips200_set_color(RGB565_BLACK, RGB565_WHITE);
        ips200_show_string(0, y, "  ");
        ips200_show_string(16, y, g_menu_pages[index].title);
    }
}

static void display_menu_draw_root(void)
{
    uint8 i = 0;

    ips200_clear(RGB565_WHITE);
    display_menu_draw_battery(1);

    for(i = 0; i < MENU_ITEM_COUNT; i++)
    {
        display_menu_draw_root_item(i);
    }
}

static void display_menu_prepare_camera_view(void)
{
    /* 摄像头页不再叠加菜单文案，只保留图像区域，其余区域统一拉黑。 */
    ips200_clear(RGB565_BLACK);
}

static void display_menu_draw_detail(display_page_t page)
{
    const display_page_info_t *info = 0;

    if((page < DISPLAY_PAGE_MENU2) || (page > DISPLAY_PAGE_ABOUT))
    {
        return;
    }

    info = &g_menu_pages[page - 1];

    ips200_clear(RGB565_WHITE);

    ips200_set_color(RGB565_BLUE, RGB565_WHITE);
    ips200_show_string(0, MENU_TITLE_Y, info->title);
    display_menu_draw_battery(1);

    ips200_set_color(RGB565_BLACK, RGB565_WHITE);
    ips200_show_string(0, MENU_DETAIL_Y + 0 * MENU_LINE_HEIGHT, info->line1);
    ips200_show_string(0, MENU_DETAIL_Y + 1 * MENU_LINE_HEIGHT, info->line2);
    ips200_show_string(0, MENU_DETAIL_Y + 2 * MENU_LINE_HEIGHT, info->line3);
    ips200_show_string(0, MENU_DETAIL_Y + 3 * MENU_LINE_HEIGHT, info->line4);
}

void display_menu_init(void)
{
    g_menu_selected = 0;
    g_menu_page = DISPLAY_PAGE_ROOT;
    g_menu_dirty = 1;
    g_menu_last_battery_percent = 0xFF;
}

void display_menu_render(void)
{
    if(DISPLAY_PAGE_CAMERA == g_menu_page)
    {
        if(g_menu_dirty)
        {
            display_menu_prepare_camera_view();
            g_menu_dirty = 0;
        }
        return;
    }

    if(!g_menu_dirty)
    {
        display_menu_draw_battery(0);
        return;
    }

    if(DISPLAY_PAGE_ROOT == g_menu_page)
    {
        display_menu_draw_root();
    }
    else
    {
        display_menu_draw_detail(g_menu_page);
    }

    g_menu_dirty = 0;
}

void display_menu_move_up(void)
{
    uint8 previous_selected = 0;

    if(DISPLAY_PAGE_ROOT != g_menu_page)
    {
        return;
    }

    previous_selected = g_menu_selected;
    if(0 == g_menu_selected)
    {
        g_menu_selected = MENU_ITEM_COUNT - 1;
    }
    else
    {
        g_menu_selected--;
    }

    if(g_menu_dirty)
    {
        display_menu_render();
    }
    else
    {
        display_menu_draw_root_item(previous_selected);
        display_menu_draw_root_item(g_menu_selected);
    }
}

void display_menu_move_down(void)
{
    uint8 previous_selected = 0;

    if(DISPLAY_PAGE_ROOT != g_menu_page)
    {
        return;
    }

    previous_selected = g_menu_selected;
    g_menu_selected++;
    if(g_menu_selected >= MENU_ITEM_COUNT)
    {
        g_menu_selected = 0;
    }

    if(g_menu_dirty)
    {
        display_menu_render();
    }
    else
    {
        display_menu_draw_root_item(previous_selected);
        display_menu_draw_root_item(g_menu_selected);
    }
}

void display_menu_enter(void)
{
    if(DISPLAY_PAGE_ROOT != g_menu_page)
    {
        display_menu_back();
        return;
    }

    g_menu_page = (display_page_t)(DISPLAY_PAGE_CAMERA + g_menu_selected);
    display_menu_mark_dirty();
    display_menu_render();
}

void display_menu_back(void)
{
    if(DISPLAY_PAGE_ROOT == g_menu_page)
    {
        return;
    }

    g_menu_page = DISPLAY_PAGE_ROOT;
    display_menu_mark_dirty();
    car_servo_set_center();
    display_menu_render();
}

uint8 display_menu_in_camera_view(void)
{
    return (DISPLAY_PAGE_CAMERA == g_menu_page) ? 1 : 0;
}