#include "headfile.h"

static void screen_test_key_init(void)
{
    gpio_init(KEY_BACK, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(KEY_UP, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(KEY_DOWN, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(KEY_ENTER, GPI, GPIO_HIGH, GPI_PULL_UP);
}

static void screen_test_color(uint16 color, const char name[])
{
    ips200_clear(color);
    if(color == RGB565_WHITE)
    {
        ips200_set_color(RGB565_BLACK, color);
    }
    else
    {
        ips200_set_color(RGB565_WHITE, color);
    }
    ips200_show_string(0, 0, "IPS200 TEST");
    ips200_show_string(0, 16, "COLOR:");
    ips200_show_string(64, 16, name);
}

static void screen_test_grid(void)
{
    uint16 x;
    uint16 y;

    ips200_clear(RGB565_BLACK);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_show_string(0, 0, "GRID TEST");

    for(x = 0; x < 135; x++)
    {
        ips200_draw_point(x, 0, RGB565_RED);
        ips200_draw_point(x, 239, RGB565_RED);
        if((x % 10) == 0)
        {
            for(y = 16; y < 240; y++)
            {
                ips200_draw_point(x, y, RGB565_BLUE);
            }
        }
    }

    for(y = 16; y < 240; y++)
    {
        ips200_draw_point(0, y, RGB565_RED);
        ips200_draw_point(134, y, RGB565_RED);
        if((y % 10) == 0)
        {
            for(x = 0; x < 135; x++)
            {
                ips200_draw_point(x, y, RGB565_GREEN);
            }
        }
    }
}

static void screen_test_key(void)
{
    ips200_clear(RGB565_BLACK);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_show_string(0, 0, "KEY TEST");
    ips200_show_string(0, 16, "BACK");
    ips200_show_string(0, 32, "UP");
    ips200_show_string(0, 48, "DOWN");
    ips200_show_string(0, 64, "ENTER");
}

void screen_test_run(void)
{
    uint8 page;
    uint8 key_level[4];

    screen_test_key_init();
    ips200_set_dir(IPS200_PORTAIT);
    ips200_init();
    page = 0;

    while(1)
    {
        if(page == 0)
        {
            screen_test_color(RGB565_RED, "RED");
        }
        else if(page == 1)
        {
            screen_test_color(RGB565_GREEN, "GREEN");
        }
        else if(page == 2)
        {
            screen_test_color(RGB565_BLUE, "BLUE");
        }
        else if(page == 3)
        {
            screen_test_color(RGB565_WHITE, "WHITE");
            ips200_set_color(RGB565_BLACK, RGB565_WHITE);
            ips200_show_string(0, 32, "BLACK WORD");
        }
        else if(page == 4)
        {
            screen_test_grid();
        }
        else
        {
            screen_test_key();
            while(1)
            {
                key_level[0] = gpio_get_level(KEY_BACK);
                key_level[1] = gpio_get_level(KEY_UP);
                key_level[2] = gpio_get_level(KEY_DOWN);
                key_level[3] = gpio_get_level(KEY_ENTER);

                ips200_show_uint8(80, 16, key_level[0]);
                ips200_show_uint8(80, 32, key_level[1]);
                ips200_show_uint8(80, 48, key_level[2]);
                ips200_show_uint8(80, 64, key_level[3]);
                system_delay_ms(20);

                if(key_level[3] == 0)
                {
                    while(gpio_get_level(KEY_ENTER) == 0);
                    break;
                }
            }
        }

        system_delay_ms(600);
        page++;
        if(page > 5)
        {
            page = 0;
        }
    }
}
