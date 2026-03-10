#ifndef MENU_H
#define MENU_H

#define NumStart  0
#define NumEnd    8

#define SetMenu Switch2&&!Switch1
#define IMAGE_M Switch1&&!Switch2
#define ST_CAR  !Switch1&&!Switch2
#define DEBUG_CAR Switch1||Switch2
#define TEST_CAR Switch1&&Switch2
//mini
#define key_up          rvsp>=3
#define key_down        rvsp<-3
#define key_left        lvsp<-3
#define key_right       P27 ==  0
#define key_press       lvsp>=3

#define Switch2         P46 ==  0
#define Switch1         P35 ==  0

#define DOWN            1
#define UP              2
#define PRESS           3
#define LEFT            4
#define RIGHT           5
#define ADD             6
#define DEC             7

#define x_main 8
#define x_sec  90

#define h_l   1

#define hang 8
#define PG0 hang*0
#define PG1 hang*1
#define PG2 hang*2
#define PG3 hang*3
#define PG4 hang*4
#define PG5 hang*5
#define PG6 hang*6
#define PG7 hang*7
#define PG8 hang*8


#define line0 0*h_l
#define line1 1*h_l
#define line2 2*h_l
#define line3 3*h_l
#define line4 4*h_l
#define line5 5*h_l
#define line6 6*h_l
#define line7 7*h_l

// #define show_str    ips114_showstr
// #define show_int    ips114_showint16
// #define show_float  ips114_showfloat
// #define clear       ips114_clear

#define show_str    oled_p6x8str_spi
#define show_int    oled_int16_spi
#define show_float  oled_printf_float_spi
#define clear       oled_fill_spi



    // #define show_str    ips114_showstr_simspi
    // #define show_int    ips114_showint16_simspi
    // #define show_float  ips114_showfloat_simspi
    // #define clear       ips114_clear_simspi


typedef struct
{
    int Num;
    int Next;
    int Back;
    int Enter;
    int Esc;
    void (*fx)();
}Menu;

void Menu_display();
void image_ori_bin();

#endif