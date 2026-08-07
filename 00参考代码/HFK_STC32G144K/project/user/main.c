/*********************************************************************************************************************
* STC32G144K Opensourec Library 即（STC32G144K 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2025 SEEKFREE 逐飞科技
*
* 本文件是STC32G144K开源库的一部分
*
* STC32G144K 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          MDK FOR C251
* 适用平台          STC32G144K
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者           备注
* 2025-11-20        大W            first version
* 2026-4            lscyzq         init
********************************************************************************************************************/
#include "zf_common_headfile.h"

int16 LCorner, RCorner, s2, centerLineCount, RUN_MODE = ZEBRA, LRoadCount = expRoadCount, RRoadCount = expRoadCount;
static int16 p, i, d, s1, imBtn, borderInTriggerDistance = 25, ab;
uint8 borderIn, switch1, switch2, ALONG = ALONG_R, topBtn, bottomBtn;
Point centerLine[expRoadCount], LRoad[expRoadCount], RRoad[expRoadCount], target;
extern uint8 outputImage[MT9V03X_H][MT9V03X_W], canNotInRamp, LThreshold, RThreshold, threshold;
static uint8 have, data_buffer[32], send_buff[32], findBorderIn;
extern int16 currentDuty;
int16 data i, j, pos;

void loadRoad();

uint8 haveZebra();

void tof() {dl1b_get_distance();}

void main(void) {
    clock_init(SYSTEM_CLOCK_96M);
    debug_init();
    ips200_init();
    while (mt9v03x_init());
    while (dl1b_init());
    pit_ms_init(TIM6_PIT, 10, tof);
#if (DEBUG_MODE)
    while (wireless_uart_init());
#endif
    gpio_init(IO_PB1, GPI, 0, GPI_PULL_UP);
    gpio_init(IO_PB0, GPI, 0, GPI_PULL_UP);
    gpio_init(IO_P02, GPI, 0, GPI_PULL_UP);
    gpio_init(IO_P01, GPI, 0, GPI_PULL_UP);
    gpio_init(IO_P32, GPI, 0, GPI_PULL_UP);
    gpio_init(IO_P90, GPO, 0, GPO_PUSH_PULL); //蜂鸣器
    //gpio_init(IO_P04, GPO, 0, GPO_PUSH_PULL); //示波器
    laserInit();
    LThreshold = RThreshold = threshold = mt9v03x_image[80][94] + mt9v03x_image[107][94] >> 1;
    displayBorderLineImage();
    while (1) {
        system_delay_ms(50);
#if (DEBUG_MODE)
        if (wireless_uart_read_buffer(data_buffer, 32)) {
            sscanf(data_buffer, "%d,%d,%d,%d,%d,%d", &p, &i, &d, &s1, &s2, &ab);
            switch2 = gpio_get_level(IO_PB1);
            switch1 = gpio_get_level(IO_PB0);
            ips200_clear(RGB565_WHITE);
            motorInit(p, i, d, s1, s2, ab, 0);
            break;
        }
#else
        switch2 = gpio_get_level(IO_PB1);
        switch1 = gpio_get_level(IO_PB0);
        topBtn = gpio_get_level(IO_P02);
        bottomBtn = gpio_get_level(IO_P01);
        imBtn = gpio_get_level(IO_P32);
        if (!imBtn) {
            while (1) {
                if (mt9v03x_finish_flag) {
                    loadRoad();
                    displayBinarizationImage();
                    for (i = 0; i < MT9V03X_H; i++)
                        for (j = 0; j < MT9V03X_W; j++)
                            outputImage[i][j] = 0;
                    for (i = 0; i < centerLineCount; i++)
                        outputImage[centerLine[i].x][centerLine[i].y] = 255;
                    for (i = 0; i < LRoadCount; i++)
                        outputImage[LRoad[i].x][LRoad[i].y] = 255;
                    for (i = 0; i < RRoadCount; i++)
                        outputImage[RRoad[i].x][RRoad[i].y] = 255;
                    ips200_displayimage03x(outputImage, MT9V03X_W, MT9V03X_H, 0, MT9V03X_H + 40);
                    ips200_show_int16(0, MT9V03X_H + 1, LRoadCount);
                    ips200_show_int16(MT9V03X_W, MT9V03X_H + 1, RRoadCount);
                    ips200_show_int16(0, MT9V03X_H + 20, LCorner);
                    ips200_show_int16(MT9V03X_W, MT9V03X_H + 20, RCorner);
                    ips200_show_int16(0, 300, RUN_MODE);
                    mt9v03x_finish_flag = 0;
                }
            }
        }
        if (!switch1 && !switch2) {
            ips200_show_string(0, MT9V03X_H + 10, "T: SPEED 1");
            ips200_show_string(0, MT9V03X_H + 40, "B: SPEED 2");
            if (!topBtn) {
                ips200_clear(RGB565_WHITE);
                ips200_show_string(0, 130, "SELECT: 1");
                system_delay_ms(500);
                system_delay_ms(500);
                motorInit(260, 1000, 26, 4500, 520, 0, 0);
                break;
            }
            if (!bottomBtn) {
                ips200_clear(RGB565_WHITE);
                ips200_show_string(0, 160, "SELECT: 2");
                system_delay_ms(500);
                system_delay_ms(500);
                motorInit(200, 800, 22, 4000, 300, 0, 0);
                break;
            }
        } else if (switch1 && !switch2) {
            ips200_show_string(0, MT9V03X_H + 10, "T: SPEED 3");
            ips200_show_string(0, MT9V03X_H + 40, "B: SPEED 4");
            if (!topBtn) {
                ips200_clear(RGB565_WHITE);
                ips200_show_string(0, 130, "SELECT: 3");
                system_delay_ms(500);
                system_delay_ms(500);
                motorInit(210, 900, 23, 4000, 320, 0, 0);
                break;
            }
            if (!bottomBtn) {
                ips200_clear(RGB565_WHITE);
                ips200_show_string(0, 160, "SELECT: 4");
                system_delay_ms(500);
                system_delay_ms(500);
                motorInit(220, 1000, 23, 4000, 340, 0, 0);
                break;
            }
        } else if (!switch1 && switch2) {
            ips200_show_string(0, MT9V03X_H + 10, "T: SPEED 5");
            ips200_show_string(0, MT9V03X_H + 40, "B: SPEED 6");
            if (!topBtn) {
                ips200_clear(RGB565_WHITE);
                ips200_show_string(0, 130, "SELECT: 5");
                system_delay_ms(500);
                system_delay_ms(500);
                motorInit(230, 1000, 24, 4000, 360, 0, 0);
                break;
            }
            if (!bottomBtn) {
                ips200_clear(RGB565_WHITE);
                ips200_show_string(0, 160, "SELECT: 6");
                system_delay_ms(500);
                system_delay_ms(500);
                motorInit(240, 1000, 24, 4000, 380, 0, 0);
                break;
            }
        } else {
            ips200_show_string(0, MT9V03X_H + 10, "T: SPEED 7");
            ips200_show_string(0, MT9V03X_H + 40, "B: SPEED 8");
            if (!topBtn) {
                ips200_clear(RGB565_WHITE);
                ips200_show_string(0, 130, "SELECT: 7");
                system_delay_ms(500);
                system_delay_ms(500);
                motorInit(250, 1000, 25, 4000, 450, 0, 0);
                break;
            }
            if (!bottomBtn) {
                ips200_clear(RGB565_WHITE);
                ips200_show_string(0, 160, "SELECT: 8");
                system_delay_ms(500);
                system_delay_ms(500);
                motorInit(260, 1000, 25, 4000, 500, 0, 0);
                break;
            }
        }
#endif
    }
    dismissLaser();
    imuInit();
    if (DEBUG_MODE) RUN_MODE = FORWARD;
    // seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_DEBUG_UART);
    // seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, outputImage, MT9V03X_W, MT9V03X_H);
    while (1) {
#if (DEBUG_MODE)
        if (wireless_uart_read_buffer(data_buffer, 32)) {
            stop = 1;
            gpio_set_level(IO_P90, 0);
            break;
        }
#endif
        if (RUN_MODE == RAMP || (!canNotInRamp && (RUN_MODE == FORWARD || RUN_MODE == BLOCK) && pitch > 7)) {
            RUN_MODE = RAMP;
            //wireless_uart_send_string("RP\n");
            runRamp();
        }
        if (mt9v03x_finish_flag) {
            // memcpy(outputImage, mt9v03x_image, MT9V03X_IMAGE_SIZE);
            // seekfree_assistant_camera_send();
#if (DEBUG_MODE)
            if (switch1) displayBorderLineImage();
#endif
            //gpio_set_level(IO_P04, 1);
            loadRoad();
            // gpio_set_level(IO_P04, 0);
            if (target.y) startLaser(target.y);
            have = haveZebra();
            if (RUN_MODE == ZEBRA || have) {
                RUN_MODE = ZEBRA;
                //wireless_uart_send_string("ZB\n");
                // sprintf(send_buff, "ZEBRA: %d,%d\n", LRoadCount, RRoadCount);
                // wireless_uart_send_buffer(send_buff, 32);
                runZebra(have);
            } else if (RUN_MODE == BLOCK) {
                runBlock(LCorner, RCorner);
                // sprintf(send_buff, "BLOCK: %d,%d\n", LRoadCount, RRoadCount);
                // wireless_uart_send_buffer(send_buff, 32);
            } else if (RUN_MODE == CROSS ||
                       (RUN_MODE == FORWARD && ((RCorner && RRoadCount > 2) || (LCorner && LRoadCount > 2)) && LRoadCount < 14 &&
                        RRoadCount < 14)) {
                RUN_MODE = CROSS;
                // sprintf(send_buff, "CROSS: %d,%d\n", LRoadCount, RRoadCount);
                // wireless_uart_send_buffer(send_buff, 32);
                runCross();
            } else if (RUN_MODE == R_ROUND ||
                       (RUN_MODE == FORWARD && LRoad[37].x < 20 && RCorner && LRoadCount > 37 && RRoadCount < 11)) {
                RUN_MODE = R_ROUND;
                borderIn = 1;
                //wireless_uart_send_string("RR\n");
                runRRound(LCorner, RCorner);
                // sprintf(send_buff, "R_ROUND: %d,%d\n", LRoadCount, RRoadCount);
                // wireless_uart_send_buffer(send_buff, 32);
            } else if (RUN_MODE == L_ROUND ||
                       (RUN_MODE == FORWARD && RRoad[37].x < 20 && LCorner && RRoadCount > 37 && LRoadCount < 11)) {
                RUN_MODE = L_ROUND;
                borderIn = 1;
                runLRound(LCorner, RCorner);
                //wireless_uart_send_string("LR\n");
                // sprintf(send_buff, "L_ROUND: %d,%d\n", LRoadCount, RRoadCount);
                // wireless_uart_send_buffer(send_buff, 32);
            } else
                alongLine(centerLine, centerLineCount, getAimPos());
            mt9v03x_finish_flag = 0;
        }
    }
}

int8 DTarget(int16 speed) {
    if (speed < 340) return 0;
    if (speed < 350) return 1;
    if (speed < 360) return 2;
    if (speed < 370) return 3;
    if (speed < 380) return 4;
    if (speed < 400) return 5;
    if (speed < 420) return 6;
    if (speed < 440) return 7;
    if (speed < 460) return 8;
    if (speed < 470) return 9;
    if (speed < 480) return 10;
    if (speed < 490) return 11;
    return 12;
}

int8 DTargetR(int16 speed) {
    if (speed < 350) return 0;
    if (speed < 370) return 1;
    return 2;
}

void findTarget() {
    if (func_abs(currentDuty) > 15) {
        target = findTargetByL(targetPos - DTargetR(currentSpeed) - func_min(func_abs(currentDuty) - 5, 30) / 5);
        if (target.y && ((currentDuty > 0 && target.y >= 101) || (currentDuty < 0 && target.y <= 89))) return;
        target = findTargetByL(targetPos - DTargetR(currentSpeed));
        if (target.y && target.y < 101 && target.y > 89) return;
        target = findTargetByL(targetPos - DTargetR(currentSpeed) + func_min(func_abs(currentDuty), 30) / 5);
    } else target = findTargetByL(targetPos - DTarget(currentSpeed));
}

void loadRoad() {
    Point LStart = getLStart(), RStart = getRStart();
    findTarget();
    LRoadCount = expRoadCount, RRoadCount = expRoadCount;
    getLBorder(LRoad, LStart, &LRoadCount, 0);
    getRBorder(RRoad, RStart, &RRoadCount, 0);
    LBorderCount = LRoadCount;
    memcpy(LBorder, LRoad, LBorderCount * sizeof(Point));
    RBorderCount = RRoadCount;
    memcpy(RBorder, RRoad, RBorderCount * sizeof(Point));
    getGodRoadLine(LRoad, LRoadCount);
    getGodRoadLine(RRoad, RRoadCount);
    filterRoad(LRoad, LRoadCount);
    filterRoad(RRoad, RRoadCount);
    resampleRoad(LRoad, &LRoadCount, 0, 3);
    resampleRoad(RRoad, &RRoadCount, 0, 3);
    LCorner = findCorner(LRoad, LRoadCount, 0);
    RCorner = findCorner(RRoad, RRoadCount, 0);
    findBorderIn = 0;
    for (i = 0; i < LRoadCount; i += 2)
        if (LRoad[i].y > 94) {
            findBorderIn = 1;
            break;
        }
    if (LRoad[i].x > borderInTriggerDistance && findBorderIn) borderIn = 1;
    else {
        for (i = 0; i < RRoadCount; i += 2)
            if (RRoad[i].y < 94) {
                findBorderIn = 1;
                break;
            }
        if (RRoad[i].x > borderInTriggerDistance && findBorderIn) borderIn = 1;
        else borderIn = 0;
    }
    if (LCorner) {
        if (LCorner < 3) LCorner = 0;
        else if (LCorner < 15 && RUN_MODE == FORWARD && LRoad[func_limit_ab(LCorner + 3, 0, LRoadCount - 1)].y > LRoad[LCorner].y) {
            RUN_MODE = BLOCK;
            ALONG = ALONG_R;
            centerLineDistance = 9;
        }
        LRoadCount = LCorner;
    }
    if (RCorner) {
        if (RCorner < 3) RCorner = 0;
        else if (RCorner < 15 && RUN_MODE == FORWARD && RRoad[RCorner].y > RRoad[func_limit_ab(RCorner + 3, 0, RRoadCount - 1)].y) {
            RUN_MODE = BLOCK;
            ALONG = ALONG_L;
            centerLineDistance = 9;
        }
        RRoadCount = RCorner;
    }
    if (ALONG == ALONG_L) {
        getCenterLineByLBorder(LRoad, centerLine, LRoadCount);
        centerLineCount = LRoadCount;
    } else {
        getCenterLineByRBorder(RRoad, centerLine, RRoadCount);
        centerLineCount = RRoadCount;
    }
    for (i = 1; i < centerLineCount; i++)
        if (centerLine[i].x < 119) {
            i--;
            break;
        }
    centerLine[i].x = MT9V03X_H - 1;
    centerLine[i].y = centerLine[i + 1].y;
    filterRoad(centerLine, centerLineCount);
    resampleRoad(centerLine, &centerLineCount, i, 2);
    ALONG = LRoadCount > RRoadCount ? ALONG_L : ALONG_R;
#if (DEBUG_MODE)
    if (switch1) {
        for (i = 0; i < MT9V03X_H; i++)
            for (j = 0; j < MT9V03X_W; j++)
                outputImage[i][j] = 0;
        for (i = 0; i < centerLineCount; i++)
            outputImage[centerLine[i].x][centerLine[i].y] = 255;
        for (i = 0; i < LRoadCount; i++)
            outputImage[LRoad[i].x][LRoad[i].y] = 255;
        for (i = 0; i < RRoadCount; i++)
            outputImage[RRoad[i].x][RRoad[i].y] = 255;
        ips200_displayimage03x(outputImage, MT9V03X_W, MT9V03X_H, 0, MT9V03X_H + 40);
        ips200_show_int16(0, MT9V03X_H + 1, LRoadCount);
        ips200_show_int16(MT9V03X_W, MT9V03X_H + 1, RRoadCount);
        ips200_show_int16(0, MT9V03X_H + 20, LCorner);
        ips200_show_int16(MT9V03X_W, MT9V03X_H + 20, RCorner);
        ips200_show_int16(0, 300, RUN_MODE);
    }
#endif
}

uint8 haveZebra() {
    int16 data i = 0;
    if (!LBorderCount) return 0;
    while (i < LBorderCount && LBorder[i].x > ZEBRA_TRIGGER_POS)
        i += 3;
    if (LBorder[i].x > ZEBRA_TRIGGER_POS) return 0;
    return findZebra(i);
}

/*　　┏┓　　┏┓
   　　┏┛┻━━━┛┻┓
   　　┃　　　　　　　 ┃ 　
   　　┃　　　━　　　 ┃
   　　┃　＞　　　＜┃
   　　┃　　　　　　　 ┃
   　　┃ . ⌒　..┃
   　　┃　　　　　　　 ┃
   　　┗━┓　　　┏━┛
     　　　　┃　　　┃　Guardian beasts, protect me; bad luck, stay away!　　　　　
     　　　　┃　　　┃  神兽护体，水逆退散！
     　　　　┃　　　┃　　　　　　　　　　　
     　　　　┃　　　┃ 　　　　　　
     　　　　┃　　　┃
     　　　　┃　　　┃　　　　　　　　　　　
     　　　　┃　　　┗━━━┓
     　　　　┃　　　　　　　┣┓
     　　　　┃　　　　　　　┏┛
     　　　　┗┓┓┏━┳┓┏┛
      　　　　　┃┫┫　┃┫┫
      　　　　　┗┻┛　┗┻┛
 */