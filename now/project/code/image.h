#ifndef __IMAGE_H__
#define __IMAGE_H__

#include "headfile.h"

#define LCDH                       (60)
#define LCDW                       (80)
#define IMAGE_H                    (LCDH)
#define IMAGE_W                    (LCDW)
#define ImageSensorMid             (39)
#define IMAGE_MID                  (ImageSensorMid) // 中心列
#define IMAGE_BLACK                (0)
#define IMAGE_WHITE                (1)
#define POINT_NUM                  (100) // 八邻域预留边界点

typedef struct
{
    uint8 ready;              /* 摄像头初始化完成标志 */
    uint8 result_ready;       /* 本帧结果可用，可供舵机控制使用 */
    uint16 sequence;          /* 已处理的图像帧计数器 */
    uint8 threshold;          /* 二值化最终阈值 */
    int16 center;             /* 加权计算得到的赛道中心列 */
    int16 error;              /* 中心与图像中线的偏差：center - IMAGE_MID */
    uint8 lost;               /* 是否丢失赛道（无可信赛道信息） */
    uint8 ring;               /* 环岛标志：0 无，1 左环岛，2 右环岛 */
    uint8 ring_step;          /* 环岛处理阶段（状态机步骤） */
    uint8 zebra;              /* 当前帧是否检测到斑马线 */
    uint8 zebra_count;        /* 累计确认的斑马线检测次数 */
    uint8 cross;              /* 判断十字标志 */
} image_data; // 图像整体数据

extern image_data Image;
extern uint8 ImageGray[IMAGE_H][IMAGE_W];
extern uint8 ImageBin[IMAGE_H][IMAGE_W];
extern uint8 Pixle[IMAGE_H][IMAGE_W];

typedef struct 
{
    uint8 point_left[POINT_NUM][2]; // 左边界 0-x-col 1-y-row
    uint8 point_right[POINT_NUM][2]; // 右边界 0-x 1-y
    uint16 left_data_num;
    uint16 right_data_num;
    uint8 dir_left[POINT_NUM]; // 左边界生长方向
    uint8 dir_right[POINT_NUM];// 右边界生长方向
}border_line; // 八邻域搜线结构体

void image_init(void);
void image_apply_camera(void);
void image_update(void);
void image_show_debug_overlay(uint16 x, uint16 y, uint16 w, uint16 h);

#endif
