/*
 * system_state.h - 系统状态定义
 *
 * 定义系统的各种状态
 */

#ifndef __SYSTEM_STATE_H__
#define __SYSTEM_STATE_H__

/************ 系统状态枚举 ************/
typedef enum {
    SYS_INIT = 0,      // 上电初始化
    SYS_PREPARE,       // 准备就绪，可运行
    SYS_RUNNING,       // 赛道运行
    SYS_STOPED,        // 停止
    SYS_EMERGENCY      // 紧急情况
} system_state_t;

#endif /* __SYSTEM_STATE_H__ */
