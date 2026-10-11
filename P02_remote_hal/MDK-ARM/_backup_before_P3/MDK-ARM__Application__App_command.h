#ifndef __APP_COMMAND__
#define __APP_COMMAND__

#include "Int_key.h"
#include "App_process_data.h"
#include "Com_debug.h"
#include "FreeRTOS.h"
#include "task.h"

/* 屏幕行1 最右一格显示的状态字母，永远只显示一个 */
#define CMD_ACTION_NONE  ' '
#define CMD_ACTION_HOLE  'H' /* 过洞 */
#define CMD_ACTION_PICK  'P' /* 取矿 */
#define CMD_ACTION_PUT   'D' /* 兑矿 */
#define CMD_ACTION_ESTOP 'E' /* 急停，反白显示 */

/* 预设姿态编号 */
#define CMD_PRESET_NONE 0
#define CMD_PRESET_HOLE 1
#define CMD_PRESET_PICK 2
#define CMD_PRESET_PUT  3

/**
 * @brief 遥控器命令状态：按键产生，发送层取用
 */
typedef struct
{
    uint8_t mode;    /* MODE_CHASSIS / MODE_ARM */
    uint8_t estop;   /* 1 = 急停锁定 */
    uint8_t gripper; /* 1 = 夹爪夹紧，0 = 张开 */
    uint8_t preset;  /* CMD_PRESET_xxx */
    char    action;  /* 屏幕上显示的字母 */
} Command_Data;

extern Command_Data cmd;

/**
 * @brief 初始化命令状态
 */
void App_command_init(void);

/**
 * @brief 扫描按键并处理事件（key_task 里每 20ms 调一次）
 *        全程非阻塞：按住任何键都不会影响其它键，急停按下即生效
 */
void App_command_scan(void);

#endif // __APP_COMMAND__
