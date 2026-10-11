#ifndef __APP_PROCESS_DATA__
#define __APP_PROCESS_DATA__

#include "Int_joystick.h"
#include "Int_key.h"
#include "Com_debug.h"
#include "Com_tool.h"

/* 控制模式
 * 由 App_command 按键切换产生，App_display 取来显示。
 * 放在这里共用一份定义，避免两个模块各写一套。 */
#define MODE_CHASSIS 0 /* 底盘（麦轮） */
#define MODE_ARM     1 /* 机械臂 */

typedef struct
{
    int16_t thr;
    int16_t yaw;
    int16_t pit;
    int16_t rol;
    uint8_t shutdown;   // 1: 关闭  0: 不关机
    uint8_t fix_height; // 1. 切换定高和不定高 0: 不切换
} Remote_Data;

/**
 * @brief 摇杆零偏校准
 *        校准前请把四个摇杆都放到中位（左杆Y 也放到中位）
 */
void App_calibrate_joystick(void);

/**
 * @brief 处理摇杆数据  => 修正极性相位和标准值
 *
 */
void App_process_joystick_data(void);

#endif // __APP_PROCESS_DATA__
