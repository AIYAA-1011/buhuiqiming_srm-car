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

/* ==================== 摇杆参数 ====================
 * 轴值域：-500 ~ +500，0 = 中位 / 停止
 * 数据包里发出去的就是这四个数，全程只有这一种表示，
 * 不再有 0-1000 和 -1.0~+1.0 之间的来回换算。
 * ================================================== */
#define AXIS_SPAN     500 /* 满量程 */
#define AXIS_DEADZONE  50 /* 死区半宽（正负对称），约占总行程 10% */

/* 极性：装机实测之后，哪个轴方向反了就把对应宏改成 1 */
#define AXIS_INV_THR 0 /* 左摇杆上下 -> 大臂     */
#define AXIS_INV_YAW 0 /* 左摇杆左右 -> 小臂     */
#define AXIS_INV_PIT 0 /* 右摇杆上下 -> 腕部俯仰 */
#define AXIS_INV_ROL 0 /* 右摇杆左右 -> 腕部转动 */

/* 处理后得到的四个轴命令值
 * 顺序：thr(左杆Y) yaw(左杆X) pit(右杆Y) rol(右杆X) */
extern int16_t axis_value[4];

/**
 * @brief 摇杆零偏校准
 *        把"四个摇杆当前位置"定义为中位（0）。
 *        调用前四个杆必须都停在中位，否则中位就定偏了。
 */
void App_calibrate_joystick(void);

/**
 * @brief 读取并处理摇杆数据（joy_task 里每 20ms 调一次）
 *
 * 处理链：ADC(0-4095) -> -500~+500 -> 极性 -> 减零偏
 *         -> 限幅 -> 死区 -> 死区外重新拉伸
 */
void App_process_joystick_data(void);

#endif // __APP_PROCESS_DATA__
