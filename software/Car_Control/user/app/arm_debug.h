//
// 机械臂调试输出（应用层）
//
// 这里放"懂机械臂"的调试代码：心跳、面包屑、门控提示、标定表打印。
// 它依赖 arm_task.h（读状态）和 debug.h（发串口），
// 但 arm_task.c 不反过来依赖它 —— 关掉时下面是空宏，一行都不会编进去。
//
// 开关：
//   现在机械臂已经调通，默认关掉它的输出，把串口让给显示/电池调试；
//   想看机械臂日志：编译时加 -DARM_DEBUG=1，或把下面的 0 改成 1
//

#ifndef CAR_CONTROL_ARM_DEBUG_H
#define CAR_CONTROL_ARM_DEBUG_H

#include <stdint.h>
#include "debug.h"
#include "arm_task.h"

#ifndef ARM_DEBUG
#define ARM_DEBUG   1
#endif

#if (ARM_DEBUG) && (DEBUG_ENABLE)

/**
 * @brief 面包屑：任务卡住之后，看这个数就知道最后走到哪一步
 *
 *   1 = 循环开始      2 = 心跳打印中     3 = 模式判断后
 *   4 = 取队列后      5 = 门控判断后     6 = 门控打印后
 *   7 = jog 之后      8 = osDelay 前     9 = osDelay 返回后
 *
 * 只是 4 字节的赋值，不打印、不占串口，卡死时还能看。
 */
extern volatile uint32_t g_arm_crumb;

#define ARM_CRUMB(n)    (g_arm_crumb = (uint32_t)(n))

/* 机械臂自己的打印宏：ARM_DEBUG=0 时整句消失，连参数都不求值 */
#define arm_debug_printf(...)           debug_printf(__VA_ARGS__)

void arm_debug_print_cfg(void);
void arm_debug_report_gate(arm_gate_t gate, uint8_t flags);
void arm_debug_log_deg(uint32_t idx, const char *tag, float deg);
void arm_debug_tick(void);

#else

#define ARM_CRUMB(n)                    ((void)0)
#define arm_debug_printf(...)           ((void)0)
#define arm_debug_print_cfg()           ((void)0)
#define arm_debug_report_gate(g, f)     ((void)0)
#define arm_debug_log_deg(i, t, d)      ((void)0)
#define arm_debug_tick()                ((void)0)

#endif /* ARM_DEBUG */

#endif // CAR_CONTROL_ARM_DEBUG_H
