//
// 调试串口打印（USART2, 115200）
//
// 这是整个工程的"唯一输出口"：所有模块都通过 debug_printf 打日志，
// 换串口 / 换编码 / 统一关掉，只改这一个文件。
// 它不认识任何业务模块（机械臂、底盘、遥控……），只负责把字符串发出去。
//
// 比赛固件里把 DEBUG_ENABLE 改成 0，所有 debug_printf 和 debug_every
// 都会编译成空操作，不占 flash、不占 CPU。
//

#ifndef BUHUIQIMING_SRM_CAR_DEBUG_H
#define BUHUIQIMING_SRM_CAR_DEBUG_H

#include <stdint.h>

/* 总开关：比赛固件改成 0；也可以在编译选项里加 -DDEBUG_ENABLE=0 覆盖 */
#ifndef DEBUG_ENABLE
#define DEBUG_ENABLE    0
#endif

#if DEBUG_ENABLE
/**
 * @brief 往 USART2 打印（阻塞发送，最长等 100ms）
 *
 * 一行控制在 60 字节以内，避免阻塞太久错过无线帧。
 * 不要放在 1ms 控制环里。
 */
void debug_printf(const char *fmt, ...);

/**
 * @brief 限速打印：每 period_ms 最多返回一次 1
 *
 * 用法：
 *   static uint32_t last;
 *   if (debug_every(&last, 2000u)) { debug_printf(...); }
 *
 * @param last_tick 调用者提供的"上次打印时刻"变量，初值给 0
 * @param period_ms 周期，毫秒
 * @return 1 = 到时间了，可以打印；0 = 还没到
 */
uint8_t debug_every(uint32_t *last_tick, uint32_t period_ms);

#else
#define debug_printf(fmt, ...)      ((void)0)
#define debug_every(last, ms)       (0u)
#endif

#endif // BUHUIQIMING_SRM_CAR_DEBUG_H
