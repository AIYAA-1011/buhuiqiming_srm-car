//
// 调试串口打印（USART2, 115200）
//
// 验证主要靠串口。比赛固件里把 DEBUG_ENABLE 改成 0，
// 所有 debug_printf 会编译成空操作，不占 flash。
//

#ifndef BUHUIQIMING_SRM_CAR_DEBUG_H
#define BUHUIQIMING_SRM_CAR_DEBUG_H

#define DEBUG_ENABLE    1

#if DEBUG_ENABLE
/**
 * @brief 往 USART2 打印（阻塞发送，最长等 100ms）
 *
 * 一行控制在 60 字节以内，避免阻塞太久错过无线帧。
 * 不要放在 1ms 控制环里。
 */
void debug_printf(const char *fmt, ...);
#else
#define debug_printf(fmt, ...)      ((void)0)
#endif

#endif // BUHUIQIMING_SRM_CAR_DEBUG_H