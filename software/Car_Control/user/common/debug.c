//
// 调试串口打印实现（USART2, 115200）
//
// 只做三件事：格式化、限速、发送。
// 不认识任何业务模块。
//

#include "debug.h"

#if DEBUG_ENABLE

#include "usart.h"
#include <stdio.h>
#include <stdarg.h>

void debug_printf(const char *fmt, ...)
{
    char    buf[96];
    va_list ap;
    int     n;

    if (fmt == 0)
    {
        return;
    }

    va_start(ap, fmt);
    n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if (n <= 0)
    {
        return;
    }

    /* vsnprintf 返回的是"本来想写多少"，可能超过缓冲，要截断 */
    if (n > (int)sizeof(buf) - 1)
    {
        n = (int)sizeof(buf) - 1;
    }

    (void)HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)n, 100);
}

uint8_t debug_every(uint32_t *last_tick, uint32_t period_ms)
{
    uint32_t now;

    if (last_tick == 0)
    {
        return 0;
    }

    now = HAL_GetTick();

    if ((uint32_t)(now - *last_tick) < period_ms)
    {
        return 0;
    }

    *last_tick = now;
    return 1;
}

#endif // DEBUG_ENABLE

/* ---------------- FreeRTOS 钩子 ----------------
 *
 * CubeMX 生成的 freertos.c 里这两个钩子是 __weak 的空函数，
 * 出问题什么都不说。这里给出强定义：打印是哪个任务，然后停住。
 *
 * 如果串口看到 "STACK OVERFLOW: 任务名"，就说明那个任务栈给小了。
 * 如果看到 "MALLOC FAILED"，说明 FreeRTOS 堆(configTOTAL_HEAP_SIZE)不够。
 */

#include "FreeRTOS.h"
#include "task.h"

void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char *pcTaskName)
{
    (void)xTask;

#if DEBUG_ENABLE
    debug_printf("\r\n[FATAL] STACK OVERFLOW: %s\r\n", (char *)pcTaskName);
#endif

    /* 停在这里，方便在 Ozone 里看栈内容 */
    taskDISABLE_INTERRUPTS();
    for (;;)
    {
    }
}

void vApplicationMallocFailedHook(void)
{
#if DEBUG_ENABLE
    debug_printf("\r\n[FATAL] MALLOC FAILED (FreeRTOS heap out)\r\n");
#endif

    taskDISABLE_INTERRUPTS();
    for (;;)
    {
    }
}
