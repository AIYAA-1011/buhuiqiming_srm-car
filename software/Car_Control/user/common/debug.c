//
// 调试串口打印实现（USART2, 115200）
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

#endif // DEBUG_ENABLE