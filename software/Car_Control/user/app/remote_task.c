//
// 遥控数据接收任务
//
//   1. 每 2ms 问一次无线芯片有没有新帧
//   2. 校验并解析成 remote_cmd_t
//   3. 覆盖写进队列，同时更新急停 / 掉线 / 时间戳

//

#include "remote_task.h"
#include "si24r1.h"
#include "debug.h"
#include "cmsis_os.h"

#define POLL_PERIOD_MS      2u      /* 每2ms查一次芯片 */
#define PRINT_EVERY_N       50u     /* 每 50 帧打印一行 ≈ 0.5s */

/* ---------------- 跨任务共享变量 ---------------- */

QueueHandle_t q_remote_cmd;

/* 上电默认"急停 + 掉线"，收到第一帧有效数据才解除 */
volatile uint8_t  g_remote_estop  = 1;//急停
volatile uint8_t  g_link_lost     = 1;//掉线
volatile uint32_t g_last_rx_tick  = 0;//最后一次有效帧时刻
volatile uint32_t g_remote_rx_ok  = 0;//累计有效帧
volatile uint32_t g_remote_rx_err = 0;//多少帧校验失败

/* 本地急停按钮。等 safety_task 写好后接管 */
volatile uint8_t  g_local_estop   = 0;

/* ---------------- 解析 ---------------- */

/**
 * @brief 把 17 字节拆成 remote_cmd_t
 * @return 0 = 成功；-1 = 帧头不对；-2 = 校验和不对
 */
static int remote_parse(const uint8_t *buf, remote_cmd_t *out)
{
    uint32_t sum = 0;
    uint32_t rx_sum;
    uint8_t  i;

    if ((buf[0] != REMOTE_HEAD0) || (buf[1] != REMOTE_HEAD1) || (buf[2] != REMOTE_HEAD2))
    {
        return -1;
    }

    /* 前 13 字节的 32 位累加和 */
    for (i = 0; i < 13; i++)
    {
        sum += buf[i];
    }

    /* 大端读取 [13..16] */
    rx_sum = ((uint32_t)buf[13] << 24) | ((uint32_t)buf[14] << 16) |
             ((uint32_t)buf[15] << 8)  |  (uint32_t)buf[16];

    if (sum != rx_sum)
    {
        return -2;
    }

    /* 大端读取四个轴（有符号，补码自动处理） */
    out->ly    = (int16_t)(((uint16_t)buf[3] << 8) | buf[4]);
    out->lx    = (int16_t)(((uint16_t)buf[5] << 8) | buf[6]);
    out->ry    = (int16_t)(((uint16_t)buf[7] << 8) | buf[8]);
    out->rx    = (int16_t)(((uint16_t)buf[9] << 8) | buf[10]);
    out->flags = buf[11];

    return 0;
}

/* ---------------- 任务 ---------------- */

void StartRemoteTask(void const *argument)
{
    uint8_t      raw[REMOTE_FRAME_LEN];
    remote_cmd_t cmd;
    uint32_t     last_ok;
#if DEBUG_ENABLE
    uint32_t     print_cnt = 0;
#endif

    (void)argument;

    if (si24r1_init() == 0)
    {
        debug_printf("[REMOTE] SI24R1 init OK\r\n");
    }
    else
    {
        debug_printf("[REMOTE] SI24R1 init FAIL\r\n");
    }

    last_ok          = osKernelSysTick();
    g_last_rx_tick   = last_ok;

    for (;;)
    {
        if (si24r1_poll(raw) == 1)
        {
            if (remote_parse(raw, &cmd) == 0)
            {
                last_ok          = osKernelSysTick();
                g_last_rx_tick   = last_ok;
                g_link_lost      = 0;
                g_remote_estop   = (cmd.flags & RCMD_ESTOP) ? 1u : 0u;
                g_remote_rx_ok++;

                /* 长度 1 的队列：新帧直接覆盖旧帧，不阻塞 */
                (void)xQueueOverwrite(q_remote_cmd, &cmd);

#if DEBUG_ENABLE
                if ((++print_cnt % PRINT_EVERY_N) == 0)
                {
                    debug_printf("[REMOTE] %d %d %d %d f=%02X\r\n",
                                 cmd.ly, cmd.lx, cmd.ry, cmd.rx, cmd.flags);
                }
#endif
            }
            else
            {
                g_remote_rx_err++;
            }
        }

        /* 掉线保护：超时就把急停标志置1，只打印一次 */
        if ((g_link_lost == 0) &&
            ((osKernelSysTick() - last_ok) > REMOTE_LINK_TIMEOUT_MS))
        {
            g_link_lost    = 1;
            g_remote_estop = 1;
            debug_printf("[REMOTE] LINK LOST\r\n");
        }

        osDelay(POLL_PERIOD_MS);
    }
}
