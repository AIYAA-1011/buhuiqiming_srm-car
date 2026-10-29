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
    uint32_t     hb_tick = 0u;
#if DEBUG_ENABLE
    uint32_t     print_cnt = 0;
#endif

    (void)argument;

    if (si24r1_init() == 0)
    {
        uint8_t a = 0u, b = 0u, c = 0u, d = 0u, e = 0u;
        uint8_t st_b = 0u, st_a = 0u;

        debug_printf("[REMOTE] SI24R1 init OK\r\n");

        /* 期望：0F 01 28 11 06（CRC16+上电+接收 / 自动应答 / 通道40 / 17字节 / 1Mbps） */
        if (si24r1_read_config(&a, &b, &c, &d, &e) == 0)
        {
            debug_printf("[REMOTE] cfg(before CE)=%02X %02X %02X %02X %02X\r\n",
                         (unsigned)a, (unsigned)b, (unsigned)c, (unsigned)d, (unsigned)e);
        }
        if (si24r1_live_config(&a, &b, &c, &d, &e) == 0)
        {
            debug_printf("[REMOTE] cfg(after  CE)=%02X %02X %02X %02X %02X\r\n",
                         (unsigned)a, (unsigned)b, (unsigned)c, (unsigned)d, (unsigned)e);
        }
        if (si24r1_ce_effect(&st_b, &st_a) == 0)
        {
            debug_printf("[REMOTE] STATUS before CE=%02X after CE=%02X\r\n",
                         (unsigned)st_b, (unsigned)st_a);
        }
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

        /* 心跳：每 2 秒报一次收包情况
         *   rx   有效帧数（不涨 = 收不到）
         *   err  收到但解析失败
         *   st   STATUS（bit6=RX_DR 有新包）
         *   fifo FIFO_STATUS（bit0=1 表示接收 FIFO 空）
         *   self 自检：0 = 芯片还活着，-1 = 没响应（供电/SPI） */
        if ((osKernelSysTick() - hb_tick) >= 2000u)
        {
            uint8_t lc = 0u, la = 0u, le = 0u, lf = 0u, lp = 0u;
            int     rc_now;

            hb_tick = osKernelSysTick();
            rc_now = si24r1_read_now(&lc, &la, &le, &lf, &lp);

            debug_printf("[REMOTE] alive rx=%lu err=%lu st=%02X fifo=%02X self=%d rpd=%u lost=%u\r\n",
                         (unsigned long)g_remote_rx_ok,
                         (unsigned long)g_remote_rx_err,
                         (unsigned)si24r1_status(),
                         (unsigned)si24r1_fifo_status(),
                         si24r1_self_test(),
                         (unsigned)si24r1_rpd(),
                         (unsigned)g_link_lost);

            /* rc_now = -1 表示"还没初始化成功，没读"（此时后面几个 0 没意义）
             * aa 期望 01：自动应答必须开，否则遥控器收不到 ACK */
            debug_printf("[REMOTE] now rc=%d cfg=%02X aa=%02X enrx=%02X ch=%02X pw0=%02X\r\n",
                         rc_now, (unsigned)lc, (unsigned)la,
                         (unsigned)le, (unsigned)lf, (unsigned)lp);

            /* cfg 期望 0F；变成 08/00 说明芯片复位了（供电问题），自动重新初始化 */
            if ((rc_now != 0) || (lc != 0x0Fu))
            {
                debug_printf("[REMOTE] radio lost config (reset?), re-init...\r\n");

                if (si24r1_init() == 0)
                {
                    debug_printf("[REMOTE] re-init OK\r\n");
                }
                else
                {
                    debug_printf("[REMOTE] re-init FAIL (power?)\r\n");
                }
            }
        }
    }
}
