//
// 控制周期实现
//

#include "control_time.h"
#include "cmsis_os.h"

/* 下一个"该醒"的时刻（tick，1 ms 一个） */
static uint32_t s_next_tick;

/* 上一次醒来的时刻 */
static uint32_t s_last_tick;

/* 最近一次测到的 dt（ms） */
static uint32_t s_dt_ms;

/* Ozone 里直接看这个，确认周期稳不稳 */
volatile uint32_t g_ctrl_dt_ms;

/* 初始化过没有 */
static uint8_t s_inited;

void ctrl_time_init(void)
{
    s_last_tick = osKernelSysTick();
    s_next_tick = s_last_tick + CTRL_PERIOD_MS;
    s_dt_ms     = CTRL_PERIOD_MS;
    s_inited    = 1u;
}

uint32_t ctrl_time_wait(void)
{
    uint32_t now;
    uint32_t dt;
    int32_t  remain;

    if (s_inited == 0u)
    {
        ctrl_time_init();
    }

    /* 睡到目标时刻 */
    now    = osKernelSysTick();
    remain = (int32_t)(s_next_tick - now);

    if (remain > 0)
    {
        osDelay((uint32_t)remain);
    }

    now = osKernelSysTick();

    /* 实测 dt */
    dt = now - s_last_tick;
    s_last_tick = now;

    if (dt < CTRL_DT_MIN_MS)
    {
        dt = CTRL_DT_MIN_MS;
    }
    if (dt > CTRL_DT_MAX_MS)
    {
        dt = CTRL_DT_MAX_MS;
    }

    s_dt_ms = dt;
    g_ctrl_dt_ms = dt;

    /* 推进目标时刻：正常情况 += 周期；
     * 如果已经落后超过一个周期（被别的任务抢占了），就重新对齐，
     * 避免之后连续"补课"把周期压得很短 */
    s_next_tick += CTRL_PERIOD_MS;

    if ((int32_t)(now - s_next_tick) > 0)
    {
        s_next_tick = now + CTRL_PERIOD_MS;
    }

    return dt;
}

uint32_t ctrl_time_dt(void)
{
    return s_dt_ms;
}
