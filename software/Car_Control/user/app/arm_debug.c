//
// 机械臂调试输出实现
//
// 只读 arm_task 的状态快照（arm_get_status / arm_get_joint_cfg），
// 不参与任何控制、不改任何位置。
//
// 四个功能：
//   arm_debug_print_cfg()       上电打印 5 行标定表
//   arm_debug_report_gate()     门控状态变化时打印一行
//   arm_debug_log_deg()         打印角度，一位小数（不用 %f）
//   arm_debug_tick()            每 2 秒一行心跳
//
// 整个文件受 (ARM_DEBUG && DEBUG_ENABLE) 控制。
// 关掉时这里编出来是空的，arm_debug.h 里的同名宏会变成 ((void)0)。
//

#include "arm_debug.h"
#include "arm_task.h"

#if (ARM_DEBUG) && (DEBUG_ENABLE)

/* FreeRTOS 头文件只为了 uxTaskGetStackHighWaterMark()，用来报栈余量 */
#include "FreeRTOS.h"
#include "task.h"

/*
 * 面包屑：任务卡住后，看这个数就知道最后走到哪一步。
 * 声明成 volatile，保证编译器不会把写入优化掉（卡住时正是要读它最后的值）。
 */
volatile uint32_t g_arm_crumb;

/* debug_every() 用的“上次打印时刻”，单位 ms（HAL_GetTick） */
static uint32_t s_hb_tick;

/* 心跳序号 = 机械臂主循环跑到第几圈。用来判断任务是否还活着 */
static uint32_t s_loop_cnt;

/*
 * 上一次打印过的门控状态。
 * 初值 0xFF 不等于任何 arm_gate_t，保证上电后第一次调用一定会打印一行。
 */
static uint8_t s_gate_last = 0xFFu;


void arm_debug_print_cfg(void)
{
    arm_status_t    st;     /* 位置 / 软限位等运行时状态 */
    arm_joint_cfg_t jc;     /* 单个关节的标定参数 */
    uint32_t        i;

    arm_get_status(&st);

    for (i = 0; i < ARM_JOINT_COUNT; i++)
    {
        arm_get_joint_cfg(i, &jc);

        debug_printf("[ARM] ch%u %-11s us %4u..%4u home %4u  deg %3d..%3d (home %3d) dir%s %udps acc%udps2 %s\r\n",
                     (unsigned)jc.ch,
                     jc.name,
                     (unsigned)jc.min_us,
                     (unsigned)jc.max_us,
                     (unsigned)jc.home_us,
                     (int)(st.soft_min_deg[i] + 0.5f),
                     (int)(st.soft_max_deg[i] + 0.5f),
                     (int)(st.pos_deg[i] + 0.5f),
                     (jc.dir > 0) ? "+1" : "-1",
                     (unsigned)jc.speed_dps,
                     (unsigned)jc.acc_dps2,
                     jc.enabled ? "on" : "DISABLED");
    }

    /* 夹爪只有两个工作位置：张开 / 闭合 */
    arm_get_joint_cfg(ARM_JOINT_COUNT - 1u, &jc);
    debug_printf("[ARM] gripper open %d deg, close %d deg  (%s)\r\n",
                 (int)(st.grip_open_deg + 0.5f),
                 (int)(st.grip_close_deg + 0.5f),
                 jc.enabled ? "on" : "DISABLED");
}

/*
 * 门控状态变化时打印一行。
 *
 * 主循环每 20 ms 调用一次，但只有状态真的变了才打印，
 * 所以正常操作时串口很安静，只在“能动了 / 被挡住了”时出现一行。
 *
 * gate  当前门控结果
 * flags 最近一帧遥控帧的标志位，用 %02X 打出来便于对照
 */
void arm_debug_report_gate(arm_gate_t gate, uint8_t flags)
{
    if ((uint8_t)gate == s_gate_last)
    {
        return;                 /* 状态没变，不打印 */
    }
    s_gate_last = (uint8_t)gate;

    switch (gate)
    {
    case ARM_GATE_OK:
        /* 机械臂模式 + 无急停 + 没掉线：可以动 */
        debug_printf("[ARM] ACTIVE: arm mode, sticks live (f=%02X)\r\n", (unsigned)flags);
        break;
    case ARM_GATE_NO_CMD:
        /* 队列里还没有帧（刚上电、remote_task 还没收到过数据） */
        debug_printf("[ARM] FROZEN: no remote frame yet (f=%02X)\r\n", (unsigned)flags);
        break;
    case ARM_GATE_NO_LINK:
        /* 超过 200 ms 没收到有效帧，remote_task 置的 g_link_lost */
        debug_printf("[ARM] FROZEN: remote link lost (f=%02X)\r\n", (unsigned)flags);
        break;
    case ARM_GATE_MODE:
        /* 遥控器在底盘模式（f 的 bit2 = 0） */
        debug_printf("[ARM] FROZEN: chassis mode (f=%02X)\r\n", (unsigned)flags);
        break;
    case ARM_GATE_ESTOP:
    default:
        /* 遥控急停（f 的 bit0）或本地急停 */
        debug_printf("[ARM] FROZEN: e-stop (f=%02X)\r\n", (unsigned)flags);
        break;
    }
}

/*
 * 打印角度，保留一位小数。
 *
 * newlib-nano 默认不支持 %f，
 * 所以手动放大 10 倍变成整数，再拆成“整数部分 + 一位小数”：
 *
 *   126.4  ->  d10 = 1264  ->  whole = 126, frac = 4   ->  "126.4"
 *   -3.2   ->  d10 = -32   ->  whole = -3,  frac = -2  ->  取绝对值 2  ->  "-3.2"
 */
void arm_debug_log_deg(uint32_t idx, const char *tag, float deg)
{
    /* 正数加 0.5、负数减 0.5，然后取整，等于四舍五入 */
    int d10   = (int)(deg * 10.0f + ((deg >= 0.0f) ? 0.5f : -0.5f));
    int whole = d10 / 10;       /* C 的整除向 0 取整，负数的整数部分正好对 */
    int frac  = d10 % 10;       /* 余数可能是负的，下面取绝对值 */

    if (frac < 0)
    {
        frac = -frac;
    }

    debug_printf("[ARM] ch%u %s %d.%d deg\r\n", (unsigned)idx, tag, whole, frac);
}

/*
 * 心跳：每 2 秒一行，证明任务还活着并带上关键状态。
 *
 * 一行举例：
 *   [ARM] alive n=5801 hwm=202 mode=0 gate=0 f=04 ly=0 lx=0 ry=268 rx=0
 *
 *   n    主循环跑了多少圈（20 ms 一圈，2 秒约 +100；不再增长 = 任务死了）
 *   hwm  栈还剩多少 words（uxTaskGetStackHighWaterMark，越小越危险）
 *   mode g_arm_mode（0 遥控 / 1 台架脉宽 / 2 台架角度）
 *   gate 门控状态（0 通 / 2 掉线 / 3 底盘 / 4 急停）
 *   f    遥控帧标志位
 *   ly lx ry rx  机械臂任务实际读到的四个轴（推杆时应该跟着变）
 */
void arm_debug_tick(void)
{
    arm_status_t st;

    s_loop_cnt++;

    /* 第一圈立刻打印，方便确认任务已经跑起来了；
     * 之后每 2000 ms 才打印一次（debug_every 内部用 HAL_GetTick 计时） */
    if ((s_loop_cnt != 1u) && (debug_every(&s_hb_tick, 2000u) == 0u))
    {
        return;
    }

    /* 只在真正要打印时才拷状态，平时不花这个开销 */
    arm_get_status(&st);

    /* 面包屑标记“正在打印心跳”，如果卡死在这里，s_crumb 就停在 2 */
    ARM_CRUMB(2);

    debug_printf("[ARM] alive n=%lu hwm=%u mode=%u gate=%u f=%02X ly=%d lx=%d ry=%d rx=%d\r\n",
                 (unsigned long)s_loop_cnt,
                 (unsigned)uxTaskGetStackHighWaterMark(NULL),   /* NULL = 当前任务 */
                 (unsigned)st.mode,
                 (unsigned)st.gate,
                 (unsigned)st.flags,
                 (int)st.axis[0],
                 (int)st.axis[1],
                 (int)st.axis[2],
                 (int)st.axis[3]);
}

#endif /* (ARM_DEBUG) && (DEBUG_ENABLE) */
