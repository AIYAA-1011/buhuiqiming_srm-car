//
// 机械臂任务
//
// 第 1 步：探测 PCA9685
// 第 2 步：初始化 50 Hz / 设脉宽 / 开关输出
// 第 3 步：台架测试（直接给脉宽）
// 第 5 步：标定表 + 角度换算 + 软限幅 + 匀速走到目标
// 第 6 步：接遥控器 —— 摇杆正负决定方向，固定速度积分，松手停
//
// 三种模式（Ozone 改 g_arm_mode）：
//   0 = 遥控（默认）
//   1 = 台架，直接给脉宽 g_test_us[]
//   2 = 台架，给角度 g_test_deg10[]（0.1° 单位），匀速走过去
//

#include "arm_task.h"
#include "arm_debug.h"
#include "pca9685.h"
#include "remote_task.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>

/* ---------------- 参数 ---------------- */

/* 上电时芯片可能还没准备好，重试几次 */
#define ARM_PROBE_RETRY         3u
#define ARM_PROBE_GAP_MS        50u

/* 扫描结果最多记几个 */
#define ARM_SCAN_MAX            8

/* 写完 home 脉宽到开输出之间等多久（一个 PWM 周期 20 ms） */
#define ARM_ENABLE_GAP_MS       50u

/* 控制循环周期 */
#define ARM_LOOP_MS             20u

/* 脉宽 ↔ 角度 的量程端点：500~2500 us 对应 0~deg_range */
#define ARM_US_LO               500.0f
#define ARM_US_SPAN             2000.0f

/* 软限位在实测机械限位的基础上再往里收一点 */
#define ARM_SOFT_MARGIN_DEG     2.0f

/* 判定"顶到限位"的容差 */
#define ARM_LIMIT_EPS_DEG       0.01f

/* ---------------- 关节参数表 ----------------
 *
 * 只改这一张表。改完重新编译，别的都不用动。
 *
 *   min_us / max_us  : 实测的机械行程端点
 *   home_us          : 上电姿态
 *   deg_range        : 舵机规格 180 或 270
 *   dir              : +1 / -1，角度变大时脉宽往哪边走
 *   speed_dps        : 摇杆推到底时的角速度（°/s）
 *   acc_dps2         : 加速度上限（°/s²）。越大启动/停止越猛，电流冲击越大
 *   enabled          : 0 = 还没标定，禁止动作
 *   open_us/close_us : 只有夹爪用 —— 两个工作档位；其它关节填 0
 *
 * 夹爪的 home_us 要和 open_us 写成一样的（上电张开）。
 */
typedef struct
{
    uint8_t     ch;
    uint16_t    min_us;
    uint16_t    max_us;
    uint16_t    home_us;
    uint16_t    deg_range;
    int8_t      dir;
    uint16_t    speed_dps;
    uint16_t    acc_dps2;
    uint8_t     enabled;
    const char *name;
    uint16_t    open_us;    /* 夹爪：工作张开 */
    uint16_t    close_us;   /* 夹爪：工作闭合 */
} arm_joint_t;

static const arm_joint_t arm_cfg[ARM_JOINT_COUNT] =
{
    /* ch  min_us max_us home   range dir  dps   acc   en  name           open    close
     * 注意：ch 是 PCA9685 的通道号（= 板上 LEDx 的 x），不是关节下标。
     * 现在的接线：LED1=大臂 LED4=小臂 LED2=腕俯仰 LED3=腕旋转 LED0=夹爪 */
    { 1u,  1420u, 2500u, 1420u, 270u, +1,  40u,  300u, 1u, "boom",          0u,      0u },
    { 4u,   700u, 2500u,  700u, 270u, +1,  40u,  300u, 1u, "forearm",       0u,      0u },
    { 2u,   500u, 2500u, 1500u, 180u, +1,  40u,  500u, 1u, "wrist_pitch",   0u,      0u },
    { 3u,   500u, 2500u, 1900u, 180u, +1,  60u,  500u, 1u, "wrist_roll",    0u,      0u },
    { 0u,   500u, 1600u, 1300u, 180u, +1,  90u,  800u, 1u, "gripper",    1300u,   650u },
};

/* 摇杆 → 关节 的符号
 *
 * 对应关系（遥控器机械臂模式）：
 *   左摇杆上下 ly = 腕旋转    左摇杆左右 lx = 腕俯仰
 *   右摇杆上下 ry = 小臂      右摇杆左右 rx = 大臂
 * 推杆方向 */
static const int8_t arm_axis_dir[4] =
{
    +1,   /* 0: 大臂    （rx） */
    -1,   /* 1: 小臂    （ry） */
    +1,   /* 2: 腕俯仰  （lx） */
    -1,   /* 3: 腕旋转  （ly） */
};

/* ---------------- 运行时状态 ---------------- */

typedef struct
{
    float deg_min;
    float deg_max;
} arm_soft_t;

static arm_soft_t s_soft[ARM_JOINT_COUNT];
static float      s_pos_deg[ARM_JOINT_COUNT];   /* 当前命令位置 */
static float      s_vel_dps[ARM_JOINT_COUNT];   /* 当前速度（受加速度限制） */
static float      s_tgt_deg[ARM_JOINT_COUNT];   /* 目标位置（台架角度模式用） */
static float      s_home_deg[ARM_JOINT_COUNT];  /* 每个关节的 home 角度 */

/* 遥控触发的"顺序回 home"：
 *   按一次遥控器上的预设键，按 夹爪->腕旋转->腕俯仰->大臂->小臂 依次回 home
 * 触发键：bit3 对孔 / bit4 取矿 / bit5 放矿，改这里换按键 */
#define ARM_HOME_TRIGGER_BIT    RCMD_PRESET_HOLE

static uint8_t    s_home_busy;                  /* 1 = 正在顺序归位 */
static uint8_t    s_home_idx;                   /* 归位到第几个 */
static uint8_t    s_home_req_last;              /* 上一帧的触发位（做上升沿） */

/* 放矿预设（bit5 放矿键触发）：
 *   小臂 1900 -> 大臂 1420 -> 腕俯仰 2400 -> 腕旋转 1900
 * 夹爪不动；一次只动一个关节；按一次执行一遍。 */
#define ARM_PRESET_TRIGGER_BIT  RCMD_PRESET_PUT
#define ARM_PRESET_STEP_COUNT   4u

static const uint8_t  s_preset_order[ARM_PRESET_STEP_COUNT] = { 1u, 0u, 2u, 3u };
static const uint16_t s_preset_us[ARM_PRESET_STEP_COUNT]    = { 1900u, 1420u, 2400u, 1900u };

static uint8_t    s_preset_busy;                /* 1 = 正在执行预设 */
static uint8_t    s_preset_idx;                 /* 第几步 */
static uint8_t    s_preset_req_last;            /* 上一帧的触发位 */
static float      s_grip_open_deg;              /* 夹爪张开角度 */
static float      s_grip_close_deg;             /* 夹爪闭合角度 */
static uint8_t    s_grip_state;                 /* 0 = 张开，1 = 闭合（只用于打印） */

/* 当前正在动的摇杆轴：0=ly 1=lx 2=ry 3=rx，ARM_AXIS_NONE 表示没动 */
#define ARM_AXIS_NONE       0xFFu

/* 滞回：当前轴只要还有"最大值"的这个百分比，就不切换 */
#define ARM_AXIS_KEEP_PCT   85

static uint8_t s_active_axis = ARM_AXIS_NONE;
static int16_t    s_req_deg10[ARM_JOINT_COUNT]; /* 上次请求的目标（0.1°） */
static uint8_t    s_moving[ARM_JOINT_COUNT];    /* 正在走（用于打印 arrived） */
static uint8_t    s_at_limit[ARM_JOINT_COUNT];  /* 已经在限位上（只提示一次） */
static uint16_t   s_last_us[ARM_JOINT_COUNT];   /* 上一次真正下发成功的脉宽 */
static uint8_t    s_i2c_fault;
static uint32_t   s_last_tick;

/* ---------------- 全局变量 ---------------- */

volatile uint8_t  g_arm_mode = ARM_MODE_REMOTE;
volatile uint16_t g_test_us[ARM_JOINT_COUNT];
volatile int16_t  g_test_deg10[ARM_JOINT_COUNT];

/* ---------------- 遥控门控 ---------------- */

static uint8_t  s_last_flags;
static uint8_t  s_gate_cur;         /* 当前门控状态 */
static int16_t  s_last_axis[4];     /* ly lx ry rx，最近一帧 */
static uint8_t  s_mode_last = ARM_MODE_REMOTE;

/* ---------------- 换算 ---------------- */

/**
 * @brief 脉宽 → 角度
 *
 * 锚点：500 us = 0°，2500 us = deg_range。
 * dir = -1 时反向。
 */
static float arm_us_to_deg(const arm_joint_t *j, uint16_t us)
{
    float t = ((float)us - ARM_US_LO) / ARM_US_SPAN;

    if (t < 0.0f)
    {
        t = 0.0f;
    }
    if (t > 1.0f)
    {
        t = 1.0f;
    }
    if (j->dir < 0)
    {
        t = 1.0f - t;
    }

    return t * (float)j->deg_range;
}

/**
 * @brief 角度 → 脉宽（三层限幅的第 1、2 层）
 *
 *   第 1 层：角度夹到 0 ~ deg_range
 *   第 2 层：脉宽夹到本关节实测的 min_us ~ max_us
 *   第 3 层：驱动里还会再夹到 500 ~ 2500
 */
static uint16_t arm_deg_to_us(const arm_joint_t *j, float deg)
{
    float    t = deg / (float)j->deg_range;
    uint16_t us;

    if (t < 0.0f)
    {
        t = 0.0f;
    }
    if (t > 1.0f)
    {
        t = 1.0f;
    }
    if (j->dir < 0)
    {
        t = 1.0f - t;
    }

    us = (uint16_t)(ARM_US_LO + t * ARM_US_SPAN + 0.5f);

    if (us < j->min_us)
    {
        us = j->min_us;
    }
    if (us > j->max_us)
    {
        us = j->max_us;
    }
    if (us < PCA9685_US_MIN)
    {
        us = PCA9685_US_MIN;
    }
    if (us > PCA9685_US_MAX)
    {
        us = PCA9685_US_MAX;
    }

    return us;
}

/* ---------------- 初始化 ---------------- */

/**
 * @brief 由标定表算出软限位、home 角度，并把台架变量初始化成 home
 *
 * 上电时 g_test_deg10 / g_test_us 就等于 home.
 */
static void arm_cal_init(void)
{
    uint32_t i;

    for (i = 0; i < ARM_JOINT_COUNT; i++)
    {
        const arm_joint_t *j = &arm_cfg[i];
        float a;
        float b;
        float lo;
        float hi;
        float home_deg;

        a = arm_us_to_deg(j, j->min_us);
        b = arm_us_to_deg(j, j->max_us);
        lo = (a < b) ? a : b;
        hi = (a > b) ? a : b;

        s_soft[i].deg_min = lo + ARM_SOFT_MARGIN_DEG;
        s_soft[i].deg_max = hi - ARM_SOFT_MARGIN_DEG;

        /* 行程特别小的关节，收完余量可能反过来，这时就不收 */
        if (s_soft[i].deg_min > s_soft[i].deg_max)
        {
            s_soft[i].deg_min = lo;
            s_soft[i].deg_max = hi;
        }

        home_deg = arm_us_to_deg(j, j->home_us);
        s_home_deg[i] = home_deg;


        if (home_deg < s_soft[i].deg_min)
        {
            s_soft[i].deg_min = home_deg;
            arm_debug_printf("[ARM] ch%u home at lower end, margin relaxed\r\n", (unsigned)arm_cfg[i].ch);
        }
        if (home_deg > s_soft[i].deg_max)
        {
            s_soft[i].deg_max = home_deg;
            arm_debug_printf("[ARM] ch%u home at upper end, margin relaxed\r\n", (unsigned)arm_cfg[i].ch);
        }

        s_pos_deg[i]  = home_deg;
        s_vel_dps[i]  = 0.0f;
        s_tgt_deg[i]  = home_deg;
        s_moving[i]   = 0u;
        s_at_limit[i] = 0u;
        s_last_us[i]  = j->home_us;

        s_req_deg10[i] = (int16_t)(home_deg * 10.0f + 0.5f);

        g_test_us[i]   = j->home_us;
        g_test_deg10[i] = s_req_deg10[i];
    }

    /* 夹爪两个工作角度：由表里的 open_us / close_us 换算，并夹到该关节行程内。
     * 如果表里漏填（0），就退回 home_us，避免把夹爪驱动到 500us 的极限。 */
    {
        uint16_t u;

        u = (arm_cfg[4].open_us != 0u) ? arm_cfg[4].open_us : arm_cfg[4].home_us;
        if (u < arm_cfg[4].min_us)
        {
            u = arm_cfg[4].min_us;
        }
        if (u > arm_cfg[4].max_us)
        {
            u = arm_cfg[4].max_us;
        }
        s_grip_open_deg = arm_us_to_deg(&arm_cfg[4], u);

        u = (arm_cfg[4].close_us != 0u) ? arm_cfg[4].close_us : arm_cfg[4].home_us;
        if (u < arm_cfg[4].min_us)
        {
            u = arm_cfg[4].min_us;
        }
        if (u > arm_cfg[4].max_us)
        {
            u = arm_cfg[4].max_us;
        }
        s_grip_close_deg = arm_us_to_deg(&arm_cfg[4], u);
    }

    s_grip_state = 0u;      /* 上电默认张开 */
}

/**
 * @brief 把 5 路写成 home，其余通道写 1500
 * @return 0 = 全部成功；-1 = 至少一路失败
 */
static int arm_write_home(void)
{
    uint32_t i;
    int      rc = 0;

    for (i = 0; i < ARM_JOINT_COUNT; i++)
    {
        if (pca9685_set_us(arm_cfg[i].ch, arm_cfg[i].home_us) != 0)
        {
            rc = -1;
        }
    }

    /* 没用的通道也写成 1500，避免它们输出 0 宽脉冲 */
    for (i = ARM_JOINT_COUNT; i < PCA9685_CH_COUNT; i++)
    {
        if (pca9685_set_us((uint8_t)i, 1500u) != 0)
        {
            rc = -1;
        }
    }

    return rc;
}

/* 上电归位顺序：夹爪 -> 腕旋转 -> 腕俯仰 -> 大臂 -> 小臂
 * 先动负载小的，大臂尽量后动，避免五个舵机同时启动造成电流冲击。 */
static const uint8_t s_home_order[ARM_JOINT_COUNT] = { 4u, 3u, 2u, 0u, 1u };

/* 遥控按"对孔"时的顺序：腕旋转 -> 腕俯仰 -> 大臂 -> 小臂
 * 不含夹爪：预设过程中夹爪保持当前状态，不参与归位。 */
#define ARM_HOME_PRESET_STEPS   4u
static const uint8_t s_home_order_preset[ARM_HOME_PRESET_STEPS] = { 3u, 2u, 0u, 1u };

/* 每个关节归位之间的间隔（ms） */
#define ARM_HOME_STEP_MS    250u

/**
 * @brief 逐个关节归位（调用前必须先打开输出）
 *
 * 上电时各通道脉宽是 0，舵机没有力矩；
 * 每写一个通道，那一路舵机才开始动，启动电流是错开的。
 */
static int arm_home_sequential(void)
{
    uint32_t i;
    int      rc = 0;

    for (i = 0u; i < ARM_JOINT_COUNT; i++)
    {
        uint8_t j = s_home_order[i];

        if (pca9685_set_us(arm_cfg[j].ch, arm_cfg[j].home_us) != 0)
        {
            rc = -1;
        }

        arm_debug_printf("[ARM] home %u/%u: %s -> %u us\r\n",
                         (unsigned)(i + 1u), (unsigned)ARM_JOINT_COUNT,
                         arm_cfg[j].name, (unsigned)arm_cfg[j].home_us);

        osDelay(ARM_HOME_STEP_MS);
    }

    return rc;
}

/* ---------------- 状态快照 ---------------- */

void arm_get_status(arm_status_t *out)
{
    uint32_t i;

    if (out == 0)
    {
        return;
    }

    for (i = 0; i < ARM_JOINT_COUNT; i++)
    {
        out->pos_deg[i]      = s_pos_deg[i];
        out->soft_min_deg[i] = s_soft[i].deg_min;
        out->soft_max_deg[i] = s_soft[i].deg_max;
        out->last_us[i]      = s_last_us[i];
    }

    out->axis[0] = s_last_axis[0];
    out->axis[1] = s_last_axis[1];
    out->axis[2] = s_last_axis[2];
    out->axis[3] = s_last_axis[3];

    out->grip_open_deg  = s_grip_open_deg;
    out->grip_close_deg = s_grip_close_deg;

    out->mode  = g_arm_mode;
    out->gate  = s_gate_cur;
    out->flags = s_last_flags;
}

void arm_get_joint_cfg(uint32_t idx, arm_joint_cfg_t *out)
{
    if ((out == 0) || (idx >= ARM_JOINT_COUNT))
    {
        return;
    }

    out->ch        = arm_cfg[idx].ch;
    out->min_us    = arm_cfg[idx].min_us;
    out->max_us    = arm_cfg[idx].max_us;
    out->home_us   = arm_cfg[idx].home_us;
    out->deg_range = arm_cfg[idx].deg_range;
    out->dir       = arm_cfg[idx].dir;
    out->speed_dps = arm_cfg[idx].speed_dps;
    out->acc_dps2  = arm_cfg[idx].acc_dps2;
    out->enabled   = arm_cfg[idx].enabled;
    out->name      = arm_cfg[idx].name;
}

/* ---------------- 输出 ---------------- */

/**
 * @brief 只在脉宽变化时才写 I2C，写失败只报一次
 */
static void arm_apply_us(uint32_t i, uint16_t us)
{
    if (us == s_last_us[i])
    {
        return;
    }

    if (pca9685_set_us(arm_cfg[i].ch, us) == 0)
    {
        s_last_us[i] = us;
        s_i2c_fault  = 0u;
    }
    else if (s_i2c_fault == 0u)
    {
        s_i2c_fault = 1u;
        arm_debug_printf("[ARM] ch%u I2C write FAIL, holding last value\r\n", (unsigned)arm_cfg[i].ch);
    }
}

/**
 * @brief 把每个关节的"当前速度"清零
 *
 * 冻结（急停/掉线/切模式）时调用：位置保持不变，但速度归零，
 * 这样恢复控制时是从 0 开始加速，而不是接着上次的速度冲出去。
 */
static void arm_stop_motion(void)
{
    uint32_t k;

    for (k = 0; k < ARM_JOINT_COUNT; k++)
    {
        s_vel_dps[k] = 0.0f;
    }
}

/* ---------------- 台架：脉宽模式 ---------------- */

static void arm_raw_tick(void)
{
    uint32_t i;

    for (i = 0; i < ARM_JOINT_COUNT; i++)
    {
        uint16_t want = g_test_us[i];
        uint16_t applied = want;

        if (applied < PCA9685_US_MIN)
        {
            applied = PCA9685_US_MIN;
        }
        if (applied > PCA9685_US_MAX)
        {
            applied = PCA9685_US_MAX;
        }

        if (applied == s_last_us[i])
        {
            continue;
        }

        if (pca9685_set_us(arm_cfg[i].ch, applied) == 0)
        {
            s_last_us[i] = applied;
            s_i2c_fault  = 0u;

            if (want != applied)
            {
                arm_debug_printf("[ARM] ch%u: %u -> %u us (clamped)\r\n",
                             (unsigned)arm_cfg[i].ch, (unsigned)want, (unsigned)applied);
            }
            else
            {
                arm_debug_printf("[ARM] ch%u -> %u us\r\n",
                             (unsigned)arm_cfg[i].ch, (unsigned)applied);
            }

            /* 手动给的脉宽也算新的命令位置，切回角度模式时不要跳 */
            s_pos_deg[i] = arm_us_to_deg(&arm_cfg[i], applied);
            s_tgt_deg[i] = s_pos_deg[i];
            s_moving[i]  = 0u;
        }
        else if (s_i2c_fault == 0u)
        {
            s_i2c_fault = 1u;
            arm_debug_printf("[ARM] ch%u I2C write FAIL, holding last value\r\n", (unsigned)arm_cfg[i].ch);
        }
    }
}

/* ---------------- 关节运动（公共） ---------------- */

/**
 * @brief 让第 i 个关节朝 s_tgt_deg[i] 走一步，带加减速限制
 *
 * 按"制动曲线"算允许速度：v = sqrt(2 * acc * 剩余距离)。
 * 离得远时受 speed_dps 限制，离得近时自动减速，
 * 这样起步、匀速、刹车都是连续的，不会冲过头也不会抖。
 *
 * 台架角度模式、夹爪、以后的预设姿态都用这一个函数。
 *
 * @return 1 = 已经到位（位置贴住目标且速度为 0）；0 = 还在走
 */
static uint8_t arm_move_toward(uint32_t i, float dt)
{
    const arm_joint_t *j   = &arm_cfg[i];
    float    tgt     = s_tgt_deg[i];
    float    pos     = s_pos_deg[i];
    float    vel     = s_vel_dps[i];
    float    dist    = tgt - pos;
    float    acc     = (float)j->acc_dps2;
    float    vmax    = (float)j->speed_dps;
    float    v_allow = sqrtf(2.0f * acc * fabsf(dist));
    float    v_tgt;

    if (v_allow > vmax)
    {
        v_allow = vmax;
    }

    if (dist > 0.0f)
    {
        v_tgt = v_allow;
    }
    else if (dist < 0.0f)
    {
        v_tgt = -v_allow;
    }
    else
    {
        v_tgt = 0.0f;
    }

    /* 每拍速度最多变化 acc*dt */
    if (v_tgt > (vel + (acc * dt)))
    {
        v_tgt = vel + (acc * dt);
    }
    if (v_tgt < (vel - (acc * dt)))
    {
        v_tgt = vel - (acc * dt);
    }
    vel = v_tgt;

    pos += vel * dt;

    /* 离散步长可能让最后一步越过目标：直接贴住并停车 */
    if (((dist > 0.0f) && (pos > tgt)) ||
        ((dist < 0.0f) && (pos < tgt)))
    {
        pos = tgt;
        vel = 0.0f;
    }

    /* 软限位再保一层 */
    if (pos < s_soft[i].deg_min)
    {
        pos = s_soft[i].deg_min;
        if (vel < 0.0f)
        {
            vel = 0.0f;
        }
    }
    if (pos > s_soft[i].deg_max)
    {
        pos = s_soft[i].deg_max;
        if (vel > 0.0f)
        {
            vel = 0.0f;
        }
    }

    s_pos_deg[i] = pos;
    s_vel_dps[i] = vel;

    arm_apply_us(i, arm_deg_to_us(j, pos));

    return ((pos == tgt) && (vel == 0.0f)) ? 1u : 0u;
}

/* ---------------- 台架：角度模式 ---------------- */

static void arm_goto_tick(float dt)
{
    uint32_t i;

    for (i = 0; i < ARM_JOINT_COUNT; i++)
    {
        const arm_joint_t *j = &arm_cfg[i];
        int16_t req;

        if (j->enabled == 0u)
        {
            continue;
        }

        req = g_test_deg10[i];

        /* 目标变了（比的是整数 0.1°，不会因为浮点误差反复触发） */
        if (req != s_req_deg10[i])
        {
            float t = (float)req / 10.0f;

            s_req_deg10[i] = req;

            if (t < s_soft[i].deg_min)
            {
                t = s_soft[i].deg_min;
            }
            if (t > s_soft[i].deg_max)
            {
                t = s_soft[i].deg_max;
            }

            if (t != ((float)req / 10.0f))
            {
                arm_debug_log_deg(i, "target clamped", t);
                arm_debug_printf("[ARM] ch%u valid %d..%d deg\r\n",
                             (unsigned)arm_cfg[i].ch,
                             (int)(s_soft[i].deg_min + 0.5f),
                             (int)(s_soft[i].deg_max + 0.5f));
            }
            else
            {
                arm_debug_log_deg(i, "target", t);
            }

            s_tgt_deg[i] = t;
        }

        if (arm_move_toward(i, dt) != 0u)
        {
            if (s_moving[i] != 0u)
            {
                s_moving[i] = 0u;
                arm_debug_log_deg(i, "arrived", s_pos_deg[i]);
            }
        }
        else
        {
            s_moving[i] = 1u;
        }
    }
}

/* ---------------- 遥控：固定速度积分 ---------------- */

/**
 * @brief 摇杆控机械臂
 *
 * 只看正负，不看大小：
 *   axis > 0 → 按 speed_dps 往正方向走
 *   axis < 0 → 往反方向走
 *   axis = 0 → 什么都不做（积分停住 = 停在原地）
 *
 * 遥控器已经做好了死区（lx/ry/rx 50，ly 100），
 * 所以"轻推"到这里就是 0，不会慢慢爬。
 */
/**
 * @brief 遥控触发的顺序回 home（非阻塞状态机）
 *
 * 按一下预设键，按 s_home_order 的顺序一个一个回 home；
 * 一次只动一个关节，其余关节停在原地。
 *
 * 必须在 arm_jog_tick 之前调用：如果正在归位，调用方要跳过摇杆。
 */
/**
 * @brief 放矿预设（bit5）：小臂 -> 大臂 -> 腕俯仰 -> 腕旋转
 *
 * 一次只动一个关节，其余保持；夹爪不动；按一次执行一遍。
 * 掉线/急停时由调用方把 s_preset_busy 清 0。
 */
static void arm_preset_tick(const remote_cmd_t *cmd, float dt)
{
    uint8_t  pressed;
    uint8_t  j;
    float    tgt;
    uint32_t i;

    pressed = ((cmd->flags & ARM_PRESET_TRIGGER_BIT) != 0u) ? 1u : 0u;

    /* 上升沿启动（按住不会反复触发） */
    if ((pressed != 0u) && (s_preset_req_last == 0u) && (s_preset_busy == 0u))
    {
        s_preset_busy = 1u;
        s_preset_idx  = 0u;
        arm_debug_printf("[ARM] PRESET start: forearm -> boom -> pitch -> roll\r\n");
    }

    s_preset_req_last = pressed;

    if (s_preset_busy == 0u)
    {
        return;
    }

    j = s_preset_order[s_preset_idx];

    /* 其余关节按住不动 */
    for (i = 0u; i < ARM_JOINT_COUNT; i++)
    {
        if (i != (uint32_t)j)
        {
            s_tgt_deg[i] = s_pos_deg[i];
            s_vel_dps[i] = 0.0f;
        }
    }

    /* 目标：脉宽 -> 角度，再夹到软限位 */
    tgt = arm_us_to_deg(&arm_cfg[j], s_preset_us[s_preset_idx]);

    if (tgt < s_soft[j].deg_min)
    {
        tgt = s_soft[j].deg_min;
    }
    if (tgt > s_soft[j].deg_max)
    {
        tgt = s_soft[j].deg_max;
    }

    s_tgt_deg[j] = tgt;

    if (arm_move_toward(j, dt) != 0u)
    {
        arm_debug_printf("[ARM] PRESET %u/%u done: %s -> %u us\r\n",
                         (unsigned)(s_preset_idx + 1u), (unsigned)ARM_PRESET_STEP_COUNT,
                         arm_cfg[j].name, (unsigned)s_preset_us[s_preset_idx]);

        s_preset_idx++;

        if (s_preset_idx >= (uint8_t)ARM_PRESET_STEP_COUNT)
        {
            s_preset_busy = 0u;
            arm_debug_printf("[ARM] PRESET complete\r\n");
        }
    }
}

static void arm_home_tick(const remote_cmd_t *cmd, float dt)
{
    uint8_t pressed;
    uint8_t j;
    uint32_t i;

    pressed = ((cmd->flags & ARM_HOME_TRIGGER_BIT) != 0u) ? 1u : 0u;

    /* 上升沿启动（按住不会反复触发） */
    if ((pressed != 0u) && (s_home_req_last == 0u) && (s_home_busy == 0u))
    {
        s_home_busy = 1u;
        s_home_idx  = 0u;
            arm_debug_printf("[ARM] HOMING start: roll -> pitch -> boom -> forearm (gripper untouched)\r\n");
    }

    s_home_req_last = pressed;

    if (s_home_busy == 0u)
    {
        return;
    }

    j = s_home_order_preset[s_home_idx];

    /* 其余关节按住不动 */
    for (i = 0u; i < ARM_JOINT_COUNT; i++)
    {
        if (i != (uint32_t)j)
        {
            s_tgt_deg[i] = s_pos_deg[i];
            s_vel_dps[i] = 0.0f;
        }
    }

    /* 当前关节去 home */
    s_tgt_deg[j] = s_home_deg[j];

    if (arm_move_toward(j, dt) != 0u)
    {
        arm_debug_printf("[ARM] HOMING %u/%u done: %s\r\n",
                         (unsigned)(s_home_idx + 1u), (unsigned)ARM_HOME_PRESET_STEPS,
                         arm_cfg[j].name);

        s_home_idx++;

        if (s_home_idx >= (uint8_t)ARM_HOME_PRESET_STEPS)
        {
            s_home_busy = 0u;
            arm_debug_printf("[ARM] HOMING complete\r\n");
        }
    }
}
static void arm_jog_tick(const remote_cmd_t *cmd, float dt)
{
    int16_t  axis[4];
    int32_t  mag[4];
    uint8_t  best = 0u;
    uint32_t i;

    axis[0] = cmd->rx;      /* 大臂     ← 右手左右 */
    axis[1] = cmd->ry;      /* 小臂     ← 右手上下 */
    axis[2] = cmd->lx;      /* 腕俯仰   ← 左手左右 */
    axis[3] = cmd->ly;      /* 腕旋转   ← 左手上下 */

    /* ---- 一次只动一个关节 ----
     * 四个轴取绝对值最大的那一个，其余三个当没推。
     * 被放弃的关节速度按加速度平滑降到 0，位置保持原地。
     */
    for (i = 0u; i < 4u; i++)
    {
        mag[i] = (axis[i] < 0) ? -(int32_t)axis[i] : (int32_t)axis[i];
    }

    for (i = 1u; i < 4u; i++)
    {
        if (mag[i] > mag[best])
        {
            best = (uint8_t)i;
        }
    }

    if (mag[best] == 0)
    {
        /* 四个轴都在死区里：全部停 */
        if (s_active_axis != ARM_AXIS_NONE)
        {
            s_active_axis = ARM_AXIS_NONE;
            arm_debug_printf("[ARM] move: idle\r\n");
        }

        for (i = 0u; i < 4u; i++)
        {
            axis[i] = 0;
        }
    }
    else
    {
        /* 滞回：当前关节只要还有最大值的 85%，就不换 */
        if ((s_active_axis < 4u) &&
            (mag[s_active_axis] * 100 >= mag[best] * ARM_AXIS_KEEP_PCT))
        {
            best = s_active_axis;
        }

        if (best != s_active_axis)
        {
            s_active_axis = best;
            arm_debug_printf("[ARM] move: %s (ly=%d lx=%d ry=%d rx=%d)\r\n",
                             arm_cfg[best].name,
                             (int)axis[0], (int)axis[1],
                             (int)axis[2], (int)axis[3]);
        }

        /* 只留最大的那个，其余清零 */
        for (i = 0u; i < 4u; i++)
        {
            if (i != best)
            {
                axis[i] = 0;
            }
        }
    }

    for (i = 0; i < 4u; i++)
    {
        const arm_joint_t *j = &arm_cfg[i];
        float    pos = s_pos_deg[i];
        float    dirf = 0.0f;
        uint16_t us;

        if (j->enabled == 0u)
        {
            continue;
        }

        if (axis[i] > 0)
        {
            dirf = (float)arm_axis_dir[i];
        }
        else if (axis[i] < 0)
        {
            dirf = -(float)arm_axis_dir[i];
        }
        else
        {
            /* 松手：目标速度 0，按加速度平滑刹停 */
            dirf = 0.0f;
        }

        /* ---- 加减速限制 ----
         * 摇杆只给"目标速度的方向"，实际速度每拍最多变化 acc*dt。
         * 启动/停止/换向都不再是阶跃，电流冲击小很多。
         */
        {
            float acc   = (float)j->acc_dps2;
            float v_tgt = dirf * (float)j->speed_dps;
            float vel   = s_vel_dps[i];

            if (v_tgt > (vel + (acc * dt)))
            {
                v_tgt = vel + (acc * dt);
            }
            if (v_tgt < (vel - (acc * dt)))
            {
                v_tgt = vel - (acc * dt);
            }
            vel = v_tgt;

            pos += vel * dt;

            /* 顶到限位：位置夹住 + 速度清零" */
            if (pos < s_soft[i].deg_min)
            {
                pos = s_soft[i].deg_min;
                if (vel < 0.0f)
                {
                    vel = 0.0f;
                }
            }
            if (pos > s_soft[i].deg_max)
            {
                pos = s_soft[i].deg_max;
                if (vel > 0.0f)
                {
                    vel = 0.0f;
                }
            }

            s_vel_dps[i] = vel;
        }

        /* 顶到限位提示一次 */
        if (((dirf > 0.0f) && (pos >= s_soft[i].deg_max - ARM_LIMIT_EPS_DEG)) ||
            ((dirf < 0.0f) && (pos <= s_soft[i].deg_min + ARM_LIMIT_EPS_DEG)))
        {
            if (s_at_limit[i] == 0u)
            {
                s_at_limit[i] = 1u;
                arm_debug_printf("[ARM] ch%u at soft limit %d deg\r\n",
                                 (unsigned)arm_cfg[i].ch, (int)pos);
            }
        }
        else
        {
            s_at_limit[i] = 0u;
        }

        s_pos_deg[i] = pos;

        us = arm_deg_to_us(j, pos);
        arm_apply_us(i, us);
    }
}

/* ---------------- 遥控：夹爪 ---------------- */

/**
 * @brief 夹爪：遥控帧 bit1 直接决定目标位置（1 = 夹紧）
 *
 * 遥控器发的是"锁存电平"，不是按键事件：
 *   按下 RIGHT_X 短按 -> bit1 翻一次，之后一直保持
 */
static void arm_grip_tick(const remote_cmd_t *cmd, float dt)
{
    uint8_t want;

    if (arm_cfg[4].enabled == 0u)
    {
        return;
    }

    want = ((cmd->flags & RCMD_GRIPPER) != 0u) ? 1u : 0u;

    if (want != s_grip_state)
    {
        s_grip_state = want;
        arm_debug_printf("[ARM] gripper -> %s\r\n", (want != 0u) ? "close" : "open");
    }

    s_tgt_deg[4] = (want != 0u) ? s_grip_close_deg : s_grip_open_deg;

    (void)arm_move_toward(4u, dt);
}

/* ---------------- 门控 ---------------- */

static arm_gate_t arm_gate_check(const remote_cmd_t *cmd)
{
    if (g_local_estop != 0u)
    {
        return ARM_GATE_ESTOP;
    }
    if (g_link_lost != 0u)
    {
        return ARM_GATE_NO_LINK;
    }
    if (cmd == 0)
    {
        return ARM_GATE_NO_CMD;
    }
    if ((cmd->flags & RCMD_ESTOP) != 0u)
    {
        return ARM_GATE_ESTOP;
    }
    if ((cmd->flags & RCMD_MODE_ARM) == 0u)
    {
        return ARM_GATE_MODE;
    }

    return ARM_GATE_OK;
}

/* ---------------- 任务 ---------------- */

void StartArmTask(void const *argument)
{
    uint8_t  mode1 = 0;
    uint8_t  prescale = 0;
    uint8_t  found[ARM_SCAN_MAX];
    uint32_t attempt;
    uint32_t i;
    int      n;
    int      ok = -1;

    (void)argument;

    /* ---------------- 1. 探测 ---------------- */

    arm_debug_printf("\r\n[ARM] probe PCA9685 on I2C2 (PB10/PB11, 0x40)\r\n");

    for (attempt = 0; attempt < ARM_PROBE_RETRY; attempt++)
    {
        ok = pca9685_detect();
        if (ok == 0)
        {
            break;
        }
        osDelay(ARM_PROBE_GAP_MS);
    }

    if (ok != 0)
    {
        arm_debug_printf("[ARM] PCA9685 NOT FOUND\r\n");
        arm_debug_printf("[ARM] check: VCC=3.3V, GND common, SCL=PB10, SDA=PB11\r\n");

        n = pca9685_scan(found, ARM_SCAN_MAX);
        if (n == 0)
        {
            arm_debug_printf("[ARM] scan: no device answers on I2C2 at all\r\n");
        }
        else
        {
            arm_debug_printf("[ARM] scan: %d device(s):", n);
            for (i = 0; i < (uint32_t)n; i++)
            {
                arm_debug_printf(" 0x%02X", found[i]);
            }
            arm_debug_printf("\r\n");
        }

        for (;;)
        {
            osDelay(1000);
        }
    }

    /* ---------------- 2. 初始化 ---------------- */

    if (pca9685_init() != 0)
    {
        (void)pca9685_read_reg(PCA9685_REG_MODE1, &mode1);
        (void)pca9685_read_reg(PCA9685_REG_PRESCALE, &prescale);
        arm_debug_printf("[ARM] PCA9685 init FAIL, MODE1=0x%02X, PRESCALE=%u\r\n",
                     mode1, (unsigned)prescale);
        arm_debug_printf("[ARM] expect MODE1 bit4(SLEEP)=0, bit5(AI)=1, PRESCALE=121\r\n");

        for (;;)
        {
            osDelay(1000);
        }
    }

    (void)pca9685_read_reg(PCA9685_REG_MODE1, &mode1);
    (void)pca9685_read_reg(PCA9685_REG_PRESCALE, &prescale);
    arm_debug_printf("[ARM] PCA9685 init OK, MODE1=0x%02X, PRESCALE=%u (50 Hz)\r\n",
                 mode1, (unsigned)prescale);

    /* ---------------- 3. 标定表 + 写 home + 开输出 ---------------- */

    arm_cal_init();

    if (arm_write_home() != 0)
    {
        arm_debug_printf("[ARM] write home FAIL, OE stays HIGH\r\n");
        for (;;)
        {
            osDelay(1000);
        }
    }

    /* ---- 先打开输出：各通道脉宽还是 0，舵机没有力矩 ---- */
    pca9685_output_enable();

    arm_debug_printf("[ARM] OE=LOW (outputs %s), homing one by one\r\n",
                     pca9685_output_enabled() ? "enabled" : "disabled");

    /* ---- 逐个归位：夹爪 -> 腕旋转 -> 腕俯仰 -> 大臂 -> 小臂 ---- */
    if (arm_home_sequential() != 0)
    {
        arm_debug_printf("[ARM] sequential home had I2C errors\r\n");
    }

    arm_debug_printf("[ARM] home done, all joints at home position\r\n");

    arm_debug_print_cfg();

    arm_debug_printf("[ARM] axes: ly=wrist_roll  lx=wrist_pitch  ry=forearm  rx=boom\r\n");

    if (g_arm_mode == ARM_MODE_REMOTE)
    {
        arm_debug_printf("[ARM] mode=0 (remote): switch remote to ARM mode to move\r\n");
    }
    else if (g_arm_mode == ARM_MODE_US)
    {
        arm_debug_printf("[ARM] mode=1 (bench us): edit g_test_us[0..4]\r\n");
    }
    else
    {
        arm_debug_printf("[ARM] mode=2 (bench deg): edit g_test_deg10[0..4], unit 0.1 deg\r\n");
    }

    /* ---------------- 4. 主循环 ---------------- */

    s_last_tick = osKernelSysTick();

    for (;;)
    {
        uint32_t now = osKernelSysTick();
        float    dt  = (float)(now - s_last_tick) * 0.001f;

        s_last_tick = now;

        if (dt <= 0.0f)
        {
            dt = 0.001f;
        }
        if (dt > 0.050f)
        {
            dt = 0.050f;
        }

        /* 心跳/面包屑都在 arm_debug.c 里，关掉时是空宏 */
        ARM_CRUMB(1);
        arm_debug_tick();

        /* 模式切换：打印一行，并把台架变量同步到"当前位置"，
         * 避免从遥控/脉宽模式切到角度模式时关节突然跳过去。 */
        if (g_arm_mode != s_mode_last)
        {
            uint32_t k;

            if (g_arm_mode == ARM_MODE_DEG)
            {
                for (k = 0; k < ARM_JOINT_COUNT; k++)
                {
                    s_tgt_deg[k]    = s_pos_deg[k];
                    s_req_deg10[k]  = (int16_t)(s_pos_deg[k] * 10.0f + 0.5f);
                    g_test_deg10[k] = s_req_deg10[k];
                }
                arm_debug_printf("[ARM] mode -> 2 (deg), target = current position\r\n");
            }
            else if (g_arm_mode == ARM_MODE_US)
            {
                for (k = 0; k < ARM_JOINT_COUNT; k++)
                {
                    g_test_us[k] = s_last_us[k];
                }
                arm_debug_printf("[ARM] mode -> 1 (us), values = current pulse\r\n");
            }
            else
            {
                arm_debug_printf("[ARM] mode -> 0 (remote)\r\n");
            }

            s_mode_last = g_arm_mode;

            /* 切模式时速度归零，避免带着上一个模式的速度冲出去 */
            arm_stop_motion();
        }

        ARM_CRUMB(3);

        /* 无论什么模式都取一次最新帧，这样心跳里的轴值永远是实时的 */
        {
            remote_cmd_t cmd = {0};
            BaseType_t   got = xQueuePeek(q_remote_cmd, &cmd, 0);

            ARM_CRUMB(4);

            if (got == pdTRUE)
            {
                s_last_flags   = cmd.flags;
                s_last_axis[0] = cmd.ly;
                s_last_axis[1] = cmd.lx;
                s_last_axis[2] = cmd.ry;
                s_last_axis[3] = cmd.rx;
            }

            if (g_arm_mode == ARM_MODE_US)
            {
                arm_raw_tick();
            }
            else if (g_arm_mode == ARM_MODE_DEG)
            {
                arm_goto_tick(dt);
            }
            else
            {
                arm_gate_t gate = arm_gate_check((got == pdTRUE) ? &cmd : 0);

                ARM_CRUMB(5);

                s_gate_cur = (uint8_t)gate;
                arm_debug_report_gate(gate, s_last_flags);

                ARM_CRUMB(6);

                if (gate == ARM_GATE_OK)
                {
                    /* 预设优先：放矿按键 -> 顺序回 home -> 摇杆 */
                    arm_preset_tick(&cmd, dt);

                    if (s_preset_busy == 0u)
                    {
                        arm_home_tick(&cmd, dt);
                    }

                    if ((s_home_busy == 0u) && (s_preset_busy == 0u))
                    {
                        arm_jog_tick(&cmd, dt);     /* 前 4 路：摇杆 */
                        arm_grip_tick(&cmd, dt);    /* 第 5 路：bit1 */
                    }
                }
                else
                {
                    s_home_busy   = 0u;            /* 掉线/急停时放弃归位和预设 */
                    s_preset_busy = 0u;
                    arm_stop_motion();
                }

                ARM_CRUMB(7);
            }
        }

        ARM_CRUMB(8);
        osDelay(ARM_LOOP_MS);
        ARM_CRUMB(9);
    }
}
