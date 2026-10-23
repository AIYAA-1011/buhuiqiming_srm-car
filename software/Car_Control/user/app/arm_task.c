//
// 机械臂任务
//
// 第 1 步：探测 PCA9685
// 第 2 步：初始化 50 Hz / 设脉宽 / 开关输出
// 第 3 步：台架测试（直接给脉宽）
// 第 5 步：标定表 + 角度换算 + 软限幅 + 匀速走到目标
//
// 上电 → 探测 → 初始化 → 按标定表把 5 路写成 home → 等 50 ms → 开 OE
// 然后进入台架模式：
//   g_test_mode=2（默认）改 g_test_deg[i]，关节按 speed_dps 匀速走过去
//   g_test_mode=1        改 g_test_us[i]，直接给脉宽（标定用）
//   g_test_mode=0        冻结，不再下发
//
// 这个版本没有接遥控器：遥控器发什么都不影响机械臂。
//

#include "arm_task.h"
#include "pca9685.h"
#include "debug.h"
#include "cmsis_os.h"

/* 上电默认进入哪个台架模式 */
#define ARM_BOOT_TEST_MODE      ARM_TEST_DEG

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

/* 软限位在实测机械限位的基础上再往里收一点，别贴着限位停 */
#define ARM_SOFT_MARGIN_DEG     2.0f

/* ---------------- 标定表 ----------------
 *
 * 只改这一张表。改完重新编译，别的都不用动。
 *
 *   min_us / max_us : 实测的机械行程端点
 *   home_us         : 上电收拢姿态
 *   deg_range       : 舵机规格 180 或 270
 *   dir             : +1 / -1，角度变大时脉宽往哪边走
 *   speed_dps       : 走目标时的角速度（°/s），别低于 30
 *   enabled         : 0 = 还没标定，禁止动作（夹爪现在就是这个状态）
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
    uint8_t     enabled;
    const char *name;
} arm_joint_t;

static const arm_joint_t arm_cfg[ARM_JOINT_COUNT] =
{
    /* ch   min_us  max_us  home   range  dir  dps   en  name          */
    { 0u,  1460u,  2500u,  1460u,  270u,  +1,  60u,  1u, "boom"        },
    { 1u,   700u,  2500u,   700u,  270u,  +1,  60u,  1u, "forearm"     },
    { 2u,   500u,  2500u,  1600u,  180u,  +1,  40u,  1u, "wrist_pitch" },
    { 3u,   500u,  2500u,  1900u,  180u,  +1,  40u,  1u, "wrist_roll"  },
    { 4u,   500u,  2500u,  1500u,  180u,  +1,  90u,  0u, "gripper"     },  /* 还没标定 */
};

/* ---------------- 运行时状态 ---------------- */

typedef struct
{
    float deg_min;
    float deg_max;
} arm_soft_t;

static arm_soft_t s_soft[ARM_JOINT_COUNT];
static float      s_pos_deg[ARM_JOINT_COUNT];   /* 当前命令位置 */
static float      s_tgt_deg[ARM_JOINT_COUNT];   /* 目标位置 */
static uint8_t    s_moving[ARM_JOINT_COUNT];    /* 正在走（用于打印 arrived） */
static uint16_t   s_last_us[ARM_JOINT_COUNT];   /* 上一次真正下发成功的脉宽 */
static uint8_t    s_i2c_fault;
static uint32_t   s_last_tick;

/* ---------------- 台架变量（Ozone 改） ---------------- */

volatile uint8_t  g_test_mode = ARM_BOOT_TEST_MODE;
volatile uint16_t g_test_us[ARM_JOINT_COUNT];
volatile float    g_test_deg[ARM_JOINT_COUNT];

/* ---------------- 换算 ---------------- */

/**
 * @brief 脉宽 → 角度
 *
 * 锚点：500 us = 0°，2500 us = deg_range。
 * dir = -1 时反向，这样"角度变大"永远等于"关节往正方向动"。
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
 * 这样上电时 g_test_deg / g_test_us 就等于 home，
 * 不会因为默认值 0 让关节一上电就飞过去。
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

        /* home 是人为选定的姿态，优先保证它可达。
         * 如果 home 正好在机械端点（比如大臂 1460 = min_us），
         * 留出来的余量会让 home 落到软限外 —— 这时把软限放宽到 home，
         * 而不是把 home 夹进来（夹进来会导致上电就动一下）。 */
        if (home_deg < s_soft[i].deg_min)
        {
            s_soft[i].deg_min = home_deg;
            debug_printf("[ARM] ch%u home at lower end, margin relaxed\r\n", (unsigned)i);
        }
        if (home_deg > s_soft[i].deg_max)
        {
            s_soft[i].deg_max = home_deg;
            debug_printf("[ARM] ch%u home at upper end, margin relaxed\r\n", (unsigned)i);
        }

        s_pos_deg[i] = home_deg;
        s_tgt_deg[i] = home_deg;
        s_moving[i]  = 0u;
        s_last_us[i] = j->home_us;

        g_test_us[i]  = j->home_us;
        g_test_deg[i] = home_deg;
    }
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

static void arm_print_cfg(void)
{
    uint32_t i;

    for (i = 0; i < ARM_JOINT_COUNT; i++)
    {
        const arm_joint_t *j = &arm_cfg[i];

        debug_printf("[ARM] ch%u %-11s us %4u..%4u home %4u  deg %3d..%3d (home %3d) dir%s %udps %s\r\n",
                     (unsigned)i,
                     j->name,
                     (unsigned)j->min_us,
                     (unsigned)j->max_us,
                     (unsigned)j->home_us,
                     (int)(s_soft[i].deg_min + 0.5f),
                     (int)(s_soft[i].deg_max + 0.5f),
                     (int)(s_pos_deg[i] + 0.5f),
                     (j->dir > 0) ? "+1" : "-1",
                     (unsigned)j->speed_dps,
                     j->enabled ? "on" : "DISABLED");
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
                debug_printf("[ARM] ch%u: %u -> %u us (clamped)\r\n",
                             (unsigned)i, (unsigned)want, (unsigned)applied);
            }
            else
            {
                debug_printf("[ARM] ch%u -> %u us\r\n",
                             (unsigned)i, (unsigned)applied);
            }

            /* 手动给的脉宽也算新的命令位置，切回角度模式时不要跳 */
            s_pos_deg[i] = arm_us_to_deg(&arm_cfg[i], applied);
            s_tgt_deg[i] = s_pos_deg[i];
            s_moving[i]  = 0u;
        }
        else if (s_i2c_fault == 0u)
        {
            s_i2c_fault = 1u;
            debug_printf("[ARM] ch%u I2C write FAIL, holding last value\r\n", (unsigned)i);
        }
    }
}

/* ---------------- 台架：角度模式 ---------------- */

static void arm_goto_tick(float dt)
{
    uint32_t i;

    for (i = 0; i < ARM_JOINT_COUNT; i++)
    {
        const arm_joint_t *j = &arm_cfg[i];
        float    tgt;
        float    pos;
        float    step;
        uint16_t us;

        if (j->enabled == 0u)
        {
            continue;
        }

        tgt = g_test_deg[i];

        /* 目标变了：先夹软限，再打印 */
        if (tgt != s_tgt_deg[i])
        {
            float t = tgt;

            if (t < s_soft[i].deg_min)
            {
                t = s_soft[i].deg_min;
            }
            if (t > s_soft[i].deg_max)
            {
                t = s_soft[i].deg_max;
            }

            if (t != tgt)
            {
                debug_printf("[ARM] ch%u target %d deg -> clamped %d deg (valid %d..%d)\r\n",
                             (unsigned)i, (int)tgt, (int)t,
                             (int)(s_soft[i].deg_min + 0.5f),
                             (int)(s_soft[i].deg_max + 0.5f));
            }
            else
            {
                debug_printf("[ARM] ch%u target %d deg\r\n", (unsigned)i, (int)tgt);
            }

            s_tgt_deg[i] = t;
        }

        pos  = s_pos_deg[i];
        step = (float)j->speed_dps * dt;

        if (pos < s_tgt_deg[i])
        {
            pos += step;
            if (pos > s_tgt_deg[i])
            {
                pos = s_tgt_deg[i];
            }
            s_moving[i] = 1u;
        }
        else if (pos > s_tgt_deg[i])
        {
            pos -= step;
            if (pos < s_tgt_deg[i])
            {
                pos = s_tgt_deg[i];
            }
            s_moving[i] = 1u;
        }
        else if (s_moving[i] != 0u)
        {
            s_moving[i] = 0u;
            debug_printf("[ARM] ch%u arrived %d deg\r\n", (unsigned)i, (int)pos);
        }

        /* 软限位再保一层 */
        if (pos < s_soft[i].deg_min)
        {
            pos = s_soft[i].deg_min;
        }
        if (pos > s_soft[i].deg_max)
        {
            pos = s_soft[i].deg_max;
        }
        s_pos_deg[i] = pos;

        us = arm_deg_to_us(j, pos);
        if (us != s_last_us[i])
        {
            if (pca9685_set_us(j->ch, us) == 0)
            {
                s_last_us[i] = us;
                s_i2c_fault  = 0u;
            }
            else if (s_i2c_fault == 0u)
            {
                s_i2c_fault = 1u;
                debug_printf("[ARM] ch%u I2C write FAIL, holding last value\r\n", (unsigned)i);
            }
        }
    }
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

    debug_printf("\r\n[ARM] probe PCA9685 on I2C2 (PB10/PB11, 0x40)\r\n");

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
        debug_printf("[ARM] PCA9685 NOT FOUND\r\n");
        debug_printf("[ARM] check: VCC=3.3V, GND common, SCL=PB10, SDA=PB11\r\n");

        n = pca9685_scan(found, ARM_SCAN_MAX);
        if (n == 0)
        {
            debug_printf("[ARM] scan: no device answers on I2C2 at all\r\n");
        }
        else
        {
            debug_printf("[ARM] scan: %d device(s):", n);
            for (i = 0; i < (uint32_t)n; i++)
            {
                debug_printf(" 0x%02X", found[i]);
            }
            debug_printf("\r\n");
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
        debug_printf("[ARM] PCA9685 init FAIL, MODE1=0x%02X, PRESCALE=%u\r\n",
                     mode1, (unsigned)prescale);
        debug_printf("[ARM] expect MODE1 bit4(SLEEP)=0, bit5(AI)=1, PRESCALE=121\r\n");

        for (;;)
        {
            osDelay(1000);
        }
    }

    (void)pca9685_read_reg(PCA9685_REG_MODE1, &mode1);
    (void)pca9685_read_reg(PCA9685_REG_PRESCALE, &prescale);
    debug_printf("[ARM] PCA9685 init OK, MODE1=0x%02X, PRESCALE=%u (50 Hz)\r\n",
                 mode1, (unsigned)prescale);

    /* ---------------- 3. 标定表 + 写 home + 开输出 ---------------- */

    arm_cal_init();

    if (arm_write_home() != 0)
    {
        debug_printf("[ARM] write home FAIL, OE stays HIGH\r\n");
        for (;;)
        {
            osDelay(1000);
        }
    }

    /* 等至少两个 PWM 周期，让每个通道都真的输出 home 脉宽 */
    osDelay(ARM_ENABLE_GAP_MS);

    pca9685_output_enable();

    debug_printf("[ARM] home written, OE=LOW (outputs %s)\r\n",
                 pca9685_output_enabled() ? "enabled" : "disabled");
    debug_printf("[ARM] arm should already be at home; if not it will snap there now\r\n");

    arm_print_cfg();

    if (g_test_mode == ARM_TEST_US)
    {
        debug_printf("[ARM] mode=1 (us):   edit g_test_us[0..4] in Ozone\r\n");
    }
    else if (g_test_mode == ARM_TEST_DEG)
    {
        debug_printf("[ARM] mode=2 (deg):  edit g_test_deg[0..4] in Ozone, moves at speed_dps\r\n");
    }
    else
    {
        debug_printf("[ARM] mode=0 (freeze): nothing will move\r\n");
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

        if (g_test_mode == ARM_TEST_US)
        {
            arm_raw_tick();
        }
        else if (g_test_mode == ARM_TEST_DEG)
        {
            arm_goto_tick(dt);
        }
        else
        {
            /* 冻结：什么都不做，PCA9685 保持最后的脉宽 */
        }

        osDelay(ARM_LOOP_MS);
    }
}
