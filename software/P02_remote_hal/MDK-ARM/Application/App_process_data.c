#include "App_process_data.h"

/* 摇杆原始 ADC 值（由 DMA 直接写进 adc_buff，这里取出来） */
Joystick_Struct joystick = {0};

/* 处理后的四个轴命令值：-500 ~ +500，0 = 中位 / 停止 */
int16_t axis_value[4] = {0};

/* ==================== 中位保存到 Flash ====================
 * STM32F103C8 是 64KB Flash、页大小 1KB。
 * 程序只占 21KB 左右，最后 1 页（0x0800FC00 ~ 0x0800FFFF）是空的，
 * 用来存每个轴的【中位】和【两端点】（都是原始 ADC），
 * 这样断电重开不用重新校准、也不用重新推杆学端点。
 *
 * 存放格式（14 个半字）：
 *   [0]     magic
 *   [1..3]  ly: 中位 / 见过的最小 raw / 见过的最大 raw
 *   [4..6]  lx: 同上
 *   [7..9]  ry: 同上
 *   [10..12] rx: 同上
 *   [13]    校验和 = [0]~[12] 逐项异或
 *
 * magic 变迁：
 *   0x5A5A  最初的"命令域零偏"格式（数值很小，通常几十）
 *   0x5A5B  改成"原始 ADC 中位"（6 个半字）
 *   0x5A5C  再改成"中位 + 两端点"（14 个半字）
 * 不换 magic，旧数据会被当成新格式读进来，算出乱七八糟的量程。
 * 所以每次升级后第一次上电都会要求重新校准一次，这是预期行为。
 *
 * 擦除后的 Flash 全是 0xFFFF，magic 对不上就当没有数据。
 * 注意：如果 Keil 下载时选了"Erase Full Chip"，这一页会被一起擦掉，
 *       烧完第一次上电需要重新校准一次，属正常现象。
 * ======================================================== */
#define CALIB_FLASH_ADDR 0x0800FC00UL
#define CALIB_MAGIC      0x5A5C
#define CALIB_WORDS      14
/* 自学习记录的极值，静默这么多拍（100 x 20ms = 2 秒）之后写一次 Flash */
#define AXIS_END_SAVE_QUIET 100

/* 每个轴的运行时参数 */
typedef struct
{
    uint8_t inv;     /* 极性：1 = 反相 */
    int16_t dz;      /* 死区半宽（对称） */
    int16_t raw_mid; /* 中位原始 ADC，由校准得到（或从 Flash 读回） */
    int16_t raw_lo;  /* 生效的正方向端点（由极值加护栏算出） */
    int16_t raw_hi;  /* 生效的负方向端点（由极值加护栏算出） */
} Axis_Cfg;

static Axis_Cfg s_axis[4] =
{
    { AXIS_INV_LY, AXIS_DEADZONE_LY, AXIS_MID_NOMINAL, AXIS_END_LO_DEFAULT, AXIS_END_HI_DEFAULT },
    { AXIS_INV_LX, AXIS_DEADZONE, AXIS_MID_NOMINAL, AXIS_END_LO_DEFAULT, AXIS_END_HI_DEFAULT },
    { AXIS_INV_RY, AXIS_DEADZONE, AXIS_MID_NOMINAL, AXIS_END_LO_DEFAULT, AXIS_END_HI_DEFAULT },
    { AXIS_INV_RX, AXIS_DEADZONE, AXIS_MID_NOMINAL, AXIS_END_LO_DEFAULT, AXIS_END_HI_DEFAULT },
};

/* 自学习：本机真的见过的极值。和生效端点 raw_lo / raw_hi 分开存，
 * 因为"保底值"可能比实测值更极端（保底 lo=0，而实测只到 300），
 * 直接拿保底值去做 min 是永远学不到东西的。 */
static int16_t s_seen_lo[4] = { 4095, 4095, 4095, 4095 };
static int16_t s_seen_hi[4] = { 0, 0, 0, 0 };

static uint16_t s_warmup    = 0; /* 开机预热计数 */
static uint8_t  s_have_calib = 0; /* 是否已经成功校准过（没校准前不自动写 Flash） */
static uint8_t  s_end_dirty  = 0; /* 学到新极值，待写 Flash */
static uint16_t s_end_quiet  = 0; /* 距上次学到新极值过了多少拍 */

/**
 * @brief 原始 ADC -> -500 ~ +500（以标定中位为锚点，两侧各自归一化到实测端点）
 *
 *   正方向： raw_mid -> raw_lo   映射到   0 -> +500
 *   负方向： raw_mid -> raw_hi   映射到   0 -> -500
 *
 * 为什么不能沿用"先按整量程换算成 ±500、再减一个零偏"：
 * 那种做法只能把整条曲线平移，中位一偏就有一侧被白白吃掉量程。
 * 实测中位 1974 时正半轴只能到 +480，而负半轴被限幅削到 -500，
 * 看起来反而"正好满"，很容易误判成没问题。
 *
 * 但光是"两侧各自归一化"还不够，端点也不能写死成 0 / 4095。
 * 电位器不一定真能推到 ADC 的两条轨上——实测本机遥控器：
 *   正方向 raw 能到 0（饱和）
 *   负方向 raw 只到 4020 左右
 * 端点写死 4095 的话，负方向最多只能到 -480。所以端点用实测自学习值。
 */
static int16_t axis_raw_to_command(int16_t raw, const Axis_Cfg *cfg)
{
    int32_t v;

    if (raw <= cfg->raw_mid)
    {
        /* ---- 正方向侧 ---- */
        int32_t den = (int32_t)cfg->raw_mid - cfg->raw_lo;

        if (den <= 0)
        {
            return AXIS_SPAN; /* 端点异常，防除零 */
        }
        v = (int32_t)(cfg->raw_mid - raw) * AXIS_SPAN / den;
    }
    else
    {
        /* ---- 负方向侧 ---- */
        int32_t den = (int32_t)cfg->raw_hi - cfg->raw_mid;

        if (den <= 0)
        {
            return -AXIS_SPAN; /* 端点异常，防除零 */
        }
        v = -((int32_t)(raw - cfg->raw_mid) * AXIS_SPAN) / den;
    }

    if (cfg->inv)
    {
        v = -v; /* 极性修正 */
    }

    return Com_limit((int16_t)v, -AXIS_SPAN, AXIS_SPAN); /* 限幅 */
}

/**
 * @brief 摇杆结构体 -> 数组，顺序 ly / lx / ry / rx
 */
static void axis_to_array(const Joystick_Struct *js, int16_t *raw)
{
    raw[0] = js->ly;
    raw[1] = js->lx;
    raw[2] = js->ry;
    raw[3] = js->rx;
}

/**
 * @brief 单个轴的处理
 */
static int16_t axis_process(int16_t raw, const Axis_Cfg *cfg)
{
    int16_t v = axis_raw_to_command(raw, cfg);

    if (v > -cfg->dz && v < cfg->dz)
    {
        return 0; /* 死区内 -> 停止 */
    }

    /* 死区外重新拉伸：让输出从 0 平滑增长，
     * 否则死区边缘会从 0 直接跳到 dz（10% 速度），精细操作很难受 */
    if (v > 0)
    {
        return (int16_t)((v - cfg->dz) * AXIS_SPAN / (AXIS_SPAN - cfg->dz));
    }
    return (int16_t)((v + cfg->dz) * AXIS_SPAN / (AXIS_SPAN - cfg->dz));
}

/**
 * @brief 从 Flash 读回中位原始 ADC
 */
uint8_t App_calib_load(void)
{
    const uint16_t *flash = (const uint16_t *)CALIB_FLASH_ADDR;
    uint16_t chk = 0;
    int16_t mid[4];
    int16_t lo[4];
    int16_t hi[4];
    uint8_t k;

    for (k = 0; k < (uint8_t)(CALIB_WORDS - 1); k++)
    {
        chk ^= flash[k];
    }

    if (flash[0] != CALIB_MAGIC || flash[CALIB_WORDS - 1] != chk)
    {
        debug_printf("CALIB: flash 无有效数据，需要校准\r\n");
        return 0;
    }

    /* 范围检查：校验和只能挡住"位翻转"，
     * 挡不住"上次存进去的本来就是个离谱的值" */
    for (k = 0; k < 4; k++)
    {
        mid[k] = (int16_t)flash[1 + k * 3];
        lo[k]  = (int16_t)flash[2 + k * 3];
        hi[k]  = (int16_t)flash[3 + k * 3];

        /* 生效端点是"极值 + 护栏"算出来的，这里一起校验 */
        {
            int16_t e_lo = (lo[k] <= AXIS_END_LO_MAX)   ? lo[k] : AXIS_END_LO_DEFAULT;
            int16_t e_hi = (hi[k] >= AXIS_END_HI_MIN)   ? hi[k] : AXIS_END_HI_DEFAULT;

            if (mid[k] < AXIS_MID_MIN || mid[k] > AXIS_MID_MAX ||
                (mid[k] - e_lo) < AXIS_MIN_HALF_RANGE ||
                (e_hi - mid[k]) < AXIS_MIN_HALF_RANGE)
            {
                debug_printf("CALIB: flash 数据越界 (axis %d mid=%d lo=%d hi=%d)，需要重新校准\r\n",
                             k, mid[k], lo[k], hi[k]);
                return 0;
            }
        }
    }

    taskENTER_CRITICAL();
    for (k = 0; k < 4; k++)
    {
        s_axis[k].raw_mid = mid[k];
        s_axis[k].raw_lo  = (lo[k] <= AXIS_END_LO_MAX) ? lo[k] : AXIS_END_LO_DEFAULT;
        s_axis[k].raw_hi  = (hi[k] >= AXIS_END_HI_MIN) ? hi[k] : AXIS_END_HI_DEFAULT;

        /* 自学习从存下来的极值继续往下学 */
        s_seen_lo[k] = lo[k];
        s_seen_hi[k] = hi[k];
    }
    s_have_calib = 1;
    taskEXIT_CRITICAL();

    debug_printf("CALIB LOAD: ly mid=%d lo=%d hi=%d | lx mid=%d lo=%d hi=%d\r\n",
                 s_axis[0].raw_mid, s_axis[0].raw_lo, s_axis[0].raw_hi,
                 s_axis[1].raw_mid, s_axis[1].raw_lo, s_axis[1].raw_hi);
    debug_printf("            ry mid=%d lo=%d hi=%d | rx mid=%d lo=%d hi=%d\r\n",
                 s_axis[2].raw_mid, s_axis[2].raw_lo, s_axis[2].raw_hi,
                 s_axis[3].raw_mid, s_axis[3].raw_lo, s_axis[3].raw_hi);
    return 1;
}

/**
 * @brief 把当前中位（原始 ADC）写进 Flash
 *
 * 注意：擦除和写入期间 Flash 不可读，CPU 会停顿几十毫秒。
 *       这是只在手动校准后发生的一次性操作，可以接受。
 */
static void calib_save(void)
{
    const uint16_t *flash = (const uint16_t *)CALIB_FLASH_ADDR;
    uint16_t data[CALIB_WORDS];
    FLASH_EraseInitTypeDef erase = {0};
    uint32_t page_err = 0;
    uint8_t k;
    uint8_t same = 1;

    data[0] = CALIB_MAGIC;
    for (k = 0; k < 4; k++)
    {
        /* 存的是"见过的极值"，不是生效端点：
         * 生效端点是极值加护栏算出来的，存极值才能在下次上电继续学 */
        data[1 + k * 3] = (uint16_t)s_axis[k].raw_mid;
        data[2 + k * 3] = (uint16_t)s_seen_lo[k];
        data[3 + k * 3] = (uint16_t)s_seen_hi[k];
    }

    data[CALIB_WORDS - 1] = 0;
    for (k = 0; k < (uint8_t)(CALIB_WORDS - 1); k++)
    {
        data[CALIB_WORDS - 1] ^= data[k];
    }

    /* 内容完全一样就不擦了，省一次 Flash 擦写寿命 */
    for (k = 0; k < CALIB_WORDS; k++)
    {
        if (flash[k] != data[k])
        {
            same = 0;
            break;
        }
    }
    if (same)
    {
        debug_printf("CALIB: Flash 内容未变，跳过写入\r\n");
        return;
    }

    HAL_FLASH_Unlock();

    erase.TypeErase   = FLASH_TYPEERASE_PAGES;
    erase.PageAddress = CALIB_FLASH_ADDR;
    erase.NbPages     = 1;
#ifdef FLASH_BANK_1
    erase.Banks = FLASH_BANK_1;
#endif

    if (HAL_FLASHEx_Erase(&erase, &page_err) != HAL_OK)
    {
        HAL_FLASH_Lock();
        debug_printf("CALIB SAVE FAIL: erase\r\n");
        return;
    }

    for (k = 0; k < CALIB_WORDS; k++)
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
                              CALIB_FLASH_ADDR + (uint32_t)k * 2,
                              (uint64_t)data[k]) != HAL_OK)
        {
            HAL_FLASH_Lock();
            debug_printf("CALIB SAVE FAIL: program\r\n");
            return;
        }
    }

    HAL_FLASH_Lock();
    debug_printf("CALIB SAVE OK\r\n");
}

/**
 * @brief 摇杆中位校准
 *
 * 采样四个摇杆"当前所在的位置"的【原始 ADC】，作为各自的中位，并存进 Flash。
 * 所以调用之前必须先把四个杆都放到中位（左杆Y 也要放中间）。
 *
 * 注意存的是原始 ADC，不是换算后的命令值：
 * 命令值域会随中位移动，只有原始 ADC 才是稳定的锚点。
 *
 * @return 1 = 成功；0 = 有轴的中位越界，本次不采纳、也不写 Flash
 */
uint8_t App_calibrate_joystick(void)
{
    int32_t acc[4] = {0};
    int16_t raw[4];
    int16_t mid[4];
    uint8_t i;
    uint8_t k;

    for (i = 0; i < 10; i++)
    {
        Int_joystick_get(&joystick);
        axis_to_array(&joystick, raw);

        for (k = 0; k < 4; k++)
        {
            acc[k] += raw[k]; /* 平均的是原始 ADC */
        }

        vTaskDelay(10); /* 10 次共约 100ms */
    }

    /* 先四个轴都查一遍，全合格才写：
     * 避免"三个轴对了、一个轴带着错误中位被存进 Flash"这种半残状态 */
    for (k = 0; k < 4; k++)
    {
        int16_t e_lo;
        int16_t e_hi;

        mid[k] = (int16_t)(acc[k] / 10);
        e_lo   = (s_seen_lo[k] <= AXIS_END_LO_MAX) ? s_seen_lo[k] : AXIS_END_LO_DEFAULT;
        e_hi   = (s_seen_hi[k] >= AXIS_END_HI_MIN) ? s_seen_hi[k] : AXIS_END_HI_DEFAULT;

        if (mid[k] < AXIS_MID_MIN || mid[k] > AXIS_MID_MAX)
        {
            debug_printf("CALIB REJECT: axis %d mid=%d 不在 [%d,%d] 内\r\n",
                         k, mid[k], AXIS_MID_MIN, AXIS_MID_MAX);
            return 0;
        }
        if ((mid[k] - e_lo) < AXIS_MIN_HALF_RANGE || (e_hi - mid[k]) < AXIS_MIN_HALF_RANGE)
        {
            debug_printf("CALIB REJECT: axis %d 行程太短 (mid=%d lo=%d hi=%d)\r\n",
                         k, mid[k], e_lo, e_hi);
            return 0;
        }
    }

    taskENTER_CRITICAL();
    for (k = 0; k < 4; k++)
    {
        /* 这里必须是 = 而不是 +=
         * 原来的代码是 +=，每校准一次中位就叠加一次，越校越偏
         * （旧代码的注释里自己也提到了这个问题） */
        s_axis[k].raw_mid = mid[k];
        s_axis[k].raw_lo  = (s_seen_lo[k] <= AXIS_END_LO_MAX) ? s_seen_lo[k] : AXIS_END_LO_DEFAULT;
        s_axis[k].raw_hi  = (s_seen_hi[k] >= AXIS_END_HI_MIN) ? s_seen_hi[k] : AXIS_END_HI_DEFAULT;
    }
    s_have_calib = 1;
    s_end_dirty  = 0;
    taskEXIT_CRITICAL();

    debug_printf("CALIB MID (raw): ly=%d lx=%d ry=%d rx=%d\r\n",
                 s_axis[0].raw_mid, s_axis[1].raw_mid,
                 s_axis[2].raw_mid, s_axis[3].raw_mid);
    debug_printf("CALIB END (raw): ly=%d..%d lx=%d..%d ry=%d..%d rx=%d..%d\r\n",
                 s_axis[0].raw_lo, s_axis[0].raw_hi,
                 s_axis[1].raw_lo, s_axis[1].raw_hi,
                 s_axis[2].raw_lo, s_axis[2].raw_hi,
                 s_axis[3].raw_lo, s_axis[3].raw_hi);

    /* 存进 Flash，下次上电直接读取 */
    calib_save();
    return 1;
}

/**
 * @brief 读取并处理摇杆数据
 */
/* ---------------------------------------------------------------
 * 调试开关：打印摇杆原始 ADC
 *
 * 用途：实测电位器的
 *   · 机械行程  -> 看 MIN / MAX 两列
 *   · 电气中位  -> 松手时看 ADC 那几列（应该在 2000~2100 附近）
 *
 * 用法：上电后把要测的杆推到两个尽头各顶死一下，
 *       然后直接读串口里的 MIN / MAX 就行（程序会一直记着见过的极值）。
 *
 * 测完把 AXIS_DEBUG_ADC 改成 0 就完全关闭，零开销。
 * --------------------------------------------------------------- */
#define AXIS_DEBUG_ADC 0
void App_process_joystick_data(void)
{
    int16_t raw[4];
    uint8_t k;

    taskENTER_CRITICAL();

    Int_joystick_get(&joystick);
    axis_to_array(&joystick, raw);

    /* 开机前几十拍不学习端点：
     * DMA 刚启动那一刻 adc_buff 里还是初值 0，
     * 会被误当成"正方向顶到底了" */
    if (s_warmup < AXIS_WARMUP_SAMPLES)
    {
        s_warmup++;
    }

    for (k = 0; k < 4; k++)
    {
        if (s_warmup >= AXIS_WARMUP_SAMPLES)
        {
            /* ---- 端点自学习 ----
             * 记下真的见过的最小 / 最大 raw。噪声只会把极值往轨上推，
             * 而护栏（AXIS_END_LO_MAX / HI_MIN）保证"不够深"的读数不会被采纳，
             * 所以不会因为噪声把量程撑坏。 */
            if (raw[k] < s_seen_lo[k])
            {
                s_seen_lo[k] = raw[k];
                s_end_dirty  = 1;
            }
            if (raw[k] > s_seen_hi[k])
            {
                s_seen_hi[k] = raw[k];
                s_end_dirty  = 1;
            }
        }

        /* 生效端点：够深就用实测值，否则退回保底值 0 / 4095 */
        s_axis[k].raw_lo = (s_seen_lo[k] <= AXIS_END_LO_MAX) ? s_seen_lo[k] : AXIS_END_LO_DEFAULT;
        s_axis[k].raw_hi = (s_seen_hi[k] >= AXIS_END_HI_MIN) ? s_seen_hi[k] : AXIS_END_HI_DEFAULT;

        axis_value[k] = axis_process(raw[k], &s_axis[k]);
    }

    taskEXIT_CRITICAL();

    /* ---- 学到新端点之后，静默 2 秒再写一次 Flash ----
     * 放在临界区外面：Flash 擦写期间 CPU 会停顿几十毫秒，不能占着临界区。
     * 只在已经校准过之后才写；而且写一次 s_end_dirty 就清零，
     * 加上 calib_save() 自己会比对内容，所以不会反复擦写。 */
    if (s_end_dirty && s_have_calib)
    {
        if (s_end_quiet < AXIS_END_SAVE_QUIET)
        {
            s_end_quiet++;
        }
        else
        {
            s_end_dirty = 0;
            s_end_quiet = 0;
            calib_save();
        }
    }

#if AXIS_DEBUG_ADC
    {
        static uint8_t s_dbg_cnt = 0;

        /* 每 25 次约 0.5 秒打一行，避免刷屏影响定时 */
        if (++s_dbg_cnt >= 25)
        {
            s_dbg_cnt = 0;
            debug_printf("ADC %4d %4d %4d %4d | MIN %4d %4d %4d %4d | MAX %4d %4d %4d %4d | %s\r\n",
                         raw[0], raw[1], raw[2], raw[3],
                         s_seen_lo[0], s_seen_lo[1], s_seen_lo[2], s_seen_lo[3],
                         s_seen_hi[0], s_seen_hi[1], s_seen_hi[2], s_seen_hi[3],
                         (s_warmup < AXIS_WARMUP_SAMPLES) ? "WARMUP" : "READY");
            debug_printf("    END ly %4d..%4d | lx %4d..%4d | ry %4d..%4d | rx %4d..%4d\r\n",
                         s_axis[0].raw_lo, s_axis[0].raw_hi,
                         s_axis[1].raw_lo, s_axis[1].raw_hi,
                         s_axis[2].raw_lo, s_axis[2].raw_hi,
                         s_axis[3].raw_lo, s_axis[3].raw_hi);
        }
    }
#endif
}

/**
 * @brief 把中位复位到电气中点（重新校准前调用）
 *
 * 注意：只改 RAM 里的中位，不动 Flash。
 *       只有校准成功之后才会写 Flash，
 *       所以中途断电/放弃不会破坏原来保存的数据。
 */
void App_calib_clear(void)
{
    uint8_t k;

    taskENTER_CRITICAL();
    for (k = 0; k < 4; k++)
    {
        /* 先把中位摆到"电气中点"：
         * 这样屏幕上的值反映的是"离电气中点还有多远"，
         * 操作者对着屏幕把四个杆摆正即可。
         * 端点这次不重置：它是机械特性，重校准中位不影响它。 */
        s_axis[k].raw_mid = AXIS_MID_NOMINAL;
    }
    s_have_calib = 0; /* 校准完成前不自动写 Flash */
    taskEXIT_CRITICAL();

    debug_printf("CALIB: 中位已复位到电气中点，请把四个杆摆到中位\r\n");
}
