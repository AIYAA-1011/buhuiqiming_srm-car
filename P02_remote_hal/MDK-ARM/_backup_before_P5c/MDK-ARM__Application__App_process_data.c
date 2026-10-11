#include "App_process_data.h"

/* 摇杆原始 ADC 值（由 DMA 直接写进 adc_buff，这里取出来） */
Joystick_Struct joystick = {0};

/* 处理后的四个轴命令值：-500 ~ +500，0 = 中位 / 停止 */
int16_t axis_value[4] = {0};

/* ==================== 零偏保存到 Flash ====================
 * STM32F103C8 是 64KB Flash、页大小 1KB。
 * 程序只占 21KB 左右，最后 1 页（0x0800FC00 ~ 0x0800FFFF）是空的，
 * 用来存 4 个 int16 的零偏，这样断电重开不用重新校准。
 *
 * 存放格式（6 个半字）：
 *   [0]     magic
 *   [1..4]  thr / yaw / pit / rol 的零偏
 *   [5]     校验和 = [0]~[4] 逐项异或
 *
 * 擦除后的 Flash 全是 0xFFFF，magic 对不上就当没有数据。
 * 注意：如果 Keil 下载时选了"Erase Full Chip"，这一页会被一起擦掉，
 *       烧完第一次上电需要重新校准一次，属正常现象。
 * ======================================================== */
#define CALIB_FLASH_ADDR 0x0800FC00UL
#define CALIB_MAGIC      0x5A5A
#define CALIB_WORDS      6

/* 每个轴的运行时参数 */
typedef struct
{
    uint8_t inv;    /* 极性：1 = 反相 */
    int16_t dz;     /* 死区半宽（对称） */
    int16_t offset; /* 零偏，由校准得到（或从 Flash 读回） */
} Axis_Cfg;

static Axis_Cfg s_axis[4] =
{
    { AXIS_INV_THR, AXIS_DEADZONE, 0 }, /* 左杆Y -> 大臂     */
    { AXIS_INV_YAW, AXIS_DEADZONE, 0 }, /* 左杆X -> 小臂     */
    { AXIS_INV_PIT, AXIS_DEADZONE, 0 }, /* 右杆Y -> 腕部俯仰 */
    { AXIS_INV_ROL, AXIS_DEADZONE, 0 }, /* 右杆X -> 腕部转动 */
};

/**
 * @brief ADC 原始值 -> -500 ~ +500
 *        这一步还没减零偏，也没做极性和死区
 */
static int16_t axis_adc_to_span(int16_t raw)
{
    return (int16_t)(AXIS_SPAN - ((int32_t)raw * (AXIS_SPAN * 2) / 4095));
}

/**
 * @brief 摇杆结构体 -> 数组，顺序 thr / yaw / pit / rol
 */
static void axis_to_array(const Joystick_Struct *js, int16_t *raw)
{
    raw[0] = js->thr;
    raw[1] = js->yaw;
    raw[2] = js->pit;
    raw[3] = js->rol;
}

/**
 * @brief 单个轴的处理
 */
static int16_t axis_process(int16_t raw, const Axis_Cfg *cfg)
{
    int16_t v = axis_adc_to_span(raw);

    if (cfg->inv)
    {
        v = -v; /* 极性修正 */
    }

    v -= cfg->offset; /* 减掉零偏 */

    v = Com_limit(v, -AXIS_SPAN, AXIS_SPAN); /* 限幅 */

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
 * @brief 从 Flash 读回零偏
 */
uint8_t App_calib_load(void)
{
    const uint16_t *flash = (const uint16_t *)CALIB_FLASH_ADDR;
    uint16_t chk = (uint16_t)(flash[0] ^ flash[1] ^ flash[2] ^ flash[3] ^ flash[4]);
    uint8_t k;

    if (flash[0] != CALIB_MAGIC || flash[5] != chk)
    {
        debug_printf("CALIB: flash 无有效数据，需要校准\r\n");
        return 0;
    }

    taskENTER_CRITICAL();
    for (k = 0; k < 4; k++)
    {
        s_axis[k].offset = (int16_t)flash[1 + k];
    }
    taskEXIT_CRITICAL();

    debug_printf("CALIB LOAD: thr=%d yaw=%d pit=%d rol=%d\r\n",
                 s_axis[0].offset, s_axis[1].offset,
                 s_axis[2].offset, s_axis[3].offset);
    return 1;
}

/**
 * @brief 把当前零偏写进 Flash
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
        data[1 + k] = (uint16_t)s_axis[k].offset;
    }
    data[5] = (uint16_t)(data[0] ^ data[1] ^ data[2] ^ data[3] ^ data[4]);

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
 * @brief 摇杆零偏校准
 *
 * 把四个摇杆"当前所在的位置"定义为中位，并存进 Flash。
 * 所以调用之前必须先把四个杆都放到中位（左杆Y 也要放中间）。
 */
void App_calibrate_joystick(void)
{
    int32_t acc[4] = {0};
    int16_t raw[4];
    uint8_t i;
    uint8_t k;

    for (i = 0; i < 10; i++)
    {
        Int_joystick_get(&joystick);
        axis_to_array(&joystick, raw);

        for (k = 0; k < 4; k++)
        {
            int16_t v = axis_adc_to_span(raw[k]);
            if (s_axis[k].inv)
            {
                v = -v;
            }
            acc[k] += v;
        }

        vTaskDelay(10); /* 10 次共约 100ms */
    }

    taskENTER_CRITICAL();
    for (k = 0; k < 4; k++)
    {
        /* 这里必须是 = 而不是 +=
         * 原来的代码是 +=，每校准一次零偏就叠加一次，越校越偏
         * （旧代码的注释里自己也提到了这个问题） */
        s_axis[k].offset = (int16_t)(acc[k] / 10);
    }
    taskEXIT_CRITICAL();

    debug_printf("CALIB: thr=%d yaw=%d pit=%d rol=%d\r\n",
                 s_axis[0].offset, s_axis[1].offset,
                 s_axis[2].offset, s_axis[3].offset);

    /* 存进 Flash，下次上电直接读取 */
    calib_save();
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
#define AXIS_DEBUG_ADC 1

void App_process_joystick_data(void)
{
    int16_t raw[4];
    uint8_t k;

    taskENTER_CRITICAL();

    Int_joystick_get(&joystick);
    axis_to_array(&joystick, raw);

    for (k = 0; k < 4; k++)
    {
        axis_value[k] = axis_process(raw[k], &s_axis[k]);
    }

    taskEXIT_CRITICAL();

#if AXIS_DEBUG_ADC
    {
        /* 每个轴见过的最大 / 最小原始 ADC，用来量机械行程 */
        static int16_t s_adc_min[4] = { 4095, 4095, 4095, 4095 };
        static int16_t s_adc_max[4] = { 0, 0, 0, 0 };
        static uint8_t s_dbg_cnt = 0;

        for (k = 0; k < 4; k++)
        {
            if (raw[k] < s_adc_min[k])
            {
                s_adc_min[k] = raw[k];
            }
            if (raw[k] > s_adc_max[k])
            {
                s_adc_max[k] = raw[k];
            }
        }

        /* 每 25 次约 0.5 秒打一行，避免刷屏影响定时 */
        if (++s_dbg_cnt >= 25)
        {
            s_dbg_cnt = 0;
            debug_printf("ADC %4d %4d %4d %4d | MIN %4d %4d %4d %4d | MAX %4d %4d %4d %4d\r\n",
                         raw[0], raw[1], raw[2], raw[3],
                         s_adc_min[0], s_adc_min[1], s_adc_min[2], s_adc_min[3],
                         s_adc_max[0], s_adc_max[1], s_adc_max[2], s_adc_max[3]);
        }
    }
#endif
}
