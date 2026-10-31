//
// 显示任务
//
// 负责：扫 I2C1、初始化 INA226 和 OLED、周期刷新电池信息
//
// 屏幕布局（4 行，每行 10x14 大字，一行最多 10 个字符）：
//   V  12.40V
//   I   3.21A
//   P  39.8W
//   SOC  78%
//

#include "display_task.h"
#include "i2c.h"
#include "ina226.h"
#include "oled.h"
#include "debug.h"
#include "cmsis_os.h"
#include <stdio.h>

/* ---------------- 扫描参数 ---------------- */

#define DISP_SCAN_FIRST     0x08u
#define DISP_SCAN_LAST      0x77u
#define DISP_SCAN_MAX       8

#define DISP_SCAN_TRIALS    2u
#define DISP_SCAN_TIMEOUT   5u

/* ---------------- 电池参数 ---------------- */

/* 采样电阻：本车 3 mΩ */
#define DISP_R_SHUNT_MOHM   3u

/* 刷新周期（ms） */
#define DISP_PERIOD_MS      200u

/* 串口打印间隔 = DISP_PERIOD_MS * DISP_LOG_EVERY 次 */
#define DISP_LOG_EVERY      5u

/* ---------------- 电量表 ----------------
 * 3S 锂离子（18650）：
 *   满电 12.60 V（4.20 V/节），空电 8.25 V（2.75 V/节）
 * 从高电压往低电压排列，中间线性插值。
 */
typedef struct
{
    uint16_t mv;
    uint8_t  soc;
} soc_point_t;

static const soc_point_t s_soc_tbl[] =
{
    {12600u, 100u},
    {12300u,  90u},
    {12000u,  80u},
    {11700u,  70u},
    {11400u,  55u},
    {11100u,  40u},
    {10800u,  25u},
    {10500u,  15u},
    {10200u,   8u},
    { 9900u,   4u},
    { 9300u,   2u},
    { 8250u,   0u},
};

#define SOC_TBL_LEN     (sizeof(s_soc_tbl) / sizeof(s_soc_tbl[0]))

/* 电压平均值（mV），只用于算电量；显示仍然用瞬时值 */
static int32_t s_v_avg_mv;

#if DISP_DEBUG
/**
 * @brief 扫一遍 I2C1，打印所有应答的 7 位地址
 */
static void i2c1_scan(void)
{
    uint8_t found[DISP_SCAN_MAX];
    uint8_t addr;
    int     n = 0;
    int     i;

    for (addr = DISP_SCAN_FIRST; addr <= DISP_SCAN_LAST; addr++)
    {
        if (HAL_I2C_IsDeviceReady(&hi2c1,
                                  (uint16_t)(addr << 1u),
                                  DISP_SCAN_TRIALS,
                                  DISP_SCAN_TIMEOUT) == HAL_OK)
        {
            if (n < DISP_SCAN_MAX)
            {
                found[n] = addr;
            }
            n++;
        }
    }

    if (n == 0)
    {
        debug_printf("[DISP] I2C1 scan: no device found\r\n");
        return;
    }

    debug_printf("[DISP] I2C1 scan: %d device(s):", n);
    for (i = 0; i < n && i < (int)DISP_SCAN_MAX; i++)
    {
        debug_printf(" 0x%02X", found[i]);
    }
    debug_printf("\r\n");
}
#endif /* DISP_DEBUG */

static int32_t abs32(int32_t v)
{
    return (v < 0) ? -v : v;
}

/**
 * @brief 按电压查表算剩余电量（线性插值）
 */
static uint8_t batt_soc_from_mv(int32_t mv)
{
    uint32_t i;

    if (mv >= (int32_t)s_soc_tbl[0].mv)
    {
        return s_soc_tbl[0].soc;
    }
    if (mv <= (int32_t)s_soc_tbl[SOC_TBL_LEN - 1u].mv)
    {
        return s_soc_tbl[SOC_TBL_LEN - 1u].soc;
    }

    for (i = 0; i + 1u < SOC_TBL_LEN; i++)
    {
        int32_t v_hi = (int32_t)s_soc_tbl[i].mv;
        int32_t v_lo = (int32_t)s_soc_tbl[i + 1u].mv;

        if ((mv <= v_hi) && (mv >= v_lo))
        {
            int32_t s_hi = (int32_t)s_soc_tbl[i].soc;
            int32_t s_lo = (int32_t)s_soc_tbl[i + 1u].soc;

            /* soc = s_lo + (mv - v_lo) * (s_hi - s_lo) / (v_hi - v_lo) */
            return (uint8_t)(s_lo + (((mv - v_lo) * (s_hi - s_lo)) / (v_hi - v_lo)));
        }
    }

    return 0u;
}

/**
 * @brief 读一次 INA226，算好电量，画 4 行并刷屏
 *
 * @return 0 = 正常；-1 = INA 读失败（保留上一帧不动）
 */
static int display_update(int32_t *out_mv, int32_t *out_ma,
                          int32_t *out_mw, uint8_t *out_soc)
{
    int32_t mv = 0;
    int32_t ma = 0;
    int32_t mw = 0;
    char    line[24];
    uint8_t soc;

    if (ina226_read(&mv, &ma, &mw) != 0)
    {
        return -1;
    }

    /* 限幅：防止异常读数把屏幕刷成天文数字 */
    if (mv < 0)      { mv = 0; }
    if (mv > 30000)  { mv = 30000; }
    if (ma < -60000) { ma = -60000; }
    if (ma > 60000)  { ma = 60000; }
    if (mw < -2000000) { mw = -2000000; }
    if (mw > 2000000)  { mw = 2000000; }

    /* 电压一阶低通：τ 约 8 个周期（1.6 s），只用于算电量 */
    if (s_v_avg_mv == 0)
    {
        s_v_avg_mv = mv;
    }
    else
    {
        s_v_avg_mv += (mv - s_v_avg_mv) / 8;
    }

    soc = batt_soc_from_mv(s_v_avg_mv);

    oled_clear();

    /* 第 1 行：电压，两位小数 */
    (void)snprintf(line, sizeof(line), "V%3d.%02dV",
                   (int)(mv / 1000), (int)(abs32(mv % 1000) / 10));
    oled_text(0u, 0u, line);

    /* 第 2 行：电流，两位小数，带符号 */
    (void)snprintf(line, sizeof(line), "I%3d.%02dA",
                   (int)(ma / 1000), (int)(abs32(ma % 1000) / 10));
    oled_text(0u, 2u, line);

    /* 第 3 行：功率，一位小数 */
    (void)snprintf(line, sizeof(line), "P%3d.%01dW",
                   (int)(mw / 1000), (int)(abs32(mw % 1000) / 100));
    oled_text(0u, 4u, line);

    /* 第 4 行：剩余电量 */
    (void)snprintf(line, sizeof(line), "SOC%3d%%", (int)soc);
    oled_text(0u, 6u, line);

    oled_flush();

    if (out_mv != 0)  { *out_mv  = mv;  }
    if (out_ma != 0)  { *out_ma  = ma;  }
    if (out_mw != 0)  { *out_mw  = mw;  }
    if (out_soc != 0) { *out_soc = soc; }

    return 0;
}

void StartDisplayTask(void const *argument)
{
    int     rc;
    uint8_t log_cnt = 0u;

    (void)argument;

    debug_printf("\r\n[DISP] start\r\n");

    /* 总线扫描：确认 OLED(0x3C) 和 INA226(0x40) 都在（只在 DISP_DEBUG=1 时编进来） */
#if DISP_DEBUG
    i2c1_scan();
#endif

    /* INA226 */
    rc = ina226_init(DISP_R_SHUNT_MOHM);
    if (rc == 0)
    {
        debug_printf("[DISP] INA226 init OK, R=%u mOhm\r\n", (unsigned)DISP_R_SHUNT_MOHM);
    }
    else
    {
        debug_printf("[DISP] INA226 init FAIL, rc=%d (-1 I2C, -2 mfg, -3 die, -4 readback)\r\n", rc);
    }

    /* OLED */
    if (oled_init() != 0)
    {
        debug_printf("[DISP] OLED init FAIL (0x3C / PB8 PB9?)\r\n");
    }

    for (;;)
    {
        int32_t mv = 0;
        int32_t ma = 0;
        int32_t mw = 0;
        uint8_t soc = 0u;

        if (display_update(&mv, &ma, &mw, &soc) == 0)
        {
            if (++log_cnt >= DISP_LOG_EVERY)
            {
                log_cnt = 0u;
                debug_printf("[DISP] V %d.%02dV  I %d.%03dA  P %d.%01dW  SOC %u%%\r\n",
                             (int)(mv / 1000), (int)(abs32(mv % 1000) / 10),
                             (int)(ma / 1000), (int)abs32(ma % 1000),
                             (int)(mw / 1000), (int)(abs32(mw % 1000) / 100),
                             (unsigned)soc);
            }
        }
        else
        {
            debug_printf("[DISP] INA226 read FAIL\r\n");
        }

        osDelay(DISP_PERIOD_MS);
    }
}
