//
// INA226 驱动实现（I2C1, 0x40）
//
// 只读写寄存器，不做单位换算以外的任何事。
// 电压/电流/功率都以"毫"为单位返回，避免浮点。
//

#include "ina226.h"
#include "i2c.h"

/* ---------------- 寄存器 ---------------- */
#define INA226_REG_CONFIG       0x00u
#define INA226_REG_SHUNT_V      0x01u
#define INA226_REG_BUS_V        0x02u
#define INA226_REG_POWER        0x03u
#define INA226_REG_CURRENT      0x04u
#define INA226_REG_CALIB        0x05u
#define INA226_REG_MASK         0x06u
#define INA226_REG_ALERT        0x07u
#define INA226_REG_MFG_ID       0xFEu
#define INA226_REG_DIE_ID       0xFFu

/* ---------------- 芯片常量 ---------------- */
#define INA226_MFG_ID_VALUE     0x5449u     /* "TI" */
#define INA226_DIE_ID_VALUE     0x2260u

/* 平均 16 次、1.1 ms 转换、分流+母线连续测量 */
#define INA226_CONFIG_VALUE     0x4527u

/* 母线电压 LSB = 1.25 mV */
#define INA226_BUS_LSB_UV       1250u

/* 分流电压满量程 = ±81.92 mV，单位 µV */
#define INA226_SHUNT_FS_UV      81920000UL

/* SHUNT 满量程对应的电流分辨率上限：2^15 */
#define INA226_CURRENT_DIV      32768UL

/* 校准公式常数：CAL = 5.12e6 / (Current_LSB[uA] * R[mΩ]) */
#define INA226_CAL_CONST        5120000UL

#define INA226_ADDR_8BIT        ((uint16_t)(INA226_ADDR_7BIT << 1u))
#define INA226_I2C_TIMEOUT_MS   10u

/* 初始化时算出来的电流分辨率，单位 µA/LSB */
static uint32_t s_lsb_ua;

/* ---------------- 底层 ---------------- */

static int ina226_write_reg(uint8_t reg, uint16_t val)
{
    uint8_t buf[2];

    buf[0] = (uint8_t)(val >> 8);
    buf[1] = (uint8_t)(val & 0xFFu);

    if (HAL_I2C_Mem_Write(&hi2c1,
                          INA226_ADDR_8BIT,
                          reg,
                          I2C_MEMADD_SIZE_8BIT,
                          buf,
                          2,
                          INA226_I2C_TIMEOUT_MS) != HAL_OK)
    {
        return -1;
    }

    return 0;
}

static int ina226_read_reg(uint8_t reg, uint16_t *val)
{
    uint8_t buf[2];

    if (val == 0)
    {
        return -1;
    }

    if (HAL_I2C_Mem_Read(&hi2c1,
                         INA226_ADDR_8BIT,
                         reg,
                         I2C_MEMADD_SIZE_8BIT,
                         buf,
                         2,
                         INA226_I2C_TIMEOUT_MS) != HAL_OK)
    {
        return -1;
    }

    *val = (uint16_t)(((uint16_t)buf[0] << 8) | buf[1]);
    return 0;
}

/* ---------------- 对外接口 ---------------- */

int ina226_init(uint16_t r_shunt_mohm)
{
    uint16_t mfg = 0;
    uint16_t die = 0;
    uint16_t chk = 0;
    uint32_t lsb_ua;
    uint32_t cal;

    if (r_shunt_mohm == 0u)
    {
        return -1;
    }

    /* 1. 探测：两个 ID 都对才认 */
    if (ina226_read_reg(INA226_REG_MFG_ID, &mfg) != 0)
    {
        return -1;
    }
    if (mfg != INA226_MFG_ID_VALUE)
    {
        return -2;
    }

    if (ina226_read_reg(INA226_REG_DIE_ID, &die) != 0)
    {
        return -1;
    }
    if (die != INA226_DIE_ID_VALUE)
    {
        return -3;
    }

    /* 2. 算电流分辨率：满量程电流 / 32768
     *    满量程电流(uA) = 81920000 / R(mΩ) */
    lsb_ua = (INA226_SHUNT_FS_UV / r_shunt_mohm) / INA226_CURRENT_DIV;

    if (lsb_ua == 0u)
    {
        lsb_ua = 1u;
    }

    /* 3. 算校准值：CAL = 5.12e6 / (LSB[uA] * R[mΩ]) */
    cal = INA226_CAL_CONST / (lsb_ua * (uint32_t)r_shunt_mohm);

    if (cal == 0u)
    {
        cal = 1u;
    }
    if (cal > 0xFFFFu)
    {
        cal = 0xFFFFu;
    }

    /* 4. 写配置和校准 */
    if (ina226_write_reg(INA226_REG_CONFIG, INA226_CONFIG_VALUE) != 0)
    {
        return -4;
    }
    if (ina226_write_reg(INA226_REG_CALIB, (uint16_t)cal) != 0)
    {
        return -4;
    }

    /* 5. 回读校验
     * CONFIG 的 bit15(RST) 和 bit14(保留) 不一定读回原值，
     * 所以只比低 12 位（平均/转换时间/模式）。 */
    if (ina226_read_reg(INA226_REG_CONFIG, &chk) != 0)
    {
        return -4;
    }
    if ((chk & 0x0FFFu) != (INA226_CONFIG_VALUE & 0x0FFFu))
    {
        return -4;
    }

    if (ina226_read_reg(INA226_REG_CALIB, &chk) != 0)
    {
        return -4;
    }
    if (chk != (uint16_t)cal)
    {
        return -4;
    }

    s_lsb_ua = lsb_ua;
    return 0;
}

int ina226_read(int32_t *mv, int32_t *ma, int32_t *mw)
{
    uint16_t bus_raw = 0;
    uint16_t cur_raw = 0;
    int16_t  cur_s;
    int32_t  v_mv;
    int32_t  i_ma;

    if ((mv == 0) || (ma == 0) || (mw == 0))
    {
        return -1;
    }

    if (s_lsb_ua == 0u)
    {
        return -1;
    }

    /* 母线电压：1.25 mV/LSB，无符号 */
    if (ina226_read_reg(INA226_REG_BUS_V, &bus_raw) != 0)
    {
        return -1;
    }

    /* 电流：有符号 16 位，LSB = s_lsb_ua（µA） */
    if (ina226_read_reg(INA226_REG_CURRENT, &cur_raw) != 0)
    {
        return -1;
    }
    cur_s = (int16_t)cur_raw;

    v_mv = (int32_t)((bus_raw * INA226_BUS_LSB_UV) / 1000u);
    i_ma = (int32_t)(((int32_t)cur_s * (int32_t)s_lsb_ua) / 1000);

    /* 方向修正 */
    i_ma = i_ma * INA226_CURRENT_SIGN;

    *mv = v_mv;
    *ma = i_ma;
    *mw = (v_mv * i_ma) / 1000;

    return 0;
}
