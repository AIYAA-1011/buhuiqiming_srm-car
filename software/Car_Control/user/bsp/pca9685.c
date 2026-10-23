//
// PCA9685 驱动
//
// 第 1 步：探测
// 第 2 步：初始化（50 Hz）/ 设脉宽 / 开关输出
//

#include "pca9685.h"
#include "i2c.h"

/* HAL 用的是 8 位地址（7 位地址左移一位） */
#define PCA9685_ADDR_8BIT       ((uint16_t)(PCA9685_ADDR_7BIT << 1u))

/* 单次 I2C 超时。PCA9685 在线时一次读写只要几十 us，
 * 超时设短一点，模块没接的时候不至于把任务卡太久 */
#define PCA9685_I2C_TIMEOUT_MS  10u

/* 扫描范围：I2C 保留地址之外的所有 7 位地址 */
#define PCA9685_SCAN_FIRST      0x08u
#define PCA9685_SCAN_LAST       0x77u

/* 50 Hz 下 1 个计数 = 20000us / 4096 */
#define PCA9685_US_PER_FRAME    20000u
#define PCA9685_COUNT_FULL      4096u

/* 唤醒后等振荡器稳定（datasheet 要求 500 us，这里给 2 ms 富余） */
#define PCA9685_WAKE_DELAY_MS   2u

/* ---------------- 底层 ---------------- */

int pca9685_read_reg(uint8_t reg, uint8_t *val)
{
    if (val == 0)
    {
        return -1;
    }

    if (HAL_I2C_Mem_Read(&hi2c2,
                         PCA9685_ADDR_8BIT,
                         reg,
                         I2C_MEMADD_SIZE_8BIT,
                         val,
                         1,
                         PCA9685_I2C_TIMEOUT_MS) != HAL_OK)
    {
        return -1;
    }

    return 0;
}

int pca9685_write_reg(uint8_t reg, uint8_t val)
{
    if (HAL_I2C_Mem_Write(&hi2c2,
                          PCA9685_ADDR_8BIT,
                          reg,
                          I2C_MEMADD_SIZE_8BIT,
                          &val,
                          1,
                          PCA9685_I2C_TIMEOUT_MS) != HAL_OK)
    {
        return -1;
    }

    return 0;
}

int pca9685_detect(void)
{
    uint8_t mode1 = 0;

    if (pca9685_read_reg(PCA9685_REG_MODE1, &mode1) != 0)
    {
        return -1;
    }

    /* 读到 0xFF 说明 SDA 被上拉、其实没人应答，HAL 偶尔会误报 OK */
    if (mode1 == 0xFFu)
    {
        return -1;
    }

    return 0;
}

int pca9685_scan(uint8_t *found, int max)
{
    int     n = 0;
    uint8_t addr;

    if ((found == 0) || (max <= 0))
    {
        return 0;
    }

    for (addr = PCA9685_SCAN_FIRST; addr <= PCA9685_SCAN_LAST; addr++)
    {
        if (HAL_I2C_IsDeviceReady(&hi2c2,
                                  (uint16_t)(addr << 1u),
                                  2,
                                  5) == HAL_OK)
        {
            if (n < max)
            {
                found[n] = addr;
                n++;
            }
        }
    }

    return n;
}

/* ---------------- 初始化 ---------------- */

int pca9685_init(void)
{
    uint8_t mode1 = 0;
    uint8_t chk   = 0;

    /* 0. 先确认芯片在 */
    if (pca9685_detect() != 0)
    {
        return -1;
    }

    /* 1. 进 SLEEP —— PRE_SCALE 只能在睡眠时改 */
    if (pca9685_read_reg(PCA9685_REG_MODE1, &mode1) != 0)
    {
        return -1;
    }
    if (pca9685_write_reg(PCA9685_REG_MODE1,
                          (uint8_t)((mode1 | PCA9685_MODE1_SLEEP) &
                                    (uint8_t)~PCA9685_MODE1_EXTCLK)) != 0)
    {
        return -1;
    }

    /* 2. 预分频 = 50 Hz */
    if (pca9685_write_reg(PCA9685_REG_PRESCALE, PCA9685_PRESCALE_50HZ) != 0)
    {
        return -1;
    }

    /* 3. 输出级配成推挽（舵机要推挽，不要开漏） */
    if (pca9685_write_reg(PCA9685_REG_MODE2, PCA9685_MODE2_OUTDRV) != 0)
    {
        return -1;
    }

    /* 4. 唤醒 + 自动递增（后面一次写 4 字节靠它） */
    if (pca9685_write_reg(PCA9685_REG_MODE1, PCA9685_MODE1_AI) != 0)
    {
        return -1;
    }

    /* 5. 等振荡器稳定 */
    HAL_Delay(PCA9685_WAKE_DELAY_MS);

    /* 6. 回读校验：必须是醒着的，而且 AI 打开 */
    if (pca9685_read_reg(PCA9685_REG_MODE1, &chk) != 0)
    {
        return -1;
    }
    if ((chk & PCA9685_MODE1_SLEEP) != 0u)
    {
        return -1;
    }
    if ((chk & PCA9685_MODE1_AI) == 0u)
    {
        return -1;
    }

    if (pca9685_read_reg(PCA9685_REG_PRESCALE, &chk) != 0)
    {
        return -1;
    }
    if (chk != PCA9685_PRESCALE_50HZ)
    {
        return -1;
    }

    return 0;
}

/* ---------------- 脉宽 ---------------- */

static uint16_t us_to_count(uint16_t us)
{
    uint32_t count = ((uint32_t)us * PCA9685_COUNT_FULL) / PCA9685_US_PER_FRAME;

    if (count >= PCA9685_COUNT_FULL)
    {
        count = PCA9685_COUNT_FULL - 1u;
    }

    return (uint16_t)count;
}

int pca9685_set_counts(uint8_t ch, uint16_t on, uint16_t off)
{
    uint8_t buf[4];
    uint8_t reg;

    if (ch >= PCA9685_CH_COUNT)
    {
        return -1;
    }

    on  &= 0x0FFFu;
    off &= 0x0FFFu;

    buf[0] = (uint8_t)(on & 0xFFu);
    buf[1] = (uint8_t)(on >> 8);
    buf[2] = (uint8_t)(off & 0xFFu);
    buf[3] = (uint8_t)(off >> 8);

    reg = (uint8_t)(PCA9685_REG_LED0_ON_L + 4u * ch);

    /* AI=1，所以一次连续写 4 个字节：ON_L, ON_H, OFF_L, OFF_H */
    if (HAL_I2C_Mem_Write(&hi2c2,
                          PCA9685_ADDR_8BIT,
                          reg,
                          I2C_MEMADD_SIZE_8BIT,
                          buf,
                          4,
                          PCA9685_I2C_TIMEOUT_MS) != HAL_OK)
    {
        return -1;
    }

    return 0;
}

int pca9685_set_us(uint8_t ch, uint16_t us)
{
    if (us < PCA9685_US_MIN)
    {
        us = PCA9685_US_MIN;
    }
    if (us > PCA9685_US_MAX)
    {
        us = PCA9685_US_MAX;
    }

    return pca9685_set_counts(ch, 0u, us_to_count(us));
}

int pca9685_set_all_us(uint16_t us)
{
    uint8_t ch;
    int     rc = 0;

    for (ch = 0; ch < PCA9685_CH_COUNT; ch++)
    {
        if (pca9685_set_us(ch, us) != 0)
        {
            rc = -1;
        }
    }

    return rc;
}

/* ---------------- 输出开关 ---------------- */

void pca9685_output_enable(void)
{
    HAL_GPIO_WritePin(SERVO_OE_N_GPIO_Port, SERVO_OE_N_Pin, GPIO_PIN_RESET);
}

void pca9685_output_disable(void)
{
    HAL_GPIO_WritePin(SERVO_OE_N_GPIO_Port, SERVO_OE_N_Pin, GPIO_PIN_SET);
}

int pca9685_output_enabled(void)
{
    return (HAL_GPIO_ReadPin(SERVO_OE_N_GPIO_Port, SERVO_OE_N_Pin) == GPIO_PIN_RESET) ? 1 : 0;
}
