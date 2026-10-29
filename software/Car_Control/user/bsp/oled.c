//
// OLED 驱动实现（SSD1306 / SSD1315，I2C1，0x3C）
//
// 结构：
//   s_gram[1024]  显存，1024 = 8 页 × 128 列
//   oled_text()   5x7 字库，绘制时放大 2 倍（10x14），字距 12 px
//   oled_flush()  把显存整块推到屏幕
//
// 注意：整屏 1024 字节 @400kHz 约 25 ms，所以刷新率别太高。
//

#include "oled.h"
#include "i2c.h"

/* SSD1306 的 I2C 控制字节 */
#define OLED_CTRL_CMD       0x00u   /* 后面全是命令 */
#define OLED_CTRL_DATA      0x40u   /* 后面全是显存数据 */

#define OLED_ADDR_8BIT      ((uint16_t)(OLED_ADDR_7BIT << 1u))

/* 显存大小：128 × 64 / 8 */
#define OLED_GRAM_SIZE      1024u
#define OLED_WIDTH          128u
#define OLED_PAGES          8u

/* 命令超时 / 刷屏超时（ms） */
#define OLED_CMD_TIMEOUT    20u
#define OLED_FLUSH_TIMEOUT  100u

/* 字符放大倍数：
 *   2,2 -> 10x14 大字（4 行，一行 10 个字符）
 *   1,1 -> 5x7  小字（8 行，一行 21 个字符）
 *   2,1 -> 10x7 宽扁字
 *   1,2 -> 5x14 瘦高字
 *
 * Y_SCALE = 2 时一个字符占 14 行，会跨两页，所以 page 必须是偶数。 */
#define OLED_TEXT_XSCALE    2u
#define OLED_TEXT_YSCALE    2u
#define OLED_TEXT_ADVANCE   ((5u * OLED_TEXT_XSCALE) + 2u)

/* 显存（放 .bss，不能放栈上） */
static uint8_t s_gram[OLED_GRAM_SIZE];

/* 初始化过没有 */
static uint8_t s_ready;

/* 初始化序列（SSD1306 128x64 通用） */
static const uint8_t s_init_seq[] =
{
    0xAEu,              /* 关显示                      */
    0xD5u, 0x80u,       /* 时钟分频                    */
    0xA8u, 0x3Fu,       /* 多路复用 1/64               */
    0xD3u, 0x00u,       /* 显示偏移 0                  */
    0x40u,              /* 起始行 0                    */
    0x8Du, 0x14u,       /* 开电荷泵（不开发屏不亮）    */
    0x20u, 0x02u,       /* ★ 页寻址模式（兼容 SSD1315 / SH1106） */
    0xA1u,              /* 列重映射                    */
    0xC8u,              /* 行重映射                    */
    0xDAu, 0x12u,       /* COM 引脚配置                */
    0x81u, 0xCFu,       /* 对比度                      */
    0xD9u, 0xF1u,       /* 预充电周期                  */
    0xDBu, 0x40u,       /* VCOMH                       */
    0xA4u,              /* 显示跟随显存                */
    0xA6u,              /* 正常显示（非反白）          */
    0x2Eu,              /* 关闭滚动                    */
    0xAFu               /* 开显示                      */
};

/* ==================================================================
 * 5x7 字库（只做本工程用到的字符，其它留空）
 *
 * 下标 = 字符 - 0x20（不是 ASCII 码本身！）
 *   空格 0x20 -> 下标 0x00
 *   '0'  0x30 -> 下标 0x10
 *   'A'  0x41 -> 下标 0x21
 *   'H'  0x48 -> 下标 0x28
 *
 * 每个字符 7 个字节，1 个字节 = 1 行，
 * 用 bit4~bit0 表示从左到右 5 个像素。
 * 绘制时放大 2 倍，所以显示出来是 10x14 像素。
 * ================================================================== */
static const uint8_t s_font5x7[96][7] =
{
    [0x00] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},   /* 空格 */
    [0x05] = {0x19, 0x1A, 0x04, 0x08, 0x13, 0x13, 0x00},   /* % */
    [0x0D] = {0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00},   /* - */
    [0x0E] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C},   /* . */
    [0x10] = {0x0E, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0E},   /* 0 */
    [0x11] = {0x04, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x0E},   /* 1 */
    [0x12] = {0x0E, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1F},   /* 2 */
    [0x13] = {0x1E, 0x01, 0x01, 0x0E, 0x01, 0x01, 0x1E},   /* 3 */
    [0x14] = {0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02},   /* 4 */
    [0x15] = {0x1F, 0x10, 0x1E, 0x01, 0x01, 0x11, 0x0E},   /* 5 */
    [0x16] = {0x06, 0x08, 0x10, 0x1E, 0x11, 0x11, 0x0E},   /* 6 */
    [0x17] = {0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08},   /* 7 */
    [0x18] = {0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E},   /* 8 */
    [0x19] = {0x0E, 0x11, 0x11, 0x0F, 0x01, 0x02, 0x0C},   /* 9 */
    [0x21] = {0x04, 0x0A, 0x11, 0x11, 0x1F, 0x11, 0x11},   /* A */
    [0x23] = {0x0E, 0x11, 0x10, 0x10, 0x10, 0x11, 0x0E},   /* C */
    [0x25] = {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x1F},   /* E */
    [0x28] = {0x11, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11},   /* H */
    [0x29] = {0x0E, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0E},   /* I */
    [0x2C] = {0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1F},   /* L */
    [0x2F] = {0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E},   /* O */
    [0x30] = {0x1E, 0x11, 0x11, 0x1E, 0x10, 0x10, 0x10},   /* P */
    [0x33] = {0x0E, 0x11, 0x10, 0x0E, 0x01, 0x11, 0x0E},   /* S */
    [0x36] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x0A, 0x04},   /* V */
    [0x37] = {0x11, 0x11, 0x11, 0x15, 0x15, 0x1B, 0x11},   /* W */
};

/* ---------------- 底层 ---------------- */

static int oled_write(uint8_t ctrl, const uint8_t *buf, uint16_t len, uint32_t timeout)
{
    if ((buf == 0) || (len == 0u))
    {
        return -1;
    }

    if (HAL_I2C_Mem_Write(&hi2c1,
                          OLED_ADDR_8BIT,
                          ctrl,
                          I2C_MEMADD_SIZE_8BIT,
                          (uint8_t *)buf,
                          len,
                          timeout) != HAL_OK)
    {
        return -1;
    }

    return 0;
}

static int oled_cmd(const uint8_t *cmd, uint16_t len)
{
    return oled_write(OLED_CTRL_CMD, cmd, len, OLED_CMD_TIMEOUT);
}

/**
 * @brief 在显存里点一个像素
 *
 * @param x 0~127
 * @param y 0~63（0 是屏幕最上面）
 *
 * 注意 bit 顺序：这块屏是 **bit0 在页的最上面、bit7 在最下面**
 * （SSD1306 手册写的是 bit7 在上，但这块 SSD1315 模块是反的）。
 * 之前按 bit7 在上写，结果整个画面上下翻转：
 *   - H / E / O 上下对称，看不出来
 *   - L 会变成 Γ，"L 颠倒"
 *   - 跨两页的字表现成"上下两半被调换"
 */
void oled_pixel(uint8_t x, uint8_t y, uint8_t on)
{
    uint16_t idx;
    uint8_t  bit;

    if ((x >= OLED_WIDTH) || (y >= 64u))
    {
        return;
    }

    bit = (uint8_t)(y & 7u);
    idx = (uint16_t)(y >> 3) * OLED_WIDTH + x;

    if (on != 0u)
    {
        s_gram[idx] |= (uint8_t)(1u << bit);
    }
    else
    {
        s_gram[idx] &= (uint8_t)~(1u << bit);
    }
}

/* ---------------- 对外接口 ---------------- */

int oled_init(void)
{
    s_ready = 0u;

    if (oled_cmd(s_init_seq, (uint16_t)sizeof(s_init_seq)) != 0)
    {
        return -1;
    }

    oled_clear();

    s_ready = 1u;

    oled_flush();

    return 0;
}

void oled_fill(uint8_t pattern)
{
    uint16_t i;

    for (i = 0; i < OLED_GRAM_SIZE; i++)
    {
        s_gram[i] = pattern;
    }
}

void oled_clear(void)
{
    oled_fill(0x00u);
}

/**
 * @brief 把显存推到屏幕
 *
 * 用"页寻址"：每页先发 0xB0+页号、0x00、0x10 指定起始列，
 * 再连续写 128 字节。这种写法在 SSD1306 / SSD1315 / SH1106 上都通用，
 * 不依赖 0x21/0x22 窗口指令（有些控制器不认）。
 */
void oled_flush(void)
{
    uint8_t page;
    uint8_t cmds[3];

    if (s_ready == 0u)
    {
        return;
    }

    for (page = 0u; page < OLED_PAGES; page++)
    {
        cmds[0] = (uint8_t)(0xB0u + page);   /* 选中第 page 页   */
        cmds[1] = 0x00u;                     /* 列地址低 4 位 = 0 */
        cmds[2] = 0x10u;                     /* 列地址高 4 位 = 0 */

        if (oled_cmd(cmds, 3u) != 0)
        {
            return;
        }

        if (oled_write(OLED_CTRL_DATA,
                       &s_gram[(uint16_t)page * OLED_WIDTH],
                       (uint16_t)OLED_WIDTH,
                       OLED_FLUSH_TIMEOUT) != 0)
        {
            return;
        }
    }
}

const uint8_t *oled_buffer(void)
{
    return s_gram;
}

void oled_text(uint8_t x, uint8_t page, const char *s)
{
    uint8_t y0;
    uint8_t idx;
    uint8_t row;
    uint8_t col;
    uint8_t bits;

    if ((s == 0) || (page > 6u))
    {
        return;
    }

    y0 = (uint8_t)(page * 8u);

    while (*s != '\0')
    {
        if ((*s < 0x20) || (*s > 0x7E))
        {
            s++;
            x = (uint8_t)(x + OLED_TEXT_ADVANCE);
            continue;
        }

        idx = (uint8_t)(*s - 0x20);

        /* 每个字符 7 行 × 5 列，按 X_SCALE / Y_SCALE 放大 */
        for (row = 0u; row < 7u; row++)
        {
            bits = s_font5x7[idx][row];

            for (col = 0u; col < 5u; col++)
            {
                if ((bits & (uint8_t)(0x10u >> col)) != 0u)
                {
                    uint8_t px = (uint8_t)(x + (col * OLED_TEXT_XSCALE));
                    uint8_t py = (uint8_t)(y0 + (row * OLED_TEXT_YSCALE));
                    uint8_t i;
                    uint8_t j;

                    for (i = 0u; i < OLED_TEXT_XSCALE; i++)
                    {
                        for (j = 0u; j < OLED_TEXT_YSCALE; j++)
                        {
                            oled_pixel((uint8_t)(px + i), (uint8_t)(py + j), 1u);
                        }
                    }
                }
            }
        }

        x = (uint8_t)(x + OLED_TEXT_ADVANCE);
        s++;
    }
}
