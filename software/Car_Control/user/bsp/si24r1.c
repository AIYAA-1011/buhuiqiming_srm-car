//
// SI24R1 接收驱动实现
//
// 参数（地址/通道/速率/载荷宽度）是从遥控器工程 P02_remote_hal 的
// Int_SI24R1.c 里 1:1 抄过来的，改任何一项都收不到数据。
//

#include "si24r1.h"
#include "spi.h"
#include "main.h"
#include <string.h>

/* ---------------- nRF24L01 / SI24R1 命令 ---------------- */
#define CMD_READ_REG        0x00
#define CMD_WRITE_REG       0x20
#define CMD_RD_RX_PLOAD     0x61
#define CMD_FLUSH_TX        0xE1
#define CMD_FLUSH_RX        0xE2

/* ---------------- 寄存器地址 ---------------- */
#define REG_CONFIG          0x00
#define REG_EN_AA           0x01
#define REG_EN_RXADDR       0x02
#define REG_SETUP_AW        0x03
#define REG_RF_CH           0x05
#define REG_RF_SETUP        0x06
#define REG_STATUS          0x07
#define REG_RX_ADDR_P0      0x0A
#define REG_TX_ADDR         0x10
#define REG_RX_PW_P0        0x11

/* ---------------- STATUS 位 ---------------- */
#define STATUS_RX_DR        0x40

/* ---------------- 必须和遥控器一致的参数 ---------------- */
#define RF_CHANNEL          40          /* 遥控器: CHANNEL 40          */
#define RF_SETUP_VAL        0x06        /* 1 Mbps, 0 dBm               */
#define CONFIG_RX           0x0F        /* EN_CRC|CRCO|PWR_UP|PRIM_RX  */
#define ADDR_WIDTH          5

/* SPI 单次传输超时：短一点，模块坏的时候任务不至于卡 100ms */
#define SPI_TIMEOUT_MS      10

/* FIFO 只有 3 层，循环读时加个保护，防止芯片异常导致死循环 */
#define RX_DRAIN_MAX        4

static const uint8_t s_addr[ADDR_WIDTH] = { 0x0A, 0x01, 0x06, 0x1E, 0x01 };

/* 1 = 初始化成功，可以轮询；0 = 初始化失败，poll 直接返回，不碰 SPI */
static uint8_t s_ready = 0;

/* ---------------- 片选 / 使能 ---------------- */
#define CS_LOW()    HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_RESET)
#define CS_HIGH()   HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_SET)
#define CE_LOW()    HAL_GPIO_WritePin(NRF_CE_GPIO_Port,  NRF_CE_Pin,  GPIO_PIN_RESET)
#define CE_HIGH()   HAL_GPIO_WritePin(NRF_CE_GPIO_Port,  NRF_CE_Pin,  GPIO_PIN_SET)

/* ---------------- 底层 SPI ---------------- */

static uint8_t spi_rw(uint8_t tx)
{
    uint8_t rx = 0;
    (void)HAL_SPI_TransmitReceive(&hspi1, &tx, &rx, 1, SPI_TIMEOUT_MS);
    return rx;
}

static uint8_t reg_write(uint8_t reg, uint8_t value)
{
    uint8_t status;

    CS_LOW();
    status = spi_rw(reg);
    (void)spi_rw(value);
    CS_HIGH();

    return status;
}

static uint8_t reg_write_buf(uint8_t reg, const uint8_t *buf, uint8_t len)
{
    uint8_t status;
    uint8_t i;

    CS_LOW();
    status = spi_rw(reg);
    for (i = 0; i < len; i++)
    {
        (void)spi_rw(buf[i]);
    }
    CS_HIGH();

    return status;
}

static uint8_t reg_read(uint8_t reg)
{
    uint8_t value;

    CS_LOW();
    (void)spi_rw(reg);
    value = spi_rw(0x00);
    CS_HIGH();

    return value;
}

static uint8_t reg_read_buf(uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t status;
    uint8_t i;

    CS_LOW();
    status = spi_rw(reg);
    for (i = 0; i < len; i++)
    {
        buf[i] = spi_rw(0x00);
    }
    CS_HIGH();

    return status;
}

/* ---------------- 配置 ---------------- */

/**
 * @brief 常驻接收模式
 *
 * 遥控器是"常驻发送"，小车这边上电配一次、一直收，不需要切模式。
 */
static void rx_mode_config(void)
{
    CE_LOW();

    /* 接收通道 0 用和遥控器 TX 完全相同的地址 */
    (void)reg_write_buf(CMD_WRITE_REG + REG_RX_ADDR_P0, s_addr, ADDR_WIDTH);

    (void)reg_write(CMD_WRITE_REG + REG_EN_AA,      0x01);   /* 通道 0 自动应答 */
    (void)reg_write(CMD_WRITE_REG + REG_EN_RXADDR,  0x01);   /* 使能通道 0     */
    (void)reg_write(CMD_WRITE_REG + REG_SETUP_AW,   0x03);   /* 地址宽度 5 字节 */
    (void)reg_write(CMD_WRITE_REG + REG_RF_CH,      RF_CHANNEL);
    (void)reg_write(CMD_WRITE_REG + REG_RX_PW_P0,   SI24R1_FRAME_LEN);
    (void)reg_write(CMD_WRITE_REG + REG_RF_SETUP,   RF_SETUP_VAL);
    (void)reg_write(CMD_WRITE_REG + REG_CONFIG,     CONFIG_RX);
    (void)reg_write(CMD_WRITE_REG + REG_STATUS,     0xFF);   /* 清所有中断标志 */

    /* 清空两个 FIFO，避免上电残留的旧数据 */
    (void)reg_write(CMD_FLUSH_RX, 0xFF);
    (void)reg_write(CMD_FLUSH_TX, 0xFF);

    CE_HIGH();                                               /* 开始接收 */
}

/* ---------------- 对外接口 ---------------- */

int si24r1_self_test(void)
{
    uint8_t buf[ADDR_WIDTH];
    uint8_t i;

    /* 先读一次，保证 SPI 已同步 */
    (void)reg_read_buf(CMD_READ_REG + REG_RX_ADDR_P0, buf, ADDR_WIDTH);

    (void)reg_write_buf(CMD_WRITE_REG + REG_TX_ADDR, s_addr, ADDR_WIDTH);
    (void)reg_read_buf(CMD_READ_REG  + REG_TX_ADDR, buf, ADDR_WIDTH);

    for (i = 0; i < ADDR_WIDTH; i++)
    {
        if (buf[i] != s_addr[i])
        {
            return -1;
        }
    }

    return 0;
}

int si24r1_init(void)
{
    uint8_t retry = 20;

    s_ready = 0;

    /* 芯片上电后需要时间稳定（遥控器那边等 200ms，这里 50ms 够） */
    HAL_Delay(50);

    while (si24r1_self_test() != 0)
    {
        if (--retry == 0)
        {
            return -1;
        }
        HAL_Delay(10);
    }

    rx_mode_config();

    s_ready = 1;
    return 0;
}

int si24r1_poll(uint8_t *newest)
{
    uint8_t status;
    uint8_t got = 0;
    uint8_t guard = 0;
    uint8_t tmp[SI24R1_FRAME_LEN];

    if ((newest == 0) || (s_ready == 0))
    {
        return 0;
    }

    for (;;)
    {
        status = reg_read(REG_STATUS);

        /* STATUS 的标志位是"写 1 清除" */
        (void)reg_write(CMD_WRITE_REG + REG_STATUS, status);

        if ((status & STATUS_RX_DR) == 0)
        {
            break;
        }

        (void)reg_read_buf(CMD_RD_RX_PLOAD, tmp, SI24R1_FRAME_LEN);
        memcpy(newest, tmp, SI24R1_FRAME_LEN);
        got = 1;

        /* FIFO 只有 3 层，读满 4 次还不空就是芯片异常，别死循环 */
        if (++guard >= RX_DRAIN_MAX)
        {
            break;
        }
    }

    return (got != 0) ? 1 : 0;
}
