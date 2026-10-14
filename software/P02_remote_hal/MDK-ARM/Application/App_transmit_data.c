#include "App_transmit_data.h"
#include "App_command.h"

/* 一帧 17 字节的发送缓冲 */
uint8_t transmit_buff[TX_PLOAD_WIDTH] = {0};

/* 每 LINK_WINDOW_SENDS 次发送结算一次成功率（发送周期约 10ms => 约 500ms） */
#define LINK_WINDOW_SENDS 50

/* 链路统计结果 */
Link_Stat link_stat = {0};

static uint16_t s_tx_total = 0;
static uint16_t s_tx_ok    = 0;

/**
 * @brief 打包并发送一帧遥控数据
 *
 * 帧格式（17 字节）：
 *   [0..2]   帧头 's' 'g' 'g'
 *   [3..10]  四个轴，有符号 16 位大端，值域 -500 ~ +500，0 = 停止
 *            顺序：ly(左杆Y) lx(左杆X) ry(右杆Y) rx(右杆X)
 *   [11]     命令标志位  bit0 急停  bit1 夹爪  bit2 模式
 *                       bit3 过洞  bit4 取矿  bit5 兑矿
 *   [12]     预留
 *   [13..16] [0..12] 共 13 字节的累加和，大端 32 位
 */
void App_transmit_data(void)
{
    uint32_t sum = 0;
    int16_t out_axis[4];
    uint8_t k;
    uint8_t res;

    /* ---------- 帧头 ---------- */
    transmit_buff[0] = FRAME_HEAD_CHECK_1;
    transmit_buff[1] = FRAME_HEAD_CHECK_2;
    transmit_buff[2] = FRAME_HEAD_CHECK_3;

    /* ---------- 四个轴 ----------
     * 未解锁（还在等摇杆回中）或者急停锁定时，
     * App_command_get_output() 会把四个轴全部置 0 */
    App_command_get_output(out_axis);

    for (k = 0; k < 4; k++)
    {
        uint16_t u = (uint16_t)out_axis[k]; /* 负数用二进制补码 */
        transmit_buff[3 + k * 2]     = (uint8_t)(u >> 8);
        transmit_buff[3 + k * 2 + 1] = (uint8_t)(u & 0xFF);
    }

    /* ---------- 命令标志位 ----------
     * 电平语义：每帧都把当前状态原样发一遍，不做"发送后清零"。
     * 这样即使某几帧丢了，下一帧会自动补上，
     * 急停 / 夹爪 / 模式 / 预设都不会因为丢一帧而永久丢失。 */
    transmit_buff[11] = (uint8_t)(
          (cmd.estop   ? 0x01 : 0)
        | (cmd.gripper ? 0x02 : 0)
        | ((cmd.mode == MODE_ARM)          ? 0x04 : 0)
        | ((cmd.preset == CMD_PRESET_HOLE) ? 0x08 : 0)
        | ((cmd.preset == CMD_PRESET_PICK) ? 0x10 : 0)
        | ((cmd.preset == CMD_PRESET_PUT)  ? 0x20 : 0));

    /* [12] 预留：将来可以放"底座回转""速度档位"之类 */
    transmit_buff[12] = 0;

    /* ---------- 累加和 ---------- */
    for (uint8_t i = 0; i < 13; i++)
    {
        sum += transmit_buff[i];
    }
    // 高位在前
    transmit_buff[13] = (sum >> 24) & 0xFF;
    transmit_buff[14] = (sum >> 16) & 0xFF;
    transmit_buff[15] = (sum >> 8) & 0xFF;
    transmit_buff[16] = sum & 0xFF;

    /* ---------- 发送 ----------
     * 上电时已经配成常驻发送模式（见 Int_SI24R1_Init），
     * 这里不用再切模式，也不用切回接收模式等回传 */
    res = Int_SI24R1_TxPacket(transmit_buff);

    /* ---------- 链路统计 ----------
     * res == 0 表示无线芯片收到了小车回的 ACK => 小车确实收到了这一帧。
     * 这一步是 SI24R1 硬件自动完成的，不需要小车端配合。 */
    s_tx_total++;
    if (res == 0)
    {
        s_tx_ok++;
    }

    if (s_tx_total >= LINK_WINDOW_SENDS)
    {
        link_stat.percent = (uint8_t)((uint32_t)s_tx_ok * 100 / s_tx_total);
        link_stat.valid   = (s_tx_ok > 0) ? 1 : 0;
        s_tx_total = 0;
        s_tx_ok    = 0;
    }
}
