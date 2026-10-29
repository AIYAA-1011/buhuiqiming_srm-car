//
// SI24R1 / nRF24L01+ 接收驱动（小车端只收不发）
//
// 参数（地址/通道/速率/载荷宽度）必须和遥控器 P02_remote_hal 一致，
// 改任何一项都收不到数据。
//

#ifndef BUHUIQIMING_SRM_CAR_SI24R1_H
#define BUHUIQIMING_SRM_CAR_SI24R1_H

#include <stdint.h>

/* 遥控器一帧固定 17 字节 */
#define SI24R1_FRAME_LEN    17

/**
 * @brief 初始化无线芯片（常驻接收模式）
 *
 * 内部会做自检：往 TX_ADDR 写地址再读回来比对，最多重试 20 次。
 *
 * @return 0 = 成功；-1 = 自检失败（模块没插好 / SPI 不通）
 */
int si24r1_init(void);

/**
 * @brief 把 FIFO 里积压的帧全部读完，只保留最新的一帧
 *
 * 不阻塞、不延时。初始化失败时直接返回 0，不会去碰 SPI。
 *
 * @param newest 至少 SI24R1_FRAME_LEN 字节
 * @return 1 = 收到过数据（newest 里是最新一帧）；0 = 没有新数据
 */
int si24r1_poll(uint8_t *newest);

/**
 * @brief 单独跑一次自检（写 TX_ADDR 再读回比对）
 * @return 0 = 通过；-1 = 失败
 */
int si24r1_self_test(void);

/* ---------------- 调试用 ---------------- */

/**
 * @brief 读 STATUS（0x07）
 *
 * bit6 = RX_DR 收到包，bit5 = TX_DS，bit4 = MAX_RT
 * 读不会清标志（清标志在 si24r1_poll 里做）。
 */
uint8_t si24r1_status(void);

/**
 * @brief 读 FIFO_STATUS（0x17）
 *
 * bit0 = RX_EMPTY（1 表示接收 FIFO 空），bit4 = TX_EMPTY
 */
uint8_t si24r1_fifo_status(void);

/**
 * @brief 取"拉高 CE 之前"的配置快照
 *
 * @param cfg   CONFIG（期望 0x0F）
 * @param en_aa EN_AA（期望 0x01）
 * @param rf_ch RF_CH（期望 0x28 = 40）
 * @param pw0   RX_PW_P0（期望 0x11 = 17）
 * @param setup RF_SETUP（期望 0x06 = 1 Mbps）
 * @return 0 = 成功
 */
int si24r1_read_config(uint8_t *cfg, uint8_t *en_aa, uint8_t *rf_ch,
                       uint8_t *pw0, uint8_t *setup);

/**
 * @brief 取"拉高 CE 之后"的配置快照（和上面一样，只是时刻不同）
 */
int si24r1_live_config(uint8_t *cfg, uint8_t *en_aa, uint8_t *rf_ch,
                       uint8_t *pw0, uint8_t *setup);

/**
 * @brief 取 CE 拉高前后的 STATUS 快照
 *
 * 如果 before 正常（0x0E）而 after 变成 0x00/0xFF，
 * 说明 CE 一拉高芯片就不响应了（供电塌陷）。
 */
int si24r1_ce_effect(uint8_t *before, uint8_t *after);

/**
 * @brief 读 RPD（Received Power Detector）
 *
 * @return 1 = 当前通道上有射频能量（遥控器在发）；0 = 没检测到
 *
 * 判据是 -64 dBm 以上，所以离得很近时也应该是 1。
 */
uint8_t si24r1_rpd(void);

/**
 * @brief 立刻读一次关键寄存器（不是快照，是现在的真实值）
 *
 * 用来确认芯片运行一段时间后是否还保持接收配置：
 *   cfg 期望 0x0F，en_rxaddr 期望 0x01，rf_ch 期望 0x28，pw0 期望 0x11
 * 如果 cfg 变成 0x08/0x00，说明芯片中途复位了（供电问题）。
 */
int si24r1_read_now(uint8_t *cfg, uint8_t *en_aa, uint8_t *en_rxaddr,
                    uint8_t *rf_ch, uint8_t *pw0);

#endif // BUHUIQIMING_SRM_CAR_SI24R1_H
