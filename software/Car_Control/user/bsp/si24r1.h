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

#endif // BUHUIQIMING_SRM_CAR_SI24R1_H
