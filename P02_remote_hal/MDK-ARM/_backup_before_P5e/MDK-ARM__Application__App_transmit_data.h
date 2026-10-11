#ifndef __APP_TRANSMIT_DATA__
#define __APP_TRANSMIT_DATA__

#include "Int_SI24R1.h"
#include "App_process_data.h"
#include "FreeRTOS.h"
#include "task.h"

// 定义帧头校验的值
#define FRAME_HEAD_CHECK_1 's'
#define FRAME_HEAD_CHECK_2 'g'
#define FRAME_HEAD_CHECK_3 'g'

/**
 * @brief 链路统计
 *
 * 靠 SI24R1 的硬件自动应答（Auto-ACK）：
 * 遥控器每发一帧，小车的无线芯片收到后会自动回一个 ACK 包，
 * Int_SI24R1_TxPacket() 返回 0 就说明小车确实收到了。
 * 这是无线芯片的硬件行为，不需要小车端写任何代码。
 *
 * 注意：ACK 是小车的【无线芯片】回的，不需要小车的 MCU 参与，
 *       所以 LNK 100% 只代表"小车无线模块在线且配置正确"，
 *       并不代表小车的 MCU 正常工作。
 */
typedef struct
{
    uint8_t valid;   /* 1 = 这个统计窗口内有成功的发送，percent 才有意义 */
    uint8_t percent; /* 0-100，发送成功率 */
} Link_Stat;

extern Link_Stat link_stat;

/**
 * @brief 把命令状态打包成一帧并发送（com_task 里每 10ms 调一次）
 */
void App_transmit_data(void);

#endif // __APP_TRANSMIT_DATA__
