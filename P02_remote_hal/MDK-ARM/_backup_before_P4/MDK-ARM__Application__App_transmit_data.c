#include "App_transmit_data.h"
#include "App_command.h"

uint8_t transmit_buff[TX_PLOAD_WIDTH] = {0};

uint8_t post_buff[TX_PLOAD_WIDTH] = {0};

/**
 * @brief 自动切换SI24R1的模式 => 将采集完成的遥控数据打包发送到飞机
 *
 */
void App_transmit_data(void)
{
    // 2. 发送数据 => 唯一性和可靠性
    // (1) => 帧头校验指定发送给对应的设备 唯一性
    // (2) => 发送数据的结尾 => 添加检验和(将发送的数据累加得到的值添加到数据的末尾) 可靠性

    uint32_t sum = 0;
    int16_t out_axis[4];
    uint8_t k;

    //  帧头校验设置为3字节 => 数据本身是10字节 => 校验和设置为4字节
    transmit_buff[0] = FRAME_HEAD_CHECK_1;
    transmit_buff[1] = FRAME_HEAD_CHECK_2;
    transmit_buff[2] = FRAME_HEAD_CHECK_3;

    // 取当前应该发出去的四个轴值
    // 未解锁（等摇杆回中）或者急停时，这个函数会把四个轴全部置 0
    App_command_get_output(out_axis);

    // 有符号 16 位，高 8 位在前（负数用二进制补码直接发）
    for (k = 0; k < 4; k++)
    {
        uint16_t u = (uint16_t)out_axis[k];
        transmit_buff[3 + k * 2]     = (uint8_t)(u >> 8);
        transmit_buff[3 + k * 2 + 1] = (uint8_t)(u & 0xFF);
    }

    // TODO(P4): byte11 / byte12 改成标志位
    //           （急停 / 夹爪 / 模式 / 三个预设），并去掉回传那一段
    transmit_buff[11] = 0;
    transmit_buff[12] = 0;

    for (uint8_t i = 0; i < 13; i++)
    {
        sum += transmit_buff[i];
    }
    // 高位在前
    transmit_buff[13] = (sum >> 24) & 0xFF;
    transmit_buff[14] = (sum >> 16) & 0xFF;
    transmit_buff[15] = (sum >> 8) & 0xFF;
    transmit_buff[16] = sum & 0xFF;

    // 1. 切换到发送模式
    Int_SI24R1_TX_Mode();
    uint8_t res = Int_SI24R1_TxPacket(transmit_buff);
    // 3. 切换回接收模式
    Int_SI24R1_RX_Mode();

    // 判断是否发送成功 => 发送成功接收回传数据
    if (res == 0)
    {
        uint16_t count = 500;
        // 2.4G同步发送逻辑 => 必须严格按照双端发送的时序才能数据
        while (Int_SI24R1_RxPacket(post_buff) == 1 && count--)
        {
        }
    }
}
