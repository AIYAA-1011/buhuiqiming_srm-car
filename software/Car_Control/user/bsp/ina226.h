//
// INA226 电流/电压/功率传感器（I2C1, 0x40）
//
// 只做寄存器层：探测、配置、校准、读电压/电流/功率。
// 不负责"电量百分比"——那是显示层的事。
//
// 接线假设：
//   采样电阻 3 mΩ，接在 IN+ / IN- 之间
//   满量程电流 = 81.92 mV / R
//   本工程 3 mΩ -> 27.31 A，分辨率 0.833 mA
//

#ifndef BUHUIQIMING_SRM_CAR_INA226_H
#define BUHUIQIMING_SRM_CAR_INA226_H

#include <stdint.h>

/* 7 位地址（A0/A1 都接地） */
#define INA226_ADDR_7BIT        0x40u

/* 电流方向：如果放电时读出来是负数，把这里改成 -1 */
#ifndef INA226_CURRENT_SIGN
#define INA226_CURRENT_SIGN     (+1)
#endif

/**
 * @brief 初始化：探测芯片、写配置、写校准
 *
 * @param r_shunt_mohm 采样电阻，单位毫欧（本车是 3）
 * @return 0 = 成功
 *        -1 = I2C 不通
 *        -2 = 制造商 ID 不对（不是 INA226）
 *        -3 = 芯片 ID 不对
 *        -4 = 写配置/校准后回读不一致
 */
int ina226_init(uint16_t r_shunt_mohm);

/**
 * @brief 读一次电压 / 电流 / 功率
 *
 * @param mv 母线电压，毫伏
 * @param ma 电流，毫安（带符号，方向见 INA226_CURRENT_SIGN）
 * @param mw 功率，毫瓦（= V × I，带符号）
 * @return 0 = 成功；-1 = I2C 失败或未初始化
 */
int ina226_read(int32_t *mv, int32_t *ma, int32_t *mw);

#endif // BUHUIQIMING_SRM_CAR_INA226_H
