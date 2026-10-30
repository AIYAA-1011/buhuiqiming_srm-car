//
// 编码器读取（TIM2/3/4/5 编码器模式，16 位）
//
// 电机参数：
//   编码器 13 线 × 4 倍频 × 减速比 30 = 每输出圈 1560 个计数
//   轮径 60 mm -> 周长 188.5 mm
//   1 个计数 ≈ 0.1208 mm
//
// 逻辑下标和电机一致：
//   0 = 左前   1 = 右前   2 = 左后   3 = 右后
//
// 哪个定时器接哪个轮子暂时按 TIM2/3/4/5 -> 0/1/2/3 排，
// 实测不对就改 s_tim 那张表。
//

#ifndef BUHUIQIMING_SRM_CAR_ENCODE_H
#define BUHUIQIMING_SRM_CAR_ENCODE_H

#include <stdint.h>

#define ENC_COUNT       4u

/**
 * @brief 启动 4 路编码器接口，计数清零
 */
void encode_init(void);

/**
 * @brief 周期调用：读计数、算增量、算速度
 *
 * @param dt_ms 距上次调用的时间（ms），用来换算速度
 */
void encode_update(uint32_t dt_ms);

/**
 * @brief 累计计数（带符号，正 = 车往前走）
 */
int32_t encode_count(uint8_t idx);

/**
 * @brief 上一次 update 的增量
 */
int32_t encode_delta(uint8_t idx);

/**
 * @brief 上一次 update 算出来的速度（mm/s）
 */
float encode_speed_mm_s(uint8_t idx);

#endif // BUHUIQIMING_SRM_CAR_ENCODE_H
