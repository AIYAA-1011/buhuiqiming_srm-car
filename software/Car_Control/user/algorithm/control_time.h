//
// 控制周期
//
// 作用：让控制循环"每 20 ms 跑一次"，并给出这一拍实际的 dt。
//
// 为什么不用裸 osDelay(20)：
//   osDelay(20) 是"干完活再睡 20 ms"，
//   实际周期 = 20 + 计算耗时，会越跑越慢，而且抖动大。
//
// 这里用"目标时刻"对齐：
//   睡到下一个整周期点，然后如实测出 dt 交给 PID 用。
//

#ifndef BUHUIQIMING_SRM_CAR_CONTROL_TIME_H
#define BUHUIQIMING_SRM_CAR_CONTROL_TIME_H

#include <stdint.h>

/* 控制周期（ms） */
#define CTRL_PERIOD_MS      20u

/* dt 合理范围：超出就当异常，夹住 */
#define CTRL_DT_MIN_MS      1u
#define CTRL_DT_MAX_MS      100u

/**
 * @brief 初始化：记录起始时刻
 */
void ctrl_time_init(void);

/**
 * @brief 睡到下一个周期点
 *
 * 每个循环末尾调用一次。
 *
 * @return 这一拍实际经过的时间（ms）
 */
uint32_t ctrl_time_wait(void);

/**
 * @brief 最近一次测到的 dt（ms），Ozone 里看周期稳不稳
 */
uint32_t ctrl_time_dt(void);

#endif // BUHUIQIMING_SRM_CAR_CONTROL_TIME_H
