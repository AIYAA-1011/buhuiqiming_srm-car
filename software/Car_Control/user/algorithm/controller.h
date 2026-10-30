//
// 速度环 PID
//
// 每个轮子一套独立参数，增益都是 volatile 全局变量，可以在 Ozone 里直接改。
//
// 输入：目标速度(mm/s)、实测速度(mm/s)、这一拍的 dt(s)
// 输出：占空比 -1000 ~ +1000
//

#ifndef BUHUIQIMING_SRM_CAR_CONTROLLER_H
#define BUHUIQIMING_SRM_CAR_CONTROLLER_H

#include <stdint.h>

#define PID_WHEEL_COUNT     4u

/* 输出限幅（和 motor 的占空比范围一致） */
#define PID_OUT_MAX         1000.0f

/* 积分限幅：防止长时间偏差把积分堆爆 */
#define PID_INTEG_MAX       2000.0f

/**
 * @brief 清空所有积分和上一拍误差（切模式/使能时调用）
 */
void pid_init(void);

/**
 * @brief 只清某一路
 */
void pid_reset(uint8_t idx);

/**
 * @brief 跑一次 PID（含前馈）
 *
 * 输出 = 前馈×目标 + P + I + D
 *
 * 前馈系数 g_pid_ff[i] 的标定方法：
 *   1. 开环模式（g_motor_mode=1）给一个固定占空比 D（比如 300）
 *   2. 读稳态速度 V（比如 380 mm/s）
 *   3. g_pid_ff[i] = D / V ≈ 0.79
 *   标好之后，同样目标下 PID 的修正量接近 0，抖动会小很多。
 *
 * @param idx    0=左前 1=右前 2=左后 3=右后
 * @param target 目标速度 mm/s
 * @param actual 实测速度 mm/s
 * @param dt     距上次调用的时间，单位秒
 * @return 占空比 -1000 ~ +1000
 */
int16_t pid_update(uint8_t idx, float target, float actual, float dt);

#endif //BUHUIQIMING_SRM_CAR_CONTROLLER_H
