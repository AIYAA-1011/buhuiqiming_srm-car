//
// 底盘电机驱动（DRV8874 × 4）
//
// 硬件：
//   TIM1 CH1~CH4 -> PA8~PA11   每路 20 kHz PWM（PSC=0, ARR=3599）
//   DIR1~DIR4    -> PB12~PB15  方向
//   MOTOR_EN     -> PC11       高 = 使能
//
// 逻辑下标（本文件对外统一用这个顺序）：
//   0 = 左前   1 = 右前   2 = 左后   3 = 右后
//
// 实物编号：
//   左前 = M4(CH4/DIR4)   右前 = M1(CH1/DIR1)
//   左后 = M3(CH3/DIR3)   右后 = M2(CH2/DIR2)
//

#ifndef BUHUIQIMING_SRM_CAR_MOTOR_H
#define BUHUIQIMING_SRM_CAR_MOTOR_H

#include <stdint.h>

/* 电机数量 */
#define MOTOR_COUNT     4u

/* 占空比满量程（-1000 ~ +1000 对应 -100% ~ +100%） */
#define MOTOR_DUTY_MAX  1000

/**
 * @brief 初始化：启动 4 路 PWM、方向清零、驱动先禁用
 *
 * 上电不使能，必须显式调用 motor_enable(1)，避免上电乱转。
 */
void motor_init(void);

/**
 * @brief 总使能（PC11）
 * @param on 1 = 使能，0 = 禁用（电机自由滑行）
 */
void motor_enable(uint8_t on);

/**
 * @brief 设置某个电机的占空比
 *
 * @param idx  0=左前 1=右前 2=左后 3=右后
 * @param duty -1000 ~ +1000；符号表示方向，0 = 停
 */
void motor_set(uint8_t idx, int16_t duty);

/**
 * @brief 四个电机全部停（占空比 0）
 */
void motor_stop_all(void);

/**
 * @brief 读回最后一次下发的占空比（调试用）
 */
int16_t motor_get_duty(uint8_t idx);

#endif // BUHUIQIMING_SRM_CAR_MOTOR_H
