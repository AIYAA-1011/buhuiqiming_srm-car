//
// PCA9685 —— 16 路 12 位 PWM（机械臂舵机用）
//
// 这里只放"硬件层"的东西：寄存器、读写、设脉宽、开关输出（OE）。
// 角度换算 / 标定表 / 软限幅 / 速度 全部在 arm_task.c 里，
// 这样以后重新标定舵机不用动 BSP。
//
// 接线（这块板子已经配好，不用改 CubeMX）：
//   SCL = PB10 , SDA = PB11   → I2C2，400 kHz
//   OE  = PC12 (SERVO_OE_N)   → 低电平使能
//   VCC = 3.3 V（逻辑） , V+ = 6.0 V（舵机）
//
// 时间基准：50 Hz（周期 20 ms），12 位分辨率
//   1 count = 20000 us / 4096 = 4.88 us
//   1500 us → 307 counts
//

#ifndef BUHUIQIMING_SRM_CAR_PCA9685_H
#define BUHUIQIMING_SRM_CAR_PCA9685_H

#include <stdint.h>

/* ---------------- I2C 地址 ----------------
 * 7 位地址：A0~A5 全部接地 = 0x40
 * HAL 要的是 8 位地址，所以驱动内部左移一位
 */
#define PCA9685_ADDR_7BIT       0x40u

/* ---------------- 寄存器 ---------------- */
#define PCA9685_REG_MODE1       0x00u
#define PCA9685_REG_MODE2       0x01u
#define PCA9685_REG_LED0_ON_L   0x06u   /* LED0~LED15 连续 64 字节 */
#define PCA9685_REG_PRESCALE    0xFEu

/* MODE1 的位 */
#define PCA9685_MODE1_RESTART   0x80u
#define PCA9685_MODE1_EXTCLK    0x40u
#define PCA9685_MODE1_AI        0x20u   /* 自动递增：连续写多字节时用 */
#define PCA9685_MODE1_SLEEP     0x10u   /* 睡眠：改 PRE_SCALE 前必须置 1 */
#define PCA9685_MODE1_ALLCALL   0x01u

/* MODE2 的位 */
#define PCA9685_MODE2_INVRT     0x10u
#define PCA9685_MODE2_OUTDRV    0x04u   /* 1 = 推挽，0 = 开漏 */
#define PCA9685_MODE2_OCH       0x08u

/* 50 Hz：25 MHz / (4096 * 50) - 1 = 121 */
#define PCA9685_PRESCALE_50HZ   121u

/* 通道数 / 脉宽安全范围 */
#define PCA9685_CH_COUNT        16u
#define PCA9685_US_MIN          500u
#define PCA9685_US_MAX          2500u

/* ==================================================================
 * 第 1 步：探测（只读，不输出 PWM）
 * ================================================================== */

/**
 * @brief 读一个寄存器
 * @return 0 = 成功；-1 = I2C 失败（没应答 / 超时）
 */
int pca9685_read_reg(uint8_t reg, uint8_t *val);

/**
 * @brief 写一个寄存器
 * @return 0 = 成功；-1 = I2C 失败
 */
int pca9685_write_reg(uint8_t reg, uint8_t val);

/**
 * @brief 探测芯片是否存在
 *
 * 判据：I2C 有应答，且 MODE1 读回来不是 0xFF。
 * （总线悬空时 HAL 偶尔也会返回 OK，所以要再看一下数据。）
 *
 * @return 0 = 在；-1 = 不在
 */
int pca9685_detect(void);

/**
 * @brief 扫一遍 I2C2 总线（只在探测失败时调用，用来查地址/接线错误）
 *
 * @param found 找到的 7 位地址列表（调用者提供缓冲）
 * @param max   缓冲最多放几个
 * @return 实际找到的器件个数
 */
int pca9685_scan(uint8_t *found, int max);

/* ==================================================================
 * 第 2 步：初始化 / 设脉宽 / 开关输出
 * ================================================================== */

/**
 * @brief 初始化成 50 Hz 舵机模式
 *
 * 顺序：探测 → SLEEP → PRESCALE=121 → MODE2 推挽 → 唤醒(AI) → 等 2 ms → 回读校验
 *
 * 注意：**这里不会开 OE**，输出仍然是关断的。
 *       要开输出必须自己再调 pca9685_output_enable()，
 *       而且必须先把 5 路都写成有效脉宽再开，否则舵机上电会乱动。
 *
 * @return 0 = 成功；-1 = 探测失败 / 写失败 / 回读校验不通过
 */
int pca9685_init(void);

/**
 * @brief 直接写某个通道的 12 位 ON/OFF 计数
 * @param on  0~4095
 * @param off 0~4095
 * @return 0 = 成功；-1 = 通道越界或 I2C 失败
 */
int pca9685_set_counts(uint8_t ch, uint16_t on, uint16_t off);

/**
 * @brief 按脉宽（us）设置某个通道
 *
 * 会自动夹到 [PCA9685_US_MIN, PCA9685_US_MAX]（500~2500 us）。
 * 关节的 min/max 是另一层限制，在 arm_task 的标定表里做。
 *
 * @return 0 = 成功；-1 = 通道越界或 I2C 失败
 */
int pca9685_set_us(uint8_t ch, uint16_t us);

/**
 * @brief 16 路全部设成同一个脉宽（上电写 1500 用）
 * @return 0 = 全部成功；-1 = 至少一路失败
 */
int pca9685_set_all_us(uint16_t us);

/**
 * @brief 开输出（OE 拉低）
 *
 * 调用前必须已经把各路设成有效脉宽，否则舵机会按残留值乱动。
 */
void pca9685_output_enable(void);

/**
 * @brief 关输出（OE 拉高）
 *
 * 注意：关掉之后舵机没有保持力矩，机械臂如果受重力会掉下来。
 */
void pca9685_output_disable(void);

/**
 * @brief 当前输出是否使能
 * @return 1 = 已使能（OE 低）；0 = 关断（OE 高）
 */
int pca9685_output_enabled(void);

#endif // BUHUIQIMING_SRM_CAR_PCA9685_H
