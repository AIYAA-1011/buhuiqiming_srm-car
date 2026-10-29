//
// OLED 驱动（SSD1306 / SSD1315，I2C1，0x3C）
//
// 128x64 单色屏，显存 1024 字节（8 页 × 128 列）。
// 只做"显存 → 屏幕"这一层，画什么由 display_task 决定。
//

#ifndef BUHUIQIMING_SRM_CAR_OLED_H
#define BUHUIQIMING_SRM_CAR_OLED_H

#include <stdint.h>

/* 7 位地址（模块背面标 0x78 就是它） */
#define OLED_ADDR_7BIT      0x3Cu

/**
 * @brief 初始化屏幕
 * @return 0 = 成功（所有命令都被 ACK）；-1 = I2C 失败
 *
 * 注意：内部会把显存清空并刷一次，所以调用后屏幕是黑的。
 */
int oled_init(void);

/**
 * @brief 显存全部填成 pattern（0x00 = 黑，0xFF = 白）
 */
void oled_fill(uint8_t pattern);

/**
 * @brief 清屏（等价于 oled_fill(0x00)）
 */
void oled_clear(void);

/**
 * @brief 把显存推到屏幕（约 25 ms，阻塞）
 */
void oled_flush(void);

/**
 * @brief 在 (x, page) 处显示字符串（5×7 字库，只占一页）
 *
 * 每个字符 5×7 像素、字距 6，一行最多 21 个字符。
 * 字符只画在 page 这一页的 8 行内，不跨页
 * （这块屏跨页时上下两半会被对调，所以纵向不放大）。
 *
 * @param x    起始列，0~127
 * @param page 页号，0~7（第 0 页是最上面 8 行）
 * @param s    以 '\0' 结尾的 ASCII 字符串
 */
void oled_text(uint8_t x, uint8_t page, const char *s);

/**
 * @brief 在显存里点一个像素（0,0 是左上角）
 */
void oled_pixel(uint8_t x, uint8_t y, uint8_t on);

/**
 * @brief 取显存指针（1024 字节，8 页 × 128 列），调试用
 */
const uint8_t *oled_buffer(void);

#endif //BUHUIQIMING_SRM_CAR_OLED_H
