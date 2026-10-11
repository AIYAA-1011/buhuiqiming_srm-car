#include "Int_key.h"

/* 去抖：连续 KEY_DEBOUNCE_CNT 次读数一致才认可
 * key_task 周期 20ms => 2 次约 20ms 的确认延迟 */
#define KEY_DEBOUNCE_CNT 2

/**
 * @brief 扫描一次按键（非阻塞）
 *
 * 低电平有效：按键另一端接地，MCU 引脚配了内部上拉。
 */
uint8_t Int_key_scan(void)
{
    static uint8_t s_stable   = 0; /* 去抖之后稳定下来的按键状态 */
    static uint8_t s_last_raw = 0; /* 上一次的原始读数 */
    static uint8_t s_cnt      = 0; /* 连续读数一致的次数 */
    uint8_t raw = 0;

    if (HAL_GPIO_ReadPin(KEY_UP_GPIO_Port,      KEY_UP_Pin)      == GPIO_PIN_RESET) raw |= KEY_UP;
    if (HAL_GPIO_ReadPin(KEY_DOWN_GPIO_Port,    KEY_DOWN_Pin)    == GPIO_PIN_RESET) raw |= KEY_DOWN;
    if (HAL_GPIO_ReadPin(KEY_LEFT_GPIO_Port,    KEY_LEFT_Pin)    == GPIO_PIN_RESET) raw |= KEY_LEFT;
    if (HAL_GPIO_ReadPin(KEY_RIGHT_GPIO_Port,   KEY_RIGHT_Pin)   == GPIO_PIN_RESET) raw |= KEY_RIGHT;
    if (HAL_GPIO_ReadPin(KEY_LEFT_X_GPIO_Port,  KEY_LEFT_X_Pin)  == GPIO_PIN_RESET) raw |= KEY_LEFT_X;
    if (HAL_GPIO_ReadPin(KEY_RIGHT_X_GPIO_Port, KEY_RIGHT_X_Pin) == GPIO_PIN_RESET) raw |= KEY_RIGHT_X;

    if (raw == s_last_raw)
    {
        if (s_cnt < KEY_DEBOUNCE_CNT)
        {
            s_cnt++;
        }
    }
    else
    {
        s_last_raw = raw;
        s_cnt = 1;
    }

    if (s_cnt >= KEY_DEBOUNCE_CNT)
    {
        s_stable = raw;
    }

    return s_stable;
}
