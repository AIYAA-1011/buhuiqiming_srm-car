//
// 底盘电机驱动实现
//
// 每路电机两样东西：
//   1 根 PWM（TIM1，20 kHz）控制速度
//   2 根 DIR（GPIO）控制方向
//   所有电机共用一个 MOTOR_EN（PC11，高有效）
//

#include "motor.h"
#include "tim.h"
#include "main.h"

/* 逻辑下标 -> 物理通道（0 表示 CH1）
 * 左前 = M4 = CH4 -> 3
 * 右前 = M1 = CH1 -> 0
 * 左后 = M3 = CH3 -> 2
 * 右后 = M2 = CH2 -> 1 */
static const uint8_t s_ch_map[MOTOR_COUNT] = { 3u, 0u, 2u, 1u };

/* 通道号 -> HAL 的 TIM_CHANNEL_x */
static const uint32_t s_tim_ch[MOTOR_COUNT] =
{
    TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4
};

/* 通道号 -> DIR 引脚 */
static const uint16_t s_dir_pin[MOTOR_COUNT] =
{
    DIR1_Pin, DIR2_Pin, DIR3_Pin, DIR4_Pin
};

static const GPIO_TypeDef *s_dir_port[MOTOR_COUNT] =
{
    DIR1_GPIO_Port, DIR2_GPIO_Port, DIR3_GPIO_Port, DIR4_GPIO_Port
};

/* 每个轮子的方向系数：
 *   +1 = 正占空比让车往前走
 *   -1 = 反过来
 *
 * 实测（+300 时）：
 *   左前 -> 往后   右前 -> 往前
 *   左后 -> 往后   右后 -> 往前
 * 左右是镜像安装的，所以左侧两路取反。 */
static const int8_t s_dir_sign[MOTOR_COUNT] = { -1, +1, -1, +1 };

/* 最近一次下发的占空比，调试用 */
static int16_t s_last_duty[MOTOR_COUNT];

/* 初始化过没有 */
static uint8_t s_inited;

/* PWM 的自动重装值（ARR），运行时读一次就够 */
static uint32_t s_arr;

/* ---------------- 内部 ---------------- */

/**
 * @brief 把 0~1000 的占空比换算成比较值
 */
static uint32_t duty_to_compare(int16_t mag)
{
    uint32_t cmp;

    /* cmp = mag * (ARR + 1) / 1000，四舍五入 */
    cmp = ((uint32_t)mag * (s_arr + 1u)) / (uint32_t)MOTOR_DUTY_MAX;

    if (cmp > s_arr)
    {
        cmp = s_arr;             /* 100% 时也不能超过 ARR，否则一直是高电平 */
    }

    return cmp;
}

/* ---------------- 对外接口 ---------------- */

void motor_init(void)
{
    uint8_t i;

    s_arr = __HAL_TIM_GET_AUTORELOAD(&htim1);

    /* 方向脚全部拉低 */
    for (i = 0u; i < MOTOR_COUNT; i++)
    {
        HAL_GPIO_WritePin((GPIO_TypeDef *)s_dir_port[i], s_dir_pin[i], GPIO_PIN_RESET);
    }

    /* 先禁用驱动，然后启动 4 路 PWM（此时占空比 0） */
    HAL_GPIO_WritePin(MOTOR_EN_GPIO_Port, MOTOR_EN_Pin, GPIO_PIN_RESET);

    (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    for (i = 0u; i < MOTOR_COUNT; i++)
    {
        __HAL_TIM_SET_COMPARE(&htim1, s_tim_ch[i], 0u);
        s_last_duty[i] = 0;
    }

    s_inited = 1u;
}

void motor_enable(uint8_t on)
{
    HAL_GPIO_WritePin(MOTOR_EN_GPIO_Port, MOTOR_EN_Pin,
                      (on != 0u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void motor_set(uint8_t idx, int16_t duty)
{
    uint8_t        ch;
    int16_t        mag;
    GPIO_PinState  dir;

    if ((idx >= MOTOR_COUNT) || (s_inited == 0u))
    {
        return;
    }

    if (duty > MOTOR_DUTY_MAX)
    {
        duty = MOTOR_DUTY_MAX;
    }
    if (duty < -MOTOR_DUTY_MAX)
    {
        duty = -MOTOR_DUTY_MAX;
    }

    mag = (duty < 0) ? (int16_t)(-duty) : duty;
    ch  = s_ch_map[idx];

    /* 正占空比 = 车往前走，再按该轮的方向系数取反 */
    dir = (duty >= 0) ? GPIO_PIN_SET : GPIO_PIN_RESET;

    if (s_dir_sign[idx] < 0)
    {
        dir = (dir == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
    }

    HAL_GPIO_WritePin((GPIO_TypeDef *)s_dir_port[ch], s_dir_pin[ch], dir);
    __HAL_TIM_SET_COMPARE(&htim1, s_tim_ch[ch], duty_to_compare(mag));

    s_last_duty[idx] = duty;
}

void motor_stop_all(void)
{
    uint8_t i;

    for (i = 0u; i < MOTOR_COUNT; i++)
    {
        motor_set(i, 0);
    }
}

int16_t motor_get_duty(uint8_t idx)
{
    if (idx >= MOTOR_COUNT)
    {
        return 0;
    }

    return s_last_duty[idx];
}
