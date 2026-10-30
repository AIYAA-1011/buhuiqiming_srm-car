//
// 编码器读取实现
//
// 只做三件事：
//   1 把定时器计数值读出来，处理 16 位回绕
//   2 累加成"总里程计数"
//   3 按时间算出速度（mm/s）
//

#include "encode.h"
#include "tim.h"

/* 1 个计数对应的位移：
 * 轮周长 = π × 60 = 188.496 mm
 * 每圈计数 = 13 × 4 × 30 = 1560
 * -> 188.496 / 1560 = 0.12083 mm */
#define ENC_MM_PER_COUNT    (188.496f / 1560.0f)

/* 哪个轮子用哪个定时器（下标 = 0 左前 1 右前 2 左后 3 右后）
 * 实测：左前轮 -> TIM5，右前轮 -> TIM2，左后轮 -> TIM4，右后轮 -> TIM3
 * 和电机的通道排列一致（3/0/2/1） */
static TIM_HandleTypeDef *const s_tim[ENC_COUNT] =
{
    &htim5,     /* 左前 */
    &htim2,     /* 右前 */
    &htim4,     /* 左后 */
    &htim3      /* 右后 */
};

/* 每路的方向系数：+1 = 车往前走时计数增加
 * 实测（sign 全 -1 时）：左前/左后仍是负的，右前/右后是正的
 * 所以左侧两路取 +1、右侧两路取 -1 */
volatile int8_t g_enc_sign[ENC_COUNT] = { +1, -1, +1, -1 };

/* ---------------- Ozone 可看 ---------------- */

volatile int32_t g_enc_count[ENC_COUNT];   /* 累计计数 */
volatile int32_t g_enc_delta[ENC_COUNT];   /* 上次 update 的增量 */
volatile float   g_enc_speed[ENC_COUNT];   /* 上次 update 的速度 mm/s */

/* ---------------- 内部状态 ---------------- */

static uint16_t s_last_raw[ENC_COUNT];
static uint8_t  s_inited;

/* ---------------- 对外接口 ---------------- */

void encode_init(void)
{
    uint8_t i;

    for (i = 0u; i < ENC_COUNT; i++)
    {
        (void)HAL_TIM_Encoder_Start(s_tim[i], TIM_CHANNEL_ALL);

        s_last_raw[i]   = (uint16_t)__HAL_TIM_GET_COUNTER(s_tim[i]);
        g_enc_count[i]  = 0;
        g_enc_delta[i]  = 0;
        g_enc_speed[i]  = 0.0f;
    }

    s_inited = 1u;
}

void encode_update(uint32_t dt_ms)
{
    uint8_t  i;
    uint16_t now;
    int16_t  diff;
    int32_t  delta;
    float    dt_s;

    if ((s_inited == 0u) || (dt_ms == 0u))
    {
        return;
    }

    dt_s = (float)dt_ms / 1000.0f;

    for (i = 0u; i < ENC_COUNT; i++)
    {
        now = (uint16_t)__HAL_TIM_GET_COUNTER(s_tim[i]);

        /* 16 位回绕：直接相减再当有符号数看 */
        diff  = (int16_t)(now - s_last_raw[i]);
        s_last_raw[i] = now;

        delta = (int32_t)diff * (int32_t)g_enc_sign[i];

        g_enc_delta[i] = delta;
        g_enc_count[i] += delta;
        g_enc_speed[i] = ((float)delta * ENC_MM_PER_COUNT) / dt_s;
    }
}

int32_t encode_count(uint8_t idx)
{
    if (idx >= ENC_COUNT)
    {
        return 0;
    }

    return g_enc_count[idx];
}

int32_t encode_delta(uint8_t idx)
{
    if (idx >= ENC_COUNT)
    {
        return 0;
    }

    return g_enc_delta[idx];
}

float encode_speed_mm_s(uint8_t idx)
{
    if (idx >= ENC_COUNT)
    {
        return 0.0f;
    }

    return g_enc_speed[idx];
}
