//
// 速度环 PID 实现（位置式 + 积分限幅 + 输出限幅）
//

#include "controller.h"

/* ---------------- Ozone 可调的增益 ---------------- */

volatile float g_pid_kp[PID_WHEEL_COUNT] = { 4.0f, 4.0f, 4.0f, 4.0f };
volatile float g_pid_ki[PID_WHEEL_COUNT] = { 2.0f, 2.0f, 2.0f, 2.0f };
volatile float g_pid_kd[PID_WHEEL_COUNT] = { 0.0f, 0.0f, 0.0f, 0.0f };

/* 静摩擦补偿：输出不为 0 但小于这个值时，直接抬到这个值。
 * 直流减速电机低速时"占空比太小根本转不动"，不加这个会一卡一卡。
 * 0 = 关闭。典型值 60~150，看你电机。 */
volatile float g_pid_min_duty[PID_WHEEL_COUNT] = { 0.0f, 0.0f, 0.0f, 0.0f };

/* 目标死区（mm/s）：目标比这个还小就当成"停"，
 * 避免零速附近积分反复累积造成抖动 */
volatile float g_pid_deadband[PID_WHEEL_COUNT] = { 5.0f, 5.0f, 5.0f, 5.0f };

/* 前馈系数（占空比 / (mm/s)）：
 * 直接把"大概需要多少占空比"算出来，PID 只负责微调。
 * 标定方法见 controller.h。0 = 关闭。 */
volatile float g_pid_ff[PID_WHEEL_COUNT] = { 1.25f, 1.25f, 1.25f, 1.25f };

/* 每路的状态 */
typedef struct
{
    float integ;      /* 积分累计 */
    float prev_err;   /* 上一拍的误差，用来算微分 */
    int16_t out;      /* 上一次输出 */
} pid_state_t;

static pid_state_t s_pid[PID_WHEEL_COUNT];

static float clampf(float v, float lo, float hi)
{
    if (v > hi)
    {
        return hi;
    }
    if (v < lo)
    {
        return lo;
    }
    return v;
}

void pid_reset(uint8_t idx)
{
    if (idx >= PID_WHEEL_COUNT)
    {
        return;
    }

    s_pid[idx].integ    = 0.0f;
    s_pid[idx].prev_err = 0.0f;
    s_pid[idx].out      = 0;
}

void pid_init(void)
{
    uint8_t i;

    for (i = 0u; i < PID_WHEEL_COUNT; i++)
    {
        pid_reset(i);
    }
}

int16_t pid_update(uint8_t idx, float target, float actual, float dt)
{
    float err;
    float deriv;
    float kp;
    float ki;
    float kd;
    float out;
    float min_duty;
    float sign;

    if ((idx >= PID_WHEEL_COUNT) || (dt <= 0.0f))
    {
        return 0;
    }

    /* 目标在死区内 = 让它停：清积分、输出 0，避免零速抖 */
    if ((target < g_pid_deadband[idx]) && (target > -g_pid_deadband[idx]))
    {
        pid_reset(idx);
        return 0;
    }

    kp = g_pid_kp[idx];
    ki = g_pid_ki[idx];
    kd = g_pid_kd[idx];

    err = target - actual;

    /* 微分项：误差变化率。dt 很小时不稳定，做个下限保护 */
    if (dt > 0.001f)
    {
        deriv = (err - s_pid[idx].prev_err) / dt;
    }
    else
    {
        deriv = 0.0f;
    }

    /* 积分：先累加再限幅，防止积分饱和 */
    s_pid[idx].integ += err * dt;
    s_pid[idx].integ = clampf(s_pid[idx].integ, -PID_INTEG_MAX, PID_INTEG_MAX);

    out = (kp * err) + (ki * s_pid[idx].integ) + (kd * deriv);

    /* 前馈：先按目标速度直接给一个基础占空比，PID 只修残差。
     * 低速时这一步能大幅减少"靠积分慢慢试"的抖动。 */
    out += g_pid_ff[idx] * target;

    /* 输出限幅；贴到限幅时把积分往回拉一点，避免"饱和后回不来" */
    if (out > PID_OUT_MAX)
    {
        out = PID_OUT_MAX;
        s_pid[idx].integ -= err * dt;
    }
    else if (out < -PID_OUT_MAX)
    {
        out = -PID_OUT_MAX;
        s_pid[idx].integ -= err * dt;
    }

    /* 静摩擦补偿：要动但输出太小，就抬到最小可转动占空比 */
    min_duty = g_pid_min_duty[idx];
    sign = (target > 0.0f) ? 1.0f : -1.0f;

    if (min_duty > 0.0f)
    {
        if ((out * sign) < min_duty)
        {
            out = sign * min_duty;
        }
    }

    s_pid[idx].prev_err = err;
    s_pid[idx].out      = (int16_t)out;

    return s_pid[idx].out;
}
