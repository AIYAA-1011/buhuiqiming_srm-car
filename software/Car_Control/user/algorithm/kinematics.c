//
// 麦轮运动学实现
//

#include "kinematics.h"

/* 旋转项系数：k = (Lx + Ly) / 2，单位 m */
static float kin_k(void)
{
    return (KIN_LX_M + KIN_LY_M) * 0.5f;
}

void chassis_inverse(float vx, float vy, float wz, float wheel[KIN_WHEEL_COUNT])
{
    float k;

    if (wheel == 0)
    {
        return;
    }

    k = kin_k() * wz;          /* 单位 m/s */

    /* 输入 m/s，输出 mm/s */
    wheel[0] = (vx - vy - k) * 1000.0f;   /* 左前 */
    wheel[1] = (vx + vy + k) * 1000.0f;   /* 右前 */
    wheel[2] = (vx + vy - k) * 1000.0f;   /* 左后 */
    wheel[3] = (vx - vy + k) * 1000.0f;   /* 右后 */
}

void chassis_forward(const float wheel[KIN_WHEEL_COUNT],
                     float *vx, float *vy, float *wz)
{
    float k;
    float w0;
    float w1;
    float w2;
    float w3;

    if ((wheel == 0) || (vx == 0) || (vy == 0) || (wz == 0))
    {
        return;
    }

    w0 = wheel[0] / 1000.0f;   /* 转回 m/s */
    w1 = wheel[1] / 1000.0f;
    w2 = wheel[2] / 1000.0f;
    w3 = wheel[3] / 1000.0f;

    k = kin_k();

    *vx = (w0 + w1 + w2 + w3) * 0.25f;
    *vy = (w1 + w2 - w0 - w3) * 0.25f;
    *wz = (w1 + w3 - w0 - w2) / (4.0f * k);
}
