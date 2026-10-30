//
// 麦轮运动学
//
// 车体坐标系（俯视）：
//   +vx = 向前
//   +vy = 向左平移
//   +wz = 逆时针旋转
//
// 轮子下标：0 = 左前，1 = 右前，2 = 左后，3 = 右后
//
// 麦轮辊子呈 X 型时的逆解：
//   k = (Lx + Ly) / 2
//   左前 = vx - vy - k·wz
//   右前 = vx + vy + k·wz
//   左后 = vx + vy - k·wz
//   右后 = vx - vy + k·wz
//
// 实测尺寸：
//   轮距 Lx（左右轮中心距）= 0.31 m
//   轴距 Ly（前后轮中心距）= 0.25 m
//

#ifndef BUHUIQIMING_SRM_CAR_KINEMATICS_H
#define BUHUIQIMING_SRM_CAR_KINEMATICS_H

#include <stdint.h>

#define KIN_WHEEL_COUNT     4u

/* 轮子直径（mm） */
#define KIN_WHEEL_DIAMETER_MM   60.0f

/* 车体尺寸（m） */
#define KIN_LX_M                0.31f
#define KIN_LY_M                0.25f

/**
 * @brief 逆解：车体速度 -> 四个轮子的线速度
 *
 * @param vx     向前速度（m/s）
 * @param vy     向左平移速度（m/s）
 * @param wz     逆时针角速度（rad/s）
 * @param wheel  输出，4 个轮子的线速度（mm/s）
 */
void chassis_inverse(float vx, float vy, float wz, float wheel[KIN_WHEEL_COUNT]);

/**
 * @brief 正解：四个轮子的线速度 -> 车体速度（里程计用）
 *
 * @param wheel 4 个轮子的线速度（mm/s）
 */
void chassis_forward(const float wheel[KIN_WHEEL_COUNT],
                     float *vx, float *vy, float *wz);

#endif //BUHUIQIMING_SRM_CAR_KINEMATICS_H
