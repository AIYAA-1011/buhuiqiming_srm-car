//
// 机械臂任务 —— 对外接口
//
// 进度：
//   第 1 步  探测 PCA9685
//   第 2 步  初始化 50 Hz / 设脉宽 / 开关输出
//   第 3 步  台架测试（直接给脉宽）
//   第 5 步  标定表 + 角度换算 + 软限幅 + 匀速走到目标
//
// 当前版本：只有台架模式，没有接遥控器。
//

#ifndef CAR_CONTROL_ARM_TASK_H
#define CAR_CONTROL_ARM_TASK_H

#include <stdint.h>

/* 关节数量：大臂 / 小臂 / 腕俯仰 / 腕旋转 / 夹爪 */
#define ARM_JOINT_COUNT     5u

/**
 * @brief 机械臂任务入口（在 freertos.c 里创建，优先级 Normal，栈 192 words）
 */
void StartArmTask(void const *argument);

/* ==================================================================
 * 台架测试变量（Ozone 里直接改，必须 volatile）
 *
 *   g_test_mode = 0   冻结：不再下发，PCA9685 保持最后一次脉宽
 *   g_test_mode = 1   脉宽模式：直接下发 g_test_us[0..4]（500~2500 us）
 *   g_test_mode = 2   角度模式：朝 g_test_deg[0..4] 匀速走（默认）
 *
 * 用法（Ozone）：
 *   Watches 里加 g_test_deg，Halt → 双击 [0] 改成 150 → Go
 *   串口会打印  [ARM] ch0 target 150 deg
 *              [ARM] ch0 arrived 150 deg
 * ================================================================== */

#define ARM_TEST_FREEZE     0u
#define ARM_TEST_US         1u
#define ARM_TEST_DEG        2u

extern volatile uint8_t  g_test_mode;
extern volatile uint16_t g_test_us[ARM_JOINT_COUNT];
extern volatile float    g_test_deg[ARM_JOINT_COUNT];

#endif // CAR_CONTROL_ARM_TASK_H
