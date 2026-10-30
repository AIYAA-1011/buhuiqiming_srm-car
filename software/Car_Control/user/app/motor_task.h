//
// 底盘任务
//
// 阶段 1：开环测试
//   Ozone 里改 g_motor_mode / g_motor_en / g_test_duty[]，逐个验证电机
//
// 后面会加：编码器速度环、麦轮解算、遥控映射
//

#ifndef CAR_CONTROL_MOTOR_TASK_H
#define CAR_CONTROL_MOTOR_TASK_H

#include <stdint.h>

/* 运行模式 */
#define MOTOR_MODE_STOP     0u   /* 停                     */
#define MOTOR_MODE_OPEN     1u   /* 开环：直接给占空比     */
#define MOTOR_MODE_SPEED    2u   /* 单轮速度环（PID）      */
#define MOTOR_MODE_CHASSIS  3u   /* 麦轮解算（Ozone 给速度）*/
#define MOTOR_MODE_REMOTE   4u   /* 遥控器控制             */

/**
 * @brief 底盘任务入口（在 freertos.c 里创建，优先级 High）
 */
void StartMotorTask(void const *argument);

#endif //CAR_CONTROL_MOTOR_TASK_H
