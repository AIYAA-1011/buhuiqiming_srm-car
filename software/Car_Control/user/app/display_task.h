//
// 显示任务 —— 对外接口
//
// 阶段 A1：只扫 I2C1 总线（确认 OLED 在不在）
// 后面会加：OLED 驱动、电池数据显示
//

#ifndef CAR_CONTROL_DISPLAY_TASK_H
#define CAR_CONTROL_DISPLAY_TASK_H

/**
 * @brief 显示任务入口（在 freertos.c 里创建，优先级 BelowNormal）
 */
void StartDisplayTask(void const *argument);

#endif //CAR_CONTROL_DISPLAY_TASK_H
