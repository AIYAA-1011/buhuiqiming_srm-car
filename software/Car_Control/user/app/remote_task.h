//
// 遥控数据接收任务 —— 对外接口
//
// 帧格式完全来自遥控器工程 P02_remote_hal/App_transmit_data.c：
//   [0..2]   's' 'g' 'g'
//   [3..10]  ly lx ry rx，int16 大端，-500 ~ +500
//   [11]     bit0 急停 / bit1 夹爪 / bit2 模式 / bit3 过洞 / bit4 取矿 / bit5 兑矿
//   [12]     预留
//   [13..16] [0..12] 的 32 位累加和，大端
//
// 轴的语义（和遥控器说明书一致）：
//   底盘模式  ry=前后 / rx=左右 / lx=自转 / ly 固定为 0
//   机械臂模式 ly=大臂 / lx=小臂 / ry=腕俯仰 / rx=腕转动
//

#ifndef CAR_CONTROL_REMOTE_TASK_H
#define CAR_CONTROL_REMOTE_TASK_H

#include <stdint.h>
#include "FreeRTOS.h"
#include "queue.h"

/* ---------------- 帧格式 ---------------- */
#define REMOTE_FRAME_LEN    17

#define REMOTE_HEAD0        's'
#define REMOTE_HEAD1        'g'
#define REMOTE_HEAD2        'g'

#define REMOTE_OFS_AXIS     3
#define REMOTE_OFS_FLAGS    11
#define REMOTE_OFS_RSV      12
#define REMOTE_OFS_SUM      13

/* 轴值范围（遥控器保证在这个范围内） */
#define REMOTE_AXIS_MIN     (-500)
#define REMOTE_AXIS_MAX     ( 500)

/* 超过这么久没收到有效帧就判定掉线 */
#define REMOTE_LINK_TIMEOUT_MS  200u

/* ---------------- 解析结果 ---------------- */
typedef struct
{
    int16_t ly;     /* 左杆上下：机械臂=大臂；底盘模式固定为 0 */
    int16_t lx;     /* 左杆左右：机械臂=小臂；底盘=自转   */
    int16_t ry;     /* 右杆上下：机械臂=腕俯仰；底盘=前后 */
    int16_t rx;     /* 右杆左右：机械臂=腕转动；底盘=左右 */
    uint8_t flags;  /* 见下面的位定义 */
} remote_cmd_t;

/* flags 的位定义 */
#define RCMD_ESTOP          (1u << 0)
#define RCMD_GRIPPER        (1u << 1)
#define RCMD_MODE_ARM       (1u << 2)
#define RCMD_PRESET_HOLE    (1u << 3)
#define RCMD_PRESET_PICK    (1u << 4)
#define RCMD_PRESET_PUT     (1u << 5)

/* ---------------- 跨任务共享 ---------------- */

/**
 * 最新一帧（长度 1 的队列）
 *
 * remote_task 用 xQueueOverwrite() 覆盖写，永远只保留最新一帧。
 * 消费方用 xQueuePeek() 读取（不取走），不要用 xQueueReceive()：
 *
 *     remote_cmd_t cmd;
 *     if (xQueuePeek(q_remote_cmd, &cmd, 0) == pdTRUE) { ... }
 */
extern QueueHandle_t q_remote_cmd;

/**
 * 急停来源（分开存，由 safety_task 汇总，不要互相覆盖）
 *
 *   stop = g_remote_estop || g_local_estop || g_link_lost
 */
extern volatile uint8_t g_remote_estop;    /* 遥控器急停位 */
extern volatile uint8_t g_local_estop;     /* 本地急停按钮（PC0），safety_task 写 */
extern volatile uint8_t g_link_lost;       /* 1 = 掉线 */

/**
 * 最近一次收到有效帧的时刻（osKernelSysTick）
 *
 * 用途：motor_task 在"刚恢复通信"的一小段时间里做速度斜坡，
 * 避免掉线期间摇杆一直推着、恢复瞬间车突然冲出去。
 *
 *     if (osKernelSysTick() - g_last_rx_tick < 500) { 限加速 }
 */
extern volatile uint32_t g_last_rx_tick;

/* 链路统计（串口 / 屏幕显示用） */
extern volatile uint32_t g_remote_rx_ok;   /* 累计有效帧 */
extern volatile uint32_t g_remote_rx_err;  /* 累计校验失败帧 */

#endif // CAR_CONTROL_REMOTE_TASK_H
