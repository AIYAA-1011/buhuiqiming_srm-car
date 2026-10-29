//
// 机械臂任务 —— 对外接口
//
// 进度：
//   第 1 步  探测 PCA9685
//   第 2 步  初始化 50 Hz / 设脉宽 / 开关输出
//   第 3 步  台架测试（直接给脉宽）
//   第 5 步  标定表 + 角度换算 + 软限幅 + 匀速走到目标
//   第 6 步  接遥控器：摇杆 → 固定速度积分 → 松手停
//
// 调试输出不在这里：见 arm_debug.c（它依赖本文件，本文件不依赖它）
//

#ifndef CAR_CONTROL_ARM_TASK_H
#define CAR_CONTROL_ARM_TASK_H

#include <stdint.h>

/* 关节数量：大臂 / 小臂 / 腕俯仰 / 腕旋转 / 夹爪 */
#define ARM_JOINT_COUNT     5u

/**
 * @brief 机械臂任务入口（在 freertos.c 里创建，优先级 Normal，栈 384 words）
 */
void StartArmTask(void const *argument);

/* ==================================================================
 * 运行模式（Ozone 里改 g_arm_mode）
 *
 *   0  ARM_MODE_REMOTE  正常：跟遥控器走（默认）
 *   1  ARM_MODE_US      台架：直接下发 g_test_us[0..4]（500~2500 us）
 *   2  ARM_MODE_DEG     台架：朝 g_test_deg10[0..4] 匀速走（单位 0.1°）
 *
 * 台架模式只用于标定/测试，比赛时保持 0。
 * ================================================================== */

#define ARM_MODE_REMOTE     0u
#define ARM_MODE_US         1u
#define ARM_MODE_DEG        2u

extern volatile uint8_t  g_arm_mode;
extern volatile uint16_t g_test_us[ARM_JOINT_COUNT];

/**
 * @brief 台架角度模式的目标，单位 0.1 度（整数，Ozone 里好改）
 *
 *   1260 = 126.0°
 *   900  =  90.0°
 *
 * 用整数而不是 float，是因为很多调试器显示/编辑 float 数组不方便。
 */
extern volatile int16_t  g_test_deg10[ARM_JOINT_COUNT];

/* ==================================================================
 * 下面这些只给 arm_debug.c 用
 * ================================================================== */

/**
 * @brief 门控状态：机械臂现在是被什么挡住的
 */
typedef enum
{
    ARM_GATE_OK = 0,        /* 放行 */
    ARM_GATE_NO_CMD,        /* 还没收到遥控帧 */
    ARM_GATE_NO_LINK,       /* 无线掉线 */
    ARM_GATE_MODE,          /* 遥控器在底盘模式 */
    ARM_GATE_ESTOP          /* 急停（遥控器或本地） */
} arm_gate_t;

/**
 * @brief 关节配置快照（只读，打印用）
 */
typedef struct
{
    uint8_t     ch;             /* PCA9685 通道号（不是关节下标） */
    uint16_t    min_us;
    uint16_t    max_us;
    uint16_t    home_us;
    uint16_t    deg_range;
    int16_t     dir;
    uint16_t    speed_dps;
    uint16_t    acc_dps2;
    uint8_t     enabled;
    const char *name;
} arm_joint_cfg_t;

/**
 * @brief 运行时状态快照
 */
typedef struct
{
    float    pos_deg[ARM_JOINT_COUNT];      /* 当前命令位置 */
    float    soft_min_deg[ARM_JOINT_COUNT]; /* 软限位下限 */
    float    soft_max_deg[ARM_JOINT_COUNT]; /* 软限位上限 */
    uint16_t last_us[ARM_JOINT_COUNT];      /* 最后一次写成功的脉宽 */
    int16_t  axis[4];                       /* ly lx ry rx，最近一帧 */
    float    grip_open_deg;                 /* 夹爪张开角度 */
    float    grip_close_deg;                /* 夹爪闭合角度 */
    uint8_t  mode;                          /* g_arm_mode */
    uint8_t  gate;                          /* arm_gate_t */
    uint8_t  flags;                         /* 遥控帧标志位 */
} arm_status_t;

/**
 * @brief 拷贝一份当前状态
 */
void arm_get_status(arm_status_t *out);

/**
 * @brief 拷贝第 idx 个关节的配置（idx >= ARM_JOINT_COUNT 时不做任何事）
 */
void arm_get_joint_cfg(uint32_t idx, arm_joint_cfg_t *out);

#endif // CAR_CONTROL_ARM_TASK_H
