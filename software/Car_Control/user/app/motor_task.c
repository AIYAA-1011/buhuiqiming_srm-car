//
// 底盘任务实现
//
// 运行模式（Ozone 里改 g_motor_mode）：
//   0 = 停
//   1 = 开环：直接按 g_test_duty[] 给占空比
//   2 = 单轮速度环：给 g_test_speed，PID 自动调节
//   3 = 麦轮解算：给 g_cmd_vx / g_cmd_vy / g_cmd_wz，四轮闭环
//   4 = 遥控：摇杆 -> 车体速度 -> 麦轮解算 -> 四轮闭环
//
// 下标：0 = 左前  1 = 右前  2 = 左后  3 = 右后
//

#include "motor_task.h"
#include "motor.h"
#include "encode.h"
#include "controller.h"
#include "kinematics.h"
#include "control_time.h"
#include "remote_task.h"
#include "cmsis_os.h"
#include "debug.h"
#include <math.h>

/* 单轮目标速度上限（mm/s）：
 * 直走 0.30 m/s 时轮子 0.30；斜走 45° 时轮子会到 0.42，
 * 所以留到 450，别让斜走被夹住 */
#define MOTOR_TARGET_MAX_MM_S   450.0f

/* 最高车体速度 */
#define MOTOR_MAX_VX_MPS        0.30f   /* 前进 m/s */
#define MOTOR_MAX_VY_MPS        0.30f   /* 平移 m/s */
#define MOTOR_MAX_WZ_RPS        1.00f   /* 旋转 rad/s */

/* 摇杆方向系数：+1 / -1，手感反了就改这里 */
#define STICK_SIGN_VX           (+1)    /* ry：推上 = 前进 */
#define STICK_SIGN_VY           (-1)    /* rx：推右 = 向右（vy 负） */
#define STICK_SIGN_WZ           (-1)    /* lx：推右 = 顺时针（wz 负） */

/* ---------------- Ozone 可改的变量 ---------------- */

volatile uint8_t g_motor_en;          /* 1 = 使能驱动板 */
volatile uint8_t g_motor_mode;        /* 0 停 / 1 开环 / 2 速度环 / 3 麦轮 / 4 遥控 */
volatile int16_t g_test_duty[MOTOR_COUNT];

volatile uint8_t g_test_wheel;        /* 速度环：测第几路 */
volatile float   g_test_speed;        /* 速度环：目标 mm/s */

volatile float   g_cmd_vx;            /* 麦轮：向前 m/s   */
volatile float   g_cmd_vy;            /* 麦轮：向左 m/s   */
volatile float   g_cmd_wz;            /* 麦轮：逆时针 rad/s */

/* 只读：实际下发值 / 目标 / PID 输出 */
volatile int16_t g_duty_out[MOTOR_COUNT];
volatile float   g_wheel_target[MOTOR_COUNT];
volatile int16_t g_wheel_duty[MOTOR_COUNT];

/* 控制周期由 control_time 统一管理（CTRL_PERIOD_MS = 20 ms） */

/* 上一个模式，用来在切换时清 PID */
static uint8_t s_mode_last;

/**
 * @brief 把四个目标轮速交给 PID，并下发
 *
 * @param tgt 4 个轮子的目标线速度（mm/s）
 */
static void motor_run_targets(const float tgt[MOTOR_COUNT], float dt_s)
{
    uint8_t i;

    for (i = 0u; i < MOTOR_COUNT; i++)
    {
        float   t = tgt[i];
        int16_t d;

        /* 限幅，防止解算出离谱的目标 */
        if (t > MOTOR_TARGET_MAX_MM_S)
        {
            t = MOTOR_TARGET_MAX_MM_S;
        }
        if (t < -MOTOR_TARGET_MAX_MM_S)
        {
            t = -MOTOR_TARGET_MAX_MM_S;
        }

        g_wheel_target[i] = t;

        d = pid_update(i, t, encode_speed_mm_s(i), dt_s);

        g_wheel_duty[i] = d;
        motor_set(i, d);
        g_duty_out[i] = motor_get_duty(i);
    }
}

/**
 * @brief 四个轮子全停（并把观测量清零）
 */
static void motor_all_zero(void)
{
    uint8_t i;

    motor_stop_all();

    for (i = 0u; i < MOTOR_COUNT; i++)
    {
        g_wheel_target[i] = 0.0f;
        g_wheel_duty[i]   = 0;
        g_duty_out[i]     = motor_get_duty(i);
    }
}

/**
 * @brief 单轮速度环：只让 g_test_wheel 那一路转到 g_test_speed，其它停
 */
static void motor_speed_loop(float dt_s)
{
    float tgt[MOTOR_COUNT];
    uint8_t i;

    if (g_test_wheel >= MOTOR_COUNT)
    {
        g_test_wheel = 0u;
    }

    for (i = 0u; i < MOTOR_COUNT; i++)
    {
        tgt[i] = (i == g_test_wheel) ? g_test_speed : 0.0f;
    }

    motor_run_targets(tgt, dt_s);
}

/**
 * @brief 麦轮：车体速度 -> 四轮目标 -> 闭环
 */
static void motor_chassis_loop(float dt_s)
{
    float tgt[MOTOR_COUNT];

    chassis_inverse(g_cmd_vx, g_cmd_vy, g_cmd_wz, tgt);
    motor_run_targets(tgt, dt_s);
}

/**
 * @brief 摇杆 -> 车体速度（合速度限幅，任何方向最快都是 0.3 m/s）
 */
static void stick_to_cmd(const remote_cmd_t *cmd,
                         float *vx, float *vy, float *wz)
{
    float x;
    float y;
    float len;

    x = ((float)cmd->ry / 1000.0f) * MOTOR_MAX_VX_MPS * (float)STICK_SIGN_VX;
    y = ((float)cmd->rx / 1000.0f) * MOTOR_MAX_VY_MPS * (float)STICK_SIGN_VY;

    /* 合速度限幅：斜着推到底时车还是 0.3 m/s，不会变成 0.42 */
    len = sqrtf((x * x) + (y * y));
    if (len > MOTOR_MAX_VX_MPS)
    {
        float s = MOTOR_MAX_VX_MPS / len;

        x *= s;
        y *= s;
    }

    *vx = x;
    *vy = y;
    *wz = ((float)cmd->lx / 1000.0f) * MOTOR_MAX_WZ_RPS * (float)STICK_SIGN_WZ;
}

/**
 * @brief 底盘门控：只有"底盘模式 + 无急停 + 没掉线"才允许动
 */
static uint8_t chassis_gate_ok(const remote_cmd_t *cmd)
{
    if (g_link_lost != 0u)
    {
        return 0u;
    }
    if ((g_remote_estop != 0u) || (g_local_estop != 0u))
    {
        return 0u;
    }
    /* bit2 = 1 表示遥控器在机械臂模式，底盘要让位 */
    if ((cmd->flags & RCMD_MODE_ARM) != 0u)
    {
        return 0u;
    }

    return 1u;
}

/**
 * @brief 遥控模式：读最新一帧 -> 摇杆映射 -> 解算 -> 闭环
 */
static void motor_remote_loop(float dt_s)
{
    remote_cmd_t cmd;
    float        tgt[MOTOR_COUNT];
    float        vx = 0.0f;
    float        vy = 0.0f;
    float        wz = 0.0f;

    /* 取最新一帧但不移除（机械臂任务也在读同一个队列） */
    if (xQueuePeek(q_remote_cmd, &cmd, 0) != pdTRUE)
    {
        motor_all_zero();
        return;
    }

    if (chassis_gate_ok(&cmd) == 0u)
    {
        motor_all_zero();
        return;
    }

    stick_to_cmd(&cmd, &vx, &vy, &wz);

    /* 暴露给 Ozone 观察 */
    g_cmd_vx = vx;
    g_cmd_vy = vy;
    g_cmd_wz = wz;

    chassis_inverse(vx, vy, wz, tgt);
    motor_run_targets(tgt, dt_s);
}

void StartMotorTask(void const *argument)
{
    uint8_t i;

    (void)argument;

    motor_init();
    motor_enable(0);
    encode_init();
    pid_init();
    ctrl_time_init();

    for (i = 0u; i < MOTOR_COUNT; i++)
    {
        g_test_duty[i]    = 0;
        g_duty_out[i]     = 0;
        g_wheel_target[i] = 0.0f;
        g_wheel_duty[i]   = 0;
    }

    debug_printf("[MOTOR] init done, enable=%u mode=%u\r\n",
                 (unsigned)g_motor_en, (unsigned)g_motor_mode);

    for (;;)
    {
        /* 睡到下一个控制周期，拿这一拍的实际 dt */
        uint32_t dt   = ctrl_time_wait();
        float    dt_s = (float)dt / 1000.0f;

        /* 先更新编码器：不管电机转不转都要读，手转轮子也能看到变化 */
        encode_update(dt);

        /* 模式切换时清 PID，避免带着旧积分/旧误差冲出去 */
        if (g_motor_mode != s_mode_last)
        {
            s_mode_last = g_motor_mode;
            pid_init();
        }

        if (g_motor_en == 0u)
        {
            /* 总使能关：四个轮子停，驱动板禁用 */
            motor_enable(0);
            motor_all_zero();
        }
        else
        {
            motor_enable(1);

            if (g_motor_mode == MOTOR_MODE_OPEN)
            {
                /* 开环：把 g_test_duty 原样下发 */
                for (i = 0u; i < MOTOR_COUNT; i++)
                {
                    motor_set(i, g_test_duty[i]);
                    g_duty_out[i] = motor_get_duty(i);
                }
            }
            else if (g_motor_mode == MOTOR_MODE_SPEED)
            {
                motor_speed_loop(dt_s);
            }
            else if (g_motor_mode == MOTOR_MODE_CHASSIS)
            {
                motor_chassis_loop(dt_s);
            }
            else if (g_motor_mode == MOTOR_MODE_REMOTE)
            {
                motor_remote_loop(dt_s);
            }
            else
            {
                motor_all_zero();
            }
        }
    }
}
