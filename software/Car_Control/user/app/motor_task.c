//
// 底盘任务实现
//
// 运行模式（Ozone 里改 g_motor_mode）：
//   0 = 停
//   1 = 开环：直接按 g_test_duty[] 给占空比
//   2 = 单轮速度环：给 g_test_speed，PID 自动调节
//   3 = 麦轮解算：给 g_cmd_vx / g_cmd_vy / g_cmd_wz，四轮闭环
//
// 下标：0 = 左前  1 = 右前  2 = 左后  3 = 右后
//

#include "motor_task.h"
#include "motor.h"
#include "encode.h"
#include "controller.h"
#include "kinematics.h"
#include "control_time.h"
#include "cmsis_os.h"
#include "debug.h"

/* 单轮目标速度上限（mm/s），约等于 1 m/s */
#define MOTOR_TARGET_MAX_MM_S   1200.0f

/* ---------------- Ozone 可改的变量 ---------------- */

volatile uint8_t g_motor_en;          /* 1 = 使能驱动板 */
volatile uint8_t g_motor_mode;        /* 0 停 / 1 开环 / 2 速度环 / 3 麦轮 */
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
            motor_stop_all();
            motor_enable(0);

            for (i = 0u; i < MOTOR_COUNT; i++)
            {
                g_duty_out[i]     = 0;
                g_wheel_duty[i]   = 0;
                g_wheel_target[i] = 0.0f;
            }
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
            else
            {
                motor_stop_all();

                for (i = 0u; i < MOTOR_COUNT; i++)
                {
                    g_duty_out[i]     = motor_get_duty(i);
                    g_wheel_duty[i]   = 0;
                    g_wheel_target[i] = 0.0f;
                }
            }
        }
    }
}
