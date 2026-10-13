#include "App_command.h"
#include <string.h>

/* ---------------------------------------------------------------
 * 串口调试开关
 *
 * 打开后，每次按键（按下/松开）和每次状态变化都会往串口打印一行。
 * 按键是人工操作，事件非常稀疏，不会影响到定时。
 * 调试完把这里改成 0 就完全关闭，零开销。
 * --------------------------------------------------------------- */
#define CMD_DEBUG_LOG 1

#if CMD_DEBUG_LOG
#define cmd_log(format, ...) debug_printf(format, ##__VA_ARGS__)
#else
#define cmd_log(format, ...)
#endif

/* 长按判定阈值（毫秒） */
#define KEY_LONG_PRESS_MS 1000

/* 首次采样零偏的方式
 *
 * 正常情况：要求四个轴都进入死区（屏幕上四个都显示 "----"）才采样，
 *           这样零偏最小，不会出现"行程一边多一边少"。
 * 保底情况：如果某个电位器机械中位偏得够不到死区，等够
 *           ARM_TIGHT_TIMEOUT 毫秒之后放宽到 ±AXIS_ARM_WINDOW，
 *           避免遥控器永远解不了锁。 */
#define AXIS_ARM_WINDOW   150
#define ARM_TIGHT_TIMEOUT 5000

/* 命令状态 */
Command_Data cmd = {0};

/* 上一次的按键掩码，用于计算边沿 */
static uint8_t s_key_last = 0;

/* RIGHT_X 按下的时刻，用于判长短按（靠时间戳，不阻塞） */
static TickType_t s_rx_press_tick = 0;

/* 零偏是否已经就绪（来自 Flash，或者刚做完校准） */
static uint8_t s_calibrated = 0;

/* 进入 HOLD 的时刻，用来算"精准窗口"等多久了 */
static TickType_t s_hold_enter_tick = 0;

/**
 * @brief 把按键掩码拼成可读字符串，方便串口查看
 *        例：KEY_UP|KEY_LEFT_X -> "UP+LX"
 */
static void key_mask_str(uint8_t mask, char *out)
{
    out[0] = '\0';

    if (mask & KEY_UP)      strcat(out, "UP+");
    if (mask & KEY_DOWN)    strcat(out, "DOWN+");
    if (mask & KEY_LEFT)    strcat(out, "LEFT+");
    if (mask & KEY_RIGHT)   strcat(out, "RIGHT+");
    if (mask & KEY_LEFT_X)  strcat(out, "L_X+");
    if (mask & KEY_RIGHT_X) strcat(out, "R_X+");

    if (out[0] == '\0')
    {
        strcpy(out, "NONE");
    }
    else
    {
        out[strlen(out) - 1] = '\0'; 
    }
}

/**
 * @brief 请求重新校准：冻结输出，并且重新走一次中位采样
 */
static void App_command_request_calib(void)
{
    cmd.armed         = 0;
    s_calibrated      = 0;

    /* 先把零偏清零：这样屏幕上的 "----" 会回到电气中位附近，
     * 而摇杆的行程中点通常也在这个范围内。
     * 否则屏幕会一直把你带回"上一次定义的中位"，
     * 如果你的中位本来就偏在一边，那个方向的行程就被吃掉了。 */
    App_calib_clear();

    s_hold_enter_tick = xTaskGetTickCount();
    cmd_log("CALIB REQUEST - center all sticks\r\n");
}

/**
 * @brief 回中解锁检查（每 20ms 调一次）
 *
 * 上电 / 切换模式 / 解除急停 / 请求校准之后，必须先让四个摇杆都回到中位，
 * 才允许输出。这样切模式的瞬间不会把旧模式的杆位当成新模式的命令。
 */
static void App_command_update_arm(void)
{
    uint8_t k;
    uint8_t in_deadzone;

    if (cmd.armed)
    {
        return;
    }

    if (!s_calibrated)
    {
        /* ---------- 第一层：四个轴都进死区才采样，零偏最准 ---------- */
        in_deadzone = 1;
        for (k = 0; k < 4; k++)
        {
            if (axis_value[k] != 0)
            {
                in_deadzone = 0;
                break;
            }
        }

        if (!in_deadzone)
        {
            /* 还没凑齐，先继续等 */
            if ((xTaskGetTickCount() - s_hold_enter_tick) < pdMS_TO_TICKS(ARM_TIGHT_TIMEOUT))
            {
                return;
            }

            /* ---------- 第二层：等够了，放宽窗口保底 ---------- */
            for (k = 0; k < 4; k++)
            {
                if (axis_value[k] > AXIS_ARM_WINDOW || axis_value[k] < -AXIS_ARM_WINDOW)
                {
                    return;
                }
            }
            cmd_log("CALIB: wide window fallback\r\n");
        }

        App_calibrate_joystick(); /* 采样定中位，并写进 Flash */
        s_calibrated = 1;
        cmd_log("CALIBRATED\r\n");
    }
    else
    {
        /* 零偏已经有了（Flash 读回 或 刚校准完）：必须真正回到死区内 */
        for (k = 0; k < 4; k++)
        {
            if (axis_value[k] != 0)
            {
                return;
            }
        }
    }

    cmd.armed = 1;
    cmd_log("ARMED\r\n");
}

/**
 * @brief 预设姿态：同一个键再按一次 = 取消；按别的键 = 直接切换
 */
static void App_preset_toggle(uint8_t target, char letter)
{
    if (cmd.preset == target)
    {
        cmd.preset = CMD_PRESET_NONE;
        cmd.action = CMD_ACTION_NONE;
        cmd_log("PRESET -> CANCEL\r\n");
    }
    else
    {
        cmd.preset = target;
        cmd.action = letter;
        cmd_log("PRESET -> %c\r\n", letter);
    }
}

/**
 * @brief 初始化命令状态
 */
void App_command_init(void)
{
    cmd.mode    = MODE_CHASSIS;
    cmd.armed   = 0; /* 上电先冻结，等摇杆回中 */
    cmd.estop   = 0;
    cmd.gripper = 0;
    cmd.preset  = CMD_PRESET_NONE;
    cmd.action  = CMD_ACTION_NONE;

    s_key_last = 0;

    /* 先从 Flash 读零偏，读到了就不用再重新采样 */
    s_calibrated      = App_calib_load();
    s_hold_enter_tick = xTaskGetTickCount();
}

/**
 * @brief 取当前应该发给小车的四个轴值
 */
void App_command_get_output(int16_t *out)
{
    uint8_t k;

    if (!cmd.armed || cmd.estop)
    {
        for (k = 0; k < 4; k++)
        {
            out[k] = 0;
        }
        return;
    }

    for (k = 0; k < 4; k++)
    {
        out[k] = axis_value[k];
    }
}

/**
 * @brief 扫描按键并处理事件（每 20ms 一次）
 */
void App_command_scan(void)
{
    uint8_t now      = Int_key_scan();
    uint8_t pressed  = (uint8_t)(now & (uint8_t)(~s_key_last)); /* 上升沿：这一拍刚按下 */
    uint8_t released = (uint8_t)((uint8_t)(~now) & s_key_last); /* 下降沿：这一拍刚松开 */
    char    name[24];

    /* ---------- 按键事件（串口调试用） ---------- */
    if (pressed)
    {
        key_mask_str(pressed, name);
        cmd_log("KEY DOWN: %s\r\n", name);
    }
    if (released)
    {
        key_mask_str(released, name);
        cmd_log("KEY UP  : %s\r\n", name);
    }

    /* ---------- UP：急停，锁定式（按一下锁、再按一下解除） ---------- */
    if (pressed & KEY_UP)
    {
        cmd.estop = cmd.estop ? 0 : 1;

        if (cmd.estop)
        {
            cmd.preset = CMD_PRESET_NONE; /* 急停时清掉预设 */
            cmd.action = CMD_ACTION_ESTOP;
        }
        else
        {
            cmd.action = CMD_ACTION_NONE;
            cmd.armed  = 0;               /* 解除急停也要重新回中确认 */
        }
        cmd_log("ESTOP = %d\r\n", cmd.estop);
    }

    /* ---------- LEFT_X：切换底盘 / 机械臂 ---------- */
    if (pressed & KEY_LEFT_X)
    {
        cmd.mode  = (cmd.mode == MODE_CHASSIS) ? MODE_ARM : MODE_CHASSIS;
        cmd.armed = 0; /* 切模式先冻结，等摇杆回中再解锁 */
        cmd_log("MODE -> %s (HOLD)\r\n", (cmd.mode == MODE_ARM) ? "ARM" : "MECANUM");
    }

    /* ---------- LEFT / RIGHT / DOWN：三个预设姿态 ----------
     * 急停锁定期间不响应（此时一切冻结） */
    if (!cmd.estop)
    {
        if (pressed & KEY_LEFT)
        {
            App_preset_toggle(CMD_PRESET_HOLE, CMD_ACTION_HOLE);
        }
        if (pressed & KEY_RIGHT)
        {
            App_preset_toggle(CMD_PRESET_PICK, CMD_ACTION_PICK);
        }
        if (pressed & KEY_DOWN)
        {
            App_preset_toggle(CMD_PRESET_PUT, CMD_ACTION_PUT);
        }
    }

    /* ---------- RIGHT_X：短按 = 夹爪开合，长按 = 请求重新校准摇杆 ---------- */
    if (pressed & KEY_RIGHT_X)
    {
        s_rx_press_tick = xTaskGetTickCount();
    }
    if (released & KEY_RIGHT_X)
    {
        TickType_t held = xTaskGetTickCount() - s_rx_press_tick;

        if (held > pdMS_TO_TICKS(KEY_LONG_PRESS_MS))
        {
            cmd_log("RIGHT_X LONG %u ms\r\n", (unsigned)held);
            App_command_request_calib();
        }
        else
        {
            cmd.gripper = cmd.gripper ? 0 : 1;
            cmd_log("GRIPPER = %d\r\n", cmd.gripper);
        }
    }

    /* ---------- 摇杆一动 = 操作员接管，清掉预设 ----------
     * 机械臂正在跑预设动作时，你一推摇杆就取消，
     * 这样"想手动微调"和"还在跑预设"不会打架。
     * 只在解锁状态下判（解锁前正在摆杆对中位，不算接管）。 */
    if (cmd.armed && !cmd.estop && cmd.preset != CMD_PRESET_NONE)
    {
        for (uint8_t k = 0; k < 4; k++)
        {
            if (axis_value[k] != 0)
            {
                cmd.preset = CMD_PRESET_NONE;
                cmd.action = CMD_ACTION_NONE;
                cmd_log("PRESET -> CANCEL (stick takeover)\r\n");
                break;
            }
        }
    }

    /* ---------- 回中解锁检查 ---------- */
    App_command_update_arm();

    s_key_last = now;
}
