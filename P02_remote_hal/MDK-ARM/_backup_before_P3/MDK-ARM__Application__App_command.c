#include "App_command.h"
#include <string.h>

/* ---------------------------------------------------------------
 * 串口调试开关
 *
 * 打开后，每次按键（按下/松开）和每次状态变化都会往串口打印一行。
 * 按键是人工操作，事件非常稀疏，不会影响到定时。
 * 调试完把这里改成 0 就完全关闭。
 * --------------------------------------------------------------- */
#define CMD_DEBUG_LOG 0

#if CMD_DEBUG_LOG
#define cmd_log(format, ...) debug_printf(format, ##__VA_ARGS__)
#else
#define cmd_log(format, ...)
#endif

/* 长按判定阈值（毫秒） */
#define KEY_LONG_PRESS_MS 1000

/* 命令状态 */
Command_Data cmd = {0};

/* 上一次的按键掩码，用于计算边沿 */
static uint8_t s_key_last = 0;

/* RIGHT_X 按下的时刻，用于判长短按（靠时间戳，不阻塞） */
static TickType_t s_rx_press_tick = 0;

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
    if (mask & KEY_LEFT_X)  strcat(out, "LX+");
    if (mask & KEY_RIGHT_X) strcat(out, "RX+");

    if (out[0] == '\0')
    {
        strcpy(out, "NONE");
    }
    else
    {
        out[strlen(out) - 1] = '\0'; /* 去掉末尾多出来的那个 '+' */
    }
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
    cmd.estop   = 0;
    cmd.gripper = 0;
    cmd.preset  = CMD_PRESET_NONE;
    cmd.action  = CMD_ACTION_NONE;

    s_key_last = 0;
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
        }
        cmd_log("ESTOP = %d\r\n", cmd.estop);
    }

    /* ---------- LEFT_X：切换底盘 / 机械臂 ---------- */
    if (pressed & KEY_LEFT_X)
    {
        cmd.mode = (cmd.mode == MODE_CHASSIS) ? MODE_ARM : MODE_CHASSIS;
        cmd_log("MODE -> %s\r\n", (cmd.mode == MODE_ARM) ? "ARM" : "MECANUM");
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

    /* ---------- RIGHT_X：短按 = 夹爪开合，长按 = 摇杆零偏校准 ---------- */
    if (pressed & KEY_RIGHT_X)
    {
        s_rx_press_tick = xTaskGetTickCount();
    }
    if (released & KEY_RIGHT_X)
    {
        TickType_t held = xTaskGetTickCount() - s_rx_press_tick;

        if (held > pdMS_TO_TICKS(KEY_LONG_PRESS_MS))
        {
            cmd_log("RIGHT_X LONG %u ms -> CALIBRATE\r\n", (unsigned)held);
            App_calibrate_joystick();
        }
        else
        {
            cmd.gripper = cmd.gripper ? 0 : 1;
            cmd_log("GRIPPER = %d\r\n", cmd.gripper);
        }
    }

    /* TODO(P3): 摇杆离开死区时清掉预设（操作员接管），
     *           需要等轴处理做完才能判 */

    s_key_last = now;
}
