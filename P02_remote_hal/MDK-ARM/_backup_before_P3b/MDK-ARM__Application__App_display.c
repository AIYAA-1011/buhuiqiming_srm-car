#include "App_display.h"
#include <stdio.h>
#include <string.h>

/* 屏幕显示数据，各模块往这里写 */
Ui_Data ui_data = {0};

/* ================================================================
 * 内部工具
 * ================================================================ */

/**
 * @brief 写一整行，不足 16 字符补空格
 *        这样上一帧的内容一定会被完全覆盖，不会出现残留
 */
static void oled_line(uint8_t y, const char *s)
{
    char buf[17];
    uint8_t i;
    uint8_t end = 0;

    for (i = 0; i < 16; i++)
    {
        if (!end && s[i] == '\0')
        {
            end = 1;
        }
        buf[i] = end ? ' ' : s[i];
    }
    buf[16] = '\0';

    OLED_ShowString(0, y, (const uint8_t *)buf, 12, 1);
}

/**
 * @brief 轴值格式化：0（死区内）显示 "----"，否则显示带符号的 4 字符
 */
static void axis_str(char *out, int16_t v)
{
    if (v == 0)
    {
        strcpy(out, "----");
    }
    else
    {
        sprintf(out, "%+4d", v);
    }
}

/**
 * @brief 拼接两个"标签 + 轴值"，固定 16 字符
 */
static void pair_str(char *out, const char *la, int16_t va,
                     const char *lb, int16_t vb)
{
    char sa[5];
    char sb[5];

    axis_str(sa, va);
    axis_str(sb, vb);
    sprintf(out, "%s %s  %s %s", la, sa, lb, sb);
}

/* ================================================================
 * 对外接口
 * ================================================================ */

/**
 * @brief 初始化显示模块
 */
void App_display_init(void)
{
    OLED_Init();
}

/**
 * @brief 记录无线模块自检结果
 */
void App_display_set_rf(uint8_t ok)
{
    ui_data.rf_ok = ok ? 1 : 0;
}

/**
 * @brief 开机自检画面
 *        板子上没有 LED，屏幕是唯一的指示手段，
 *        所以开机必须把状态明确显示出来
 */
void App_display_boot(void)
{
    /* --- 第 1 屏：启动 --- */
    OLED_Clear();
    oled_line(LINE1_Y, "STG REMOTE");
    oled_line(LINE2_Y, "BOOTING...");
    OLED_Refresh_Gram();
    vTaskDelay(pdMS_TO_TICKS(600));

    /* --- 第 2 屏：无线模块自检结果 --- */
    OLED_Clear();
    oled_line(LINE1_Y, "RF CHECK");
    oled_line(LINE2_Y, ui_data.rf_ok ? "RF OK" : "RF FAIL");
    OLED_Refresh_Gram();
    vTaskDelay(pdMS_TO_TICKS(600));

    /* --- 第 3 屏：提示摇杆回中 ---
     * 真正的回中判定在 App_command 里（未解锁就一直是 HOLD 状态） */
    OLED_Clear();
    oled_line(LINE1_Y, "CENTER STICKS");
    OLED_Refresh_Gram();
    vTaskDelay(pdMS_TO_TICKS(600));
}

/**
 * @brief 循环执行刷写屏幕
 */
void App_display_show(void)
{
    char buf[17];
    char lnk[9];
    char sw[5];
    uint8_t i;

    /* ---------- 行1：链路成功率 + 状态字母 ---------- */
    if (ui_data.link_valid)
    {
        sprintf(lnk, "LNK %3d%%", ui_data.link_percent);
    }
    else
    {
        strcpy(lnk, "LNK   --");
    }

    for (i = 0; i < 16; i++)
    {
        buf[i] = ' ';
    }
    for (i = 0; lnk[i] != '\0' && i < 16; i++)
    {
        buf[i] = lnk[i];
    }
    buf[15] = ui_data.action ? ui_data.action : ' ';
    buf[16] = '\0';
    OLED_ShowString(0, LINE1_Y, (const uint8_t *)buf, 12, 1);

    /* 急停用反白显示：不闪烁，但比普通字母醒目的多 */
    if (ui_data.action == ACT_ESTOP)
    {
        OLED_ShowChar(15 * 8, LINE1_Y, ACT_ESTOP, 12, 0);
    }

    /* ---------- 行2：当前模式 + 是否已解锁 ---------- */
    sprintf(buf, "%s %s",
            (ui_data.mode == MODE_ARM) ? "ARM" : "MECANUM",
            ui_data.armed ? "READY" : "CENTER");
    oled_line(LINE2_Y, buf);

    /* ---------- 行3 / 行4 ---------- */
    if (!ui_data.armed)
    {
        /* 还没解锁：显示四个物理摇杆轴，方便对着屏幕把杆都放回中位 */
        pair_str(buf, "TH", ui_data.axis[0], "YW", ui_data.axis[1]);
        oled_line(LINE3_Y, buf);

        pair_str(buf, "PT", ui_data.axis[2], "RL", ui_data.axis[3]);
        oled_line(LINE4_Y, buf);
    }
    else if (ui_data.mode == MODE_ARM)
    {
        /* thr=大臂  yaw=小臂  pit=腕俯仰  rol=腕转动 */
        pair_str(buf, "BM", ui_data.axis[0], "EX", ui_data.axis[1]);
        oled_line(LINE3_Y, buf);

        pair_str(buf, "WP", ui_data.axis[2], "WR", ui_data.axis[3]);
        oled_line(LINE4_Y, buf);
    }
    else
    {
        /* pit=前后VX  rol=左右VY  yaw=自转W  thr 不使用 */
        pair_str(buf, "VX", ui_data.axis[2], "VY", ui_data.axis[3]);
        oled_line(LINE3_Y, buf);

        axis_str(sw, ui_data.axis[1]);
        sprintf(buf, "W  %s", sw);
        oled_line(LINE4_Y, buf);
    }

    /* 调用刷写才能真的显示 */
    OLED_Refresh_Gram();
}

/**
 * @brief 进度条（保留：字库里有 13 级进度条字模，需要时直接用）
 */
void App_display_show_bar(uint8_t x, uint8_t y, uint8_t count)
{
    if (count < 13)
    {
        OLED_Show_CH(x, y, 12 + count, 12, 1);
    }
}
