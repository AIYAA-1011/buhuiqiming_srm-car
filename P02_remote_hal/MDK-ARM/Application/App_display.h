#ifndef __APP_DISPLAY__
#define __APP_DISPLAY__

#include "Inf_OLED.h"
#include "Int_SI24R1.h"
#include "App_process_data.h"

/* ===================== 屏幕布局 =====================
 * OLED 128x64，使用 8x12 的 ASCII 字体，每行正好 16 个字符
 *
 *   行1 (y=0)   LNK 98%         E     链路成功率 + 状态字母
 *   行2 (y=14)  MECANUM / ARM          当前模式
 *   行3 (y=26)  VX +512  VY ----       轴1 + 轴2
 *   行4 (y=38)  W  ----                轴3 (+ 轴4)
 *
 * 轴值域 -500 ~ +500，0 表示中位/停止，死区内显示 "----"
 * 状态字母永远只显示一个，放在行1 最右一格
 * ==================================================== */
#define LINE1_Y 0
#define LINE2_Y 14
#define LINE3_Y 26
#define LINE4_Y 38

/* 模式 */
#define MODE_CHASSIS 0
#define MODE_ARM     1

/* 状态字母 */
#define ACT_NONE  ' '
#define ACT_HOLE  'H' /* 过洞 */
#define ACT_PICK  'P' /* 取矿 */
#define ACT_PUT   'D' /* 兑矿 */
#define ACT_ESTOP 'E' /* 急停，反白显示 */

/**
 * @brief 屏幕显示数据
 *        各模块只管往这里写，显示层只负责画出来
 */
typedef struct
{
    uint8_t rf_ok;        /* 1 = 无线模块自检通过 */
    uint8_t link_valid;   /* 1 = 收到过 ACK，link_percent 有效 */
    uint8_t link_percent; /* 0-100，发送成功率 */
    uint8_t mode;         /* MODE_CHASSIS / MODE_ARM */
    int16_t axis[4];      /* -500 ~ +500，顺序 thr / yaw / pit / rol */
    char    action;       /* 状态字母 ACT_xxx */
} Ui_Data;

extern Ui_Data ui_data;

/**
 * @brief 初始化 OLED 硬件
 */
void App_display_init(void);

/**
 * @brief 记录无线模块自检结果（由 main 在启动 RTOS 之前调用）
 */
void App_display_set_rf(uint8_t ok);

/**
 * @brief 开机自检画面（oled_task 启动时调一次，内部带延时）
 */
void App_display_boot(void);

/**
 * @brief 循环执行刷写屏幕
 */
void App_display_show(void);

/**
 * @brief 进度条（字库里有 13 级进度条字模，需要时直接调用）
 * @param x 起始列  y 起始行  count 0-12
 */
void App_display_show_bar(uint8_t x, uint8_t y, uint8_t count);

#endif // __APP_DISPLAY__
