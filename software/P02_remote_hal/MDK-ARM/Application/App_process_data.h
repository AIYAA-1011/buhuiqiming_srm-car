#ifndef __APP_PROCESS_DATA__
#define __APP_PROCESS_DATA__

#include "Int_joystick.h"
#include "Int_key.h"
#include "Com_debug.h"
#include "Com_tool.h"

/* 控制模式
 * 由 App_command 按键切换产生，App_display 取来显示。
 * 放在这里共用一份定义，避免两个模块各写一套。 */
#define MODE_CHASSIS 0 /* 底盘（麦轮） */
#define MODE_ARM     1 /* 机械臂 */

/* ==================== 摇杆参数 ====================
 * 轴值域：-500 ~ +500，0 = 中位 / 停止
 * 数据包里发出去的就是这四个数，全程只有这一种表示，
 * 不再有 0-1000 和 -1.0~+1.0 之间的来回换算。
 * ================================================== */
#define AXIS_SPAN     500 /* 满量程 */
#define AXIS_DEADZONE  50 /* 默认死区半宽（正负对称），约占总行程 10%
                           * 必须小于 AXIS_SPAN，否则死区外重拉伸会除零 */

/* 每个轴可以单独覆盖死区半宽。
 *
 * ly（左杆上下 = 机械臂大臂）单独加大：
 * 这根杆回中的一致性比其它轴差一些，死区太小时松手后大臂会自己慢慢爬。
 *
 * 加大的代价只是"中位附近一段行程不响应"：死区外会重新拉伸
 * （out = (v - dz) * 500 / (500 - dz)），所以推到底仍然是满量程 ±500，
 * 不会丢极速，只是中段的手感变"钝"一些。
 *
 * 觉得还不够就继续加大 AXIS_DEADZONE_LY；想给别的轴也单独设，
 * 照抄一行改成对应的 AXIS_DEADZONE_xx 即可。 */
#define AXIS_DEADZONE_LY 100 /* 左摇杆上下 -> 大臂，加大到约 20% */

/* ADC 满量程（12 位） */
#define AXIS_ADC_FULL 4095
/* 电气中点：摇杆理想的中位原始 ADC */
#define AXIS_MID_NOMINAL 2048

/* 标定中位的合法范围（原始 ADC）
 *
 * 两个用途：
 *   1. 丢掉 Flash 里的垃圾数据（校验和只能挡位翻转，挡不住"存进去时就是错的"）
 *   2. 拒绝明显摆偏了的中位
 * 范围取得比较宽，是为了不误伤"电位器机械中位本来就偏"的摇杆——
 * 宁可让某个轴的灵敏度左右不对称，也不能让遥控器永远解不了锁。
 */
#define AXIS_MID_MIN 1300
#define AXIS_MID_MAX 2750

/* ==================== 端点自学习 ====================
 * 电位器不一定真能推到 ADC 的两条轨上。实测本机遥控器：
 *   正方向 raw 能到 0（饱和）
 *   负方向 raw 只到 4020 左右，到不了 4095
 * 所以端点不能写死。做法是运行时记录每个轴"真的见过"的极值，
 * 但只有推得够深（越过下面两个门槛）才认账，避免噪声把量程撑坏。
 * ================================================== */
#define AXIS_END_LO_DEFAULT 0    /* 正方向端点保底值（还没学到时用） */
#define AXIS_END_HI_DEFAULT 4095 /* 负方向端点保底值（还没学到时用） */
#define AXIS_END_LO_MAX 600      /* 实测 raw 小于它，才算"正方向真的推到底了" */
#define AXIS_END_HI_MIN 3500     /* 实测 raw 大于它，才算"负方向真的推到底了" */

/* 中位到任一端点至少要有这么多计数，否则认为这个轴坏了 */
#define AXIS_MIN_HALF_RANGE 600

/* 开机前这么多拍不学习端点：DMA 刚启动时 adc_buff 还是初值 0，
 * 会被误当成"正方向顶到底"。50 拍 x 20ms = 1 秒 */
#define AXIS_WARMUP_SAMPLES 50

/* 极性：装机实测之后，哪个轴方向反了就把对应宏改成 1 */
#define AXIS_INV_LY 0 /* 左摇杆上下 -> 大臂     */
#define AXIS_INV_LX 0 /* 左摇杆左右 -> 小臂     */
#define AXIS_INV_RY 0 /* 右摇杆上下 -> 腕部俯仰 */
#define AXIS_INV_RX 0 /* 右摇杆左右 -> 腕部转动 */

/* 处理后得到的四个轴命令值
 * 顺序：ly(左杆Y) lx(左杆X) ry(右杆Y) rx(右杆X) */
extern int16_t axis_value[4];

/**
 * @brief 从 Flash 读回上次保存的【中位原始 ADC】
 *
 * 上电时调一次。读到有效数据（且四个数都落在 AXIS_MID_MIN~MAX 内）
 * 就把它应用到四个轴上并返回 1，这样开机不用重新采样校准，
 * 只需要把杆摆回中位解锁。
 *
 * @return 1 = 读到有效数据，0 = 无有效数据（首次上电 / 被擦除 / 数值越界）
 */
uint8_t App_calib_load(void);

/**
 * @brief 把中位复位到电气中点
 *
 * 重新校准前调用。这样屏幕上的值反映的是"离电气中点还有多远"，
 * 操作者对着屏幕把四个杆摆到中位即可。
 */
void App_calib_clear(void);

/**
 * @brief 摇杆中位校准
 *
 * 采样"四个摇杆当前位置"的原始 ADC，作为各自的中位（0），并存进 Flash。
 * 调用前四个杆必须都停在中位，否则中位就定偏了。
 *
 * @return 1 = 校准成功；0 = 某个轴的中位越界（AXIS_MID_MIN~MAX），本次拒绝
 */
uint8_t App_calibrate_joystick(void);

/**
 * @brief 读取并处理摇杆数据（joy_task 里每 20ms 调一次）
 *
 * 处理链：ADC(0-4095) -> 以标定中位为锚点、两侧各自归一化到 -500~+500
 *         -> 极性 -> 限幅 -> 死区 -> 死区外重新拉伸
 */
void App_process_joystick_data(void);

#endif // __APP_PROCESS_DATA__
