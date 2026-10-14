#ifndef __INT_JOYSTICK__
#define __INT_JOYSTICK__

#include "adc.h"

/* 摇杆原始 ADC 值（0 ~ 4095）
 *
 * 四个名字和 ADC 扫描顺序一一对应，必须对齐（顺序写错就会张冠李戴）：
 *
 *   ly  = 左摇杆上下
 *   lx  = 左摇杆左右
 *   ry  = 右摇杆上下
 *   rx  = 右摇杆左右
 *
 * 对应到功能：
 *   底盘模式  ->  ry 前后 / rx 左右 / lx 自转 / ly 不使用
 *   机械臂模式 ->  ly 大臂 / lx 小臂 / ry 腕部俯仰 / rx 腕部转动
 */
typedef struct
{
    int16_t ly;
    int16_t lx;
    int16_t ry;
    int16_t rx;
} Joystick_Struct;

/**
 * @brief 初始化ADC遥控  打开ADC
 *
 */
void Int_joystick_init(void);

/**
 * @brief 读取ADC数据 保存到结构体地址中
 * 
 * @param joystick 
 */
void Int_joystick_get(Joystick_Struct *joystick);

#endif // __INT_JOYSTICK__
