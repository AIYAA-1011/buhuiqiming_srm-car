#ifndef __INT_KEY__
#define __INT_KEY__

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

/* 按键位掩码：一个字节同时表示"哪几个键被按下"，每一位对应一个物理按键 */
#define KEY_UP      0x01
#define KEY_DOWN    0x02
#define KEY_LEFT    0x04
#define KEY_RIGHT   0x08
#define KEY_LEFT_X  0x10
#define KEY_RIGHT_X 0x20

/**
 * @brief 扫描一次按键（非阻塞，立即返回）
 *
 * 每 20ms 调用一次。内部用计数器去抖：连续 2 次读数一致才认可
 * （采样间隔 20ms 远大于机械抖动时间，所以比"延时 5ms 再查一次"更稳）。
 *
 * 注意：这个函数绝对不等待松手。
 *       原来的 Int_key_get() 里有一个 while(按下) 死等循环，会导致
 *       按住任意一个键就把整个 key_task 卡住，其它键全部失灵、
 *       急停也发不出去、长按判定还要阻塞整整 1 秒。
 *
 * @return uint8_t 位掩码，哪几个键此刻是按下的（0 = 都没按）
 */
uint8_t Int_key_scan(void);

#endif // __INT_KEY__
