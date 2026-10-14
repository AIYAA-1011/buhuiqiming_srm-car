#ifndef __COM_DEBUG_H__
#define __COM_DEBUG_H__

#include "usart.h"
#include "stdio.h"
#include "stdarg.h"
#include <string.h>
// 日志输出打印在CPU运行上非常占用资源 => 通过比特率可以计算  打印10字节左右大概需要1ms
// 遥控器上影响相对小（按键、校准这类事件都很稀疏），
// 但如果以后要在周期任务里加高频打印（比如每 10ms 打一帧数据），一定要关掉
// 设计一个日志输出打印开关
#define DEBUG_LOG_ENABLE 1

#ifdef DEBUG_LOG_ENABLE

// 使用宏定义的方式 只打印文件名称 不打印路径名称
// strrchr()从后向前查找字符串中的字符
#define __FILE_NAME__ (strrchr(__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 : __FILE__)
#define __FILE_NAME (strrchr(__FILE_NAME__, '/') ? strrchr(__FILE_NAME__, '/') + 1 : __FILE_NAME__)

// 使用宏定义的方式能实现打印日志之前 先添加文件名和行号
#define debug_printf(format, ...) printf("[%s:%d]  " format, __FILE_NAME, __LINE__, ##__VA_ARGS__)

#else
// 如果没有开启日志输出打印
#define debug_printf(format, ...)
#endif // ifdef


#endif //
