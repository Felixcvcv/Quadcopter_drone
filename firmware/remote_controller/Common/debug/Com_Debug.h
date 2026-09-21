#ifndef __COM_DEBUG_H
#define __COM_DEBUG_H

#include "usart.h"
#include "string.h"
#include "stdio.h"

/* 1: 打开调试串口输出   0: 全部编译为空, 不影响实时性 */
#define DEBUG (0)

#if(DEBUG == 1)

/* 只取 __FILE__ 里的文件名部分, 避免打印整条编译路径 */
#define FILENAME (strrchr(__FILE__, '\\') ? (strrchr(__FILE__, '\\') + 1) : __FILE__)

#define debug_start() Com_Debug_Start()
#define debug_printf(format, ...) printf("[%15s:%4d] -- " format, FILENAME, __LINE__, ##__VA_ARGS__)
#define debug_printfln(format, ...) \
    printf("[%15s:%4d] -- " format "\r\n", FILENAME, __LINE__, ##__VA_ARGS__)

#else

#define debug_start()
#define debug_printf(format, ...)
#define debug_printfln(format, ...)

#endif

void Com_Debug_Start(void);

#endif
