#include "Com_Debug.h"

/* 调试串口: USART2, 用 printf 重定向到 fputc 输出.
   注意: 使用 printf 需要在 Keil 的 Target 选项里勾选 "Use MicroLIB". */
#define DEBUG_UART        (&huart2)
#define DEBUG_UART_TIMEOUT (1000U)

void Com_Debug_Start(void)
{
    /* 预留: 如果以后要用中断/DMA方式发送日志, 在这里初始化 */
}

/**
 * @description: 把 printf 的输出重定向到调试串口
 * @param {int} c 待输出的字符
 * @param {FILE} *file
 * @return {*} 输出的字符
 */
int fputc(int c, FILE *file)
{
    HAL_UART_Transmit(DEBUG_UART, (uint8_t *)&c, 1, DEBUG_UART_TIMEOUT);

    return c;
}
