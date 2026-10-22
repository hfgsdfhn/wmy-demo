#ifndef APP_INIT_H
#define APP_INIT_H /* 应用初始化头文件保护宏 */

#include <stdbool.h>

#include "bsp_can.h"
#include "bsp_uart.h"

extern bsp_uart_t uart10;
extern bsp_uart_t uart1;
extern bsp_uart_t uart8;

bool AppInit_Run(void);

#endif /* APP_INIT_H */
