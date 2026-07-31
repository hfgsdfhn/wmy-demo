#ifndef BSP_UART_H
#define BSP_UART_H

#include <stdbool.h>
#include <stdint.h>

#include "main.h"

#ifndef BSP_UART_TX_TIMEOUT_MS
#define BSP_UART_TX_TIMEOUT_MS 100U
#endif

#define BSP_UART_MAX_INSTANCES 3U

typedef enum
{
    BSP_UART_TX_BLOCKING = 0,   //阻塞模式
    BSP_UART_TX_DMA = 1         //DMA   
} bsp_uart_tx_mode_t;

typedef struct bsp_uart bsp_uart_t;

typedef void (*bsp_uart_rx_callback_t)(bsp_uart_t *uart,
                                       const uint8_t *data,
                                       uint16_t len);

struct bsp_uart
{
    UART_HandleTypeDef *huart;
    bsp_uart_tx_mode_t tx_mode;
    uint8_t *rx_buf;
    uint16_t rx_size;
    volatile uint8_t tx_busy;
    bsp_uart_rx_callback_t rx_callback;
    uint8_t instance_index;
    bool initialized;
};

bool Bsp_Uart_Init(bsp_uart_t *uart,
                   UART_HandleTypeDef *huart,
                   bsp_uart_tx_mode_t tx_mode,
                   uint8_t *rx_buf,
                   uint16_t rx_size,
                   uint8_t instance_index,
                   bsp_uart_rx_callback_t rx_callback);

bool Bsp_Uart_Send(bsp_uart_t *uart, uint8_t *data, uint16_t len);

#endif /* BSP_UART_H */
