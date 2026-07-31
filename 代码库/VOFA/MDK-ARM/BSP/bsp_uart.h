#ifndef BSP_UART_H
#define BSP_UART_H

#include <stdbool.h>
#include <stdint.h>

#include "main.h"

#ifndef BSP_UART_TX_TIMEOUT_MS
#define BSP_UART_TX_TIMEOUT_MS 100U
#endif

typedef enum
{
    BSP_UART_TX_BLOCKING = 0,
    BSP_UART_TX_DMA = 1
} bsp_uart_tx_mode_t;

typedef void (*bsp_uart_rx_callback_t)(uint8_t *data, uint16_t len);

typedef struct
{
    UART_HandleTypeDef *huart;
    bsp_uart_tx_mode_t tx_mode;
    uint8_t *rx_buf; /* NULL selects transmit-only operation. */
    uint16_t rx_size;
    volatile uint8_t tx_busy;
    bsp_uart_rx_callback_t rx_callback;
} bsp_uart_t;


bool Bsp_Uart_Init(bsp_uart_t *uart,
                  UART_HandleTypeDef *huart,
                  bsp_uart_tx_mode_t tx_mode,
                  uint8_t *rx_buf,
                  uint16_t rx_size);


bool Bsp_Uart_Send(bsp_uart_t *uart, uint8_t *data, uint16_t len);


void Bsp_Uart_RegisterRxCallback(bsp_uart_t *uart,
                                 bsp_uart_rx_callback_t callback);

void Bsp_Uart_RxEventCallback(bsp_uart_t *uart, uint16_t size);

void Bsp_Uart_TxCpltCallback(bsp_uart_t *uart);

#endif /* BSP_UART_H */
