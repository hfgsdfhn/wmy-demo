/**
 * @file bsp_uart.c
 * @author 王梦阳 wmy07823@163.com
 * @brief 串口封装实现
 * @version 0.1
 * @date 2026-07-25
 * 
 * @details 串口支持阻塞发送和DMA发送模式,模式选择可以再初始化时指定.接收固定DMA+IDLE
 * 
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#include "bsp_uart.h"

/**
 * @brief 开启串口接收
 * 
 * @param uart 
 * @return true 
 * @return false 
 */
static bool Bsp_Uart_StartReceive(bsp_uart_t *uart)
{
    if (uart->rx_buf == NULL)
    {
        return true;
    }

    if (uart->rx_size == 0U)
    {
        return false;
    }

    return HAL_UARTEx_ReceiveToIdle_DMA(uart->huart, uart->rx_buf,
                                         uart->rx_size) == HAL_OK;
}

/**
 * @brief 串口初始化
 * 
 * @param uart 串口BSP实例
 * @param huart 串口句柄
 * @param tx_mode 发送模式0为阻塞发送，1为DMA发送
 * @param rx_buf 接收缓冲区指针，NULL表示只发送不接收
 * @param rx_size 接收缓冲区大小
 * @return true 
 * @return false 
 */
bool Bsp_Uart_Init(bsp_uart_t *uart,
                  UART_HandleTypeDef *huart,
                  bsp_uart_tx_mode_t tx_mode,
                  uint8_t *rx_buf,
                  uint16_t rx_size)
{
    if ((uart == NULL) || (huart == NULL)
        || (tx_mode > BSP_UART_TX_DMA)
        || ((rx_buf != NULL) && (rx_size == 0U)))
    {
        return false;
    }

    uart->huart = huart;
    uart->tx_mode = tx_mode;
    uart->rx_buf = rx_buf;
    uart->rx_size = rx_size;
    uart->tx_busy = 0U;
    uart->rx_callback = NULL;

    return Bsp_Uart_StartReceive(uart);
}

/**
 * @brief 串口发送
 * 
 * @param uart 串口BSP实例
 * @param data 发送数据指针
 * @param len 发送数据长度
 * @return true 
 * @return false 
 */
bool Bsp_Uart_Send(bsp_uart_t *uart, uint8_t *data, uint16_t len)
{
    if ((uart == NULL) || (uart->huart == NULL)
        || ((data == NULL) && (len != 0U)))
    {
        return false;
    }

    if (uart->tx_mode == BSP_UART_TX_DMA)
    {
        if (uart->tx_busy != 0U)
        {
            return false;
        }

        uart->tx_busy = 1U;
        if (HAL_UART_Transmit_DMA(uart->huart, data, len) != HAL_OK)
        {
            uart->tx_busy = 0U;
            return false;
        }
        return true;
    }

    return HAL_UART_Transmit(uart->huart, data, len,
                             BSP_UART_TX_TIMEOUT_MS) == HAL_OK;
}

/**
 * @brief 注册接收回调函数
 * 
 * @param uart 串口BSP实例
 * @param callback 回调函数指针
 */
void Bsp_Uart_RegisterRxCallback(bsp_uart_t *uart,
                                bsp_uart_rx_callback_t callback)
{
    if (uart != NULL)
    {
        uart->rx_callback = callback;
    }
}

/*
 * @brief 
 * 
 * 
 * @param uart 串口BSP实例
 * @param size 接收到的数据长度
 */
void Bsp_Uart_RxEventCallback(bsp_uart_t *uart, uint16_t size)
{
    if ((uart == NULL) || (uart->rx_buf == NULL))
    {
        return;
    }

    if ((size > 0U) && (uart->rx_callback != NULL))
    {
        uart->rx_callback(uart->rx_buf, size);
    }

    (void)Bsp_Uart_StartReceive(uart);
}

/**
 * @brief 发送完成回调函数
 * 
 * @param uart 
 */
void Bsp_Uart_TxCpltCallback(bsp_uart_t *uart)
{
    if ((uart != NULL) && (uart->tx_mode == BSP_UART_TX_DMA))
    {
        uart->tx_busy = 0U;
    }
}
