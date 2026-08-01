#include "bsp_uart.h"

static bsp_uart_t *uart_instances[BSP_UART_MAX_INSTANCES];

static bool Bsp_Uart_StartReceive(bsp_uart_t *uart)
{
    if (uart->rx_buf == NULL)
    {
        return true;
    }

    return HAL_UARTEx_ReceiveToIdle_DMA(uart->huart, uart->rx_buf,
                                        uart->rx_size) == HAL_OK;
}

static bsp_uart_t *Bsp_Uart_FindInstance(UART_HandleTypeDef *huart)
{
    uint8_t index;

    for (index = 0U; index < BSP_UART_MAX_INSTANCES; index++)
    {
        if ((uart_instances[index] != NULL)
            && uart_instances[index]->initialized
            && (uart_instances[index]->huart == huart))
        {
            return uart_instances[index];
        }
    }

    return NULL;
}

/**
 * @brief 
 * 
 * @param uart 
 * @param huart 
 * @param tx_mode 0（阻塞）/1（DMA）
 * @param rx_buf 
 * @param rx_size 
 * @param instance_index 
 * @param rx_callback 
 * @return true 
 * @return false 
 */
bool Bsp_Uart_Init(bsp_uart_t *uart,
                   UART_HandleTypeDef *huart,
                   bsp_uart_tx_mode_t tx_mode,
                   uint8_t *rx_buf,
                   uint16_t rx_size,
                   uint8_t instance_index,              //串口索引
                   bsp_uart_rx_callback_t rx_callback)
{
    if ((uart == NULL) || (huart == NULL)
        || (tx_mode > BSP_UART_TX_DMA)
        || ((rx_buf != NULL) && (rx_size == 0U))
        || (instance_index >= BSP_UART_MAX_INSTANCES)
        || (uart_instances[instance_index] != NULL))
    {
        return false;
    }

    uart->huart = huart;
    uart->tx_mode = tx_mode;
    uart->rx_buf = rx_buf;
    uart->rx_size = rx_size;
    uart->tx_busy = 0U;
    uart->rx_callback = rx_callback;
    uart->instance_index = instance_index;
    uart->initialized = false;
    uart_instances[instance_index] = uart;

    if (!Bsp_Uart_StartReceive(uart))
    {
        uart_instances[instance_index] = NULL;
        uart->huart = NULL;
        uart->rx_callback = NULL;
        return false;
    }

    uart->initialized = true;
    return true;
}

bool Bsp_Uart_Send(bsp_uart_t *uart, uint8_t *data, uint16_t len)
{
    if ((uart == NULL) || !uart->initialized || (uart->huart == NULL)
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

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    bsp_uart_t *uart = Bsp_Uart_FindInstance(huart);

    if ((uart == NULL) || (uart->rx_buf == NULL))
    {
        return;
    }

    if ((size > 0U) && (uart->rx_callback != NULL))
    {
        uart->rx_callback(uart, uart->rx_buf, size);
    }

    (void)Bsp_Uart_StartReceive(uart);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    bsp_uart_t *uart = Bsp_Uart_FindInstance(huart);

    if ((uart != NULL) && (uart->tx_mode == BSP_UART_TX_DMA))
    {
        uart->tx_busy = 0U;
    }
}
