#include "bsp_spi.h"

/**
 * @brief 初始化 SPI BSP 对象并将片选信号置为无效电平。
 * @param spi BSP 对象。
 * @param hal_handle STM32 HAL SPI 句柄。
 * @param cs_port 片选 GPIO 端口。
 * @param cs_pin 片选 GPIO 引脚。
 * @return 参数有效时返回 true。
 */
bool BspSpi_Init(bsp_spi_t *spi,
                 void *hal_handle,
                 GPIO_TypeDef *cs_port,
                 uint16_t cs_pin)
{
    if ((spi == NULL) || (hal_handle == NULL) || (cs_port == NULL)
        || (cs_pin == 0U))
    {
        return false;
    }

    spi->hal_handle = hal_handle;
    spi->cs_port = cs_port;
    spi->cs_pin = cs_pin;
    spi->initialized = true;

    HAL_GPIO_WritePin(spi->cs_port, spi->cs_pin, GPIO_PIN_SET);
    return true;
}

/**
 * @brief 使用阻塞方式同时发送和接收指定字节数。
 * @note 本函数不控制片选信号，由上层驱动决定事务边界。
 */
bool BspSpi_TransferBytes(bsp_spi_t *spi,
                          const uint8_t *tx,
                          uint8_t *rx,
                          uint16_t length)
{
    if ((spi == NULL) || !spi->initialized || (spi->hal_handle == NULL)
        || (tx == NULL) || (rx == NULL) || (length == 0U))
    {
        return false;
    }

    return HAL_SPI_TransmitReceive((SPI_HandleTypeDef *)spi->hal_handle,
                                   (uint8_t *)tx, rx, length,
                                   BSP_SPI_TRANSFER_TIMEOUT_MS) == HAL_OK;
}

/**
 * @brief 使用阻塞方式发送指定字节数。
 * @note 本函数不控制片选信号。
 */
bool BspSpi_TransmitBytes(bsp_spi_t *spi,
                          const uint8_t *data,
                          uint16_t length)
{
    if ((spi == NULL) || !spi->initialized || (spi->hal_handle == NULL)
        || (data == NULL) || (length == 0U))
    {
        return false;
    }

    return HAL_SPI_Transmit((SPI_HandleTypeDef *)spi->hal_handle,
                            (uint8_t *)data, length,
                            BSP_SPI_TRANSFER_TIMEOUT_MS) == HAL_OK;
}

/**
 * @brief 启动一次 DMA 全双工传输。
 * @note 函数仅负责启动 DMA，传输完成状态可通过 BspSpi_IsDmaReady 查询。
 */
bool BspSpi_TransferDma(bsp_spi_t *spi,
                        const uint8_t *tx,
                        uint8_t *rx,
                        uint16_t length)
{
    if ((spi == NULL) || !spi->initialized || (spi->hal_handle == NULL)
        || (tx == NULL) || (rx == NULL) || (length == 0U))
    {
        return false;
    }

    return HAL_SPI_TransmitReceive_DMA((SPI_HandleTypeDef *)spi->hal_handle,
                                       (uint8_t *)tx, rx, length) == HAL_OK;
}

/**
 * @brief 查询 SPI DMA 事务是否已经结束。
 */
bool BspSpi_IsDmaReady(const bsp_spi_t *spi)
{
    if ((spi == NULL) || !spi->initialized || (spi->hal_handle == NULL))
    {
        return false;
    }

    return HAL_SPI_GetState((SPI_HandleTypeDef *)spi->hal_handle)
           == HAL_SPI_STATE_READY;
}

/**
 * @brief 获取最近一次 SPI 事务的 HAL 错误码。
 */
uint32_t BspSpi_GetError(const bsp_spi_t *spi)
{
    if ((spi == NULL) || !spi->initialized || (spi->hal_handle == NULL))
    {
        return HAL_SPI_ERROR_FLAG;
    }

    return HAL_SPI_GetError((SPI_HandleTypeDef *)spi->hal_handle);
}

/**
 * @brief 中止当前 SPI DMA 事务，用于超时或异常恢复。
 */
bool BspSpi_AbortDma(bsp_spi_t *spi)
{
    if ((spi == NULL) || !spi->initialized || (spi->hal_handle == NULL))
    {
        return false;
    }

    return HAL_SPI_Abort((SPI_HandleTypeDef *)spi->hal_handle) == HAL_OK;
}

/** @brief 拉低片选，开始一个 SPI 从机事务。 */
void BspSpi_CS_Low(bsp_spi_t *spi)
{
    if ((spi != NULL) && spi->initialized && (spi->cs_port != NULL))
    {
        HAL_GPIO_WritePin(spi->cs_port, spi->cs_pin, GPIO_PIN_RESET);
    }
}

/** @brief 拉高片选，结束一个 SPI 从机事务。 */
void BspSpi_CS_High(bsp_spi_t *spi)
{
    if ((spi != NULL) && spi->initialized && (spi->cs_port != NULL))
    {
        HAL_GPIO_WritePin(spi->cs_port, spi->cs_pin, GPIO_PIN_SET);
    }
}
