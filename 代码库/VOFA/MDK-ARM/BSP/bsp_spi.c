/**
 * @file Bsp_Spi.c
 * @author 王梦阳 wmy07823@163.com
 * @brief  spi驱动实现
 * @version 0.1
 * @date 2026-07-24
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#include "Bsp_Spi.h"

/**
 * @brief spi初始化
 * 
 * @param spi 
 * @param hal_handle 
 * @param cs_port 
 * @param cs_pin 
 * @return true 
 * @return false 
 */
bool BspSpi_Init(bsp_spi_t *spi,
                 void *hal_handle,
                 GPIO_TypeDef *cs_port,
                 uint16_t cs_pin)
{
    if (spi == NULL)
    {
        return false;
    }

    spi->initialized = false;

    if ((hal_handle == NULL) || (cs_port == NULL) || (cs_pin == 0U))
    {
        spi->hal_handle = NULL;
        spi->cs_port = NULL;
        spi->cs_pin = 0U;
        return false;
    }

    spi->hal_handle = hal_handle;
    spi->cs_port = cs_port;
    spi->cs_pin = cs_pin;

    HAL_GPIO_WritePin(spi->cs_port, spi->cs_pin, GPIO_PIN_SET);
    spi->initialized = true;
    return true;
}

/**
 * @brief spi传输
 * 
 * @param spi 
 * @param tx 
 * @param rx 
 * @param length 
 * @return true 
 * @return false 
 */
bool BspSpi_Transfer(bsp_spi_t *spi,
                     uint8_t *tx,
                     uint8_t *rx,
                     uint16_t length)
{
    HAL_StatusTypeDef status;

    if (!spi || !tx || !rx || length == 0U)
    {
        return false;
    }

    BspSpi_CS_Low(spi);
    status = HAL_SPI_TransmitReceive((SPI_HandleTypeDef *)spi->hal_handle,
                                    tx,
                                    rx,
                                    length,
                                    BSP_SPI_TRANSFER_TIMEOUT_MS);
    BspSpi_CS_High(spi);

    return status == HAL_OK;
}
/**
 * @brief CS引脚拉低
 * 
 * @param spi 
 */
void BspSpi_CS_Low(bsp_spi_t *spi)
{
    if (spi && spi->initialized && spi->cs_port)
    {
        HAL_GPIO_WritePin(spi->cs_port, spi->cs_pin, GPIO_PIN_RESET);
    }
}

/**
 * @brief CS引脚拉高
 * 
 * @param spi 
 */
void BspSpi_CS_High(bsp_spi_t *spi)
{
    if (spi && spi->initialized && spi->cs_port)
    {
        HAL_GPIO_WritePin(spi->cs_port, spi->cs_pin, GPIO_PIN_SET);
    }
}
