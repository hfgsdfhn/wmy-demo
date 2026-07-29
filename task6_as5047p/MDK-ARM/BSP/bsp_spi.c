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
    uint16_t index;
    uint16_t tx_word;
    uint16_t rx_word;

    if (!spi || !tx || !rx || length == 0U || (length & 1U) != 0U)
    {
        return false;
    }

    BspSpi_CS_Low(spi);
    status = HAL_OK;
    for (index = 0U; index < length; index += 2U)
    {
        tx_word = (uint16_t)(((uint16_t)tx[index] << 8U) | tx[index + 1U]);
        status = HAL_SPI_TransmitReceive((SPI_HandleTypeDef *)spi->hal_handle,
                                         (uint8_t *)&tx_word,
                                         (uint8_t *)&rx_word,
                                         1U,
                                         BSP_SPI_TRANSFER_TIMEOUT_MS);
        if (status != HAL_OK)
        {
            break;
        }
        rx[index] = (uint8_t)(rx_word >> 8U);
        rx[index + 1U] = (uint8_t)rx_word;
    }
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
