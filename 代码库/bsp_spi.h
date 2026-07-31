
#ifndef BSP_SPI_H
#define BSP_SPI_H

#include <stdbool.h>
#include <stdint.h>

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif


#ifndef BSP_SPI_TRANSFER_TIMEOUT_MS
#define BSP_SPI_TRANSFER_TIMEOUT_MS  10U
#endif

typedef struct
{
    void *hal_handle;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
    bool initialized;
} bsp_spi_t;

bool BspSpi_Init(bsp_spi_t *spi,
                 void *hal_handle,
                 GPIO_TypeDef *cs_port,
                 uint16_t cs_pin);

bool BspSpi_Transfer(bsp_spi_t *spi,
                     uint8_t *tx,
                     uint8_t *rx,
                     uint16_t length);

bool BspSpi_TransferBytes(bsp_spi_t *spi,
                          const uint8_t *tx,
                          uint8_t *rx,
                          uint16_t length);

bool BspSpi_TransferDma(bsp_spi_t *spi,
                        const uint8_t *tx,
                        uint8_t *rx,
                        uint16_t length);

bool BspSpi_IsDmaReady(const bsp_spi_t *spi);

bool BspSpi_AbortDma(bsp_spi_t *spi);

void BspSpi_CS_Low(bsp_spi_t *spi);

void BspSpi_CS_High(bsp_spi_t *spi);

#ifdef __cplusplus
}
#endif

#endif /* BSP_SPI_H */
