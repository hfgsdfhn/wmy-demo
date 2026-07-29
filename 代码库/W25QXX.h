#ifndef W25QXX_H
#define W25QXX_H

#include <stdbool.h>
#include <stdint.h>

#include "bsp_spi.h"

#define W25QXX_PAGE_SIZE             256U
#define W25QXX_SECTOR_SIZE           4096U
#define W25Q64_CAPACITY_BYTES        (8U * 1024U * 1024U)
#define W25Q128_CAPACITY_BYTES       (16U * 1024U * 1024U)

#define W25QXX_JEDEC_ID_W25Q64       0xEF4017UL
#define W25QXX_JEDEC_ID_W25Q128      0xEF4018UL

typedef enum
{
    W25QXX_MODEL_UNKNOWN = 0,
    W25QXX_MODEL_W25Q64,
    W25QXX_MODEL_W25Q128
} w25qxx_model_t;

typedef struct
{
    bsp_spi_t *spi;
    uint32_t jedec_id;
    uint32_t capacity_bytes;
    w25qxx_model_t model;
    bool initialized;
} w25qxx_t;

bool W25QXX_Init(w25qxx_t *flash, bsp_spi_t *spi);
bool W25QXX_ReadJedecId(w25qxx_t *flash, uint32_t *jedec_id);
uint32_t W25QXX_GetCapacity(const w25qxx_t *flash);
bool W25QXX_WaitReady(w25qxx_t *flash, uint32_t timeout_ms);
bool W25QXX_Read(w25qxx_t *flash, uint32_t address, uint8_t *data,
                  uint32_t length);
bool W25QXX_Write(w25qxx_t *flash, uint32_t address, const uint8_t *data,
                   uint32_t length);
bool W25QXX_EraseSector(w25qxx_t *flash, uint32_t address);

#endif /* W25QXX_H */
