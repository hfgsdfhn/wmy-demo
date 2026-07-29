#ifndef BSP_I2C_H
#define BSP_I2C_H

#include <stdbool.h>
#include <stdint.h>

#include "main.h"

#define BSP_I2C_TRANSFER_TIMEOUT_MS  10U

typedef struct
{
    I2C_HandleTypeDef *hi2c;
    bool initialized;
} bsp_i2c_t;

bool BspI2c_Init(bsp_i2c_t *i2c, I2C_HandleTypeDef *hi2c);

bool BspI2c_ReadRegister(bsp_i2c_t *i2c,
                         uint8_t device_address,
                         uint8_t register_address,
                         uint8_t *data,
                         uint16_t length);

bool BspI2c_WriteRegister(bsp_i2c_t *i2c,
                          uint8_t device_address,
                          uint8_t register_address,
                          const uint8_t *data,
                          uint16_t length);

#endif /* BSP_I2C_H */
