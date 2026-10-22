#ifndef INA226_H
#define INA226_H

#include <stdbool.h>
#include <stdint.h>

#include "bsp_i2c.h"

#define INA226_DEFAULT_ADDRESS       0x40U
#define INA226_BUS_VOLTAGE_REGISTER  0x02U
#define INA226_BUS_VOLTAGE_LSB_V     0.00125f

typedef struct
{
    bsp_i2c_t *i2c;
    uint8_t address;
    bool valid;
} ina226_t;

bool INA226_Init(ina226_t *ina226, bsp_i2c_t *i2c, uint8_t address);
bool INA226_ReadBusVoltage(ina226_t *ina226, float *bus_voltage_v);

#endif /* INA226_H */
