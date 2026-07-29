#ifndef AS5047P_H
#define AS5047P_H

#include <stdbool.h>
#include <stdint.h>

#include "Bsp_Spi.h"

#define AS5047P_ANGLE_REGISTER  0x3FFFU
#define AS5047P_DATA_MASK       0x3FFFU
#define AS5047P_READ_BIT        0x4000U
#define AS5047P_NOP             0x0000U

typedef struct
{
    bsp_spi_t *spi;
    uint16_t raw_angle;
    float angle_rad;
    bool valid;
} as5047p_t;

uint16_t AS5047P_AddParity(uint16_t data);
bool AS5047P_Init(as5047p_t *encoder, bsp_spi_t *spi);
bool AS5047P_ReadAngle(as5047p_t *encoder);
float AS5047P_GetAngle(as5047p_t *encoder);
float AS5047P_WrapAngleError(float error);

#endif
