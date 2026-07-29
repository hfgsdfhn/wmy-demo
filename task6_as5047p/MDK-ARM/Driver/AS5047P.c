#include "AS5047P.h"

#define AS5047P_PI               3.14159265358979323846f
#define AS5047P_TWO_PI           (2.0f * AS5047P_PI)
#define AS5047P_RAD_PER_COUNT    (AS5047P_TWO_PI / 16384.0f)

// 计算16位数据的偶校验
static uint16_t AS5047P_Pack(uint16_t word)
{
    return AS5047P_AddParity(word & 0x7FFFU);
}

// 检查16位数据的偶校验
static bool AS5047P_HasEvenParity(uint16_t word)
{
    bool parity = false;
    while (word != 0U)
    {
        parity = !parity;
        word &= (uint16_t)(word - 1U);
    }
    return !parity;
}
/**
 * @brief 读取编码器寄存器值
 * 
 * @param encoder 
 * @param address 
 * @param value 
 * @return true 
 * @return false 
 */
static bool AS5047P_ReadRegister(as5047p_t *encoder, uint16_t address,
                                  uint16_t *value)
{
    uint8_t tx[2];
    uint8_t rx[2];
    uint16_t response;
    uint16_t command;

    if ((encoder == NULL) || (encoder->spi == NULL) || (value == NULL)
        || (address > AS5047P_DATA_MASK))
    {
        return false;
    }

    command = AS5047P_Pack((uint16_t)(AS5047P_READ_BIT | address));
    tx[0] = (uint8_t)(command >> 8U);
    tx[1] = (uint8_t)command;
    if (!BspSpi_Transfer(encoder->spi, tx, rx, 2U))
    {
        return false;
    }

    tx[0] = 0U;
    tx[1] = AS5047P_NOP;
    if (!BspSpi_Transfer(encoder->spi, tx, rx, 2U))
    {
        return false;
    }

    response = (uint16_t)(((uint16_t)rx[0] << 8U) | rx[1]);
    if (!AS5047P_HasEvenParity(response))
    {
        return false;
    }
    *value = response & AS5047P_DATA_MASK;
    return true;
}

/**
 * @brief 为16位数据添加偶校验
 * 
 * @param data 
 * @return uint16_t 
 */
uint16_t AS5047P_AddParity(uint16_t data)
{
    data &= 0x7FFFU;
    if (!AS5047P_HasEvenParity(data))
    {
        data |= 0x8000U;
    }
    return data;
}

/**
 * @brief 初始化编码器
 * 
 * @param encoder 
 * @param spi 
 * @return true 
 * @return false 
 */
bool AS5047P_Init(as5047p_t *encoder, bsp_spi_t *spi)
{
    if ((encoder == NULL) || (spi == NULL) || !spi->initialized)
    {
        return false;
    }

    encoder->spi = spi;
    encoder->raw_angle = 0U;
    encoder->angle_rad = 0.0f;
    encoder->valid = false;
    return true;
}

/**
 * @brief 读取编码器角度
 * 
 * @param encoder 
 * @return true 
 * @return false 
 */
bool AS5047P_ReadAngle(as5047p_t *encoder)
{
    uint16_t value;
    if (!AS5047P_ReadRegister(encoder, AS5047P_ANGLE_REGISTER, &value))
    {
        if (encoder != NULL)
        {
            encoder->valid = false;
        }
        return false;
    }
    encoder->raw_angle = value;
    encoder->angle_rad = (float)value * AS5047P_RAD_PER_COUNT;
    encoder->valid = true;
    return true;
}

/**
 * @brief 获取编码器角度（弧度）
 * 
 * @param encoder 
 * @return float 
 */
float AS5047P_GetAngle(as5047p_t *encoder)
{
    return (encoder != NULL) ? encoder->angle_rad : 0.0f;
}

/**
 * @brief 让电机转最少到达目标位置
 * 
 * @param error 
 * @return float 
 */
float AS5047P_WrapAngleError(float error)
{
    while (error > 180.0f)
    {
        error -= 360.0f;
    }
    while (error <= -180.0f)
    {
        error += 360.0f;
    }
    return error;
}
