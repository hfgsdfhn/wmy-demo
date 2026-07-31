/**
 * @file INA226.c
 * @brief INA226母线电压驱动实现
 */
#include "INA226.h"

/**
 * @brief 初始化INA226实例
 * @param ina226 INA226实例
 * @param i2c 已初始化的I2C BSP实例
 * @param address INA226的7位I2C地址
 * @retval true 初始化成功
 * @retval false 初始化失败
 */
bool INA226_Init(ina226_t *ina226, bsp_i2c_t *i2c, uint8_t address)
{
    if ((ina226 == NULL) || (i2c == NULL) || !i2c->initialized
        || (address > 0x7FU))
    {
        return false;
    }

    ina226->i2c = i2c;
    ina226->address = address;
    ina226->valid = false;
    return true;
}

/**
 * @brief 读取并换算INA226母线电压
 * @param ina226 INA226实例
 * @param bus_voltage_v 母线电压输出，单位V
 * @retval true 读取成功
 * @retval false 读取失败
 */
bool INA226_ReadBusVoltage(ina226_t *ina226, float *bus_voltage_v)
{
    uint8_t data[2];
    uint16_t raw_voltage;

    if ((ina226 == NULL) || (ina226->i2c == NULL)
        || (bus_voltage_v == NULL))
    {
        return false;
    }

    if (!BspI2c_ReadRegister(ina226->i2c,
                             ina226->address,
                             INA226_BUS_VOLTAGE_REGISTER,
                             data,
                             sizeof(data)))
    {
        ina226->valid = false;
        return false;
    }

    raw_voltage = (uint16_t)(((uint16_t)data[0] << 8U) | data[1]);
    *bus_voltage_v = (float)raw_voltage * INA226_BUS_VOLTAGE_LSB_V;
    ina226->valid = true;
    return true;
}
