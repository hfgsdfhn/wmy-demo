#include "bsp_i2c.h"

/**
 * @brief 初始化I2C BSP实例
 * @param i2c I2C BSP实例
 * @param hi2c HAL I2C句柄，例如&hi2c1
 * @retval true 初始化成功
 * @retval false 初始化失败
 */
bool BspI2c_Init(bsp_i2c_t *i2c, I2C_HandleTypeDef *hi2c)
{
    if ((i2c == NULL) || (hi2c == NULL))
    {
        return false;
    }

    i2c->hi2c = hi2c;
    i2c->initialized = true;
    return true;
}

/**
 * @brief 读取从设备8位寄存器数据
 * @param i2c I2C BSP实例
 * @param device_address 从设备7位地址
 * @param register_address 8位寄存器地址
 * @param data 接收缓冲区
 * @param length 接收字节数
 * @retval true 读取成功
 * @retval false 读取失败
 */
bool BspI2c_ReadRegister(bsp_i2c_t *i2c,
                         uint8_t device_address,
                         uint8_t register_address,
                         uint8_t *data,
                         uint16_t length)
{
    if ((i2c == NULL) || !i2c->initialized || (i2c->hi2c == NULL)
        || (device_address > 0x7FU) || (data == NULL) || (length == 0U))
    {
        return false;
    }

    return HAL_I2C_Mem_Read(i2c->hi2c,
                            (uint16_t)(device_address << 1U),
                            register_address,
                            I2C_MEMADD_SIZE_8BIT,
                            data,
                            length,
                            BSP_I2C_TRANSFER_TIMEOUT_MS) == HAL_OK;
}

/**
 * @brief 写入从设备8位寄存器数据
 * @param i2c I2C BSP实例
 * @param device_address 从设备7位地址
 * @param register_address 8位寄存器地址
 * @param data 发送数据缓冲区
 * @param length 发送字节数
 * @retval true 写入成功
 * @retval false 写入失败
 */
bool BspI2c_WriteRegister(bsp_i2c_t *i2c,
                          uint8_t device_address,
                          uint8_t register_address,
                          const uint8_t *data,
                          uint16_t length)
{
    if ((i2c == NULL) || !i2c->initialized || (i2c->hi2c == NULL)
        || (device_address > 0x7FU) || (data == NULL) || (length == 0U))
    {
        return false;
    }

    return HAL_I2C_Mem_Write(i2c->hi2c,
                             (uint16_t)(device_address << 1U),
                             register_address,
                             I2C_MEMADD_SIZE_8BIT,
                             (uint8_t *)data,
                             length,
                             BSP_I2C_TRANSFER_TIMEOUT_MS) == HAL_OK;
}
