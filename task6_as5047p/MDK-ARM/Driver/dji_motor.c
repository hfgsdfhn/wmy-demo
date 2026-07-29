#include "dji_motor.h"

#include <stddef.h>

//读取16位大端有符号整数
static int16_t DjiMotor_ReadI16Be(const uint8_t *data)
{
    return (int16_t)(((uint16_t)data[0] << 8U) | data[1]);
}

//写入16位大端有符号整数
static void DjiMotor_WriteI16Be(uint8_t *data, int16_t value)
{
    uint16_t raw = (uint16_t)value;

    data[0] = (uint8_t)(raw >> 8U);
    data[1] = (uint8_t)raw;
}

/**
 * @brief 电机初始化
 * 
 * @param motor     电机结构体指针
 * @param can       CAN句柄指针
 * @param id        电机ID
 * @param type      电机类型
 */
void DjiMotor_Init(dji_motor_t *motor, bsp_can_t *can, uint8_t id,
                   dji_motor_type_t type)
{
    if ((motor == NULL) || (can == NULL) || (id < DJI_MOTOR_MIN_ID)
        || (id > DJI_MOTOR_MAX_ID) || (type > DJI_MOTOR_TYPE_M3508))
    {
        return;
    }

    motor->id = id;
    motor->type = type;
    motor->encoder = 0U;
    motor->last_encoder = 0U;
    motor->encoder_turns = 0;
    motor->total_encoder = 0;
    motor->encoder_ready = 0U;
    motor->speed_rpm = 0;
    motor->current = 0;
    motor->temperature = 0U;
    motor->target_current = 0;
    motor->can = can;
}

/**
 * @brief 设置电机目标电流
 * 
 * @param motor 
 * @param current 
 */
void DjiMotor_SetCurrent(dji_motor_t *motor, int16_t current)
{
    int16_t limit;

    if (motor == NULL)
    {
        return;
    }

    limit = (motor->type == DJI_MOTOR_TYPE_M2006) ? DJI_M2006_MAX_CURRENT
                                                   : DJI_M3508_MAX_CURRENT;
    if (current > limit)
    {
        current = limit;
    }
    else if (current < -limit)
    {
        current = -limit;
    }

    motor->target_current = current;
}

/**
 * @brief 解析电机反馈
 * 
 * @param motor 
 * @param data 
 */
void DjiMotor_ParseFeedback(dji_motor_t *motor, const uint8_t *data)
{
    uint16_t encoder;
    int32_t delta;

    if ((motor == NULL) || (data == NULL))
    {
        return;
    }

    encoder = (uint16_t)(((uint16_t)data[0] << 8U) | data[1]);
    if (motor->encoder_ready != 0U)
    {
        delta = (int32_t)encoder - (int32_t)motor->last_encoder;
        if (delta > 4096)
        {
            delta -= 8192;
            motor->encoder_turns--;
        }
        else if (delta < -4096)
        {
            delta += 8192;
            motor->encoder_turns++;
        }
        motor->total_encoder += delta;
    }
    else
    {
        motor->encoder_ready = 1U;
    }
    motor->encoder = encoder;
    motor->last_encoder = encoder;
    motor->speed_rpm = DjiMotor_ReadI16Be(&data[2]);
    motor->current = DjiMotor_ReadI16Be(&data[4]);
    motor->temperature = data[6];
}

/**
 * @brief 发送电机目标电流
 * 
 * @param motor 指向一个连续ID组的电机数组: 1-4 或 5-8
 * @param number 电机数量
 */
bool DjiMotor_SendGroup(dji_motor_t *motor, uint8_t number)
{
    uint8_t data[BSP_CAN_MAX_DATA_LENGTH] = {0};
    uint8_t index;
    uint32_t can_id;

    if ((motor == NULL) || (motor->can == NULL) || (number == 0U)
        || (number > 4U) || (motor->id < DJI_MOTOR_MIN_ID)
        || (motor->id > DJI_MOTOR_MAX_ID))
    {
        return false;
    }

    can_id = (motor->id <= 4U) ? DJI_MOTOR_CONTROL_ID_1_4
                               : DJI_MOTOR_CONTROL_ID_5_8;
    for (index = 0U; index < number; index++)
    {
        DjiMotor_WriteI16Be(&data[index * 2U], motor[index].target_current);
    }

    return BspCan_Send(motor->can, can_id, data, BSP_CAN_MAX_DATA_LENGTH);
}
