/**
 * @file ak_motor.c
 * @author 王梦阳
 * @brief ak60电机速度模式
 * @version 0.1
 * @date 2026-07-30
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#include "ak_motor.h"

#include <stddef.h>

#define AK_MOTOR_CAN_DATA_LENGTH          8U
#define AK_MOTOR_SERVO_SPEED_COMMAND      0xA2U
#define AK_MOTOR_SERVO_FEEDBACK_COMMAND   0x9CU
#define AK_MOTOR_DEG_PER_RAD              57.2957795131f
#define AK_MOTOR_SERVO_SPEED_SCALE        100.0f


//限幅函数
static float AkMotor_Clamp(float value, float minimum, float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

static int16_t AkMotor_ReadI16Le(const uint8_t *data)
{
    return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8U));
}

static uint16_t AkMotor_ReadU16Le(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

static void AkMotor_WriteI32Le(uint8_t *data, int32_t value)
{
    uint32_t raw = (uint32_t)value;

    data[0] = (uint8_t)raw;
    data[1] = (uint8_t)(raw >> 8U);
    data[2] = (uint8_t)(raw >> 16U);
    data[3] = (uint8_t)(raw >> 24U);
}

static bool AkMotor_SendServoSpeedCommand(const ak_motor_t *motor)
{
    uint8_t data[AK_MOTOR_CAN_DATA_LENGTH] = {0};
    float speed;
    int32_t speed_raw;

    speed = AkMotor_Clamp(motor->target_speed, AK_MOTOR_SPEED_MIN,
                          AK_MOTOR_SPEED_MAX);
    speed_raw = (int32_t)(speed * AK_MOTOR_DEG_PER_RAD
                          * AK_MOTOR_SERVO_SPEED_SCALE);
    data[0] = AK_MOTOR_SERVO_SPEED_COMMAND;
    AkMotor_WriteI32Le(&data[4], speed_raw);
    return BspCan_Send(motor->can, motor->id, data, AK_MOTOR_CAN_DATA_LENGTH);
}

/**
 * @brief ak电机初始化
 * 
 * @param motor 
 * @param can 
 * @param id    电机id
 * @return true 
 * @return false 
 */
bool AkMotor_Init(ak_motor_t *motor, bsp_can_t *can, uint8_t id)
{
    if ((motor == NULL) || (can == NULL) || (id < AK_MOTOR_MIN_ID)
        || (id > AK_MOTOR_MAX_ID))
    {
        return false;
    }

    motor->id = id;
    motor->can = can;
    motor->target_speed = 0.0f;
    motor->feedback.position = 0.0f;
    motor->feedback.speed = 0.0f;
    motor->feedback.torque = 0.0f;
    motor->feedback.temperature = 0U;
    motor->feedback.valid = false;
    return true;
}

/**
 * @brief 设置电机速度
 * 
 * @param motor 
 * @param speed 
 */
bool AkMotor_SetSpeed(ak_motor_t *motor, float speed)
{
    if ((motor == NULL) || (motor->can == NULL))
    {
        return false;
    }

    motor->target_speed = AkMotor_Clamp(speed, AK_MOTOR_SPEED_MIN,
                                        AK_MOTOR_SPEED_MAX);
    return AkMotor_SendServoSpeedCommand(motor);
}

/**
 * @brief 信息处理
 * 
 * @param motor 
 * @param data 
 * @param length 
 * @return true 
 * @return false 
 */
bool AkMotor_ParseFeedback(ak_motor_t *motor, const uint8_t *data,
                           uint8_t length)
{
    if ((motor == NULL) || (data == NULL)
        || (length != AK_MOTOR_CAN_DATA_LENGTH)
        || (data[0] != AK_MOTOR_SERVO_FEEDBACK_COMMAND))
    {
        return false;
    }

    motor->feedback.temperature = data[1];
    motor->feedback.torque = (float)AkMotor_ReadI16Le(&data[2]) / 100.0f;
    motor->feedback.speed = (float)AkMotor_ReadI16Le(&data[4])
                               / AK_MOTOR_DEG_PER_RAD;
    motor->feedback.position = (float)AkMotor_ReadU16Le(&data[6])
                               / (AK_MOTOR_SERVO_SPEED_SCALE
                                  * AK_MOTOR_DEG_PER_RAD);

    motor->feedback.valid = true;
    return true;
}
