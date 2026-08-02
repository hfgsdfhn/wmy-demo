#include "rs01_motor.h"

#include <stddef.h>

#define RS01_MOTOR_DATA_LENGTH        8U

#define RS01_MOTOR_MODE_CONTROL       0x01U
#define RS01_MOTOR_MODE_FEEDBACK      0x02U
#define RS01_MOTOR_MODE_ENABLE        0x03U
#define RS01_MOTOR_MODE_DISABLE       0x04U
#define RS01_MOTOR_MODE_SET_ZERO      0x06U

static float Rs01Motor_Clamp(float value, float minimum, float maximum)
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

static uint16_t Rs01Motor_FloatToUint(float value, float minimum,
                                      float maximum, uint16_t maximum_raw)
{
    float scaled;

    value = Rs01Motor_Clamp(value, minimum, maximum);
    scaled = (value - minimum) * (float)maximum_raw / (maximum - minimum);
    return (uint16_t)scaled;
}

static float Rs01Motor_UintToFloat(uint16_t value, float minimum,
                                   float maximum, uint16_t maximum_raw)
{
    return (float)value * (maximum - minimum) / (float)maximum_raw + minimum;
}

static uint32_t Rs01Motor_MakeExtendedId(uint8_t mode, uint16_t data,
                                         uint8_t id)
{
    return ((uint32_t)mode << 24U) | ((uint32_t)data << 8U) | id;
}

static bool Rs01Motor_SendModeCommand(const rs01_motor_t *motor,
                                      uint8_t mode)
{
    uint8_t data[RS01_MOTOR_DATA_LENGTH] = {0U};
    uint32_t can_id;

    if ((motor == NULL) || (motor->can == NULL))
    {
        return false;
    }

    can_id = Rs01Motor_MakeExtendedId(mode, motor->master_id, motor->id);
    return BspCan_SendExtended(motor->can, can_id, data,
                               RS01_MOTOR_DATA_LENGTH);
}

bool Rs01Motor_Init(rs01_motor_t *motor, bsp_can_t *can, uint8_t id)
{
    if ((motor == NULL) || (can == NULL) || (id < RS01_MOTOR_MIN_ID)
        || (id > RS01_MOTOR_MAX_ID))
    {
        return false;
    }

    motor->id = id;
    motor->master_id = RS01_MOTOR_MASTER_ID;
    motor->can = can;
    motor->feedback.position = 0.0f;
    motor->feedback.speed = 0.0f;
    motor->feedback.torque = 0.0f;
    motor->feedback.temperature = 0.0f;
    motor->feedback.valid = false;
    return true;
}

bool Rs01Motor_SetZero(const rs01_motor_t *motor)
{
    uint8_t data[RS01_MOTOR_DATA_LENGTH] = {0U};
    uint32_t can_id;

    if ((motor == NULL) || (motor->can == NULL))
    {
        return false;
    }

    data[0] = 1U;
    can_id = Rs01Motor_MakeExtendedId(RS01_MOTOR_MODE_SET_ZERO,
                                      motor->master_id, motor->id);
    return BspCan_SendExtended(motor->can, can_id, data,
                               RS01_MOTOR_DATA_LENGTH);
}

bool Rs01Motor_Enable(const rs01_motor_t *motor)
{
    return Rs01Motor_SendModeCommand(motor, RS01_MOTOR_MODE_ENABLE);
}

bool Rs01Motor_Disable(const rs01_motor_t *motor)
{
    return Rs01Motor_SendModeCommand(motor, RS01_MOTOR_MODE_DISABLE);
}

bool Rs01Motor_SendMotionCommand(const rs01_motor_t *motor, float position,
                                 float speed, float kp, float kd,
                                 float torque)
{
    uint8_t data[RS01_MOTOR_DATA_LENGTH];
    uint16_t position_raw;
    uint16_t speed_raw;
    uint16_t kp_raw;
    uint16_t kd_raw;
    uint16_t torque_raw;

    if ((motor == NULL) || (motor->can == NULL))
    {
        return false;
    }

    position_raw = Rs01Motor_FloatToUint(position, RS01_MOTOR_POSITION_MIN,
                                          RS01_MOTOR_POSITION_MAX, 0xFFFFU);
    speed_raw = Rs01Motor_FloatToUint(speed, RS01_MOTOR_SPEED_MIN,
                                       RS01_MOTOR_SPEED_MAX, 0xFFFFU);
    kp_raw = Rs01Motor_FloatToUint(kp, RS01_MOTOR_KP_MIN,
                                    RS01_MOTOR_KP_MAX, 0xFFFFU);
    kd_raw = Rs01Motor_FloatToUint(kd, RS01_MOTOR_KD_MIN,
                                    RS01_MOTOR_KD_MAX, 0xFFFFU);
    torque_raw = Rs01Motor_FloatToUint(torque, RS01_MOTOR_TORQUE_MIN,
                                        RS01_MOTOR_TORQUE_MAX, 0xFFFFU);

    data[0] = (uint8_t)(position_raw >> 8U);
    data[1] = (uint8_t)position_raw;
    data[2] = (uint8_t)(speed_raw >> 8U);
    data[3] = (uint8_t)speed_raw;
    data[4] = (uint8_t)(kp_raw >> 8U);
    data[5] = (uint8_t)kp_raw;
    data[6] = (uint8_t)(kd_raw >> 8U);
    data[7] = (uint8_t)kd_raw;

    return BspCan_SendExtended(motor->can,
                               Rs01Motor_MakeExtendedId(
                                   RS01_MOTOR_MODE_CONTROL, torque_raw,
                                   motor->id),
                               data, RS01_MOTOR_DATA_LENGTH);
}

bool Rs01Motor_ProcessData(rs01_motor_t *motor, uint32_t can_id,
                           const uint8_t *data, uint8_t length)
{
    uint16_t position_raw;
    uint16_t speed_raw;
    uint16_t torque_raw;
    uint16_t temperature_raw;

    if ((motor == NULL) || (data == NULL)
        || ((can_id >> 24U) != RS01_MOTOR_MODE_FEEDBACK)
        || (((can_id >> 8U) & 0xFFU) != motor->id)
        || ((can_id & 0xFFU) != motor->master_id)
        || (length != RS01_MOTOR_DATA_LENGTH))
    {
        return false;
    }

    position_raw = ((uint16_t)data[0] << 8U) | data[1];
    speed_raw = ((uint16_t)data[2] << 8U) | data[3];
    torque_raw = ((uint16_t)data[4] << 8U) | data[5];
    temperature_raw = ((uint16_t)data[6] << 8U) | data[7];

    motor->feedback.position = Rs01Motor_UintToFloat(position_raw,
                                                      RS01_MOTOR_POSITION_MIN,
                                                      RS01_MOTOR_POSITION_MAX,
                                                      0xFFFFU);
    motor->feedback.speed = Rs01Motor_UintToFloat(speed_raw,
                                                   RS01_MOTOR_SPEED_MIN,
                                                   RS01_MOTOR_SPEED_MAX,
                                                   0xFFFFU);
    motor->feedback.torque = Rs01Motor_UintToFloat(torque_raw,
                                                    RS01_MOTOR_TORQUE_MIN,
                                                    RS01_MOTOR_TORQUE_MAX,
                                                    0xFFFFU);
    motor->feedback.temperature = (float)temperature_raw * 0.1f;
    motor->feedback.valid = true;
    return true;
}
