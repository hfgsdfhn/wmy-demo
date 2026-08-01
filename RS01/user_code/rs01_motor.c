#include "rs01_motor.h"

#include <stddef.h>

#define RS01_MOTOR_COMMAND_ID         0x7FFU
#define RS01_MOTOR_DATA_LENGTH        8U
#define RS01_MOTOR_ENABLE_COMMAND     0xFCU
#define RS01_MOTOR_DISABLE_COMMAND    0xFDU

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

static bool Rs01Motor_SendModeCommand(const rs01_motor_t *motor,
                                      uint8_t command)
{
    uint8_t data[RS01_MOTOR_DATA_LENGTH] = {0xFFU, 0xFFU, 0xFFU, 0xFFU,
                                             0xFFU, 0xFFU, 0xFFU, 0x00U};

    if ((motor == NULL) || (motor->can == NULL))
    {
        return false;
    }

    data[7] = command;
    return BspCan_Send(motor->can, RS01_MOTOR_COMMAND_ID, data,
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
    motor->can = can;
    motor->feedback.position = 0.0f;
    motor->feedback.speed = 0.0f;
    motor->feedback.torque = 0.0f;
    motor->feedback.temperature = 0.0f;
    motor->feedback.valid = false;
    return true;
}

bool Rs01Motor_Enable(const rs01_motor_t *motor)
{
    return Rs01Motor_SendModeCommand(motor, RS01_MOTOR_ENABLE_COMMAND);
}

bool Rs01Motor_Disable(const rs01_motor_t *motor)
{
    return Rs01Motor_SendModeCommand(motor, RS01_MOTOR_DISABLE_COMMAND);
}

bool Rs01Motor_SendMitCommand(const rs01_motor_t *motor, float position,
                              float speed, float kp, float kd, float torque)
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
                                       RS01_MOTOR_SPEED_MAX, 0x0FFFU);
    kp_raw = Rs01Motor_FloatToUint(kp, RS01_MOTOR_KP_MIN,
                                    RS01_MOTOR_KP_MAX, 0x0FFFU);
    kd_raw = Rs01Motor_FloatToUint(kd, RS01_MOTOR_KD_MIN,
                                    RS01_MOTOR_KD_MAX, 0x0FFFU);
    torque_raw = Rs01Motor_FloatToUint(torque, RS01_MOTOR_TORQUE_MIN,
                                        RS01_MOTOR_TORQUE_MAX, 0x0FFFU);

    data[0] = (uint8_t)(position_raw >> 8U);
    data[1] = (uint8_t)position_raw;
    data[2] = (uint8_t)(speed_raw >> 4U);
    data[3] = (uint8_t)((speed_raw << 4U) | (kp_raw >> 8U));
    data[4] = (uint8_t)kp_raw;
    data[5] = (uint8_t)(kd_raw >> 4U);
    data[6] = (uint8_t)((kd_raw << 4U) | (torque_raw >> 8U));
    data[7] = (uint8_t)torque_raw;

    return BspCan_Send(motor->can, motor->id, data, RS01_MOTOR_DATA_LENGTH);
}

bool Rs01Motor_ProcessData(rs01_motor_t *motor, uint32_t can_id,
                           const uint8_t *data, uint8_t length)
{
    uint16_t position_raw;
    uint16_t speed_raw;
    uint16_t torque_raw;
    uint16_t temperature_raw;

    if ((motor == NULL) || (data == NULL) || (can_id != motor->id)
        || (length != RS01_MOTOR_DATA_LENGTH) || (data[0] != motor->id))
    {
        return false;
    }

    position_raw = ((uint16_t)data[1] << 8U) | data[2];
    speed_raw = ((uint16_t)data[3] << 4U) | (data[4] >> 4U);
    torque_raw = ((uint16_t)(data[4] & 0x0FU) << 8U) | data[5];
    temperature_raw = ((uint16_t)data[6] << 8U) | data[7];

    motor->feedback.position = Rs01Motor_UintToFloat(position_raw,
                                                      RS01_MOTOR_POSITION_MIN,
                                                      RS01_MOTOR_POSITION_MAX,
                                                      0xFFFFU);
    motor->feedback.speed = Rs01Motor_UintToFloat(speed_raw,
                                                   RS01_MOTOR_SPEED_MIN,
                                                   RS01_MOTOR_SPEED_MAX,
                                                   0x0FFFU);
    motor->feedback.torque = Rs01Motor_UintToFloat(torque_raw,
                                                    RS01_MOTOR_TORQUE_MIN,
                                                    RS01_MOTOR_TORQUE_MAX,
                                                    0x0FFFU);
    motor->feedback.temperature = (float)temperature_raw * 0.1f;
    motor->feedback.valid = true;
    return true;
}
