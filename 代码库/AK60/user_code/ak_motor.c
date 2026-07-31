#include "ak_motor.h"

#include <stddef.h>

#include "main.h"

#define AK_MOTOR_SERVO_SPEED_MODE         3U
#define AK_MOTOR_SERVO_SPEED_DATA_LENGTH  4U
#define AK_MOTOR_SERVO_FEEDBACK_LENGTH    8U
#define AK_MOTOR_SPEED_RPM_MAX            100000.0f

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

static float AkMotor_MoveToward(float current, float target, float step)
{
    if (current < target)
    {
        return AkMotor_Clamp(current + step, current, target);
    }
    if (current > target)
    {
        return AkMotor_Clamp(current - step, target, current);
    }
    return current;
}

static int16_t AkMotor_ReadI16Be(const uint8_t *data)
{
    return (int16_t)(((uint16_t)data[0] << 8U) | data[1]);
}

static void AkMotor_WriteI32Be(uint8_t *data, int32_t value)
{
    uint32_t raw = (uint32_t)value;

    data[0] = (uint8_t)(raw >> 24U);
    data[1] = (uint8_t)(raw >> 16U);
    data[2] = (uint8_t)(raw >> 8U);
    data[3] = (uint8_t)raw;
}

static bool AkMotor_SendServoSpeedCommand(const ak_motor_t *motor)
{
    uint8_t data[AK_MOTOR_SERVO_SPEED_DATA_LENGTH];
    float speed;
    int32_t speed_raw;
    uint32_t can_id;

    speed = AkMotor_Clamp(motor->command_speed, AK_MOTOR_SPEED_MIN,
                          AK_MOTOR_SPEED_MAX);
    speed_raw = (int32_t)speed;
    speed_raw = (int32_t)AkMotor_Clamp((float)speed_raw,
                                       -AK_MOTOR_SPEED_RPM_MAX,
                                       AK_MOTOR_SPEED_RPM_MAX);
    can_id = motor->id | ((uint32_t)AK_MOTOR_SERVO_SPEED_MODE << 8U);
    AkMotor_WriteI32Be(data, speed_raw);
    return BspCan_SendExtended(motor->can, can_id, data,
                               AK_MOTOR_SERVO_SPEED_DATA_LENGTH);
}

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
    motor->command_speed = 0.0f;
    motor->step_speed = 0.0f;
    motor->step_interval_ms = 0U;
    motor->last_step_tick = 0U;
    motor->step_started = false;
    motor->feedback.position = 0.0f;
    motor->feedback.speed = 0.0f;
    motor->feedback.torque = 0.0f;
    motor->feedback.temperature = 0U;
    motor->feedback.error_code = 0U;
    motor->feedback.valid = false;
    return true;
}

bool AkMotor_SetSpeed(ak_motor_t *motor, float speed)
{
    if ((motor == NULL) || (motor->can == NULL))
    {
        return false;
    }

    motor->target_speed = AkMotor_Clamp(speed, AK_MOTOR_SPEED_MIN,
                                        AK_MOTOR_SPEED_MAX);
    motor->command_speed = motor->target_speed;
    motor->step_started = false;
    return AkMotor_SendServoSpeedCommand(motor);
}

bool AkMotor_SetSpeedStep(ak_motor_t *motor, float target_rpm,
                          float step_rpm, uint32_t interval_ms)
{
    uint32_t now;

    if ((motor == NULL) || (motor->can == NULL) || (step_rpm <= 0.0f)
        || (interval_ms == 0U))
    {
        return false;
    }

    motor->target_speed = AkMotor_Clamp(target_rpm, AK_MOTOR_SPEED_MIN,
                                        AK_MOTOR_SPEED_MAX);
    motor->step_speed = step_rpm;
    motor->step_interval_ms = interval_ms;
    now = HAL_GetTick();

    if (!motor->step_started
        || ((uint32_t)(now - motor->last_step_tick) >= interval_ms))
    {
        motor->command_speed = AkMotor_MoveToward(motor->command_speed,
                                                   motor->target_speed,
                                                   step_rpm);
        motor->last_step_tick = now;
        motor->step_started = true;
        return AkMotor_SendServoSpeedCommand(motor);
    }

    return true;
}

bool AkMotor_ParseFeedback(ak_motor_t *motor, const uint8_t *data,
                           uint8_t length)
{
    if ((motor == NULL) || (data == NULL)
        || (length != AK_MOTOR_SERVO_FEEDBACK_LENGTH))
    {
        return false;
    }

    motor->feedback.position = (float)AkMotor_ReadI16Be(&data[0]) * 0.1f;
    motor->feedback.speed = (float)AkMotor_ReadI16Be(&data[2]) * 10.0f;
    motor->feedback.torque = (float)AkMotor_ReadI16Be(&data[4]) * 0.01f;
    motor->feedback.temperature = data[6];
    motor->feedback.error_code = data[7];
    motor->feedback.valid = true;
    return true;
}
