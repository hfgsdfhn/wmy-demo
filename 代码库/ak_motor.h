#ifndef AK_MOTOR_H
#define AK_MOTOR_H

#include <stdbool.h>
#include <stdint.h>

#include "Bsp_Can.h"

#define AK_MOTOR_MIN_ID                  1U
#define AK_MOTOR_MAX_ID                  0x7FU
#define AK_MOTOR_SPEED_MIN               (-45.0f)
#define AK_MOTOR_SPEED_MAX               45.0f

typedef struct
{
    float position;
    float speed;
    float torque;
    uint8_t temperature;
    bool valid;
} ak_motor_feedback_t;

typedef struct
{
    uint8_t id;
    bsp_can_t *can;
    float target_speed;
    ak_motor_feedback_t feedback;
} ak_motor_t;

bool AkMotor_Init(ak_motor_t *motor, bsp_can_t *can, uint8_t id);
bool AkMotor_SetSpeed(ak_motor_t *motor, float speed);
bool AkMotor_ParseFeedback(ak_motor_t *motor, const uint8_t *data,
                           uint8_t length);

#endif
