#ifndef RS01_MOTOR_H
#define RS01_MOTOR_H

#include <stdbool.h>
#include <stdint.h>

#include "bsp_can.h"

#define RS01_MOTOR_MIN_ID            1U
#define RS01_MOTOR_MAX_ID            0x7FU
#define RS01_MOTOR_MASTER_ID          0U

#define RS01_MOTOR_POSITION_MIN      (-12.57f)
#define RS01_MOTOR_POSITION_MAX      12.57f
#define RS01_MOTOR_SPEED_MIN         (-44.0f)
#define RS01_MOTOR_SPEED_MAX         44.0f
#define RS01_MOTOR_KP_MIN            0.0f
#define RS01_MOTOR_KP_MAX            500.0f
#define RS01_MOTOR_KD_MIN            0.0f
#define RS01_MOTOR_KD_MAX            5.0f
#define RS01_MOTOR_TORQUE_MIN        (-17.0f)
#define RS01_MOTOR_TORQUE_MAX        17.0f

typedef struct
{
    float position;
    float speed;
    float torque;
    float temperature;
    bool valid;
} rs01_motor_feedback_t;

typedef struct
{
    uint8_t id;
    uint8_t master_id;
    bsp_can_t *can;
    rs01_motor_feedback_t feedback;
} rs01_motor_t;

bool Rs01Motor_Init(rs01_motor_t *motor, bsp_can_t *can, uint8_t id);
bool Rs01Motor_SetZero(const rs01_motor_t *motor);
bool Rs01Motor_Enable(const rs01_motor_t *motor);
bool Rs01Motor_Disable(const rs01_motor_t *motor);
bool Rs01Motor_SendMotionCommand(const rs01_motor_t *motor, float position,
                                 float speed, float kp, float kd,
                                 float torque);
bool Rs01Motor_ProcessData(rs01_motor_t *motor, uint32_t can_id,
                           const uint8_t *data, uint8_t length);

#endif
