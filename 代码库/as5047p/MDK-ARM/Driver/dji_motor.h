#ifndef DJI_MOTOR_H
#define DJI_MOTOR_H

#include <stdint.h>

#include "Bsp_Can.h"

#define DJI_MOTOR_FEEDBACK_ID_BASE  0x201U
#define DJI_MOTOR_CONTROL_ID_1_4    0x200U
#define DJI_MOTOR_CONTROL_ID_5_8    0x1FFU
#define DJI_MOTOR_MIN_ID            1U
#define DJI_MOTOR_MAX_ID            8U
#define DJI_M2006_MAX_CURRENT        10000
#define DJI_M3508_MAX_CURRENT        16384

typedef enum
{
    DJI_MOTOR_TYPE_M2006 = 0,
    DJI_MOTOR_TYPE_M3508
} dji_motor_type_t;

typedef struct
{
    uint8_t id;
    dji_motor_type_t type;
    uint16_t encoder;
    uint16_t last_encoder;
    int32_t encoder_turns;
    int32_t total_encoder;
    uint8_t encoder_ready;
    int16_t speed_rpm;
    int16_t current;
    uint8_t temperature;
    int16_t target_current;
    bsp_can_t *can;
} dji_motor_t;

void DjiMotor_Init(dji_motor_t *motor, bsp_can_t *can, uint8_t id,
                   dji_motor_type_t type);
void DjiMotor_SetCurrent(dji_motor_t *motor, int16_t current);
void DjiMotor_ParseFeedback(dji_motor_t *motor, const uint8_t *data);

bool DjiMotor_SendGroup(dji_motor_t *motor, uint8_t number);

#endif
