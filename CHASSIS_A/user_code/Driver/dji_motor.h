#ifndef DJI_MOTOR_H
#define DJI_MOTOR_H

#include <stdint.h>

#include "bsp_can.h"

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
    uint8_t id;             //电机id
    dji_motor_type_t type;  //电机种类（2006/3508）
    uint16_t encoder;       //编码器返回值
    uint16_t last_encoder;
    int32_t encoder_turns;  //转子转动圈数
    int32_t total_encoder;  
    uint8_t encoder_ready;
    int16_t speed_rpm;      //转子速度
    int16_t current;        //电流
    uint8_t temperature;    //温度
    int16_t target_current;
    bsp_can_t *can;
} dji_motor_t;

void DjiMotor_Init(dji_motor_t *motor, bsp_can_t *can, uint8_t id,
                   dji_motor_type_t type);
void DjiMotor_SetCurrent(dji_motor_t *motor, int16_t current);
void DjiMotor_ParseFeedback(dji_motor_t *motor, const uint8_t *data);

bool DjiMotor_SendGroup(dji_motor_t *motor, uint8_t number);

#endif
