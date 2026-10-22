#ifndef AK_MOTOR_H
#define AK_MOTOR_H /* AK 电机驱动头文件保护宏 */

#include <stdbool.h>
#include <stdint.h>

#include "bsp_can.h"

#define AK_MOTOR_MIN_ID                  1U     /* 有效电机 ID 最小值 */
#define AK_MOTOR_MAX_ID                  0x7FU  /* 有效电机 ID 最大值 */

//速度限幅
#define AK_MOTOR_SPEED_MIN               (-100000.0f) /* 允许的最小电机转速，单位 eRPM */
#define AK_MOTOR_SPEED_MAX               100000.0f    /* 允许的最大电机转速，单位 eRPM */

typedef struct
{
    float position;
    float speed;
    float torque;
    uint8_t temperature;
    uint8_t error_code;
    bool valid;
} ak_motor_feedback_t;

typedef struct
{
    uint8_t id;
    bsp_can_t *can;
    float target_speed;
    float command_speed;
    float step_speed;
    uint32_t step_interval_ms;
    uint32_t last_step_tick;
    bool step_started;
    ak_motor_feedback_t feedback;
} ak_motor_t;

bool AkMotor_Init(ak_motor_t *motor, bsp_can_t *can, uint8_t id);
bool AkMotor_SetSpeed(ak_motor_t *motor, float speed);
bool AkMotor_SetSpeedStep(ak_motor_t *motor, float target_rpm,
                          float step_rpm, uint32_t interval_ms);
bool AkMotor_ParseFeedback(ak_motor_t *motor, const uint8_t *data,
                           uint8_t length);

#endif
