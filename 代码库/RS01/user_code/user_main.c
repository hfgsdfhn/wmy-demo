#include "user_main.h"

#include <stddef.h>

#include "can.h"

#define RS01_CAN_INSTANCE_INDEX       0U
#define RS01_MOTOR_ID                 1U
#define RS01_MOTION_SEND_INTERVAL_MS  5U

static bsp_can_t can1;
static uint32_t last_motion_send_tick;

rs01_motor_t rs01_motor;

volatile float rs01_motion_position = 0.0f;
volatile float rs01_motion_speed = 0.0f;
volatile float rs01_motion_kp = 8.0f;
volatile float rs01_motion_kd = 1.0f;
volatile float rs01_motion_torque = 0.0f;

static void Can1_RxCallback(bsp_can_t *can, uint32_t can_id,
                            const uint8_t *data, uint8_t length)
{
    if ((can == &can1) && (data != NULL))
    {
        Rs01Motor_ProcessData(&rs01_motor, can_id, data, length);
    }
}

bool UserMain_Init(void)
{
    if (!BspCan_Init(&can1, &hcan1, RS01_CAN_INSTANCE_INDEX,
                     Can1_RxCallback)
        || !Rs01Motor_Init(&rs01_motor, &can1, RS01_MOTOR_ID))
    {
        return false;
    }

    if (!Rs01Motor_SetZero(&rs01_motor))
    {
        return false;
    }

    last_motion_send_tick = HAL_GetTick();
    return Rs01Motor_Enable(&rs01_motor);
}

void UserMain_Loop(void)
{
    uint32_t now;

    if (!BspCan_Process(&can1))
    {
        return;
    }

    now = HAL_GetTick();
    if ((uint32_t)(now - last_motion_send_tick)
        < RS01_MOTION_SEND_INTERVAL_MS)
    {
        return;
    }

    last_motion_send_tick = now;
    Rs01Motor_SendMotionCommand(&rs01_motor, rs01_motion_position,
                                 rs01_motion_speed, rs01_motion_kp,
                                 rs01_motion_kd, rs01_motion_torque);
}
