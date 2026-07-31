#include "user_main.h"

#include <stddef.h>

#include "ak_motor.h"
#include "fdcan.h"

static bsp_can_t can1;
ak_motor_t ak60;
float speed = 0.0f;

#define AK60_SPEED_STEP_RPM          100.0f
#define AK60_SPEED_STEP_INTERVAL_MS  10U

static void can1_callback(bsp_can_t *can, uint32_t can_id,
                          const uint8_t *data, uint8_t length)
{
    if ((can != &can1) || (data == NULL))
    {
        return;
    }
    if ((can_id & 0xFFU) == ak60.id)
    {
        AkMotor_ParseFeedback(&ak60, data, length);
        return;
    }
}

void Init(void)
{
    if (!BspCan_Init(&can1, &hfdcan1, 0U, can1_callback))
    {
        return;
    }
    AkMotor_Init(&ak60, &can1, 2U);
}

void Loop(void)
{
    if (BspCan_Process(&can1))
    {
        AkMotor_SetSpeedStep(&ak60, speed, AK60_SPEED_STEP_RPM,
                             AK60_SPEED_STEP_INTERVAL_MS);
    }
    HAL_Delay(2);
}
