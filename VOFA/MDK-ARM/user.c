#include "main.h"
#include "math.h"
#include "vofa.h"

extern UART_HandleTypeDef huart4;

static float sinx(float elapsed_seconds)
{
    return sinf(6.28318530718f * elapsed_seconds);
}

void _loop(void)
{
    static uint32_t start_tick;
    static uint32_t last_sample_tick;
    static uint8_t initialized;
    uint32_t now = HAL_GetTick();

    if (initialized == 0U)
    {
        start_tick = now;
        last_sample_tick = now - 10U;
        initialized = 1U;
    }

    if ((uint32_t)(now - last_sample_tick) < 10U)
    {
        return;
    }
    last_sample_tick = now;

    vofa.fdata[0] = 10*sinx((float)((now - start_tick) % 1000U) / 1000.0f);
    vofa.send(&vofa, &huart4);
}
