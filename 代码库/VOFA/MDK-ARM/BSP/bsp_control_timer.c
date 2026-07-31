#include "Bsp_Control_Timer.h"

#include "main.h"

extern TIM_HandleTypeDef htim7;

static volatile uint32_t timer_1ms_count;
static uint8_t timer_started;

uint32_t BspControlTimer_Get1msCount(void)
{
    if (timer_started == 0U)
    {
        if (HAL_TIM_Base_Start_IT(&htim7) == HAL_OK)
        {
            timer_started = 1U;
        }
    }

    return timer_1ms_count;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM7)
    {
        timer_1ms_count++;
    }
}
