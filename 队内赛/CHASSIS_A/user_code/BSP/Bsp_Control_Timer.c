#include "Bsp_Control_Timer.h"

#include <stddef.h>

static bsp_tim_t *bsp_tim_instances[BSP_TIM_MAX_INSTANCES];

bool BspTim_Init(bsp_tim_t *tim, TIM_HandleTypeDef *htim,
                 uint8_t instance_index, bsp_tim_callback_t callback)
{
    if ((tim == NULL) || (htim == NULL) || (callback == NULL)
        || (instance_index >= BSP_TIM_MAX_INSTANCES)
        || (bsp_tim_instances[instance_index] != NULL))
    {
        return false;
    }

    tim->htim = htim;
    tim->callback = callback;
    tim->instance_index = instance_index;
    tim->initialized = false;
    bsp_tim_instances[instance_index] = tim;

    if (HAL_TIM_Base_Start_IT(htim) != HAL_OK)
    {
        bsp_tim_instances[instance_index] = NULL;
        tim->htim = NULL;
        tim->callback = NULL;
        return false;
    }

    tim->initialized = true;
    return true;
}

void BspTim_DispatchPeriodElapsedFromISR(TIM_HandleTypeDef *htim)
{
    bsp_tim_t *tim;
    uint8_t index;

    for (index = 0U; index < BSP_TIM_MAX_INSTANCES; index++)
    {
        tim = bsp_tim_instances[index];
        if ((tim != NULL) && tim->initialized && (tim->htim == htim))
        {
            tim->callback();
            return;
        }
    }
}
