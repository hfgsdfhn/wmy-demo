#ifndef BSP_CONTROL_TIMER_H
#define BSP_CONTROL_TIMER_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"

#define BSP_TIM_MAX_INSTANCES  14U

typedef void (*bsp_tim_callback_t)(void);

typedef struct
{
    TIM_HandleTypeDef *htim;
    bsp_tim_callback_t callback;
    uint8_t instance_index;
    bool initialized;
} bsp_tim_t;

bool BspTim_Init(bsp_tim_t *tim, TIM_HandleTypeDef *htim,
                 uint8_t instance_index, bsp_tim_callback_t callback);

#endif
