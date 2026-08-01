#include "Bsp_Control_Timer.h"

static bsp_tim_t *tim_instances[BSP_TIM_MAX_INSTANCES];

/**
 * @brief 定时器初始化
 * 
 * @param tim               传入bsp_tim_t指针
 * @param htim              传入句柄指针
 * @param instance_index    索引，用来选择传入的定时器结构体数组
 * @param callback          设置中断回调函数
 * @return true 
 * @return false 
 */
bool BspTim_Init(bsp_tim_t *tim, TIM_HandleTypeDef *htim,
                 uint8_t instance_index, bsp_tim_callback_t callback)
{
    if ((tim == NULL) || (htim == NULL) || (callback == NULL)
        || (instance_index >= BSP_TIM_MAX_INSTANCES)
        || (tim_instances[instance_index] != NULL))
    {
        return false;
    }

    tim->htim = htim;
    tim->callback = callback;
    tim->instance_index = instance_index;
    tim->initialized = false;
    tim_instances[instance_index] = tim;

    if (HAL_TIM_Base_Start_IT(htim) != HAL_OK)
    {
        tim_instances[instance_index] = NULL;
        tim->htim = NULL;
        tim->callback = NULL;
        return false;
    }

    tim->initialized = true;
    return true;
}

/**
 * @brief HAL库底层中断回调处理函数
 *        它触发后说明有tim进入中断
 *        之后轮询查找进入具体的实例中断
 * @param htim 
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    uint8_t index;
    bsp_tim_t *tim;

    for (index = 0U; index < BSP_TIM_MAX_INSTANCES; index++)    //轮询
    {
        tim = tim_instances[index];         //这里做中断选择

        if ((tim != NULL) && tim->initialized && (tim->htim == htim))
        {
            if (tim->callback != NULL)
            {
                tim->callback();        //指向选好的中断函数
            }
            return;
        }
    }
}
