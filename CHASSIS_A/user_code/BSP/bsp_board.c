#include "bsp_board.h"

#include <stddef.h>

#include "fdcan.h"

static bsp_can_t bsp_board_can[BSP_CAN_MAX_INSTANCES];
static bool bsp_board_can_initialized;

bool BspBoardCan_Init(bsp_can_rx_callback_t rx_callback,
                      bsp_can_error_callback_t error_callback)
{
    static void * const handles[BSP_CAN_MAX_INSTANCES] =
    {
        &hfdcan1,
        &hfdcan2,
        &hfdcan3
    };
    uint8_t index;

    if (bsp_board_can_initialized)
    {
        return true;
    }
    if (rx_callback == NULL)
    {
        return false;
    }

    for (index = 0U; index < BSP_CAN_MAX_INSTANCES; index++)
    {
        if (!BspCan_Init(&bsp_board_can[index], handles[index], index,
                         rx_callback, error_callback))
        {
            return false;
        }
    }

    bsp_board_can_initialized = true;
    return true;
}

bsp_can_t *BspBoardCan_Get(uint8_t index)
{
    if (!bsp_board_can_initialized || (index >= BSP_CAN_MAX_INSTANCES))
    {
        return NULL;
    }
    return &bsp_board_can[index];
}

bool BspBoardCan_ProcessAll(void)
{
    bool healthy = bsp_board_can_initialized;
    uint8_t index;

    for (index = 0U; index < BSP_CAN_MAX_INSTANCES; index++)
    {
        healthy = BspCan_Process(&bsp_board_can[index]) && healthy;
    }
    return healthy;
}
