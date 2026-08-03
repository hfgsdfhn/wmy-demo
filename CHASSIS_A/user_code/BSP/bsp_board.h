#ifndef BSP_BOARD_H
#define BSP_BOARD_H

#include <stdbool.h>
#include <stdint.h>

#include "bsp_can.h"

bool BspBoardCan_Init(bsp_can_rx_callback_t rx_callback,
                      bsp_can_error_callback_t error_callback);
bsp_can_t *BspBoardCan_Get(uint8_t index);
bool BspBoardCan_ProcessAll(void);

#endif /* BSP_BOARD_H */
