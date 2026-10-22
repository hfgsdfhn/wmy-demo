#ifndef BSP_CAN_H
#define BSP_CAN_H

#include <stdbool.h>
#include <stdint.h>


#define BSP_CAN_MAX_DATA_LENGTH  8U
#define BSP_CAN_MAX_INSTANCES    3U
#define BSP_CAN_MAX_EXTENDED_ID   0x1FFFFFFFU

typedef struct bsp_can bsp_can_t;

typedef void (*bsp_can_rx_callback_t)(bsp_can_t *can,
                                      uint32_t can_id,
                                      const uint8_t *data,
                                      uint8_t length);

/**
 * @brief CAN实例结构体
 */
struct bsp_can
{
    uint8_t instance_index;
    void *hal_handle;
    bsp_can_rx_callback_t rx_callback;
    bool initialized;
};

bool BspCan_Init(bsp_can_t *can, void *hal_handle, uint8_t instance_index,
                 bsp_can_rx_callback_t callback);
bool BspCan_Process(bsp_can_t *can);
bool BspCan_Send(bsp_can_t *can, uint32_t can_id,
                 const uint8_t *data, uint8_t length);
bool BspCan_SendExtended(bsp_can_t *can, uint32_t can_id,
                         const uint8_t *data, uint8_t length);

#endif
