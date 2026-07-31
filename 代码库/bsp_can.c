/**
 * @file Bsp_Can.c
 * @author 王梦阳 wmy07823@163.com
 * @brief CAN总线驱动实现
 * @version 0.2
 * @date 2026-07-24
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#include "Bsp_Can.h"

// Automatically select the CAN peripheral available on the target device.
#if !defined(BSP_CAN_BACKEND_FDCAN) && !defined(BSP_CAN_BACKEND_BXCAN)
#include "main.h"
#if defined(FDCAN1) || defined(FDCAN2)
#define BSP_CAN_BACKEND_FDCAN  1U
#elif defined(CAN1) || defined(CAN2)
#define BSP_CAN_BACKEND_BXCAN  1U
#else
#error "No supported CAN peripheral found. Define BSP_CAN_BACKEND_FDCAN or BSP_CAN_BACKEND_BXCAN."
#endif
#endif
#if defined(BSP_CAN_BACKEND_FDCAN) && (BSP_CAN_BACKEND_FDCAN != 0U)
#include "fdcan.h"
#define BSP_CAN_HAS_FDCAN  1U
#else
#define BSP_CAN_HAS_FDCAN  0U
#endif
#if !BSP_CAN_HAS_FDCAN \
    && defined(BSP_CAN_BACKEND_BXCAN) && (BSP_CAN_BACKEND_BXCAN != 0U)
#include "main.h"
#define BSP_CAN_HAS_BXCAN  1U
#else
#define BSP_CAN_HAS_BXCAN  0U
#endif

    static bsp_can_t *can_instances[BSP_CAN_MAX_INSTANCES];//CAN实例数组

#if BSP_CAN_HAS_FDCAN
/// FDCAN驱动相关函数
static FDCAN_HandleTypeDef *BspCan_Handle(const bsp_can_t *can)
{
    if ((can == 0) || (can->hal_handle == 0))
    {
        return 0;
    }
    return (FDCAN_HandleTypeDef *)can->hal_handle;
}
// 配置全局过滤器，拒绝所有远程帧和非匹配帧
static bool BspCan_ConfigGlobalFilter(FDCAN_HandleTypeDef *handle)
{
    return HAL_FDCAN_ConfigGlobalFilter(handle,
                                        FDCAN_ACCEPT_IN_RX_FIFO0,
                                        FDCAN_REJECT,
                                        FDCAN_REJECT_REMOTE,
                                        FDCAN_REJECT_REMOTE) == HAL_OK;
}
// 配置默认过滤器，接收所有标准ID帧
static bool BspCan_ConfigDefaultFilter(FDCAN_HandleTypeDef *handle)
{
    FDCAN_FilterTypeDef filter = {0};

    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0U;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = 0U;
    filter.FilterID2 = 0U;
    return HAL_FDCAN_ConfigFilter(handle, &filter) == HAL_OK;
}
// 启动FDCAN硬件
static bool BspCan_StartHardware(bsp_can_t *can)
{
    FDCAN_HandleTypeDef *handle = BspCan_Handle(can);

    if ((handle == 0) || !BspCan_ConfigDefaultFilter(handle)
        || !BspCan_ConfigGlobalFilter(handle))
    {
        return false;
    }
    if (HAL_FDCAN_Start(handle) != HAL_OK)
    {
        return false;
    }
    if (HAL_FDCAN_ActivateNotification(handle,
                                       FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
                                       0U) != HAL_OK)
    {
        HAL_FDCAN_Stop(handle);
        return false;
    }
    return true;
}
#endif

#if BSP_CAN_HAS_BXCAN
// BxCAN驱动相关函数
static CAN_HandleTypeDef *BspCan_BxHandle(const bsp_can_t *can)
{
    return ((can != 0) && (can->hal_handle != 0))
        ? (CAN_HandleTypeDef *)can->hal_handle : 0;
}
// 启动BxCAN硬件
static bool BspCan_BxStartHardware(bsp_can_t *can)
{
    CAN_HandleTypeDef *handle = BspCan_BxHandle(can);
    CAN_FilterTypeDef filter = {0};

    if (handle == 0)
    {
        return false;
    }
    filter.FilterBank = 0U;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = 0U;
    filter.FilterIdLow = 0U;
    filter.FilterMaskIdHigh = 0U;
    filter.FilterMaskIdLow = 0U;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14U;

    return HAL_CAN_ConfigFilter(handle, &filter) == HAL_OK
        && HAL_CAN_Start(handle) == HAL_OK
        && HAL_CAN_ActivateNotification(handle,
                                         CAN_IT_RX_FIFO0_MSG_PENDING) == HAL_OK;
}
#endif
/**
 * @brief can初始化
 * 
 * @param can 
 * @param hal_handle 
 * @param instance_index 
 * @return true 
 * @return false 
 */
bool BspCan_Init(bsp_can_t *can, void *hal_handle, uint8_t instance_index,
                 bsp_can_rx_callback_t callback)
{
    if ((can == 0) || (hal_handle == 0) || (callback == 0)
        || (instance_index >= BSP_CAN_MAX_INSTANCES)
        || (can_instances[instance_index] != 0))
    {
        return false;
    }

    can->instance_index = instance_index;
    can->hal_handle = hal_handle;
    can->rx_callback = callback;
    can->initialized = false;
    can_instances[instance_index] = can;

#if BSP_CAN_HAS_FDCAN
    if (!BspCan_StartHardware(can))
    {
        can_instances[instance_index] = 0;
        can->hal_handle = 0;
        return false;
    }
    can->initialized = true;
    return true;
#elif BSP_CAN_HAS_BXCAN
    if (!BspCan_BxStartHardware(can))
    {
        can_instances[instance_index] = 0;
        can->hal_handle = 0;
        return false;
    }
    can->initialized = true;
    return true;
#else
    can_instances[instance_index] = 0;
    can->hal_handle = 0;
    return false;
#endif
}

/**
 * @brief 接收回调函数设置
 * 
 * @param can 
 * @param callback 
 */
/**
 * @brief can发送数据
 * 
 * @param can 
 * @param can_id 
 * @param data 
 * @param length 
 * @return true 
 * @return false 
 */
bool BspCan_Send(bsp_can_t *can, uint32_t can_id,
                 const uint8_t *data, uint8_t length)
{
#if BSP_CAN_HAS_FDCAN
    FDCAN_TxHeaderTypeDef header = {0};

    if ((can == 0) || !can->initialized || (can_id > 0x7FFU)
        || (length > BSP_CAN_MAX_DATA_LENGTH)
        || ((length > 0U) && (data == 0)))
    {
        return false;
    }

    header.Identifier = can_id;
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = (uint32_t)(length << 16U);
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    return HAL_FDCAN_AddMessageToTxFifoQ(BspCan_Handle(can),
                                         &header, (uint8_t *)data) == HAL_OK;
#elif BSP_CAN_HAS_BXCAN
    CAN_TxHeaderTypeDef header = {0};
    uint32_t mailbox;

    if ((can == 0) || !can->initialized || (can_id > 0x7FFU)
        || (length > BSP_CAN_MAX_DATA_LENGTH)
        || ((length > 0U) && (data == 0)))
    {
        return false;
    }
    header.StdId = can_id;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = length;
    return HAL_CAN_AddTxMessage(BspCan_BxHandle(can), &header,
                                (uint8_t *)data, &mailbox) == HAL_OK;
#else
    (void)can;
    (void)can_id;
    (void)data;
    (void)length;
    return false;
#endif
}

#if BSP_CAN_HAS_BXCAN
/**
 * @brief BxCAN中断选择
 * 
 * @param handle 
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *handle)
{
    CAN_RxHeaderTypeDef header;
    uint8_t data[BSP_CAN_MAX_DATA_LENGTH];
    bsp_can_t *can = 0;
    uint8_t index;

    for (index = 0U; index < BSP_CAN_MAX_INSTANCES; index++)    //轮询查找需要进入中断的函数           
    {
        if ((can_instances[index] != 0) &&
            (can_instances[index]->hal_handle == handle))
        {
            can = can_instances[index];
            break;
        }
    }
    if (can == 0)
    {
        return;
    }
    while (HAL_CAN_GetRxFifoFillLevel(handle, CAN_RX_FIFO0) > 0U)
    {
        if (HAL_CAN_GetRxMessage(handle, CAN_RX_FIFO0, &header, data) != HAL_OK)
        {
            break;
        }
        if ((header.IDE == CAN_ID_STD) && (can->rx_callback != 0))
        {
            can->rx_callback(can, header.StdId, data, header.DLC);      //进入具体任务中断
        }
    }
}
#endif

#if BSP_CAN_HAS_FDCAN
/**
 * @brief FDCAN接收中断回调函数
 * 
 * @param handle 
 * @param interrupt_flags 
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *handle,
                               uint32_t interrupt_flags)
{
    FDCAN_RxHeaderTypeDef header;
    uint8_t data[BSP_CAN_MAX_DATA_LENGTH];
    bsp_can_t *can = 0;
    uint8_t index;

    for (index = 0U; index < BSP_CAN_MAX_INSTANCES; index++)//轮询查找需要进入中断的函数    
    {
        if ((can_instances[index] != 0)
            && (can_instances[index]->hal_handle == handle))
        {
            can = can_instances[index];
            break;
        }
    }

    if ((can == 0)
        || ((interrupt_flags & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U))
    {
        return;
    }

    while (HAL_FDCAN_GetRxFifoFillLevel(handle, FDCAN_RX_FIFO0) > 0U)
    {
        if (HAL_FDCAN_GetRxMessage(handle, FDCAN_RX_FIFO0,
                                  &header, data) != HAL_OK)
        {
            break;
        }
        if ((header.IdType == FDCAN_STANDARD_ID)
            && (can->rx_callback != 0))
        {
            can->rx_callback(can, header.Identifier, data,                   //进入具体任务中断
                             (uint8_t)(header.DataLength >> 16U));
        }
    }
}
#endif
