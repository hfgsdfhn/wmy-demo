#include "app_init.h"

#include "DM_Imu.h"
#include "app_chassis.h"
#include "app_monitor.h"
#include "app_messages.h"
#include "app_rtos.h"
#include "bsp_uart.h"
#include "fdcan.h"
#include "usart.h"

#define APP_CHASSIS_CAN_INDEX            0U  /* 底盘 CAN 总线索引 */
#define APP_DM_IMU_DEVICE_ID              1U  /* DM-IMU 从机设备 ID */
#define APP_DM_IMU_UART_INSTANCE_INDEX    0U  /* DM-IMU 使用的串口实例索引 */
#define APP_DM_IMU_UART_RX_BUFFER_SIZE   64U  /* DM-IMU 串口接收缓冲区字节数 */
#define APP_DT35_UART_RX_BUFFER_SIZE     32U  /* DT35 串口接收缓冲区字节数 */
#define APP_DUAL_UART_RX_BUFFER_SIZE     32U  /* 双 MCU 串口接收缓冲区字节数 */
#define APP_PC_UART_RX_BUFFER_SIZE       64U  /* PC 串口接收缓冲区字节数 */

#define APP_REMOTE_UART_RX_BUFFER_SIZE 64U

static bool app_init_complete;
volatile uint32_t app_init_stage;

/* DMA1 cannot access DTCM (0x20000000); keep UART buffers in AXI SRAM. */
static uint8_t app_init_imu_uart_rx_buffer[APP_DM_IMU_UART_RX_BUFFER_SIZE]
    __attribute__((section(".dma_buffer"), aligned(32)));
static uint8_t app_init_dt35_uart_rx_buffer[APP_DT35_UART_RX_BUFFER_SIZE]
    __attribute__((section(".dma_buffer"), aligned(32)));
static uint8_t app_init_pc_uart_rx_buffer[APP_PC_UART_RX_BUFFER_SIZE]
    __attribute__((section(".dma_buffer"), aligned(32)));
static uint8_t app_init_remote_uart_rx_buffer[APP_REMOTE_UART_RX_BUFFER_SIZE]
    __attribute__((section(".dma_buffer"), aligned(32)));
static uint8_t app_init_dual_uart_rx_buffer[APP_DUAL_UART_RX_BUFFER_SIZE]
    __attribute__((section(".dma_buffer"), aligned(32)));

bsp_can_t can1;
bsp_uart_t uart10;
bsp_uart_t uart1;
bsp_uart_t uart2;
bsp_uart_t uart4;
bsp_uart_t uart5;
bsp_uart_t uart8;

static void AppInit_OnPcRx(bsp_uart_t *uart, const uint8_t *data, uint16_t len)
{
    (void)uart;
    AppRtos_UartRxFromISR(APP_UART_RX_PC, data, len);
}

static void AppInit_OnRemoteRx(bsp_uart_t *uart, const uint8_t *data, uint16_t len)
{
    (void)uart;
    AppRtos_UartRxFromISR(APP_UART_RX_REMOTE, data, len);
}

bool AppInit_Run(void)
{
    if (app_init_complete)
    {
        return true;
    }
    app_init_stage = 1U;
    if (!AppSafety_Init() || !AppMonitor_Init())
    {
        return false;
    }
    app_init_stage = 2U;
    if (!BspCan_Init(&can1, &hfdcan1, APP_CHASSIS_CAN_INDEX,
                     AppMessages_OnCanRx, AppMessages_OnCanError))
    {
        return false;
    }
    app_init_stage = 3U;
    if (!AppChassis_Init(&can1))
    {
        return false;
    }
    AppMessages_Init();
    app_init_stage = 4U;
    if (!DM_IMU_Init(APP_DM_IMU_DEVICE_ID))
    {
        return false;
    }
    app_init_stage = 5U;
    if (!Bsp_Uart_Init(&uart10, &huart10, BSP_UART_TX_BLOCKING,
                       app_init_pc_uart_rx_buffer, sizeof(app_init_pc_uart_rx_buffer),
                       3U, AppInit_OnPcRx))
    {
        return false;
    }
    app_init_stage = 6U;
    if (!Bsp_Uart_Init(&uart1, &huart1, BSP_UART_TX_DMA,
                       app_init_remote_uart_rx_buffer, sizeof(app_init_remote_uart_rx_buffer),
                       4U, AppInit_OnRemoteRx))
    {
        return false;
    }
    app_init_stage = 7U;
    if (!Bsp_Uart_Init(&uart4, &huart4, BSP_UART_TX_BLOCKING,
                      app_init_dt35_uart_rx_buffer, sizeof(app_init_dt35_uart_rx_buffer),
                      1U, AppMessages_OnDt35Rx))
    {
        return false;
    }
    app_init_stage = 8U;
    if (!Bsp_Uart_Init(&uart8, &huart8, BSP_UART_TX_BLOCKING,
                        app_init_dual_uart_rx_buffer, sizeof(app_init_dual_uart_rx_buffer),
                        2U, AppMessages_OnDualMcuRx))
    {
        return false;
    }
    app_init_stage = 9U;
    if (!Bsp_Uart_Init(&uart5, &huart5, BSP_UART_TX_BLOCKING,
                       app_init_imu_uart_rx_buffer, sizeof(app_init_imu_uart_rx_buffer),
                       APP_DM_IMU_UART_INSTANCE_INDEX, AppMessages_OnImuRx))
    {
        return false;
    }
    app_init_stage = 10U;
    app_init_complete = true;
    return true;
}
