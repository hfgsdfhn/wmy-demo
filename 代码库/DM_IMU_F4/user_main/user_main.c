#include "user_main.h"

#include "DM_Imu.h"
#include "VOFA.h"
#include "bsp_uart.h"
#include "usart.h"

#define IMU_UART_RX_BUFFER_SIZE 128U
#define VOFA_SEND_PERIOD_MS     10U

static bsp_uart_t imu_uart;
static bsp_uart_t vofa_uart;
static vofa_t vofa;
static uint8_t imu_uart_rx_buffer[IMU_UART_RX_BUFFER_SIZE];
static float imu_yaw;

bool UserMain_Init(void)
{
    if (!DM_IMU_Init(0x01U))
    {
        return false;
    }

    if (!Bsp_Uart_Init(&imu_uart, &huart6, BSP_UART_TX_BLOCKING,
                       imu_uart_rx_buffer, sizeof(imu_uart_rx_buffer), 3U,
                       DM_IMU_RxCallback))
    {
        return false;
    }

    if (!Bsp_Uart_Init(&vofa_uart, &huart1, BSP_UART_TX_BLOCKING,
                       NULL, 0U, 1U, NULL)
        || !Vofa_Init(&vofa, &vofa_uart)
        || !Vofa_Register(&vofa, "Yaw", &imu_yaw))
    {
        return false;
    }

    Vofa_SetPeriod(&vofa, VOFA_SEND_PERIOD_MS);
    return true;
}

void UserMain_Process(void)
{
    imu_yaw = DM_IMU_GetYaw();
    Vofa_Process(&vofa);
}
 