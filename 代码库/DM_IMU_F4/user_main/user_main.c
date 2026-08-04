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
static float imu_accel_x;
static float imu_accel_y;
static float imu_velocity_x;
static float imu_velocity_y;
static float imu_position_x;
static float imu_position_y;

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
        || !Vofa_Register(&vofa, "Yaw", &imu_yaw)
        || !Vofa_Register(&vofa, "Accel_X", &imu_accel_x)
        || !Vofa_Register(&vofa, "Accel_Y", &imu_accel_y)
        || !Vofa_Register(&vofa, "Velocity_X", &imu_velocity_x)
        || !Vofa_Register(&vofa, "Velocity_Y", &imu_velocity_y)
        || !Vofa_Register(&vofa, "Position_X", &imu_position_x)
        || !Vofa_Register(&vofa, "Position_Y", &imu_position_y))
    {
        return false;
    }

    Vofa_SetPeriod(&vofa, VOFA_SEND_PERIOD_MS);
    return true;
}

void UserMain_Process(void)
{
    imu_yaw = DM_IMU_GetYaw();
    imu_accel_x = DM_IMU_GetAccelX();
    imu_accel_y = DM_IMU_GetAccelY();
    imu_velocity_x = DM_IMU_GetVelocityX();
    imu_velocity_y = DM_IMU_GetVelocityY();
    imu_position_x = DM_IMU_GetPositionX();
    imu_position_y = DM_IMU_GetPositionY();
    Vofa_Process(&vofa);
}
 
