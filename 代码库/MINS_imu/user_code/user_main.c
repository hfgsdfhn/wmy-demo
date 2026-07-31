#include "user_main.h"

#include "bsp_uart.h"
#include "mins_imu.h"
#include "usart.h"
#include "vofa.h"

#define VOFA_SEND_PERIOD_MS 20U

static uint8_t mins_rx_buf[32];
static bsp_uart_t mins_uart;
static mins_imu_t mins_imu;
static bsp_uart_t vofa_uart;
static vofa_t vofa;

static void Mins_Imu_RxCallback(bsp_uart_t *uart, const uint8_t *data,
                                uint16_t len)
{
    Mins_Imu_OnRx(&mins_imu, data, len);
}

void User_Main_Init(void)
{
    Mins_Imu_Init(&mins_imu, &mins_uart);
    if (Bsp_Uart_Init(&mins_uart, &huart1, BSP_UART_TX_BLOCKING,
                      mins_rx_buf, sizeof(mins_rx_buf), 0U,
                      Mins_Imu_RxCallback))
    {
        Mins_Imu_SetAutoOutput(&mins_imu, MINS_IMU_RATE_100HZ);
    }

    if (Bsp_Uart_Init(&vofa_uart, &huart2, BSP_UART_TX_BLOCKING,NULL, 0U, 1U, NULL) && Vofa_Init(&vofa, &vofa_uart))
    {
        Vofa_Register(&vofa, "pitch", &mins_imu.euler.pitch);
        Vofa_Register(&vofa, "roll", &mins_imu.euler.roll);
        Vofa_Register(&vofa, "yaw", &mins_imu.euler.yaw);
        Vofa_SetPeriod(&vofa, VOFA_SEND_PERIOD_MS);
    }
}

void User_Main_Loop(void)
{
    Vofa_Process(&vofa);
}
