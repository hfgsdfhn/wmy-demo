#include "user_main.h"

#include <stdbool.h>
#include <stdint.h>

#include "W25QXX.h"
#include "bsp_spi.h"
#include "bsp_uart.h"
#include "main.h"
#include "spi.h"
#include "usart.h"

#define W25Q128_TEST_ADDRESS       0x000000UL
#define W25Q128_TEST_DATA_LENGTH   16U
#define UART_SEND_PERIOD_MS        200U

static bsp_spi_t g_spi;
static bsp_uart_t g_uart;
static w25qxx_t g_flash;
static uint8_t g_read_data[W25Q128_TEST_DATA_LENGTH];
static uint32_t g_last_send_tick;
static bool g_flash_ready;

static const uint8_t g_write_data[W25Q128_TEST_DATA_LENGTH] =
{
    0x00U, 0x11U, 0x22U, 0x33U,
    0x44U, 0x55U, 0x66U, 0x77U,
    0x88U, 0x99U, 0xAAU, 0xBBU,
    0xCCU, 0xDDU, 0xEEU, 0xFFU
};

void init(void)
{
    g_flash_ready = false;

    if (!Bsp_Uart_Init(&g_uart, &huart10, BSP_UART_TX_BLOCKING,
                       NULL, 0U, 0U, NULL))
    {
        return;
    }

    if (!BspSpi_Init(&g_spi, &hspi1, SPI1_CS_GPIO_Port, SPI1_CS_Pin)
        || !W25QXX_Init(&g_flash, &g_spi)
        || (g_flash.model != W25QXX_MODEL_W25Q128)
        || !W25QXX_Unprotect(&g_flash)
        || !W25QXX_EraseSector(&g_flash, W25Q128_TEST_ADDRESS)
        || !W25QXX_Write(&g_flash, W25Q128_TEST_ADDRESS, g_write_data,
                          W25Q128_TEST_DATA_LENGTH)
        || !W25QXX_Read(&g_flash, W25Q128_TEST_ADDRESS, g_read_data,
                         W25Q128_TEST_DATA_LENGTH))
    {
        return;
    }

    g_flash_ready = true;
    g_last_send_tick = HAL_GetTick() - UART_SEND_PERIOD_MS;
}

void loop(void)
{
    if (!g_flash_ready
        || ((uint32_t)(HAL_GetTick() - g_last_send_tick) < UART_SEND_PERIOD_MS))
    {
        return;
    }

    g_last_send_tick = HAL_GetTick();
    if (W25QXX_Read(&g_flash, W25Q128_TEST_ADDRESS, g_read_data,
                    W25Q128_TEST_DATA_LENGTH))
    {
        Bsp_Uart_Send(&g_uart, g_read_data, sizeof(g_read_data));
    }
}
