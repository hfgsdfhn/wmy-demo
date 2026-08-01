/**
 * @file DM_Imu.h
 * @brief DM-IMU-L1 RS485 driver.
 */
#ifndef DM_IMU_H
#define DM_IMU_H

#include <stdbool.h>
#include <stdint.h>

#include "bsp_uart.h"

typedef enum
{
    DM_IMU_BAUD_115200 = 1U,
    DM_IMU_BAUD_230400,
    DM_IMU_BAUD_460800,
    DM_IMU_BAUD_500000,
    DM_IMU_BAUD_921600,
    DM_IMU_BAUD_1000000
} dm_imu_baud_t;

bool DM_IMU_Init(bsp_uart_t *uart, uint8_t device_id, dm_imu_baud_t baud);

float DM_IMU_GetYaw(void);

float DM_IMU_GetGyroZ(void);

#endif /* DM_IMU_H */
