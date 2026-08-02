/**
 * @file DM_Imu.h
 * @brief DM-IMU-L1 RS485 driver.
 */
#ifndef DM_IMU_H
#define DM_IMU_H

#include <stdbool.h>
#include <stdint.h>

#include "bsp_uart.h"

bool DM_IMU_Init(uint8_t device_id);

void DM_IMU_RxCallback(bsp_uart_t *uart, const uint8_t *data, uint16_t len);

float DM_IMU_GetYaw(void);

float DM_IMU_GetGyroZ(void);

#endif /* DM_IMU_H */
