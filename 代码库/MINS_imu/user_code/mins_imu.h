#ifndef MINS_IMU_H
#define MINS_IMU_H

#include <stdint.h>

#include "bsp_uart.h"

typedef enum
{
    MINS_IMU_RATE_50HZ = 0x05U,
    MINS_IMU_RATE_100HZ = 0x06U,
    MINS_IMU_RATE_200HZ = 0x07U,
    MINS_IMU_RATE_500HZ = 0x08U
} mins_imu_rate_t;

typedef struct
{
    float pitch;
    float roll;
    float yaw;
} mins_imu_euler_t;

typedef struct
{
    bsp_uart_t *uart;
    mins_imu_euler_t euler;
    float yaw_last_raw;
    float yaw_total;
    uint8_t yaw_last_valid;
    uint32_t update_count;
} mins_imu_t;

void Mins_Imu_Init(mins_imu_t *imu, bsp_uart_t *uart);

void Mins_Imu_RequestEuler(mins_imu_t *imu);

void Mins_Imu_SetAutoOutput(mins_imu_t *imu, mins_imu_rate_t rate);

void Mins_Imu_OnRx(mins_imu_t *imu, const uint8_t *data, uint16_t len);

#endif /* MINS_IMU_H */
