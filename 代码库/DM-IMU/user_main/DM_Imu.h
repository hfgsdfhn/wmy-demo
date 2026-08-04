/**
 * @file DM_Imu.h
 * @brief DM-IMU-L1 RS485 driver.
 */
#ifndef DM_IMU_H
#define DM_IMU_H

#include <stdbool.h>
#include <stdint.h>

#define DM_IMU_FRAME_HEAD_0         0x55U
#define DM_IMU_FRAME_HEAD_1         0xAAU
#define DM_IMU_FRAME_TAIL           0x0AU
#define DM_IMU_FRAME_SIZE            19U
#define DM_IMU_ID_OFFSET              2U
#define DM_IMU_REGISTER_OFFSET        3U
#define DM_IMU_DATA_OFFSET            4U
#define DM_IMU_CRC_OFFSET            16U

#define DM_IMU_REGISTER_GYRO          0x02U
#define DM_IMU_REGISTER_EULER         0x03U

typedef struct
{
    uint8_t device_id;
    volatile float yaw;
    volatile float gyro_z;
    uint8_t rx_frame[DM_IMU_FRAME_SIZE];
    uint16_t rx_frame_len;
} dm_imu_t;

extern dm_imu_t dm_imu;

bool DM_IMU_Init(uint8_t device_id);

bool DM_IMU_ParseFeedback(const uint8_t *data, uint16_t len);

float DM_IMU_GetYaw(void);

float DM_IMU_GetGyroZ(void);

#endif /* DM_IMU_H */
