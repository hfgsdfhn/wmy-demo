/**
 * @file DM_Imu.h
 * @brief DM-IMU-L1 RS485 driver.
 */
#ifndef DM_IMU_H
#define DM_IMU_H /* DM-IMU 驱动头文件保护宏 */

#include <stdbool.h>
#include <stdint.h>

#define DM_IMU_FRAME_HEAD_0         0x55U /* 数据帧帧头第 1 字节 */
#define DM_IMU_FRAME_HEAD_1         0xAAU /* 数据帧帧头第 2 字节 */
#define DM_IMU_FRAME_TAIL           0x0AU /* 数据帧帧尾字节 */
#define DM_IMU_FRAME_SIZE            19U  /* 完整 IMU 数据帧字节数 */
#define DM_IMU_ID_OFFSET              2U  /* 设备 ID 在帧中的偏移 */
#define DM_IMU_REGISTER_OFFSET        3U  /* 寄存器号在帧中的偏移 */
#define DM_IMU_DATA_OFFSET            4U  /* 数据区在帧中的起始偏移 */
#define DM_IMU_CRC_OFFSET            16U  /* CRC 在帧中的起始偏移 */

#define DM_IMU_REGISTER_GYRO          0x02U /* 角速度寄存器号 */
#define DM_IMU_REGISTER_EULER         0x03U /* 欧拉角寄存器号 */
#define DM_IMU_DEG_TO_RAD              0.01745329251994329577f /* 角度转弧度比例 */

#define DM_IMU_GYRO_BIAS_CALIBRATION_MS 500U

typedef struct
{
    uint8_t device_id;
    volatile float yaw;        /* filtered relative Euler yaw, rad */
    volatile float raw_yaw;    /* raw Euler yaw, rad */
    volatile float yaw_offset; /* first valid Euler yaw after calibration, rad */
    volatile float gyro_z;     /* bias-corrected Z gyro, rad/s */
    float gyro_bias_sum;
    float gyro_bias;
    float yaw_kalman_estimate;
    float yaw_kalman_covariance;
    uint32_t gyro_bias_start_ms;
    uint16_t gyro_bias_sample_count;
    bool gyro_bias_ready;
    bool yaw_zeroed;
    bool yaw_kalman_valid;
    uint8_t rx_frame[DM_IMU_FRAME_SIZE];
    uint16_t rx_frame_len;
} dm_imu_t;

extern dm_imu_t dm_imu;

bool DM_IMU_Init(uint8_t device_id);

bool DM_IMU_ParseFeedback(const uint8_t *data, uint16_t len);

float DM_IMU_GetYaw(void);

bool DM_IMU_ZeroYaw(void);

bool DM_IMU_IsYawZeroed(void);

bool DM_IMU_IsGyroCalibrated(void);

float DM_IMU_GetGyroZ(void);

#endif /* DM_IMU_H */
