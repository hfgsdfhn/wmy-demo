#include "DM_Imu.h"

#include <string.h>

#define DM_IMU_FRAME_HEAD_0         0x55U
#define DM_IMU_FRAME_HEAD_1         0xAAU
#define DM_IMU_FRAME_TAIL           0x0AU
#define DM_IMU_FRAME_SIZE            19U
#define DM_IMU_ID_OFFSET              2U
#define DM_IMU_REGISTER_OFFSET        3U
#define DM_IMU_DATA_OFFSET            4U
#define DM_IMU_CRC_OFFSET            16U

#define DM_IMU_REGISTER_ACCEL         0x01U
#define DM_IMU_REGISTER_GYRO          0x02U
#define DM_IMU_REGISTER_EULER         0x03U

#define DM_IMU_KALMAN_PROCESS_NOISE   0.01f
#define DM_IMU_KALMAN_MEASURE_NOISE   0.10f
#define DM_IMU_MS_TO_S                 0.001f

typedef struct
{
    float estimate;
    float covariance;
} dm_imu_kalman_t;

typedef struct
{
    uint8_t device_id;
    volatile float yaw;
    volatile float gyro_z;
    volatile float accel_x;
    volatile float accel_y;
    volatile float velocity_x;
    volatile float velocity_y;
    volatile float position_x;
    volatile float position_y;
    float previous_accel_x;
    float previous_accel_y;
    uint32_t previous_accel_ms;
    bool accel_initialized;
    dm_imu_kalman_t accel_x_filter;
    dm_imu_kalman_t accel_y_filter;
    uint8_t rx_frame[DM_IMU_FRAME_SIZE];
    uint16_t rx_frame_len;
} dm_imu_t;

static dm_imu_t dm_imu;

static float DM_IMU_DecodeFloat(const uint8_t *data)
{
    float value;

    memcpy(&value, data, sizeof(value));
    return value;
}

static float DM_IMU_KalmanUpdate(dm_imu_kalman_t *filter, float measurement)
{
    float gain;

    filter->covariance += DM_IMU_KALMAN_PROCESS_NOISE;
    gain = filter->covariance / (filter->covariance + DM_IMU_KALMAN_MEASURE_NOISE);
    filter->estimate += gain * (measurement - filter->estimate);
    filter->covariance *= (1.0f - gain);
    return filter->estimate;
}

static void DM_IMU_UpdateAcceleration(float raw_accel_x, float raw_accel_y)
{
    uint32_t now_ms = HAL_GetTick();
    float filtered_accel_x;
    float filtered_accel_y;
    float dt;
    float previous_velocity_x;
    float previous_velocity_y;

    if (!dm_imu.accel_initialized)
    {
        dm_imu.accel_x_filter.estimate = raw_accel_x;
        dm_imu.accel_x_filter.covariance = 1.0f;
        dm_imu.accel_y_filter.estimate = raw_accel_y;
        dm_imu.accel_y_filter.covariance = 1.0f;
        dm_imu.accel_x = raw_accel_x;
        dm_imu.accel_y = raw_accel_y;
        dm_imu.previous_accel_x = raw_accel_x;
        dm_imu.previous_accel_y = raw_accel_y;
        dm_imu.previous_accel_ms = now_ms;
        dm_imu.accel_initialized = true;
        return;
    }

    filtered_accel_x = DM_IMU_KalmanUpdate(&dm_imu.accel_x_filter, raw_accel_x);
    filtered_accel_y = DM_IMU_KalmanUpdate(&dm_imu.accel_y_filter, raw_accel_y);
    dt = (float)(now_ms - dm_imu.previous_accel_ms) * DM_IMU_MS_TO_S;

    previous_velocity_x = dm_imu.velocity_x;
    previous_velocity_y = dm_imu.velocity_y;
    dm_imu.velocity_x += 0.5f * (dm_imu.previous_accel_x + filtered_accel_x) * dt;
    dm_imu.velocity_y += 0.5f * (dm_imu.previous_accel_y + filtered_accel_y) * dt;
    dm_imu.position_x += 0.5f * (previous_velocity_x + dm_imu.velocity_x) * dt;
    dm_imu.position_y += 0.5f * (previous_velocity_y + dm_imu.velocity_y) * dt;

    dm_imu.accel_x = filtered_accel_x;
    dm_imu.accel_y = filtered_accel_y;
    dm_imu.previous_accel_x = filtered_accel_x;
    dm_imu.previous_accel_y = filtered_accel_y;
    dm_imu.previous_accel_ms = now_ms;
}

static uint16_t DM_IMU_CalculateCrcTableEntry(uint8_t value)
{
    uint16_t crc = (uint16_t)value << 8U;
    uint8_t bit;

    for (bit = 0U; bit < 8U; bit++)
    {
        crc = ((crc & 0x8000U) != 0U) ? (uint16_t)((crc << 1U) ^ 0x1021U)
                                        : (uint16_t)(crc << 1U);
    }

    return crc;
}

static uint16_t DM_IMU_CalculateCrc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFU;
    uint16_t index;
    uint8_t table_index;

    for (index = 0U; index < len; index++)
    {
        table_index = (uint8_t)((crc >> 8U) ^ data[index]);
        crc = (uint16_t)((crc << 1U)
                         ^ DM_IMU_CalculateCrcTableEntry(table_index));
    }

    return crc;
}

static void DM_IMU_ParseFrame(const uint8_t *frame)
{
    uint16_t received_crc;

    if ((frame[0] != DM_IMU_FRAME_HEAD_0) || (frame[1] != DM_IMU_FRAME_HEAD_1)
        || (frame[DM_IMU_ID_OFFSET] != dm_imu.device_id)
        || (frame[DM_IMU_FRAME_SIZE - 1U] != DM_IMU_FRAME_TAIL))
    {
        return;
    }

    received_crc = (uint16_t)frame[DM_IMU_CRC_OFFSET]
                   | ((uint16_t)frame[DM_IMU_CRC_OFFSET + 1U] << 8U);
    if (DM_IMU_CalculateCrc16(frame, DM_IMU_CRC_OFFSET) != received_crc)
    {
        return;
    }

    if (frame[DM_IMU_REGISTER_OFFSET] == DM_IMU_REGISTER_ACCEL)
    {
        DM_IMU_UpdateAcceleration(DM_IMU_DecodeFloat(&frame[DM_IMU_DATA_OFFSET]),
                                  DM_IMU_DecodeFloat(&frame[DM_IMU_DATA_OFFSET + 4U]));
    }
    else if (frame[DM_IMU_REGISTER_OFFSET] == DM_IMU_REGISTER_GYRO)
    {
        dm_imu.gyro_z = DM_IMU_DecodeFloat(&frame[DM_IMU_DATA_OFFSET + 8U]);
    }
    else if (frame[DM_IMU_REGISTER_OFFSET] == DM_IMU_REGISTER_EULER)
    {
        dm_imu.yaw = DM_IMU_DecodeFloat(&frame[DM_IMU_DATA_OFFSET + 8U]);
    }
}

void DM_IMU_RxCallback(bsp_uart_t *uart, const uint8_t *data, uint16_t len)
{
    uint16_t index;

    (void)uart;
    if (data == NULL)
    {
        return;
    }

    for (index = 0U; index < len; index++)
    {
        if (dm_imu.rx_frame_len == 0U)
        {
            if (data[index] == DM_IMU_FRAME_HEAD_0)
            {
                dm_imu.rx_frame[0] = data[index];
                dm_imu.rx_frame_len = 1U;
            } 
            continue;
        }

        if ((dm_imu.rx_frame_len == 1U) && (data[index] != DM_IMU_FRAME_HEAD_1))
        {
            dm_imu.rx_frame_len = 0U;
            continue;
        }

        dm_imu.rx_frame[dm_imu.rx_frame_len] = data[index];
        dm_imu.rx_frame_len++;

        if (dm_imu.rx_frame_len == DM_IMU_FRAME_SIZE)
        {
            DM_IMU_ParseFrame(dm_imu.rx_frame);
            dm_imu.rx_frame_len = 0U;
        }
    }
}

bool DM_IMU_Init(uint8_t device_id)
{
    memset(&dm_imu, 0, sizeof(dm_imu));
    dm_imu.device_id = device_id;
    return true;
}

float DM_IMU_GetYaw(void)
{
    return dm_imu.yaw;
}

float DM_IMU_GetGyroZ(void)
{
    return dm_imu.gyro_z;
}

float DM_IMU_GetAccelX(void)
{
    return dm_imu.accel_x;
}

float DM_IMU_GetAccelY(void)
{
    return dm_imu.accel_y;
}

float DM_IMU_GetVelocityX(void)
{
    return dm_imu.velocity_x;
}

float DM_IMU_GetVelocityY(void)
{
    return dm_imu.velocity_y;
}

float DM_IMU_GetPositionX(void)
{
    return dm_imu.position_x;
}

float DM_IMU_GetPositionY(void)
{
    return dm_imu.position_y;
}
