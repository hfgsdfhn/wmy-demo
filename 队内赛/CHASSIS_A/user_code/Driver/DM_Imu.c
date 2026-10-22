#include "DM_Imu.h"
#include <math.h>
#include <string.h>

#include "main.h"

#define DM_IMU_PI                       3.14159265358979323846f /* 圆周率 */
#define DM_IMU_TWO_PI                   (2.0f * DM_IMU_PI) /* 一周对应的弧度 */
#define DM_IMU_YAW_KALMAN_Q_RAD2        0.00000609234839573f /* YAW 卡尔曼过程噪声方差，单位 rad^2 */
#define DM_IMU_YAW_KALMAN_R_RAD2        0.00091385225938565f /* YAW 卡尔曼测量噪声方差，单位 rad^2 */

dm_imu_t dm_imu;

static float DM_IMU_DecodeFloat(const uint8_t *data)
{
    float value;

    memcpy(&value, data, sizeof(value));
    return value;
}

static float DM_IMU_NormalizeAngle(float angle_rad)
{
    while (angle_rad >= DM_IMU_PI) angle_rad -= DM_IMU_TWO_PI;
    while (angle_rad < -DM_IMU_PI) angle_rad += DM_IMU_TWO_PI;
    return angle_rad;
}

static float DM_IMU_ApplyYawKalman(float measured_yaw_rad)
{
    float innovation;
    float gain;

    if (!dm_imu.yaw_kalman_valid)
    {
        dm_imu.yaw_kalman_estimate = DM_IMU_NormalizeAngle(measured_yaw_rad);
        dm_imu.yaw_kalman_covariance = 1.0f;
        dm_imu.yaw_kalman_valid = true;
        return dm_imu.yaw_kalman_estimate;
    }

    dm_imu.yaw_kalman_covariance += DM_IMU_YAW_KALMAN_Q_RAD2;
    innovation = DM_IMU_NormalizeAngle(measured_yaw_rad - dm_imu.yaw_kalman_estimate);
    gain = dm_imu.yaw_kalman_covariance
         / (dm_imu.yaw_kalman_covariance + DM_IMU_YAW_KALMAN_R_RAD2);
    dm_imu.yaw_kalman_estimate = DM_IMU_NormalizeAngle(
        dm_imu.yaw_kalman_estimate + gain * innovation);
    dm_imu.yaw_kalman_covariance *= (1.0f - gain);
    return dm_imu.yaw_kalman_estimate;
}

static void DM_IMU_ProcessGyroZ(float gyro_z_radps)
{
    uint32_t now = HAL_GetTick();

    if (!dm_imu.gyro_bias_ready)
    {
        dm_imu.gyro_bias_sum += gyro_z_radps;
        dm_imu.gyro_bias_sample_count++;
        if (((uint32_t)(now - dm_imu.gyro_bias_start_ms)
             >= DM_IMU_GYRO_BIAS_CALIBRATION_MS)
            && (dm_imu.gyro_bias_sample_count > 0U))
        {
            dm_imu.gyro_bias = dm_imu.gyro_bias_sum
                              / (float)dm_imu.gyro_bias_sample_count;
            dm_imu.gyro_bias_ready = true;
        }
        dm_imu.gyro_z = 0.0f;
        return;
    }

    dm_imu.gyro_z = gyro_z_radps - dm_imu.gyro_bias;
}

static void DM_IMU_ProcessEulerYaw(float yaw_deg)
{
    float raw_yaw_rad = DM_IMU_NormalizeAngle(yaw_deg * DM_IMU_DEG_TO_RAD);

    dm_imu.raw_yaw = raw_yaw_rad;
    if (!dm_imu.gyro_bias_ready)
    {
        return;
    }

    if (!dm_imu.yaw_zeroed)
    {
        dm_imu.yaw_offset = raw_yaw_rad;
        dm_imu.yaw = 0.0f;
        dm_imu.yaw_kalman_estimate = 0.0f;
        dm_imu.yaw_kalman_covariance = 1.0f;
        dm_imu.yaw_kalman_valid = true;
        dm_imu.yaw_zeroed = true;
        return;
    }

    dm_imu.yaw = DM_IMU_ApplyYawKalman(
        DM_IMU_NormalizeAngle(raw_yaw_rad - dm_imu.yaw_offset));
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

static bool DM_IMU_ParseFrame(const uint8_t *frame)
{
    uint16_t received_crc;

    if ((frame[0] != DM_IMU_FRAME_HEAD_0) || (frame[1] != DM_IMU_FRAME_HEAD_1)
        || (frame[DM_IMU_ID_OFFSET] != dm_imu.device_id)
        || (frame[DM_IMU_FRAME_SIZE - 1U] != DM_IMU_FRAME_TAIL))
    {
        return false;
    }

    received_crc = (uint16_t)frame[DM_IMU_CRC_OFFSET]
                   | ((uint16_t)frame[DM_IMU_CRC_OFFSET + 1U] << 8U);
    if (DM_IMU_CalculateCrc16(frame, DM_IMU_CRC_OFFSET) != received_crc)
    {
        return false;
    }

    if (frame[DM_IMU_REGISTER_OFFSET] == DM_IMU_REGISTER_GYRO)
    {
        /* DM-IMU gyro register is already in rad/s. */
        float gyro_z_radps = DM_IMU_DecodeFloat(&frame[DM_IMU_DATA_OFFSET + 8U]);

        if (!isfinite(gyro_z_radps) || (fabsf(gyro_z_radps) > 35.0f))
        {
            return false;
        }
        DM_IMU_ProcessGyroZ(gyro_z_radps);
        return false;
    }
    else if (frame[DM_IMU_REGISTER_OFFSET] == DM_IMU_REGISTER_EULER)
    {
        float yaw_deg = DM_IMU_DecodeFloat(&frame[DM_IMU_DATA_OFFSET + 8U]);

        if (!isfinite(yaw_deg) || (fabsf(yaw_deg) > 360.0f))
        {
            return false;
        }
        DM_IMU_ProcessEulerYaw(yaw_deg);
        return true;
    }

    return false;
}
bool DM_IMU_ParseFeedback(const uint8_t *data, uint16_t len)
{
    uint16_t index;
    bool euler_updated = false;

    if (data == NULL)
    {
        return false;
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
            dm_imu.rx_frame_len = (data[index] == DM_IMU_FRAME_HEAD_0) ? 1U : 0U;
            continue;
        }

        dm_imu.rx_frame[dm_imu.rx_frame_len] = data[index];
        dm_imu.rx_frame_len++;

        if (dm_imu.rx_frame_len == DM_IMU_FRAME_SIZE)
        {
            if (DM_IMU_ParseFrame(dm_imu.rx_frame))
            {
                euler_updated = true;
            }
            dm_imu.rx_frame_len = 0U;
        }
    }

    return euler_updated;
}

bool DM_IMU_Init(uint8_t device_id)
{
    memset(&dm_imu, 0, sizeof(dm_imu));
    dm_imu.device_id = device_id;
    dm_imu.gyro_bias_start_ms = HAL_GetTick();
    return true;
}

float DM_IMU_GetYaw(void)
{
    return dm_imu.yaw_zeroed ? dm_imu.yaw : 0.0f;
}

bool DM_IMU_ZeroYaw(void)
{
    if (!dm_imu.yaw_zeroed) return false;
    dm_imu.yaw_offset = dm_imu.raw_yaw;
    dm_imu.yaw = 0.0f;
    dm_imu.yaw_kalman_estimate = 0.0f;
    dm_imu.yaw_kalman_covariance = 1.0f;
    dm_imu.yaw_kalman_valid = true;
    return true;
}

bool DM_IMU_IsYawZeroed(void)
{
    return dm_imu.yaw_zeroed;
}

bool DM_IMU_IsGyroCalibrated(void)
{
    return dm_imu.gyro_bias_ready;
}

float DM_IMU_GetGyroZ(void)
{
    return dm_imu.gyro_bias_ready ? dm_imu.gyro_z : 0.0f;
}
