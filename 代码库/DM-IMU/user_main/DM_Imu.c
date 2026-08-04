#include "DM_Imu.h"
#include <string.h>

dm_imu_t dm_imu;

static float DM_IMU_DecodeFloat(const uint8_t *data)
{
    float value;

    memcpy(&value, data, sizeof(value));
    return value;
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
        dm_imu.gyro_z = DM_IMU_DecodeFloat(&frame[DM_IMU_DATA_OFFSET + 8U]);
        return false;
    }
    else if (frame[DM_IMU_REGISTER_OFFSET] == DM_IMU_REGISTER_EULER)
    {
        dm_imu.yaw = DM_IMU_DecodeFloat(&frame[DM_IMU_DATA_OFFSET + 8U]);
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
