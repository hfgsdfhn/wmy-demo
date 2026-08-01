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

static dm_imu_t dm_imu;

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

    if (frame[DM_IMU_REGISTER_OFFSET] == DM_IMU_REGISTER_GYRO)
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
