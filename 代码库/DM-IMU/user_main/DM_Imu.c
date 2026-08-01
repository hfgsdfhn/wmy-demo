#include "DM_Imu.h"

#include <string.h>

#define DM_IMU_FRAME_HEAD           0xA5U
#define DM_IMU_FRAME_TAIL           0x5AU
#define DM_IMU_FLOAT_SIZE            4U
#define DM_IMU_DATA_FLOAT_COUNT      3U
#define DM_IMU_DATA_SIZE             (DM_IMU_FLOAT_SIZE * DM_IMU_DATA_FLOAT_COUNT)
#define DM_IMU_FRAME_MIN_SIZE        (2U + DM_IMU_DATA_SIZE + 1U)
#define DM_IMU_FRAME_MAX_SIZE        20U

#define DM_IMU_FRAME_ACCEL           0x00U
#define DM_IMU_FRAME_GYRO            0x01U
#define DM_IMU_FRAME_EULER           0x02U
#define DM_IMU_FRAME_QUATERNION      0x03U

typedef struct
{
    uint8_t device_id;
    volatile float yaw;
    volatile float gyro_z;
} dm_imu_t;

static dm_imu_t dm_imu;

static uint32_t DM_IMU_GetBaudRate(dm_imu_baud_t baud)
{
    static const uint32_t baud_rates[] =
    {
        0U,
        115200U,
        230400U,
        460800U,
        500000U,
        921600U,
        1000000U
    };

    if ((baud < DM_IMU_BAUD_115200) || (baud > DM_IMU_BAUD_1000000))
    {
        return 0U;
    }

    return baud_rates[baud];
}

static float DM_IMU_DecodeFloat(const uint8_t *data)
{
    float value;

    memcpy(&value, data, sizeof(value));
    return value;
}

static bool DM_IMU_GetFrameInfo(const uint8_t *frame, uint16_t frame_len,
                                uint8_t *frame_id, uint16_t *payload_offset)
{
    if ((frame_len < DM_IMU_FRAME_MIN_SIZE) || (frame[0] != DM_IMU_FRAME_HEAD)
        || (frame[frame_len - 1U] != DM_IMU_FRAME_TAIL))
    {
        return false;
    }

    if ((frame_len >= (DM_IMU_FRAME_MIN_SIZE + 1U))
        && (frame[1] == dm_imu.device_id)
        && (frame[2] <= DM_IMU_FRAME_QUATERNION))
    {
        *frame_id = frame[2];
        *payload_offset = 3U;
        return (uint16_t)(frame_len - *payload_offset - 1U) >= DM_IMU_DATA_SIZE;
    }

    if (frame[1] <= DM_IMU_FRAME_QUATERNION)
    {
        *frame_id = frame[1];
        *payload_offset = 2U;
        return (uint16_t)(frame_len - *payload_offset - 1U) >= DM_IMU_DATA_SIZE;
    }

    return false;
}

static void DM_IMU_ParseFrame(const uint8_t *frame, uint16_t frame_len)
{
    uint8_t frame_id;
    uint16_t payload_offset;

    if (!DM_IMU_GetFrameInfo(frame, frame_len, &frame_id, &payload_offset))
    {
        return;
    }

    if (frame_id == DM_IMU_FRAME_GYRO)
    {
        dm_imu.gyro_z = DM_IMU_DecodeFloat(&frame[payload_offset + 8U]);
    }
    else if (frame_id == DM_IMU_FRAME_EULER)
    {
        dm_imu.yaw = DM_IMU_DecodeFloat(&frame[payload_offset + 4U]);
    }
}

static void DM_IMU_RxCallback(bsp_uart_t *uart, const uint8_t *data,
                              uint16_t len)
{
    uint16_t start;
    uint16_t end;

    (void)uart;
    if (data == NULL)
    {
        return;
    }

    start = 0U;
    while (start < len)
    {
        while ((start < len) && (data[start] != DM_IMU_FRAME_HEAD))
        {
            start++;
        }

        if (start >= len)
        {
            break;
        }

        end = (uint16_t)(start + 1U);
        while ((end < len) && (data[end] != DM_IMU_FRAME_TAIL)
               && ((uint16_t)(end - start) < DM_IMU_FRAME_MAX_SIZE))
        {
            end++;
        }

        if ((end < len) && (data[end] == DM_IMU_FRAME_TAIL))
        {
            DM_IMU_ParseFrame(&data[start], (uint16_t)(end - start + 1U));
            start = (uint16_t)(end + 1U);
        }
        else
        {
            start++;
        }
    }
}

bool DM_IMU_Init(bsp_uart_t *uart, uint8_t device_id, dm_imu_baud_t baud)
{
    uint32_t baud_rate;

    if ((uart == NULL) || !uart->initialized || (uart->huart == NULL)
        || (uart->rx_buf == NULL) || (uart->rx_size == 0U))
    {
        return false;
    }

    baud_rate = DM_IMU_GetBaudRate(baud);
    if (baud_rate == 0U)
    {
        return false;
    }

    if (HAL_UART_AbortReceive(uart->huart) != HAL_OK)
    {
        return false;
    }

    uart->huart->Init.BaudRate = baud_rate;
    if (HAL_UART_Init(uart->huart) != HAL_OK)
    {
        return false;
    }

    memset(&dm_imu, 0, sizeof(dm_imu));
    dm_imu.device_id = device_id;
    uart->rx_callback = DM_IMU_RxCallback;
    return HAL_UARTEx_ReceiveToIdle_DMA(uart->huart, uart->rx_buf,
                                        uart->rx_size) == HAL_OK;
}

float DM_IMU_GetYaw(void)
{
    return dm_imu.yaw;
}

float DM_IMU_GetGyroZ(void)
{
    return dm_imu.gyro_z;
}
