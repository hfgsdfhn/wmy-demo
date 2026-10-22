#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "DM_Imu.h"

static uint32_t test_tick_ms;

uint32_t HAL_GetTick(void)
{
    return test_tick_ms;
}

static uint16_t crc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;
    uint16_t index;
    uint8_t bit;

    for (index = 0U; index < length; ++index) {
        uint8_t table_index = (uint8_t)((crc >> 8U) ^ data[index]);
        uint16_t table_entry = (uint16_t)table_index << 8U;

        for (bit = 0U; bit < 8U; ++bit) {
            table_entry = (table_entry & 0x8000U)
                ? (uint16_t)((table_entry << 1U) ^ 0x1021U)
                : (uint16_t)(table_entry << 1U);
        }
        crc = (uint16_t)((crc << 1U) ^ table_entry);
    }
    return crc;
}

static void feed_frame(uint8_t register_id, float z_value, int valid_crc)
{
    uint8_t frame[DM_IMU_FRAME_SIZE] = {0};
    uint16_t checksum;

    frame[0] = DM_IMU_FRAME_HEAD_0;
    frame[1] = DM_IMU_FRAME_HEAD_1;
    frame[2] = 1U;
    frame[3] = register_id;
    memcpy(&frame[DM_IMU_DATA_OFFSET + 8U], &z_value, sizeof(z_value));
    checksum = crc16(frame, DM_IMU_CRC_OFFSET);
    if (!valid_crc) checksum ^= 0xFFFFU;
    frame[DM_IMU_CRC_OFFSET] = (uint8_t)checksum;
    frame[DM_IMU_CRC_OFFSET + 1U] = (uint8_t)(checksum >> 8U);
    frame[DM_IMU_FRAME_SIZE - 1U] = DM_IMU_FRAME_TAIL;
    (void)DM_IMU_ParseFeedback(frame, sizeof(frame));
}

int main(void)
{
    float yaw_before_wrap;
    float yaw_after_wrap;
    float yaw_after_zero;

    assert(DM_IMU_Init(1U));
    feed_frame(DM_IMU_REGISTER_EULER, 45.0f, 1);
    assert(!DM_IMU_IsGyroCalibrated());
    assert(!DM_IMU_IsYawZeroed());
    assert(fabsf(DM_IMU_GetYaw()) < 0.000001f);
    assert(fabsf(DM_IMU_GetGyroZ()) < 0.000001f);

    feed_frame(DM_IMU_REGISTER_GYRO, 5.0f, 1);
    test_tick_ms = DM_IMU_GYRO_BIAS_CALIBRATION_MS;
    feed_frame(DM_IMU_REGISTER_GYRO, 5.0f, 1);
    assert(DM_IMU_IsGyroCalibrated());
    assert(fabsf(DM_IMU_GetGyroZ()) < 0.000001f);
    feed_frame(DM_IMU_REGISTER_GYRO, 5.0f, 1);
    assert(fabsf(DM_IMU_GetGyroZ()) < 0.000001f);
    feed_frame(DM_IMU_REGISTER_GYRO, 6.0f, 1);
    assert(fabsf(DM_IMU_GetGyroZ() - 1.0f) < 0.000001f);

    feed_frame(DM_IMU_REGISTER_EULER, 170.0f, 1);
    assert(DM_IMU_IsYawZeroed());
    assert(fabsf(DM_IMU_GetYaw()) < 0.000001f);
    feed_frame(DM_IMU_REGISTER_EULER, 179.0f, 1);
    yaw_before_wrap = DM_IMU_GetYaw();
    feed_frame(DM_IMU_REGISTER_EULER, -179.0f, 1);
    yaw_after_wrap = DM_IMU_GetYaw();
    assert(yaw_before_wrap > 0.0f);
    assert(yaw_after_wrap > yaw_before_wrap);
    assert(fabsf(yaw_after_wrap - yaw_before_wrap) < 0.2f);

    assert(DM_IMU_ZeroYaw());
    assert(fabsf(DM_IMU_GetYaw()) < 0.000001f);
    feed_frame(DM_IMU_REGISTER_EULER, -170.0f, 1);
    assert(DM_IMU_GetYaw() > 0.0f);
    yaw_after_zero = DM_IMU_GetYaw();

    feed_frame(DM_IMU_REGISTER_EULER, NAN, 1);
    assert(fabsf(DM_IMU_GetYaw() - yaw_after_zero) < 0.000001f);
    feed_frame(DM_IMU_REGISTER_EULER, INFINITY, 1);
    assert(fabsf(DM_IMU_GetYaw() - yaw_after_zero) < 0.000001f);
    feed_frame(DM_IMU_REGISTER_EULER, 90.0f, 0);
    assert(fabsf(DM_IMU_GetYaw() - yaw_after_zero) < 0.000001f);

    puts("dm_imu_test passed");
    return 0;
}
