#include "vesc_can.h"

#define VESC_CAN_NODE_ID_MASK       0xFFU
#define VESC_CAN_POSITION_SCALE     1000000.0f
#define VESC_CAN_POSITION_MAX_DEG   2147.4835f
#define VESC_CAN_POSITION_MIN_DEG  -2147.4836f

static uint32_t VescCan_MakeExtendedId(uint8_t command, uint8_t controller_id)
{
    return ((uint32_t)command << 8U) | (uint32_t)controller_id;
}

static void VescCan_PutInt32Be(uint8_t data[4], int32_t value)
{
    uint32_t raw = (uint32_t)value;

    data[0] = (uint8_t)(raw >> 24U);
    data[1] = (uint8_t)(raw >> 16U);
    data[2] = (uint8_t)(raw >> 8U);
    data[3] = (uint8_t)raw;
}

static bool VescCan_IsReady(const vesc_can_t *vesc)
{
    return (vesc != 0) && vesc->initialized && (vesc->can != 0)
        && vesc->can->initialized;
}

bool VescCan_Init(vesc_can_t *vesc, bsp_can_t *can, uint8_t controller_id,
                  vesc_can_sensor_mode_t sensor_mode)
{
    if ((vesc == 0) || (can == 0) || !can->initialized
        || (sensor_mode != VESC_CAN_SENSORLESS
            && sensor_mode != VESC_CAN_SENSORED))
    {
        return false;
    }

    vesc->can = can;
    vesc->controller_id = controller_id & VESC_CAN_NODE_ID_MASK;
    vesc->sensor_mode = sensor_mode;
    vesc->initialized = true;
    return true;
}

bool VescCan_SetSpeedRpm(vesc_can_t *vesc, int32_t rpm)
{
    uint8_t data[4];

    if (!VescCan_IsReady(vesc))
    {
        return false;
    }

    VescCan_PutInt32Be(data, rpm);
    return BspCan_SendExtended(vesc->can,
                               VescCan_MakeExtendedId(VESC_CAN_PACKET_SET_RPM,
                                                      vesc->controller_id),
                               data, sizeof(data));
}

bool VescCan_SetPositionDeg(vesc_can_t *vesc, float position_deg)
{
    uint8_t data[4];
    int32_t position_raw;

    if (!VescCan_IsReady(vesc) || (position_deg != position_deg)
        || (position_deg < VESC_CAN_POSITION_MIN_DEG)
        || (position_deg > VESC_CAN_POSITION_MAX_DEG))
    {
        return false;
    }

    position_raw = (int32_t)(position_deg * VESC_CAN_POSITION_SCALE);
    VescCan_PutInt32Be(data, position_raw);
    return BspCan_SendExtended(vesc->can,
                               VescCan_MakeExtendedId(VESC_CAN_PACKET_SET_POS,
                                                      vesc->controller_id),
                               data, sizeof(data));
}

vesc_can_sensor_mode_t VescCan_GetSensorMode(const vesc_can_t *vesc)
{
    return (vesc == 0) ? VESC_CAN_SENSORLESS : vesc->sensor_mode;
}
