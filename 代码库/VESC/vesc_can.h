#ifndef VESC_CAN_H
#define VESC_CAN_H

#include <stdbool.h>
#include <stdint.h>

#include "bsp_can.h"

/* VESC CAN extended-id command values used by this driver. */
#define VESC_CAN_PACKET_SET_RPM  3U
#define VESC_CAN_PACKET_SET_POS  4U

typedef enum
{
    VESC_CAN_SENSORLESS = 0U,
    VESC_CAN_SENSORED = 1U
} vesc_can_sensor_mode_t;

typedef struct
{
    bsp_can_t *can;
    uint8_t controller_id;
    vesc_can_sensor_mode_t sensor_mode;
    bool initialized;
} vesc_can_t;

/**
 * @brief Bind a VESC node to an initialized BSP CAN instance.
 *
 * Sensor mode is retained as application configuration. The VESC FOC sensor
 * mode itself must be configured and stored in the VESC, since standard VESC
 * real-time CAN packets do not switch it.
 */
bool VescCan_Init(vesc_can_t *vesc, bsp_can_t *can, uint8_t controller_id,
                  vesc_can_sensor_mode_t sensor_mode);

/** @brief Send the VESC closed-loop speed target in electrical RPM. */
bool VescCan_SetSpeedRpm(vesc_can_t *vesc, int32_t rpm);

/** @brief Send the VESC closed-loop position target in mechanical degrees. */
bool VescCan_SetPositionDeg(vesc_can_t *vesc, float position_deg);

/** @brief Return the configured motor feedback mode. */
vesc_can_sensor_mode_t VescCan_GetSensorMode(const vesc_can_t *vesc);

#endif
