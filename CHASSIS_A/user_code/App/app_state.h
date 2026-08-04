#ifndef APP_STATE_H
#define APP_STATE_H

#include <stdint.h>

#define APP_EVENT_INIT_DONE               (1UL << 0)
#define APP_EVENT_INIT_FAILED             (1UL << 1)
#define APP_EVENT_ESTOP                   (1UL << 2)
#define APP_EVENT_MOTOR_FAULT             (1UL << 3)
#define APP_EVENT_CAN1_READY              (1UL << 4)
#define APP_EVENT_CAN2_READY              (1UL << 5)
#define APP_EVENT_CAN3_READY              (1UL << 6)
#define APP_EVENT_DT35_ONLINE             (1UL << 7)
#define APP_EVENT_MAIN_CTRL_ONLINE        (1UL << 8)
#define APP_EVENT_USB_ONLINE              (1UL << 9)
#define APP_EVENT_IMU_ONLINE              (1UL << 10)
#define APP_EVENT_CAN_FAULT               (1UL << 11)
#define APP_EVENT_ALL_CAN_READY           (APP_EVENT_CAN1_READY | \
                                           APP_EVENT_CAN2_READY | \
                                           APP_EVENT_CAN3_READY)

typedef struct
{
    uint32_t timestamp_ms;
    float position_x_m;
    float position_y_m;
    float yaw_rad;
    float velocity_x_mps;
    float velocity_y_mps;
    float yaw_rate_radps;
    float dt35_distance_m[2];
    float rod_angle_rad[4];
    float rear_wheel_speed_radps[2];
    uint32_t status_flags;
} app_robot_state_t;

typedef struct
{
    uint32_t timestamp_ms;
    uint32_t system_events;
    uint32_t can_rx_count;
    uint32_t can_rx_dropped;
    uint32_t comm_rx_count;
    uint32_t comm_rx_dropped;
    uint32_t imu_rx_count;
    uint32_t imu_rx_dropped;
    uint32_t dt35_rx_count;
    uint32_t dt35_rx_dropped;
    uint32_t control_cycle_count;
    uint32_t control_overrun_count;
    uint32_t can_error_mask;
} app_runtime_stats_t;

#endif /* APP_STATE_H */
