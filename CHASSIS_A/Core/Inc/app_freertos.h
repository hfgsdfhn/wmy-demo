#ifndef APP_FREERTOS_H
#define APP_FREERTOS_H

#include <stdbool.h>
#include <stdint.h>

#include "cmsis_os2.h"

#define APP_CAN_DATA_MAX_SIZE             8U
#define APP_COMM_PAYLOAD_MAX_SIZE         64U
#define APP_TRAJECTORY_QUEUE_LENGTH       32U

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

typedef enum
{
    APP_CAN_BUS_1 = 0,
    APP_CAN_BUS_2,
    APP_CAN_BUS_3
} app_can_bus_t;

typedef enum
{
    APP_COMM_SOURCE_DT35 = 0,
    APP_COMM_SOURCE_MAIN_CONTROLLER,
    APP_COMM_SOURCE_USB,
    APP_COMM_SOURCE_DM_IMU
} app_comm_source_t;

typedef enum
{
    APP_FREERTOS_FAULT_NONE = 0,
    APP_FREERTOS_FAULT_OBJECT_CREATE,
    APP_FREERTOS_FAULT_STACK_OVERFLOW,
    APP_FREERTOS_FAULT_MALLOC_FAILED,
    APP_FREERTOS_FAULT_ASSERT
} app_freertos_fault_t;

typedef struct
{
    uint32_t timestamp_ms;
    uint32_t identifier;
    app_can_bus_t bus;
    uint8_t is_extended_id;
    uint8_t length;
    uint8_t data[APP_CAN_DATA_MAX_SIZE];
} app_can_frame_t;

typedef struct
{
    uint32_t sequence;
    uint32_t timestamp_ms;
    float velocity_x_mps;
    float velocity_y_mps;
    float yaw_rate_radps;
    uint8_t valid;
} app_chassis_command_t;

typedef struct
{
    uint32_t sequence;
    uint32_t timestamp_ms;
    float rod_angle_rad[4];
    float rear_wheel_speed_radps[2];
    uint32_t control_flags;
    uint8_t valid;
} app_mechanism_command_t;

typedef struct
{
    uint32_t sequence;
    float position_x_m;
    float position_y_m;
    float yaw_rad;
    float target_speed_mps;
    uint32_t flags;
} app_trajectory_point_t;

typedef struct
{
    uint32_t timestamp_ms;
    app_comm_source_t source;
    uint16_t length;
    uint8_t data[APP_COMM_PAYLOAD_MAX_SIZE];
} app_comm_packet_t;

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
    uint32_t control_cycle_count;
    uint32_t control_overrun_count;
    uint32_t can_error_mask;
} app_runtime_stats_t;

extern osThreadId_t appCanRxTaskHandle;
extern osThreadId_t appControlTaskHandle;
extern osThreadId_t appNavigationTaskHandle;
extern osThreadId_t appCommunicationTaskHandle;
extern osThreadId_t appSafetyTaskHandle;
extern osThreadId_t appMonitorTaskHandle;

extern osMessageQueueId_t appCanRxQueueHandle;
extern osMessageQueueId_t appChassisCommandQueueHandle;
extern osMessageQueueId_t appMechanismCommandQueueHandle;
extern osMessageQueueId_t appTrajectoryQueueHandle;
extern osMessageQueueId_t appCommunicationQueueHandle;
extern osMessageQueueId_t appTelemetryQueueHandle;
extern osEventFlagsId_t appSystemEventHandle;
extern osMutexId_t appRobotStateMutexHandle;

extern volatile app_freertos_fault_t appFreertosFaultCode;
extern volatile uint32_t appFreertosFaultLine;
extern const char * volatile appFreertosFaultFile;

bool AppFreertos_Init(void);
void AppFreertos_Bootstrap(void);

bool AppFreertos_SubmitChassisCommand(const app_chassis_command_t *command,
                                      uint32_t timeout_ms);
bool AppFreertos_SubmitMechanismCommand(
    const app_mechanism_command_t *command, uint32_t timeout_ms);
bool AppFreertos_SubmitTrajectoryPoint(const app_trajectory_point_t *point,
                                       uint32_t timeout_ms);
bool AppFreertos_SubmitCommunicationPacket(const app_comm_packet_t *packet,
                                            uint32_t timeout_ms);
bool AppFreertos_PublishRobotState(const app_robot_state_t *state,
                                   uint32_t timeout_ms);
bool AppFreertos_GetRobotState(app_robot_state_t *state,
                               uint32_t timeout_ms);
void AppFreertos_GetRuntimeStats(app_runtime_stats_t *stats);

void AppFreertos_NotifyCanErrorFromISR(app_can_bus_t bus);
void App_FreertosAssert(const char *file, uint32_t line);
void AppFreertos_Panic(app_freertos_fault_t fault);

void App_OnCanFrame(const app_can_frame_t *frame);
void App_ControlStep(const app_chassis_command_t *chassis_command,
                     const app_mechanism_command_t *mechanism_command,
                     uint32_t system_events);
void App_NavigationStep(const app_trajectory_point_t *point,
                        bool point_available);
void App_OnCommunicationPacket(const app_comm_packet_t *packet);
void App_SafetyStep(uint32_t system_events);
void App_MonitorStep(const app_runtime_stats_t *stats);

#endif /* APP_FREERTOS_H */
