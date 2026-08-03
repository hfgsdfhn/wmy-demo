#include "app_rtos_internal.h"

#include <stddef.h>
#include <string.h>

#include "FreeRTOS.h"
#include "app_init.h"
#include "bsp_can.h"
#include "cmsis_compiler.h"
#include "control_config.h"
#include "task.h"

#define APP_CAN_RX_QUEUE_LENGTH           64U
#define APP_COMMAND_QUEUE_LENGTH           4U
#define APP_ACTION_QUEUE_LENGTH            8U
#define APP_IMU_QUEUE_LENGTH               4U
#define APP_DT35_QUEUE_LENGTH              2U
#define APP_COMM_QUEUE_LENGTH             16U
#define APP_TELEMETRY_QUEUE_LENGTH         4U
#define APP_TRAJECTORY_QUEUE_LENGTH       32U

osMessageQueueId_t appRtosCanRxQueue;
osMessageQueueId_t appRtosChassisCommandQueue;
osMessageQueueId_t appRtosMechanismCommandQueue;
osMessageQueueId_t appRtosTrajectoryQueue;
osMessageQueueId_t appRtosActionQueue;
osMessageQueueId_t appRtosImuQueue;
osMessageQueueId_t appRtosDt35Queue;
osMessageQueueId_t appRtosCommunicationQueue;
osMessageQueueId_t appRtosTelemetryQueue;
osEventFlagsId_t appRtosSystemEvents;

static osMutexId_t app_rtos_robot_state_mutex;
static osThreadId_t app_rtos_can_rx_task;
static osThreadId_t app_rtos_control_task;
static osThreadId_t app_rtos_mechanism_task;
static osThreadId_t app_rtos_sensor_task;
static osThreadId_t app_rtos_navigation_task;
static osThreadId_t app_rtos_communication_task;
static osThreadId_t app_rtos_safety_task;
static osThreadId_t app_rtos_monitor_task;

app_runtime_stats_t appRtosRuntimeStats;
volatile uint32_t appRtosCanErrorMask;

volatile app_rtos_fault_t appRtosFaultCode = APP_RTOS_FAULT_NONE;
volatile uint32_t appRtosFaultLine;
const char * volatile appRtosFaultFile;

static app_robot_state_t app_rtos_robot_state;

static const osThreadAttr_t app_rtos_can_rx_task_attributes =
{
    .name = "canRxTask",
    .stack_size = APP_CAN_RX_TASK_STACK_SIZE,
    .priority = osPriorityHigh
};

static const osThreadAttr_t app_rtos_control_task_attributes =
{
    .name = "controlTask",
    .stack_size = APP_CONTROL_TASK_STACK_SIZE,
    .priority = osPriorityRealtime
};

static const osThreadAttr_t app_rtos_mechanism_task_attributes =
{
    .name = "mechanismTask",
    .stack_size = APP_MECHANISM_TASK_STACK_SIZE,
    .priority = osPriorityAboveNormal1
};

static const osThreadAttr_t app_rtos_sensor_task_attributes =
{
    .name = "sensorTask",
    .stack_size = APP_SENSOR_TASK_STACK_SIZE,
    .priority = osPriorityHigh1
};

static const osThreadAttr_t app_rtos_navigation_task_attributes =
{
    .name = "navigationTask",
    .stack_size = APP_NAVIGATION_TASK_STACK_SIZE,
    .priority = osPriorityAboveNormal
};

static const osThreadAttr_t app_rtos_communication_task_attributes =
{
    .name = "communicationTask",
    .stack_size = APP_COMM_TASK_STACK_SIZE,
    .priority = osPriorityNormal
};

static const osThreadAttr_t app_rtos_safety_task_attributes =
{
    .name = "safetyTask",
    .stack_size = APP_SAFETY_TASK_STACK_SIZE,
    .priority = osPriorityHigh
};

static const osThreadAttr_t app_rtos_monitor_task_attributes =
{
    .name = "monitorTask",
    .stack_size = APP_MONITOR_TASK_STACK_SIZE,
    .priority = osPriorityLow
};

static const osMessageQueueAttr_t app_rtos_can_rx_queue_attributes =
{
    .name = "canRxQueue"
};

static const osMessageQueueAttr_t app_rtos_chassis_queue_attributes =
{
    .name = "chassisCmdQueue"
};

static const osMessageQueueAttr_t app_rtos_mechanism_queue_attributes =
{
    .name = "mechanismCmdQueue"
};

static const osMessageQueueAttr_t app_rtos_trajectory_queue_attributes =
{
    .name = "trajectoryQueue"
};

static const osMessageQueueAttr_t app_rtos_action_queue_attributes =
{
    .name = "actionQueue"
};

static const osMessageQueueAttr_t app_rtos_imu_queue_attributes =
{
    .name = "imuQueue"
};

static const osMessageQueueAttr_t app_rtos_dt35_queue_attributes =
{
    .name = "dt35Queue"
};

static const osMessageQueueAttr_t app_rtos_communication_queue_attributes =
{
    .name = "communicationQueue"
};

static const osMessageQueueAttr_t app_rtos_telemetry_queue_attributes =
{
    .name = "telemetryQueue"
};

static const osEventFlagsAttr_t app_rtos_event_attributes =
{
    .name = "systemEvents"
};

static const osMutexAttr_t app_rtos_state_mutex_attributes =
{
    .name = "robotStateMutex",
    .attr_bits = osMutexPrioInherit
};

uint32_t AppRtos_MsToTicks(uint32_t milliseconds)
{
    uint64_t ticks;

    ticks = ((uint64_t)milliseconds * osKernelGetTickFreq() + 999U) / 1000U;
    if ((milliseconds > 0U) && (ticks == 0U))
    {
        ticks = 1U;
    }
    return (uint32_t)ticks;
}

bool AppRtos_WaitForInitialization(void)
{
    uint32_t flags;

    flags = osEventFlagsWait(appRtosSystemEvents,
                             APP_EVENT_INIT_DONE | APP_EVENT_INIT_FAILED,
                             osFlagsWaitAny | osFlagsNoClear, osWaitForever);
    return ((flags & osFlagsError) == 0U)
        && ((flags & APP_EVENT_INIT_DONE) != 0U);
}

static bool AppRtos_PutLatest(osMessageQueueId_t queue, const void *item,
                              void *discarded, uint32_t timeout_ms,
                              bool *discarded_oldest)
{
    if (discarded_oldest != NULL)
    {
        *discarded_oldest = false;
    }
    if (osMessageQueuePut(queue, item, 0U, AppRtos_MsToTicks(timeout_ms))
        == osOK)
    {
        return true;
    }
    if (osMessageQueueGet(queue, discarded, NULL, 0U) != osOK)
    {
        return false;
    }
    if (discarded_oldest != NULL)
    {
        *discarded_oldest = true;
    }
    return osMessageQueuePut(queue, item, 0U, 0U) == osOK;
}

static bool AppRtos_GetCanBus(const bsp_can_t *can, app_can_bus_t *bus)
{
    if ((can == NULL) || (bus == NULL)
        || (can->instance_index >= (uint8_t)APP_CAN_BUS_COUNT))
    {
        return false;
    }
    *bus = (app_can_bus_t)can->instance_index;
    return true;
}

static void AppRtos_CanRxFromISR(bsp_can_t *can, uint32_t can_id,
                                 bool is_extended_id, const uint8_t *data,
                                 uint8_t length)
{
    app_can_frame_t frame;
    TickType_t tick_count;

    if ((appRtosCanRxQueue == NULL) || (data == NULL)
        || (length > APP_CAN_DATA_MAX_SIZE)
        || !AppRtos_GetCanBus(can, &frame.bus))
    {
        return;
    }

    tick_count = xTaskGetTickCountFromISR();
    frame.timestamp_ms = (uint32_t)(((uint64_t)tick_count * 1000U)
                                    / configTICK_RATE_HZ);
    frame.identifier = can_id;
    frame.is_extended_id = is_extended_id ? 1U : 0U;
    frame.length = length;
    memset(frame.data, 0, sizeof(frame.data));
    memcpy(frame.data, data, length);
    if (osMessageQueuePut(appRtosCanRxQueue, &frame, 0U, 0U) != osOK)
    {
        appRtosRuntimeStats.can_rx_dropped++;
    }
}

static void AppRtos_CanErrorFromISR(bsp_can_t *can, uint32_t error_flags)
{
    app_can_bus_t bus;

    (void)error_flags;
    if (AppRtos_GetCanBus(can, &bus))
    {
        appRtosCanErrorMask |= 1UL << (uint32_t)bus;
    }
}

bool AppRtos_Init(void)
{
    memset(&app_rtos_robot_state, 0, sizeof(app_rtos_robot_state));
    memset(&appRtosRuntimeStats, 0, sizeof(appRtosRuntimeStats));
    appRtosCanErrorMask = 0U;

    appRtosCanRxQueue = osMessageQueueNew(
        APP_CAN_RX_QUEUE_LENGTH, sizeof(app_can_frame_t),
        &app_rtos_can_rx_queue_attributes);
    appRtosChassisCommandQueue = osMessageQueueNew(
        APP_COMMAND_QUEUE_LENGTH, sizeof(app_chassis_command_t),
        &app_rtos_chassis_queue_attributes);
    appRtosMechanismCommandQueue = osMessageQueueNew(
        APP_COMMAND_QUEUE_LENGTH, sizeof(app_mechanism_command_t),
        &app_rtos_mechanism_queue_attributes);
    appRtosTrajectoryQueue = osMessageQueueNew(
        APP_TRAJECTORY_QUEUE_LENGTH, sizeof(app_trajectory_point_t),
        &app_rtos_trajectory_queue_attributes);
    appRtosActionQueue = osMessageQueueNew(
        APP_ACTION_QUEUE_LENGTH, sizeof(app_action_command_t),
        &app_rtos_action_queue_attributes);
    appRtosImuQueue = osMessageQueueNew(
        APP_IMU_QUEUE_LENGTH, sizeof(app_imu_sample_t),
        &app_rtos_imu_queue_attributes);
    appRtosDt35Queue = osMessageQueueNew(
        APP_DT35_QUEUE_LENGTH, sizeof(app_dt35_sample_t),
        &app_rtos_dt35_queue_attributes);
    appRtosCommunicationQueue = osMessageQueueNew(
        APP_COMM_QUEUE_LENGTH, sizeof(app_comm_packet_t),
        &app_rtos_communication_queue_attributes);
    appRtosTelemetryQueue = osMessageQueueNew(
        APP_TELEMETRY_QUEUE_LENGTH, sizeof(app_runtime_stats_t),
        &app_rtos_telemetry_queue_attributes);
    appRtosSystemEvents = osEventFlagsNew(&app_rtos_event_attributes);
    app_rtos_robot_state_mutex = osMutexNew(
        &app_rtos_state_mutex_attributes);

    if ((appRtosCanRxQueue == NULL)
        || (appRtosChassisCommandQueue == NULL)
        || (appRtosMechanismCommandQueue == NULL)
        || (appRtosTrajectoryQueue == NULL)
        || (appRtosActionQueue == NULL)
        || (appRtosImuQueue == NULL)
        || (appRtosDt35Queue == NULL)
        || (appRtosCommunicationQueue == NULL)
        || (appRtosTelemetryQueue == NULL)
        || (appRtosSystemEvents == NULL)
        || (app_rtos_robot_state_mutex == NULL))
    {
        return false;
    }

    app_rtos_can_rx_task = osThreadNew(
        AppTask_CanRx, NULL, &app_rtos_can_rx_task_attributes);
    app_rtos_control_task = osThreadNew(
        AppTask_Control, NULL, &app_rtos_control_task_attributes);
    app_rtos_mechanism_task = osThreadNew(
        AppTask_Mechanism, NULL, &app_rtos_mechanism_task_attributes);
    app_rtos_sensor_task = osThreadNew(
        AppTask_Sensor, NULL, &app_rtos_sensor_task_attributes);
    app_rtos_navigation_task = osThreadNew(
        AppTask_Navigation, NULL, &app_rtos_navigation_task_attributes);
    app_rtos_communication_task = osThreadNew(
        AppTask_Communication, NULL,
        &app_rtos_communication_task_attributes);
    app_rtos_safety_task = osThreadNew(
        AppTask_Safety, NULL, &app_rtos_safety_task_attributes);
    app_rtos_monitor_task = osThreadNew(
        AppTask_Monitor, NULL, &app_rtos_monitor_task_attributes);

    return (app_rtos_can_rx_task != NULL)
        && (app_rtos_control_task != NULL)
        && (app_rtos_mechanism_task != NULL)
        && (app_rtos_sensor_task != NULL)
        && (app_rtos_navigation_task != NULL)
        && (app_rtos_communication_task != NULL)
        && (app_rtos_safety_task != NULL)
        && (app_rtos_monitor_task != NULL);
}

void AppRtos_Bootstrap(void)
{
    if (AppInit_Run(AppRtos_CanRxFromISR, AppRtos_CanErrorFromISR))
    {
        (void)osEventFlagsSet(appRtosSystemEvents,
                              APP_EVENT_ALL_CAN_READY | APP_EVENT_INIT_DONE);
    }
    else
    {
        (void)osEventFlagsSet(appRtosSystemEvents,
                              APP_EVENT_INIT_FAILED | APP_EVENT_ESTOP);
    }
}

bool AppRtos_SubmitChassisCommand(const app_chassis_command_t *command,
                                  uint32_t timeout_ms)
{
    app_chassis_command_t discarded;

    return (command != NULL) && (appRtosChassisCommandQueue != NULL)
        && AppRtos_PutLatest(appRtosChassisCommandQueue, command, &discarded,
                             timeout_ms, NULL);
}

bool AppRtos_SubmitMechanismCommand(
    const app_mechanism_command_t *command, uint32_t timeout_ms)
{
    app_mechanism_command_t discarded;

    return (command != NULL) && (appRtosMechanismCommandQueue != NULL)
        && AppRtos_PutLatest(appRtosMechanismCommandQueue, command,
                             &discarded, timeout_ms, NULL);
}

bool AppRtos_SubmitTrajectoryPoint(const app_trajectory_point_t *point,
                                   uint32_t timeout_ms)
{
    return (point != NULL) && (appRtosTrajectoryQueue != NULL)
        && (osMessageQueuePut(appRtosTrajectoryQueue, point, 0U,
                              AppRtos_MsToTicks(timeout_ms)) == osOK);
}

bool AppRtos_SubmitActionCommand(const app_action_command_t *command,
                                 uint32_t timeout_ms)
{
    return (command != NULL) && (appRtosActionQueue != NULL)
        && (osMessageQueuePut(appRtosActionQueue, command, 0U,
                              AppRtos_MsToTicks(timeout_ms)) == osOK);
}

bool AppRtos_SubmitImuSample(const app_imu_sample_t *sample,
                             uint32_t timeout_ms)
{
    app_imu_sample_t discarded;
    bool discarded_oldest;
    bool queued;

    if ((sample == NULL) || (appRtosImuQueue == NULL)
        || (app_rtos_sensor_task == NULL))
    {
        return false;
    }
    queued = AppRtos_PutLatest(appRtosImuQueue, sample, &discarded,
                               timeout_ms, &discarded_oldest);
    if (discarded_oldest)
    {
        appRtosRuntimeStats.imu_rx_dropped++;
    }
    if (!queued)
    {
        return false;
    }
    return (osThreadFlagsSet(app_rtos_sensor_task, APP_SENSOR_FLAG_IMU)
            & osFlagsError) == 0U;
}

bool AppRtos_SubmitDt35Sample(const app_dt35_sample_t *sample,
                              uint32_t timeout_ms)
{
    app_dt35_sample_t discarded;
    bool discarded_oldest;
    bool queued;

    if ((sample == NULL) || (appRtosDt35Queue == NULL)
        || (app_rtos_sensor_task == NULL))
    {
        return false;
    }
    queued = AppRtos_PutLatest(appRtosDt35Queue, sample, &discarded,
                               timeout_ms, &discarded_oldest);
    if (discarded_oldest)
    {
        appRtosRuntimeStats.dt35_rx_dropped++;
    }
    if (!queued)
    {
        return false;
    }
    return (osThreadFlagsSet(app_rtos_sensor_task, APP_SENSOR_FLAG_DT35)
            & osFlagsError) == 0U;
}

bool AppRtos_SubmitCommunicationPacket(const app_comm_packet_t *packet,
                                       uint32_t timeout_ms)
{
    if ((packet == NULL) || (packet->length > APP_COMM_PAYLOAD_MAX_SIZE)
        || (appRtosCommunicationQueue == NULL))
    {
        return false;
    }
    if (osMessageQueuePut(appRtosCommunicationQueue, packet, 0U,
                          AppRtos_MsToTicks(timeout_ms)) != osOK)
    {
        appRtosRuntimeStats.comm_rx_dropped++;
        return false;
    }
    return true;
}

bool AppRtos_PublishRobotState(const app_robot_state_t *state,
                               uint32_t timeout_ms)
{
    if ((state == NULL) || (app_rtos_robot_state_mutex == NULL)
        || (osMutexAcquire(app_rtos_robot_state_mutex,
                           AppRtos_MsToTicks(timeout_ms)) != osOK))
    {
        return false;
    }
    app_rtos_robot_state = *state;
    return osMutexRelease(app_rtos_robot_state_mutex) == osOK;
}

bool AppRtos_GetRobotState(app_robot_state_t *state, uint32_t timeout_ms)
{
    if ((state == NULL) || (app_rtos_robot_state_mutex == NULL)
        || (osMutexAcquire(app_rtos_robot_state_mutex,
                           AppRtos_MsToTicks(timeout_ms)) != osOK))
    {
        return false;
    }
    *state = app_rtos_robot_state;
    return osMutexRelease(app_rtos_robot_state_mutex) == osOK;
}

void AppRtos_GetRuntimeStats(app_runtime_stats_t *stats)
{
    if (stats != NULL)
    {
        *stats = appRtosRuntimeStats;
        stats->timestamp_ms = osKernelGetTickCount();
        stats->system_events = AppRtos_GetEvents();
        stats->can_error_mask = appRtosCanErrorMask;
    }
}

bool AppRtos_SetEvents(uint32_t flags)
{
    return (appRtosSystemEvents != NULL)
        && ((osEventFlagsSet(appRtosSystemEvents, flags) & osFlagsError) == 0U);
}

bool AppRtos_ClearEvents(uint32_t flags)
{
    return (appRtosSystemEvents != NULL)
        && ((osEventFlagsClear(appRtosSystemEvents, flags) & osFlagsError)
            == 0U);
}

uint32_t AppRtos_GetEvents(void)
{
    uint32_t flags;

    if (appRtosSystemEvents == NULL)
    {
        return 0U;
    }
    flags = osEventFlagsGet(appRtosSystemEvents);
    return ((flags & osFlagsError) == 0U) ? flags : 0U;
}

void AppRtos_Assert(const char *file, uint32_t line)
{
    appRtosFaultFile = file;
    appRtosFaultLine = line;
    AppRtos_Panic(APP_RTOS_FAULT_ASSERT);
}

void AppRtos_Panic(app_rtos_fault_t fault)
{
    appRtosFaultCode = fault;
    __disable_irq();
    for (;;)
    {
        __NOP();
    }
}
