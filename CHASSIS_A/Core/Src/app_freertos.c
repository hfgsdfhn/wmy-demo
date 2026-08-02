#include "app_freertos.h"

#include <string.h>

#include "FreeRTOS.h"
#include "fdcan.h"
#include "task.h"

#define APP_CAN_RX_QUEUE_LENGTH           64U
#define APP_COMMAND_QUEUE_LENGTH           4U
#define APP_COMM_QUEUE_LENGTH             16U
#define APP_TELEMETRY_QUEUE_LENGTH         4U

#define APP_CONTROL_PERIOD_MS              1U
#define APP_NAVIGATION_PERIOD_MS          10U
#define APP_SAFETY_PERIOD_MS              10U
#define APP_MONITOR_PERIOD_MS            100U

#define APP_CAN_RX_TASK_STACK_SIZE       1536U
#define APP_CONTROL_TASK_STACK_SIZE      2048U
#define APP_NAVIGATION_TASK_STACK_SIZE   3072U
#define APP_COMM_TASK_STACK_SIZE         2048U
#define APP_SAFETY_TASK_STACK_SIZE       1024U
#define APP_MONITOR_TASK_STACK_SIZE      1024U

osThreadId_t appCanRxTaskHandle;
osThreadId_t appControlTaskHandle;
osThreadId_t appNavigationTaskHandle;
osThreadId_t appCommunicationTaskHandle;
osThreadId_t appSafetyTaskHandle;
osThreadId_t appMonitorTaskHandle;

osMessageQueueId_t appCanRxQueueHandle;
osMessageQueueId_t appChassisCommandQueueHandle;
osMessageQueueId_t appMechanismCommandQueueHandle;
osMessageQueueId_t appTrajectoryQueueHandle;
osMessageQueueId_t appCommunicationQueueHandle;
osMessageQueueId_t appTelemetryQueueHandle;
osEventFlagsId_t appSystemEventHandle;
osMutexId_t appRobotStateMutexHandle;

volatile app_freertos_fault_t appFreertosFaultCode = APP_FREERTOS_FAULT_NONE;
volatile uint32_t appFreertosFaultLine;
const char * volatile appFreertosFaultFile;

static app_robot_state_t app_robot_state;
static app_runtime_stats_t app_runtime_stats;
static volatile uint32_t app_can_error_mask;

static void AppCanRxTask(void *argument);
static void AppControlTask(void *argument);
static void AppNavigationTask(void *argument);
static void AppCommunicationTask(void *argument);
static void AppSafetyTask(void *argument);
static void AppMonitorTask(void *argument);

static const osThreadAttr_t app_can_rx_task_attributes =
{
    .name = "canRxTask",
    .stack_size = APP_CAN_RX_TASK_STACK_SIZE,
    .priority = osPriorityHigh
};

static const osThreadAttr_t app_control_task_attributes =
{
    .name = "controlTask",
    .stack_size = APP_CONTROL_TASK_STACK_SIZE,
    .priority = osPriorityRealtime
};

static const osThreadAttr_t app_navigation_task_attributes =
{
    .name = "navigationTask",
    .stack_size = APP_NAVIGATION_TASK_STACK_SIZE,
    .priority = osPriorityAboveNormal
};

static const osThreadAttr_t app_communication_task_attributes =
{
    .name = "communicationTask",
    .stack_size = APP_COMM_TASK_STACK_SIZE,
    .priority = osPriorityNormal
};

static const osThreadAttr_t app_safety_task_attributes =
{
    .name = "safetyTask",
    .stack_size = APP_SAFETY_TASK_STACK_SIZE,
    .priority = osPriorityHigh
};

static const osThreadAttr_t app_monitor_task_attributes =
{
    .name = "monitorTask",
    .stack_size = APP_MONITOR_TASK_STACK_SIZE,
    .priority = osPriorityLow
};

static const osMessageQueueAttr_t app_can_rx_queue_attributes =
{
    .name = "canRxQueue"
};

static const osMessageQueueAttr_t app_chassis_command_queue_attributes =
{
    .name = "chassisCmdQueue"
};

static const osMessageQueueAttr_t app_mechanism_command_queue_attributes =
{
    .name = "mechanismCmdQueue"
};

static const osMessageQueueAttr_t app_trajectory_queue_attributes =
{
    .name = "trajectoryQueue"
};

static const osMessageQueueAttr_t app_communication_queue_attributes =
{
    .name = "communicationQueue"
};

static const osMessageQueueAttr_t app_telemetry_queue_attributes =
{
    .name = "telemetryQueue"
};

static const osEventFlagsAttr_t app_system_event_attributes =
{
    .name = "systemEvents"
};

static const osMutexAttr_t app_state_mutex_attributes =
{
    .name = "robotStateMutex",
    .attr_bits = osMutexPrioInherit
};

static uint32_t App_MsToTicks(uint32_t milliseconds)
{
    uint64_t ticks;

    ticks = ((uint64_t)milliseconds * osKernelGetTickFreq() + 999U) / 1000U;
    if ((milliseconds > 0U) && (ticks == 0U))
    {
        ticks = 1U;
    }
    return (uint32_t)ticks;
}

static bool App_WaitForInitialization(void)
{
    uint32_t flags;

    flags = osEventFlagsWait(appSystemEventHandle,
                             APP_EVENT_INIT_DONE | APP_EVENT_INIT_FAILED,
                             osFlagsWaitAny | osFlagsNoClear, osWaitForever);
    return ((flags & osFlagsError) == 0U)
        && ((flags & APP_EVENT_INIT_DONE) != 0U);
}

static bool App_StartCan(FDCAN_HandleTypeDef *handle, uint32_t ready_event)
{
    FDCAN_FilterTypeDef filter = {0};

    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0U;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = 0U;
    filter.FilterID2 = 0U;
    if (HAL_FDCAN_ConfigFilter(handle, &filter) != HAL_OK)
    {
        return false;
    }

    filter.IdType = FDCAN_EXTENDED_ID;
    if (HAL_FDCAN_ConfigFilter(handle, &filter) != HAL_OK)
    {
        return false;
    }

    if (HAL_FDCAN_ConfigGlobalFilter(handle, FDCAN_REJECT,
                                     FDCAN_REJECT, FDCAN_REJECT_REMOTE,
                                     FDCAN_REJECT_REMOTE) != HAL_OK)
    {
        return false;
    }

    if (HAL_FDCAN_Start(handle) != HAL_OK)
    {
        return false;
    }

    if (HAL_FDCAN_ActivateNotification(
            handle, FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_BUS_OFF,
            0U) != HAL_OK)
    {
        (void)HAL_FDCAN_Stop(handle);
        return false;
    }

    (void)osEventFlagsSet(appSystemEventHandle, ready_event);
    return true;
}

static uint8_t App_FdcanDataLengthToBytes(uint32_t data_length)
{
    switch (data_length)
    {
    case FDCAN_DLC_BYTES_0:
        return 0U;
    case FDCAN_DLC_BYTES_1:
        return 1U;
    case FDCAN_DLC_BYTES_2:
        return 2U;
    case FDCAN_DLC_BYTES_3:
        return 3U;
    case FDCAN_DLC_BYTES_4:
        return 4U;
    case FDCAN_DLC_BYTES_5:
        return 5U;
    case FDCAN_DLC_BYTES_6:
        return 6U;
    case FDCAN_DLC_BYTES_7:
        return 7U;
    case FDCAN_DLC_BYTES_8:
        return 8U;
    default:
        return 0U;
    }
}

static bool App_GetCanBus(const FDCAN_HandleTypeDef *handle,
                          app_can_bus_t *bus)
{
    if (handle == &hfdcan1)
    {
        *bus = APP_CAN_BUS_1;
    }
    else if (handle == &hfdcan2)
    {
        *bus = APP_CAN_BUS_2;
    }
    else if (handle == &hfdcan3)
    {
        *bus = APP_CAN_BUS_3;
    }
    else
    {
        return false;
    }
    return true;
}

static bool App_PutLatestChassisCommand(
    const app_chassis_command_t *command, uint32_t timeout_ticks)
{
    app_chassis_command_t discarded;

    if (osMessageQueuePut(appChassisCommandQueueHandle, command, 0U,
                          timeout_ticks) == osOK)
    {
        return true;
    }
    if (osMessageQueueGet(appChassisCommandQueueHandle, &discarded, NULL,
                          0U) != osOK)
    {
        return false;
    }
    return osMessageQueuePut(appChassisCommandQueueHandle, command, 0U,
                             0U) == osOK;
}

static bool App_PutLatestMechanismCommand(
    const app_mechanism_command_t *command, uint32_t timeout_ticks)
{
    app_mechanism_command_t discarded;

    if (osMessageQueuePut(appMechanismCommandQueueHandle, command, 0U,
                          timeout_ticks) == osOK)
    {
        return true;
    }
    if (osMessageQueueGet(appMechanismCommandQueueHandle, &discarded, NULL,
                          0U) != osOK)
    {
        return false;
    }
    return osMessageQueuePut(appMechanismCommandQueueHandle, command, 0U,
                             0U) == osOK;
}

bool AppFreertos_Init(void)
{
    memset(&app_robot_state, 0, sizeof(app_robot_state));
    memset(&app_runtime_stats, 0, sizeof(app_runtime_stats));
    app_can_error_mask = 0U;

    appCanRxQueueHandle = osMessageQueueNew(APP_CAN_RX_QUEUE_LENGTH,
        sizeof(app_can_frame_t), &app_can_rx_queue_attributes);
    appChassisCommandQueueHandle = osMessageQueueNew(
        APP_COMMAND_QUEUE_LENGTH, sizeof(app_chassis_command_t),
        &app_chassis_command_queue_attributes);
    appMechanismCommandQueueHandle = osMessageQueueNew(
        APP_COMMAND_QUEUE_LENGTH, sizeof(app_mechanism_command_t),
        &app_mechanism_command_queue_attributes);
    appTrajectoryQueueHandle = osMessageQueueNew(
        APP_TRAJECTORY_QUEUE_LENGTH, sizeof(app_trajectory_point_t),
        &app_trajectory_queue_attributes);
    appCommunicationQueueHandle = osMessageQueueNew(
        APP_COMM_QUEUE_LENGTH, sizeof(app_comm_packet_t),
        &app_communication_queue_attributes);
    appTelemetryQueueHandle = osMessageQueueNew(
        APP_TELEMETRY_QUEUE_LENGTH, sizeof(app_runtime_stats_t),
        &app_telemetry_queue_attributes);
    appSystemEventHandle = osEventFlagsNew(&app_system_event_attributes);
    appRobotStateMutexHandle = osMutexNew(&app_state_mutex_attributes);

    if ((appCanRxQueueHandle == NULL)
        || (appChassisCommandQueueHandle == NULL)
        || (appMechanismCommandQueueHandle == NULL)
        || (appTrajectoryQueueHandle == NULL)
        || (appCommunicationQueueHandle == NULL)
        || (appTelemetryQueueHandle == NULL)
        || (appSystemEventHandle == NULL)
        || (appRobotStateMutexHandle == NULL))
    {
        return false;
    }

    appCanRxTaskHandle = osThreadNew(AppCanRxTask, NULL,
                                     &app_can_rx_task_attributes);
    appControlTaskHandle = osThreadNew(AppControlTask, NULL,
                                       &app_control_task_attributes);
    appNavigationTaskHandle = osThreadNew(AppNavigationTask, NULL,
                                          &app_navigation_task_attributes);
    appCommunicationTaskHandle = osThreadNew(
        AppCommunicationTask, NULL, &app_communication_task_attributes);
    appSafetyTaskHandle = osThreadNew(AppSafetyTask, NULL,
                                      &app_safety_task_attributes);
    appMonitorTaskHandle = osThreadNew(AppMonitorTask, NULL,
                                       &app_monitor_task_attributes);

    return (appCanRxTaskHandle != NULL)
        && (appControlTaskHandle != NULL)
        && (appNavigationTaskHandle != NULL)
        && (appCommunicationTaskHandle != NULL)
        && (appSafetyTaskHandle != NULL)
        && (appMonitorTaskHandle != NULL);
}

void AppFreertos_Bootstrap(void)
{
    bool initialized;

    initialized = App_StartCan(&hfdcan1, APP_EVENT_CAN1_READY);
    initialized = App_StartCan(&hfdcan2, APP_EVENT_CAN2_READY) && initialized;
    initialized = App_StartCan(&hfdcan3, APP_EVENT_CAN3_READY) && initialized;

    if (initialized)
    {
        (void)osEventFlagsSet(appSystemEventHandle, APP_EVENT_INIT_DONE);
    }
    else
    {
        (void)osEventFlagsSet(appSystemEventHandle,
                              APP_EVENT_INIT_FAILED | APP_EVENT_ESTOP);
    }
}

bool AppFreertos_SubmitChassisCommand(const app_chassis_command_t *command,
                                      uint32_t timeout_ms)
{
    if ((command == NULL) || (appChassisCommandQueueHandle == NULL))
    {
        return false;
    }
    return App_PutLatestChassisCommand(command, App_MsToTicks(timeout_ms));
}

bool AppFreertos_SubmitMechanismCommand(
    const app_mechanism_command_t *command, uint32_t timeout_ms)
{
    if ((command == NULL) || (appMechanismCommandQueueHandle == NULL))
    {
        return false;
    }
    return App_PutLatestMechanismCommand(command, App_MsToTicks(timeout_ms));
}

bool AppFreertos_SubmitTrajectoryPoint(const app_trajectory_point_t *point,
                                       uint32_t timeout_ms)
{
    if ((point == NULL) || (appTrajectoryQueueHandle == NULL))
    {
        return false;
    }
    return osMessageQueuePut(appTrajectoryQueueHandle, point, 0U,
                             App_MsToTicks(timeout_ms)) == osOK;
}

bool AppFreertos_SubmitCommunicationPacket(const app_comm_packet_t *packet,
                                            uint32_t timeout_ms)
{
    if ((packet == NULL) || (packet->length > APP_COMM_PAYLOAD_MAX_SIZE)
        || (appCommunicationQueueHandle == NULL))
    {
        return false;
    }

    if (osMessageQueuePut(appCommunicationQueueHandle, packet, 0U,
                          App_MsToTicks(timeout_ms)) != osOK)
    {
        app_runtime_stats.comm_rx_dropped++;
        return false;
    }
    return true;
}

bool AppFreertos_PublishRobotState(const app_robot_state_t *state,
                                   uint32_t timeout_ms)
{
    if ((state == NULL) || (appRobotStateMutexHandle == NULL)
        || (osMutexAcquire(appRobotStateMutexHandle,
                           App_MsToTicks(timeout_ms)) != osOK))
    {
        return false;
    }
    app_robot_state = *state;
    (void)osMutexRelease(appRobotStateMutexHandle);
    return true;
}

bool AppFreertos_GetRobotState(app_robot_state_t *state,
                               uint32_t timeout_ms)
{
    if ((state == NULL) || (appRobotStateMutexHandle == NULL)
        || (osMutexAcquire(appRobotStateMutexHandle,
                           App_MsToTicks(timeout_ms)) != osOK))
    {
        return false;
    }
    *state = app_robot_state;
    (void)osMutexRelease(appRobotStateMutexHandle);
    return true;
}

void AppFreertos_GetRuntimeStats(app_runtime_stats_t *stats)
{
    if (stats != NULL)
    {
        *stats = app_runtime_stats;
        stats->timestamp_ms = osKernelGetTickCount();
        stats->system_events = (appSystemEventHandle != NULL)
            ? osEventFlagsGet(appSystemEventHandle) : 0U;
        stats->can_error_mask = app_can_error_mask;
    }
}

void AppFreertos_NotifyCanErrorFromISR(app_can_bus_t bus)
{
    if (bus <= APP_CAN_BUS_3)
    {
        app_can_error_mask |= (1UL << (uint32_t)bus);
    }
}

static void AppCanRxTask(void *argument)
{
    app_can_frame_t frame;
    uint32_t ready_event;

    (void)argument;
    if (!App_WaitForInitialization())
    {
        osThreadExit();
    }

    for (;;)
    {
        if (osMessageQueueGet(appCanRxQueueHandle, &frame, NULL,
                              osWaitForever) == osOK)
        {
            app_runtime_stats.can_rx_count++;
            ready_event = APP_EVENT_CAN1_READY << (uint32_t)frame.bus;
            (void)osEventFlagsSet(appSystemEventHandle, ready_event);
            App_OnCanFrame(&frame);
        }
    }
}

static void AppControlTask(void *argument)
{
    app_chassis_command_t chassis_command = {0};
    app_mechanism_command_t mechanism_command = {0};
    uint32_t period_ticks;
    uint32_t next_wake;
    uint32_t system_events;

    (void)argument;
    if (!App_WaitForInitialization())
    {
        osThreadExit();
    }

    next_wake = osKernelGetTickCount();
    period_ticks = App_MsToTicks(APP_CONTROL_PERIOD_MS);
    for (;;)
    {
        while (osMessageQueueGet(appChassisCommandQueueHandle,
                                 &chassis_command, NULL, 0U) == osOK)
        {
        }
        while (osMessageQueueGet(appMechanismCommandQueueHandle,
                                 &mechanism_command, NULL, 0U) == osOK)
        {
        }

        system_events = osEventFlagsGet(appSystemEventHandle);
        if ((system_events & (APP_EVENT_ESTOP | APP_EVENT_MOTOR_FAULT
                              | APP_EVENT_CAN_FAULT)) != 0U)
        {
            chassis_command.valid = 0U;
            mechanism_command.valid = 0U;
        }

        App_ControlStep(&chassis_command, &mechanism_command, system_events);
        app_runtime_stats.control_cycle_count++;

        next_wake += period_ticks;
        if ((int32_t)(osKernelGetTickCount() - next_wake) >= 0)
        {
            app_runtime_stats.control_overrun_count++;
        }
        (void)osDelayUntil(next_wake);
    }
}

static void AppNavigationTask(void *argument)
{
    app_trajectory_point_t point = {0};
    bool point_available;
    uint32_t next_wake;

    (void)argument;
    if (!App_WaitForInitialization())
    {
        osThreadExit();
    }

    next_wake = osKernelGetTickCount();
    for (;;)
    {
        point_available = osMessageQueueGet(appTrajectoryQueueHandle, &point,
                                            NULL, 0U) == osOK;
        App_NavigationStep(&point, point_available);
        next_wake += App_MsToTicks(APP_NAVIGATION_PERIOD_MS);
        (void)osDelayUntil(next_wake);
    }
}

static void AppCommunicationTask(void *argument)
{
    app_comm_packet_t packet;

    (void)argument;
    if (!App_WaitForInitialization())
    {
        osThreadExit();
    }

    for (;;)
    {
        if (osMessageQueueGet(appCommunicationQueueHandle, &packet, NULL,
                              osWaitForever) == osOK)
        {
            app_runtime_stats.comm_rx_count++;
            App_OnCommunicationPacket(&packet);
        }
    }
}

static void AppSafetyTask(void *argument)
{
    uint32_t next_wake;
    uint32_t system_events;

    (void)argument;
    next_wake = osKernelGetTickCount();
    for (;;)
    {
        if (app_can_error_mask != 0U)
        {
            (void)osEventFlagsSet(appSystemEventHandle,
                                  APP_EVENT_CAN_FAULT | APP_EVENT_ESTOP);
        }
        system_events = osEventFlagsGet(appSystemEventHandle);
        App_SafetyStep(system_events);

        next_wake += App_MsToTicks(APP_SAFETY_PERIOD_MS);
        (void)osDelayUntil(next_wake);
    }
}

static void AppMonitorTask(void *argument)
{
    app_runtime_stats_t stats;
    app_runtime_stats_t discarded;
    uint32_t next_wake;

    (void)argument;
    next_wake = osKernelGetTickCount();
    for (;;)
    {
        AppFreertos_GetRuntimeStats(&stats);
        if (osMessageQueuePut(appTelemetryQueueHandle, &stats, 0U, 0U)
            != osOK)
        {
            (void)osMessageQueueGet(appTelemetryQueueHandle, &discarded,
                                    NULL, 0U);
            (void)osMessageQueuePut(appTelemetryQueueHandle, &stats, 0U, 0U);
        }
        App_MonitorStep(&stats);

        next_wake += App_MsToTicks(APP_MONITOR_PERIOD_MS);
        (void)osDelayUntil(next_wake);
    }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *handle,
                               uint32_t rx_fifo0_its)
{
    FDCAN_RxHeaderTypeDef header;
    app_can_frame_t frame;

    (void)rx_fifo0_its;
    if ((appCanRxQueueHandle == NULL) || !App_GetCanBus(handle, &frame.bus))
    {
        return;
    }

    while (HAL_FDCAN_GetRxFifoFillLevel(handle, FDCAN_RX_FIFO0) > 0U)
    {
        if (HAL_FDCAN_GetRxMessage(handle, FDCAN_RX_FIFO0, &header,
                                   frame.data) != HAL_OK)
        {
            AppFreertos_NotifyCanErrorFromISR(frame.bus);
            break;
        }

        frame.timestamp_ms = HAL_GetTick();
        frame.identifier = header.Identifier;
        frame.is_extended_id =
            (header.IdType == FDCAN_EXTENDED_ID) ? 1U : 0U;
        frame.length = App_FdcanDataLengthToBytes(header.DataLength);
        if (osMessageQueuePut(appCanRxQueueHandle, &frame, 0U, 0U) != osOK)
        {
            app_runtime_stats.can_rx_dropped++;
        }
    }
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *handle,
                                   uint32_t error_status_its)
{
    app_can_bus_t bus;

    (void)error_status_its;
    if (App_GetCanBus(handle, &bus))
    {
        AppFreertos_NotifyCanErrorFromISR(bus);
    }
}

void App_FreertosAssert(const char *file, uint32_t line)
{
    appFreertosFaultFile = file;
    appFreertosFaultLine = line;
    AppFreertos_Panic(APP_FREERTOS_FAULT_ASSERT);
}

void AppFreertos_Panic(app_freertos_fault_t fault)
{
    appFreertosFaultCode = fault;
    __disable_irq();
    for (;;)
    {
        __NOP();
    }
}

__weak void App_OnCanFrame(const app_can_frame_t *frame)
{
    (void)frame;
}

__weak void App_ControlStep(const app_chassis_command_t *chassis_command,
                            const app_mechanism_command_t *mechanism_command,
                            uint32_t system_events)
{
    (void)chassis_command;
    (void)mechanism_command;
    (void)system_events;
}

__weak void App_NavigationStep(const app_trajectory_point_t *point,
                               bool point_available)
{
    (void)point;
    (void)point_available;
}

__weak void App_OnCommunicationPacket(const app_comm_packet_t *packet)
{
    (void)packet;
}

__weak void App_SafetyStep(uint32_t system_events)
{
    (void)system_events;
}

__weak void App_MonitorStep(const app_runtime_stats_t *stats)
{
    (void)stats;
}
