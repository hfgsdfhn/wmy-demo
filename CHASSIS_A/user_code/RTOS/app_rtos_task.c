#include "app_rtos_task.h"

#include <string.h>

#include "app_can_router.h"
#include "app_chassis.h"
#include "app_climb.h"
#include "app_communication.h"
#include "app_monitor.h"
#include "app_navigation.h"
#include "app_rtos_event.h"
#include "app_rtos_queue.h"
#include "app_safety.h"
#include "app_sensors.h"
#include "bsp_board.h"
#include "control_config.h"

static osThreadId_t appRtosCanRxTask;
static osThreadId_t appRtosControlTask;
static osThreadId_t appRtosMechanismTask;
static osThreadId_t appRtosSensorTask;
static osThreadId_t appRtosNavigationTask;
static osThreadId_t appRtosCommunicationTask;
static osThreadId_t appRtosSafetyTask;
static osThreadId_t appRtosMonitorTask;
static app_runtime_stats_t appRtosRuntimeStats;

static const osThreadAttr_t appRtosCanRxTaskAttributes = { .name = "canRxTask", .stack_size = APP_CAN_RX_TASK_STACK_SIZE, .priority = osPriorityHigh };
static const osThreadAttr_t appRtosControlTaskAttributes = { .name = "controlTask", .stack_size = APP_CONTROL_TASK_STACK_SIZE, .priority = osPriorityRealtime };
static const osThreadAttr_t appRtosMechanismTaskAttributes = { .name = "mechanismTask", .stack_size = APP_MECHANISM_TASK_STACK_SIZE, .priority = osPriorityAboveNormal1 };
static const osThreadAttr_t appRtosSensorTaskAttributes = { .name = "sensorTask", .stack_size = APP_SENSOR_TASK_STACK_SIZE, .priority = osPriorityHigh1 };
static const osThreadAttr_t appRtosNavigationTaskAttributes = { .name = "navigationTask", .stack_size = APP_NAVIGATION_TASK_STACK_SIZE, .priority = osPriorityAboveNormal };
static const osThreadAttr_t appRtosCommunicationTaskAttributes = { .name = "communicationTask", .stack_size = APP_COMM_TASK_STACK_SIZE, .priority = osPriorityNormal };
static const osThreadAttr_t appRtosSafetyTaskAttributes = { .name = "safetyTask", .stack_size = APP_SAFETY_TASK_STACK_SIZE, .priority = osPriorityHigh };
static const osThreadAttr_t appRtosMonitorTaskAttributes = { .name = "monitorTask", .stack_size = APP_MONITOR_TASK_STACK_SIZE, .priority = osPriorityLow };

static void AppTask_CanRx(void *argument);
static void AppTask_Control(void *argument);
static void AppTask_Mechanism(void *argument);
static void AppTask_Sensor(void *argument);
static void AppTask_Navigation(void *argument);
static void AppTask_Communication(void *argument);
static void AppTask_Safety(void *argument);
static void AppTask_Monitor(void *argument);

bool AppRtos_TaskCreate(void)
{
    memset(&appRtosRuntimeStats, 0, sizeof(appRtosRuntimeStats));
    appRtosCanRxTask = osThreadNew(AppTask_CanRx, NULL, &appRtosCanRxTaskAttributes);
    appRtosControlTask = osThreadNew(AppTask_Control, NULL, &appRtosControlTaskAttributes);
    appRtosMechanismTask = osThreadNew(AppTask_Mechanism, NULL, &appRtosMechanismTaskAttributes);
    appRtosSensorTask = osThreadNew(AppTask_Sensor, NULL, &appRtosSensorTaskAttributes);
    appRtosNavigationTask = osThreadNew(AppTask_Navigation, NULL, &appRtosNavigationTaskAttributes);
    appRtosCommunicationTask = osThreadNew(AppTask_Communication, NULL, &appRtosCommunicationTaskAttributes);
    appRtosSafetyTask = osThreadNew(AppTask_Safety, NULL, &appRtosSafetyTaskAttributes);
    appRtosMonitorTask = osThreadNew(AppTask_Monitor, NULL, &appRtosMonitorTaskAttributes);

    return (appRtosCanRxTask != NULL) && (appRtosControlTask != NULL)
        && (appRtosMechanismTask != NULL) && (appRtosSensorTask != NULL)
        && (appRtosNavigationTask != NULL) && (appRtosCommunicationTask != NULL)
        && (appRtosSafetyTask != NULL) && (appRtosMonitorTask != NULL);
}

bool AppRtos_SensorTaskReady(void)
{
    return appRtosSensorTask != NULL;
}

bool AppRtos_NotifySensor(uint32_t flags)
{
    return (osThreadFlagsSet(appRtosSensorTask, flags) & osFlagsError) == 0U;
}

void AppRtos_RecordCanRxDrop(void) { appRtosRuntimeStats.can_rx_dropped++; }
void AppRtos_RecordCommunicationDrop(void) { appRtosRuntimeStats.comm_rx_dropped++; }
void AppRtos_RecordImuDrop(void) { appRtosRuntimeStats.imu_rx_dropped++; }
void AppRtos_RecordDt35Drop(void) { appRtosRuntimeStats.dt35_rx_dropped++; }

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

static void AppTask_CanRx(void *argument)
{
    app_can_frame_t frame;

    (void)argument;
    if (!AppRtosEvent_WaitForInitialization())
    {
        osThreadExit();
    }
    for (;;)
    {
        if (osMessageQueueGet(appRtosCanRxQueue, &frame, NULL, osWaitForever) == osOK)
        {
            appRtosRuntimeStats.can_rx_count++;
            AppCanRouter_Dispatch(&frame);
        }
    }
}

static void AppTask_Control(void *argument)
{
    app_chassis_command_t chassisCommand = {0};
    app_mechanism_command_t mechanismCommand = {0};
    uint32_t nextWake;
    uint32_t periodTicks;
    uint32_t systemEvents;

    (void)argument;
    if (!AppRtosEvent_WaitForInitialization())
    {
        osThreadExit();
    }
    nextWake = osKernelGetTickCount();
    periodTicks = AppRtos_MsToTicks(APP_CONTROL_PERIOD_MS);
    for (;;)
    {
        while (osMessageQueueGet(appRtosChassisCommandQueue, &chassisCommand, NULL, 0U) == osOK) { }
        while (osMessageQueueGet(appRtosMechanismCommandQueue, &mechanismCommand, NULL, 0U) == osOK) { }
        systemEvents = AppRtos_GetEvents();
        AppChassis_ControlStep(&chassisCommand, systemEvents);
        AppClimb_ControlStep(&mechanismCommand, systemEvents);
        appRtosRuntimeStats.control_cycle_count++;
        nextWake += periodTicks;
        if ((int32_t)(osKernelGetTickCount() - nextWake) >= 0)
        {
            appRtosRuntimeStats.control_overrun_count++;
        }
        osDelayUntil(nextWake);
    }
}

static void AppTask_Mechanism(void *argument)
{
    app_action_command_t command = {0};
    uint32_t nextWake;

    (void)argument;
    if (!AppRtosEvent_WaitForInitialization())
    {
        osThreadExit();
    }
    nextWake = osKernelGetTickCount();
    for (;;)
    {
        bool available = osMessageQueueGet(appRtosActionQueue, &command, NULL, 0U) == osOK;
        AppClimb_ActionStep(&command, available, AppRtos_GetEvents());
        nextWake += AppRtos_MsToTicks(APP_MECHANISM_PERIOD_MS);
        osDelayUntil(nextWake);
    }
}

static void AppTask_Sensor(void *argument)
{
    app_imu_sample_t imuSample;
    app_dt35_sample_t dt35Sample;

    (void)argument;
    if (!AppRtosEvent_WaitForInitialization())
    {
        osThreadExit();
    }
    for (;;)
    {
        uint32_t flags = osThreadFlagsWait(APP_SENSOR_FLAG_IMU | APP_SENSOR_FLAG_DT35, osFlagsWaitAny, osWaitForever);
        if ((flags & osFlagsError) != 0U)
        {
            continue;
        }
        if ((flags & APP_SENSOR_FLAG_IMU) != 0U)
        {
            while (osMessageQueueGet(appRtosImuQueue, &imuSample, NULL, 0U) == osOK)
            {
                appRtosRuntimeStats.imu_rx_count++;
                AppSensors_UpdateImu(&imuSample);
            }
        }
        if ((flags & APP_SENSOR_FLAG_DT35) != 0U)
        {
            while (osMessageQueueGet(appRtosDt35Queue, &dt35Sample, NULL, 0U) == osOK)
            {
                appRtosRuntimeStats.dt35_rx_count++;
                AppSensors_UpdateDt35(&dt35Sample);
            }
        }
    }
}

static void AppTask_Navigation(void *argument)
{
    PathPoint point = {0};
    uint32_t nextWake;

    (void)argument;
    if (!AppRtosEvent_WaitForInitialization())
    {
        osThreadExit();
    }
    nextWake = osKernelGetTickCount();
    for (;;)
    {
        bool available = osMessageQueueGet(appRtosTrajectoryQueue, &point, NULL, 0U) == osOK;
        AppNavigation_Step(&point, available);
        nextWake += AppRtos_MsToTicks(APP_NAVIGATION_PERIOD_MS);
        osDelayUntil(nextWake);
    }
}

static void AppTask_Communication(void *argument)
{
    app_comm_packet_t packet;

    (void)argument;
    if (!AppRtosEvent_WaitForInitialization())
    {
        osThreadExit();
    }
    for (;;)
    {
        if (osMessageQueueGet(appRtosCommunicationQueue, &packet, NULL, osWaitForever) == osOK)
        {
            appRtosRuntimeStats.comm_rx_count++;
            AppCommunication_OnPacket(&packet);
        }
    }
}

static void AppTask_Safety(void *argument)
{
    bool initialized;
    uint32_t nextWake;
    uint32_t initFlags;
    uint32_t systemEvents;

    (void)argument;
    initFlags = osEventFlagsWait(appRtosSystemEvents, APP_EVENT_INIT_DONE | APP_EVENT_INIT_FAILED,
                                 osFlagsWaitAny | osFlagsNoClear, osWaitForever);
    initialized = ((initFlags & osFlagsError) == 0U) && ((initFlags & APP_EVENT_INIT_DONE) != 0U);
    nextWake = osKernelGetTickCount();
    for (;;)
    {
        if ((initialized && !BspBoardCan_ProcessAll()) || (appRtosCanErrorMask != 0U))
        {
            AppRtos_SetEvents(APP_EVENT_CAN_FAULT | APP_EVENT_ESTOP);
        }
        systemEvents = AppRtos_GetEvents();
        AppSafety_Step(systemEvents);
        nextWake += AppRtos_MsToTicks(APP_SAFETY_PERIOD_MS);
        osDelayUntil(nextWake);
    }
}

static void AppTask_Monitor(void *argument)
{
    app_runtime_stats_t stats;
    app_runtime_stats_t discarded;
    uint32_t nextWake;

    (void)argument;
    nextWake = osKernelGetTickCount();
    for (;;)
    {
        AppRtos_GetRuntimeStats(&stats);
        if (osMessageQueuePut(appRtosTelemetryQueue, &stats, 0U, 0U) != osOK)
        {
            osMessageQueueGet(appRtosTelemetryQueue, &discarded, NULL, 0U);
            osMessageQueuePut(appRtosTelemetryQueue, &stats, 0U, 0U);
        }
        AppMonitor_Step(&stats);
        nextWake += AppRtos_MsToTicks(APP_MONITOR_PERIOD_MS);
        osDelayUntil(nextWake);
    }
}
