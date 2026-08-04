#include "app_rtos.h"

#include <stddef.h>
#include <string.h>

#include "FreeRTOS.h"
#include "app_init.h"
#include "app_can_router.h"
#include "app_chassis.h"
#include "app_climb.h"
#include "app_communication.h"
#include "app_monitor.h"
#include "app_navigation.h"
#include "app_safety.h"
#include "app_sensors.h"
#include "bsp_board.h"
#include "bsp_can.h"
#include "cmsis_compiler.h"
#include "cmsis_os2.h"
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
#define APP_SENSOR_FLAG_IMU                (1UL << 0)
#define APP_SENSOR_FLAG_DT35               (1UL << 1)

static osMessageQueueId_t appRtosCanRxQueue;
static osMessageQueueId_t appRtosChassisCommandQueue;
static osMessageQueueId_t appRtosMechanismCommandQueue;
static osMessageQueueId_t appRtosTrajectoryQueue;
static osMessageQueueId_t appRtosActionQueue;
static osMessageQueueId_t appRtosImuQueue;
static osMessageQueueId_t appRtosDt35Queue;
static osMessageQueueId_t appRtosCommunicationQueue;
static osMessageQueueId_t appRtosTelemetryQueue;
static osEventFlagsId_t appRtosSystemEvents;

static osMutexId_t app_rtos_robot_state_mutex;
static osThreadId_t app_rtos_can_rx_task;
static osThreadId_t app_rtos_control_task;
static osThreadId_t app_rtos_mechanism_task;
static osThreadId_t app_rtos_sensor_task;
static osThreadId_t app_rtos_navigation_task;
static osThreadId_t app_rtos_communication_task;
static osThreadId_t app_rtos_safety_task;
static osThreadId_t app_rtos_monitor_task;

static app_runtime_stats_t appRtosRuntimeStats;
static volatile uint32_t appRtosCanErrorMask;

volatile app_rtos_fault_t appRtosFaultCode = APP_RTOS_FAULT_NONE;
volatile uint32_t appRtosFaultLine;
const char * volatile appRtosFaultFile;

static app_robot_state_t app_rtos_robot_state;

static uint32_t AppRtos_MsToTicks(uint32_t milliseconds);
static bool AppRtos_WaitForInitialization(void);
static void AppTask_CanRx(void *argument);
static void AppTask_Control(void *argument);
static void AppTask_Mechanism(void *argument);
static void AppTask_Sensor(void *argument);
static void AppTask_Communication(void *argument);
static void AppTask_Navigation(void *argument);
static void AppTask_Safety(void *argument);
static void AppTask_Monitor(void *argument);

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

/* 将毫秒数换算为系统 tick 数。 */
static uint32_t AppRtos_MsToTicks(uint32_t milliseconds)
{
    uint64_t ticks;

    ticks = ((uint64_t)milliseconds * osKernelGetTickFreq() + 999U) / 1000U;
    if ((milliseconds > 0U) && (ticks == 0U))
    {
        ticks = 1U;
    }
    return (uint32_t)ticks;
}

/* 等待系统初始化完成或初始化失败事件。 */
static bool AppRtos_WaitForInitialization(void)
{
    uint32_t flags;

    flags = osEventFlagsWait(appRtosSystemEvents,
                             APP_EVENT_INIT_DONE | APP_EVENT_INIT_FAILED,
                             osFlagsWaitAny | osFlagsNoClear, osWaitForever);
    return ((flags & osFlagsError) == 0U)
        && ((flags & APP_EVENT_INIT_DONE) != 0U);
}

/* 将消息写入队列，必要时丢弃旧消息以保证最新状态生效。 */
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

/* 从 CAN 驱动对象中提取应用层总线编号。 */
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

/* 将中断接收到的 CAN 帧封装后送入 RTOS 接收队列。 */
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

/* 记录 CAN 总线错误状态，供安全任务和控制任务使用。 */
static void AppRtos_CanErrorFromISR(bsp_can_t *can, uint32_t error_flags)
{
    app_can_bus_t bus;

    (void)error_flags;
    if (AppRtos_GetCanBus(can, &bus))
    {
        appRtosCanErrorMask |= 1UL << (uint32_t)bus;
    }
}

/* 初始化 RTOS 相关的队列、互斥量、事件标志和各个任务。 */
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
        APP_TRAJECTORY_QUEUE_LENGTH, sizeof(PathPoint),
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

/* 执行应用初始化并发布系统启动完成或失败事件。 */
void AppRtos_Bootstrap(void)
{
    if (AppInit_Run(AppRtos_CanRxFromISR, AppRtos_CanErrorFromISR))
    {
        osEventFlagsSet(appRtosSystemEvents,
                        APP_EVENT_ALL_CAN_READY | APP_EVENT_INIT_DONE);
    }
    else
    {
        osEventFlagsSet(appRtosSystemEvents,
                        APP_EVENT_INIT_FAILED | APP_EVENT_ESTOP);
    }
}

/* 提交底盘控制命令到控制任务队列。 */
bool AppRtos_SubmitChassisCommand(const app_chassis_command_t *command,
                                  uint32_t timeout_ms)
{
    app_chassis_command_t discarded;

    return (command != NULL) && (appRtosChassisCommandQueue != NULL)
        && AppRtos_PutLatest(appRtosChassisCommandQueue, command, &discarded,
                             timeout_ms, NULL);
}

/* 提交机构控制命令到机构任务队列。 */
bool AppRtos_SubmitMechanismCommand(
    const app_mechanism_command_t *command, uint32_t timeout_ms)
{
    app_mechanism_command_t discarded;

    return (command != NULL) && (appRtosMechanismCommandQueue != NULL)
        && AppRtos_PutLatest(appRtosMechanismCommandQueue, command,
                             &discarded, timeout_ms, NULL);
}

/* 提交一条轨迹点到导航任务队列。 */
bool AppRtos_SubmitTrajectoryPoint(const PathPoint *point,
                                   uint32_t timeout_ms)
{
    return (point != NULL) && (appRtosTrajectoryQueue != NULL)
        && (osMessageQueuePut(appRtosTrajectoryQueue, point, 0U,
                              AppRtos_MsToTicks(timeout_ms)) == osOK);
}

/* 提交动作命令到动作执行任务队列。 */
bool AppRtos_SubmitActionCommand(const app_action_command_t *command,
                                 uint32_t timeout_ms)
{
    return (command != NULL) && (appRtosActionQueue != NULL)
        && (osMessageQueuePut(appRtosActionQueue, command, 0U,
                              AppRtos_MsToTicks(timeout_ms)) == osOK);
}

/* 提交 IMU 采样数据到传感器处理任务。 */
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

/* 提交 DT35 采样数据到传感器处理任务。 */
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

/* 提交通信报文到通信任务队列。 */
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

/* 发布当前机器人状态，使用互斥锁保证状态一致性。 */
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

/* 读取当前机器人状态。 */
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

/* 拷贝当前运行时统计信息，并补充系统事件和 CAN 错误掩码。 */
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

/* 设置系统事件标志。 */
bool AppRtos_SetEvents(uint32_t flags)
{
    return (appRtosSystemEvents != NULL)
        && ((osEventFlagsSet(appRtosSystemEvents, flags) & osFlagsError) == 0U);
}

/* 清除指定的系统事件标志。 */
bool AppRtos_ClearEvents(uint32_t flags)
{
    return (appRtosSystemEvents != NULL)
        && ((osEventFlagsClear(appRtosSystemEvents, flags) & osFlagsError)
            == 0U);
}

/* 获取当前系统事件标志。 */
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

/* 记录断言失败位置，随后触发 RTOS 故障处理。 */
void AppRtos_Assert(const char *file, uint32_t line)
{
    appRtosFaultFile = file;
    appRtosFaultLine = line;
    AppRtos_Panic(APP_RTOS_FAULT_ASSERT);
}

/* 进入不可恢复故障状态，阻塞当前线程。 */
void AppRtos_Panic(app_rtos_fault_t fault)
{
    appRtosFaultCode = fault;
    __disable_irq();
    for (;;)
    {
        __NOP();
    }
}

/* CAN 接收任务，持续从队列读取并分发 CAN 帧。 */
static void AppTask_CanRx(void *argument)
{
    app_can_frame_t frame;

    (void)argument;
    if (!AppRtos_WaitForInitialization())
    {
        osThreadExit();
    }

    for (;;)
    {
        if (osMessageQueueGet(appRtosCanRxQueue, &frame, NULL,
                              osWaitForever) == osOK)
        {
            appRtosRuntimeStats.can_rx_count++;
            AppCanRouter_Dispatch(&frame);
        }
    }
}

/* 控制任务，周期性执行底盘和机构控制逻辑。 */
static void AppTask_Control(void *argument)
{
    app_chassis_command_t chassis_command = {0};
    app_mechanism_command_t mechanism_command = {0};
    uint32_t next_wake;
    uint32_t period_ticks;
    uint32_t system_events;

    (void)argument;
    if (!AppRtos_WaitForInitialization())
    {
        osThreadExit();
    }

    next_wake = osKernelGetTickCount();
    period_ticks = AppRtos_MsToTicks(APP_CONTROL_PERIOD_MS);
    for (;;)
    {
        while (osMessageQueueGet(appRtosChassisCommandQueue,
                                 &chassis_command, NULL, 0U) == osOK)
        {
        }
        while (osMessageQueueGet(appRtosMechanismCommandQueue,
                                 &mechanism_command, NULL, 0U) == osOK)
        {
        }

        system_events = AppRtos_GetEvents();
        AppChassis_ControlStep(&chassis_command, system_events);
        AppClimb_ControlStep(&mechanism_command, system_events);
        appRtosRuntimeStats.control_cycle_count++;

        next_wake += period_ticks;
        if ((int32_t)(osKernelGetTickCount() - next_wake) >= 0)
        {
            appRtosRuntimeStats.control_overrun_count++;
        }
        osDelayUntil(next_wake);
    }
}

/* 机构任务，周期性执行动作命令处理。 */
static void AppTask_Mechanism(void *argument)
{
    app_action_command_t command = {0};
    bool command_available;
    uint32_t next_wake;

    (void)argument;
    if (!AppRtos_WaitForInitialization())
    {
        osThreadExit();
    }

    next_wake = osKernelGetTickCount();
    for (;;)
    {
        command_available = osMessageQueueGet(appRtosActionQueue, &command,
                                              NULL, 0U) == osOK;
        AppClimb_ActionStep(&command, command_available, AppRtos_GetEvents());

        next_wake += AppRtos_MsToTicks(APP_MECHANISM_PERIOD_MS);
        osDelayUntil(next_wake);
    }
}

/* 传感器任务，处理 IMU 和 DT35 的采样数据。 */
static void AppTask_Sensor(void *argument)
{
    app_imu_sample_t imu_sample;
    app_dt35_sample_t dt35_sample;
    uint32_t flags;

    (void)argument;
    if (!AppRtos_WaitForInitialization())
    {
        osThreadExit();
    }

    for (;;)
    {
        flags = osThreadFlagsWait(APP_SENSOR_FLAG_IMU | APP_SENSOR_FLAG_DT35,
                                  osFlagsWaitAny, osWaitForever);
        if ((flags & osFlagsError) != 0U)
        {
            continue;
        }

        if ((flags & APP_SENSOR_FLAG_IMU) != 0U)
        {
            while (osMessageQueueGet(appRtosImuQueue, &imu_sample,
                                     NULL, 0U) == osOK)
            {
                appRtosRuntimeStats.imu_rx_count++;
                AppSensors_UpdateImu(&imu_sample);
            }
        }
        if ((flags & APP_SENSOR_FLAG_DT35) != 0U)
        {
            while (osMessageQueueGet(appRtosDt35Queue, &dt35_sample,
                                     NULL, 0U) == osOK)
            {
                appRtosRuntimeStats.dt35_rx_count++;
                AppSensors_UpdateDt35(&dt35_sample);
            }
        }
    }
}

/* 导航任务，周期性读取轨迹点并更新导航目标。 */
static void AppTask_Navigation(void *argument)
{
    PathPoint point = {0};
    bool point_available;
    uint32_t next_wake;

    (void)argument;
    if (!AppRtos_WaitForInitialization())
    {
        osThreadExit();
    }

    next_wake = osKernelGetTickCount();
    for (;;)
    {
        point_available = osMessageQueueGet(appRtosTrajectoryQueue, &point,
                                            NULL, 0U) == osOK;
        AppNavigation_Step(&point, point_available);

        next_wake += AppRtos_MsToTicks(APP_NAVIGATION_PERIOD_MS);
        osDelayUntil(next_wake);
    }
}

/* 通信任务，持续处理通信队列中的报文。 */
static void AppTask_Communication(void *argument)
{
    app_comm_packet_t packet;

    (void)argument;
    if (!AppRtos_WaitForInitialization())
    {
        osThreadExit();
    }

    for (;;)
    {
        if (osMessageQueueGet(appRtosCommunicationQueue, &packet, NULL,
                              osWaitForever) == osOK)
        {
            appRtosRuntimeStats.comm_rx_count++;
            AppCommunication_OnPacket(&packet);
        }
    }
}

/* 安全任务，监视 CAN 状态并触发保护事件。 */
static void AppTask_Safety(void *argument)
{
    bool initialized;
    uint32_t init_flags;
    uint32_t next_wake;
    uint32_t system_events;

    (void)argument;
    init_flags = osEventFlagsWait(appRtosSystemEvents,
                                  APP_EVENT_INIT_DONE | APP_EVENT_INIT_FAILED,
                                  osFlagsWaitAny | osFlagsNoClear,
                                  osWaitForever);
    initialized = ((init_flags & osFlagsError) == 0U)
        && ((init_flags & APP_EVENT_INIT_DONE) != 0U);
    next_wake = osKernelGetTickCount();
    for (;;)
    {
        if ((initialized && !BspBoardCan_ProcessAll())
            || (appRtosCanErrorMask != 0U))
        {
            AppRtos_SetEvents(APP_EVENT_CAN_FAULT | APP_EVENT_ESTOP);
        }
        system_events = AppRtos_GetEvents();
        AppSafety_Step(system_events);

        next_wake += AppRtos_MsToTicks(APP_SAFETY_PERIOD_MS);
        osDelayUntil(next_wake);
    }
}

/* 监控任务，周期性上报运行时统计信息。 */
static void AppTask_Monitor(void *argument)
{
    app_runtime_stats_t stats;
    app_runtime_stats_t discarded;
    uint32_t next_wake;

    (void)argument;
    next_wake = osKernelGetTickCount();
    for (;;)
    {
        AppRtos_GetRuntimeStats(&stats);
        if (osMessageQueuePut(appRtosTelemetryQueue, &stats, 0U, 0U)
            != osOK)
        {
            osMessageQueueGet(appRtosTelemetryQueue, &discarded,
                              NULL, 0U);
            osMessageQueuePut(appRtosTelemetryQueue, &stats, 0U, 0U);
        }
        AppMonitor_Step(&stats);

        next_wake += AppRtos_MsToTicks(APP_MONITOR_PERIOD_MS);
        osDelayUntil(next_wake);
    }
}
