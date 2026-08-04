#include "app_rtos_queue.h"

#include <stddef.h>
#include <string.h>

#include "FreeRTOS.h"
#include "app_rtos_task.h"
#include "bsp_can.h"
#include "task.h"

#define APP_CAN_RX_QUEUE_LENGTH       64U
#define APP_COMMAND_QUEUE_LENGTH        4U
#define APP_ACTION_QUEUE_LENGTH         8U
#define APP_IMU_QUEUE_LENGTH            4U
#define APP_DT35_QUEUE_LENGTH           2U
#define APP_COMM_QUEUE_LENGTH          16U
#define APP_TELEMETRY_QUEUE_LENGTH      4U
#define APP_TRAJECTORY_QUEUE_LENGTH    32U

osMessageQueueId_t appRtosCanRxQueue;
osMessageQueueId_t appRtosChassisCommandQueue;
osMessageQueueId_t appRtosMechanismCommandQueue;
osMessageQueueId_t appRtosTrajectoryQueue;
osMessageQueueId_t appRtosActionQueue;
osMessageQueueId_t appRtosImuQueue;
osMessageQueueId_t appRtosDt35Queue;
osMessageQueueId_t appRtosCommunicationQueue;
osMessageQueueId_t appRtosTelemetryQueue;
volatile uint32_t appRtosCanErrorMask;

static const osMessageQueueAttr_t appRtosCanRxQueueAttributes = { .name = "canRxQueue" };
static const osMessageQueueAttr_t appRtosChassisQueueAttributes = { .name = "chassisCmdQueue" };
static const osMessageQueueAttr_t appRtosMechanismQueueAttributes = { .name = "mechanismCmdQueue" };
static const osMessageQueueAttr_t appRtosTrajectoryQueueAttributes = { .name = "trajectoryQueue" };
static const osMessageQueueAttr_t appRtosActionQueueAttributes = { .name = "actionQueue" };
static const osMessageQueueAttr_t appRtosImuQueueAttributes = { .name = "imuQueue" };
static const osMessageQueueAttr_t appRtosDt35QueueAttributes = { .name = "dt35Queue" };
static const osMessageQueueAttr_t appRtosCommunicationQueueAttributes = { .name = "communicationQueue" };
static const osMessageQueueAttr_t appRtosTelemetryQueueAttributes = { .name = "telemetryQueue" };

uint32_t AppRtos_MsToTicks(uint32_t milliseconds)
{
    uint64_t ticks = ((uint64_t)milliseconds * osKernelGetTickFreq() + 999U) / 1000U;

    if ((milliseconds > 0U) && (ticks == 0U))
    {
        ticks = 1U;
    }
    return (uint32_t)ticks;
}

bool AppRtos_QueueInit(void)
{
    appRtosCanErrorMask = 0U;
    appRtosCanRxQueue = osMessageQueueNew(APP_CAN_RX_QUEUE_LENGTH, sizeof(app_can_frame_t), &appRtosCanRxQueueAttributes);
    appRtosChassisCommandQueue = osMessageQueueNew(APP_COMMAND_QUEUE_LENGTH, sizeof(app_chassis_command_t), &appRtosChassisQueueAttributes);
    appRtosMechanismCommandQueue = osMessageQueueNew(APP_COMMAND_QUEUE_LENGTH, sizeof(app_mechanism_command_t), &appRtosMechanismQueueAttributes);
    appRtosTrajectoryQueue = osMessageQueueNew(APP_TRAJECTORY_QUEUE_LENGTH, sizeof(PathPoint), &appRtosTrajectoryQueueAttributes);
    appRtosActionQueue = osMessageQueueNew(APP_ACTION_QUEUE_LENGTH, sizeof(app_action_command_t), &appRtosActionQueueAttributes);
    appRtosImuQueue = osMessageQueueNew(APP_IMU_QUEUE_LENGTH, sizeof(app_imu_sample_t), &appRtosImuQueueAttributes);
    appRtosDt35Queue = osMessageQueueNew(APP_DT35_QUEUE_LENGTH, sizeof(app_dt35_sample_t), &appRtosDt35QueueAttributes);
    appRtosCommunicationQueue = osMessageQueueNew(APP_COMM_QUEUE_LENGTH, sizeof(app_comm_packet_t), &appRtosCommunicationQueueAttributes);
    appRtosTelemetryQueue = osMessageQueueNew(APP_TELEMETRY_QUEUE_LENGTH, sizeof(app_runtime_stats_t), &appRtosTelemetryQueueAttributes);

    return (appRtosCanRxQueue != NULL) && (appRtosChassisCommandQueue != NULL)
        && (appRtosMechanismCommandQueue != NULL) && (appRtosTrajectoryQueue != NULL)
        && (appRtosActionQueue != NULL) && (appRtosImuQueue != NULL)
        && (appRtosDt35Queue != NULL) && (appRtosCommunicationQueue != NULL)
        && (appRtosTelemetryQueue != NULL);
}

bool AppRtos_PutLatest(osMessageQueueId_t queue, const void *item, void *discarded,
                       uint32_t timeoutMs, bool *discardedOldest)
{
    if (discardedOldest != NULL)
    {
        *discardedOldest = false;
    }
    if (osMessageQueuePut(queue, item, 0U, AppRtos_MsToTicks(timeoutMs)) == osOK)
    {
        return true;
    }
    if (osMessageQueueGet(queue, discarded, NULL, 0U) != osOK)
    {
        return false;
    }
    if (discardedOldest != NULL)
    {
        *discardedOldest = true;
    }
    return osMessageQueuePut(queue, item, 0U, 0U) == osOK;
}

bool AppRtos_SubmitChassisCommand(const app_chassis_command_t *command, uint32_t timeoutMs)
{
    app_chassis_command_t discarded;
    return (command != NULL) && (appRtosChassisCommandQueue != NULL)
        && AppRtos_PutLatest(appRtosChassisCommandQueue, command, &discarded, timeoutMs, NULL);
}

bool AppRtos_SubmitMechanismCommand(const app_mechanism_command_t *command, uint32_t timeoutMs)
{
    app_mechanism_command_t discarded;
    return (command != NULL) && (appRtosMechanismCommandQueue != NULL)
        && AppRtos_PutLatest(appRtosMechanismCommandQueue, command, &discarded, timeoutMs, NULL);
}

bool AppRtos_SubmitTrajectoryPoint(const PathPoint *point, uint32_t timeoutMs)
{
    return (point != NULL) && (appRtosTrajectoryQueue != NULL)
        && (osMessageQueuePut(appRtosTrajectoryQueue, point, 0U, AppRtos_MsToTicks(timeoutMs)) == osOK);
}

bool AppRtos_SubmitActionCommand(const app_action_command_t *command, uint32_t timeoutMs)
{
    return (command != NULL) && (appRtosActionQueue != NULL)
        && (osMessageQueuePut(appRtosActionQueue, command, 0U, AppRtos_MsToTicks(timeoutMs)) == osOK);
}

bool AppRtos_SubmitImuSample(const app_imu_sample_t *sample, uint32_t timeoutMs)
{
    app_imu_sample_t discarded;
    bool discardedOldest;

    if ((sample == NULL) || (appRtosImuQueue == NULL) || !AppRtos_SensorTaskReady())
    {
        return false;
    }
    if (!AppRtos_PutLatest(appRtosImuQueue, sample, &discarded, timeoutMs, &discardedOldest))
    {
        return false;
    }
    if (discardedOldest)
    {
        AppRtos_RecordImuDrop();
    }
    return AppRtos_NotifySensor(APP_SENSOR_FLAG_IMU);
}

bool AppRtos_SubmitDt35Sample(const app_dt35_sample_t *sample, uint32_t timeoutMs)
{
    app_dt35_sample_t discarded;
    bool discardedOldest;

    if ((sample == NULL) || (appRtosDt35Queue == NULL) || !AppRtos_SensorTaskReady())
    {
        return false;
    }
    if (!AppRtos_PutLatest(appRtosDt35Queue, sample, &discarded, timeoutMs, &discardedOldest))
    {
        return false;
    }
    if (discardedOldest)
    {
        AppRtos_RecordDt35Drop();
    }
    return AppRtos_NotifySensor(APP_SENSOR_FLAG_DT35);
}

bool AppRtos_SubmitCommunicationPacket(const app_comm_packet_t *packet, uint32_t timeoutMs)
{
    if ((packet == NULL) || (packet->length > APP_COMM_PAYLOAD_MAX_SIZE)
        || (appRtosCommunicationQueue == NULL))
    {
        return false;
    }
    if (osMessageQueuePut(appRtosCommunicationQueue, packet, 0U, AppRtos_MsToTicks(timeoutMs)) != osOK)
    {
        AppRtos_RecordCommunicationDrop();
        return false;
    }
    return true;
}

void AppRtos_CanRxFromISR(bsp_can_t *can, uint32_t canId, bool isExtendedId,
                          const uint8_t *data, uint8_t length)
{
    app_can_frame_t frame;
    TickType_t tickCount;

    if ((appRtosCanRxQueue == NULL) || (data == NULL) || (length > APP_CAN_DATA_MAX_SIZE)
        || (can == NULL) || (can->instance_index >= (uint8_t)APP_CAN_BUS_COUNT))
    {
        return;
    }
    tickCount = xTaskGetTickCountFromISR();
    frame.timestamp_ms = (uint32_t)(((uint64_t)tickCount * 1000U) / configTICK_RATE_HZ);
    frame.identifier = canId;
    frame.bus = (app_can_bus_t)can->instance_index;
    frame.is_extended_id = isExtendedId ? 1U : 0U;
    frame.length = length;
    memset(frame.data, 0, sizeof(frame.data));
    memcpy(frame.data, data, length);
    if (osMessageQueuePut(appRtosCanRxQueue, &frame, 0U, 0U) != osOK)
    {
        AppRtos_RecordCanRxDrop();
    }
}

void AppRtos_CanErrorFromISR(bsp_can_t *can, uint32_t errorFlags)
{
    (void)errorFlags;
    if ((can != NULL) && (can->instance_index < (uint8_t)APP_CAN_BUS_COUNT))
    {
        appRtosCanErrorMask |= 1UL << can->instance_index;
    }
}
