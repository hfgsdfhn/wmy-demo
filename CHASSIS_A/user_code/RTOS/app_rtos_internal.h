#ifndef APP_RTOS_INTERNAL_H
#define APP_RTOS_INTERNAL_H

#include <stdbool.h>
#include <stdint.h>

#include "app_rtos.h"
#include "cmsis_os2.h"

#define APP_SENSOR_FLAG_IMU              (1UL << 0)
#define APP_SENSOR_FLAG_DT35             (1UL << 1)

extern osMessageQueueId_t appRtosCanRxQueue;
extern osMessageQueueId_t appRtosChassisCommandQueue;
extern osMessageQueueId_t appRtosMechanismCommandQueue;
extern osMessageQueueId_t appRtosTrajectoryQueue;
extern osMessageQueueId_t appRtosActionQueue;
extern osMessageQueueId_t appRtosImuQueue;
extern osMessageQueueId_t appRtosDt35Queue;
extern osMessageQueueId_t appRtosCommunicationQueue;
extern osMessageQueueId_t appRtosTelemetryQueue;
extern osEventFlagsId_t appRtosSystemEvents;

extern app_runtime_stats_t appRtosRuntimeStats;
extern volatile uint32_t appRtosCanErrorMask;

uint32_t AppRtos_MsToTicks(uint32_t milliseconds);
bool AppRtos_WaitForInitialization(void);

void AppTask_CanRx(void *argument);
void AppTask_Control(void *argument);
void AppTask_Mechanism(void *argument);
void AppTask_Sensor(void *argument);
void AppTask_Communication(void *argument);
void AppTask_Navigation(void *argument);
void AppTask_Safety(void *argument);
void AppTask_Monitor(void *argument);

#endif /* APP_RTOS_INTERNAL_H */
