#ifndef APP_RTOS_QUEUE_H
#define APP_RTOS_QUEUE_H

#include "app_rtos.h"
#include "bsp_can.h"
#include "cmsis_os2.h"

#define APP_SENSOR_FLAG_IMU   (1UL << 0)
#define APP_SENSOR_FLAG_DT35  (1UL << 1)

extern osMessageQueueId_t appRtosCanRxQueue;
extern osMessageQueueId_t appRtosChassisCommandQueue;
extern osMessageQueueId_t appRtosMechanismCommandQueue;
extern osMessageQueueId_t appRtosTrajectoryQueue;
extern osMessageQueueId_t appRtosActionQueue;
extern osMessageQueueId_t appRtosImuQueue;
extern osMessageQueueId_t appRtosDt35Queue;
extern osMessageQueueId_t appRtosCommunicationQueue;
extern osMessageQueueId_t appRtosTelemetryQueue;
extern volatile uint32_t appRtosCanErrorMask;

bool AppRtos_QueueInit(void);
uint32_t AppRtos_MsToTicks(uint32_t milliseconds);
bool AppRtos_PutLatest(osMessageQueueId_t queue, const void *item, void *discarded,
                       uint32_t timeoutMs, bool *discardedOldest);
void AppRtos_CanRxFromISR(bsp_can_t *can, uint32_t canId, bool isExtendedId,
                          const uint8_t *data, uint8_t length);
void AppRtos_CanErrorFromISR(bsp_can_t *can, uint32_t errorFlags);

#endif /* APP_RTOS_QUEUE_H */
