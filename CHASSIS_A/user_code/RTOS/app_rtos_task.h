#ifndef APP_RTOS_TASK_H
#define APP_RTOS_TASK_H

#include <stdbool.h>

#include <stdint.h>

#include "app_rtos.h"

bool AppRtos_TaskCreate(void);
bool AppRtos_SensorTaskReady(void);
bool AppRtos_NotifySensor(uint32_t flags);
void AppRtos_RecordCanRxDrop(void);
void AppRtos_RecordCommunicationDrop(void);
void AppRtos_RecordImuDrop(void);
void AppRtos_RecordDt35Drop(void);

#endif /* APP_RTOS_TASK_H */
