#ifndef APP_RTOS_EVENT_H
#define APP_RTOS_EVENT_H

#include <stdbool.h>
#include <stdint.h>

#include "app_rtos.h"
#include "cmsis_os2.h"

extern osEventFlagsId_t appRtosSystemEvents;

bool AppRtos_EventInit(void);
bool AppRtosEvent_WaitForInitialization(void);

#endif /* APP_RTOS_EVENT_H */
