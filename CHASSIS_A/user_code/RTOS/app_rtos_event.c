#include "app_rtos_event.h"

#include "app_init.h"
#include "app_rtos_queue.h"
#include "cmsis_compiler.h"

osEventFlagsId_t appRtosSystemEvents;
volatile app_rtos_fault_t appRtosFaultCode = APP_RTOS_FAULT_NONE;
volatile uint32_t appRtosFaultLine;
const char * volatile appRtosFaultFile;

static const osEventFlagsAttr_t appRtosEventAttributes = { .name = "systemEvents" };

bool AppRtos_EventInit(void)
{
    appRtosSystemEvents = osEventFlagsNew(&appRtosEventAttributes);
    return appRtosSystemEvents != NULL;
}

void AppRtos_Bootstrap(void)
{
    if (AppInit_Run(AppRtos_CanRxFromISR, AppRtos_CanErrorFromISR))
    {
        AppRtos_SetEvents(APP_EVENT_ALL_CAN_READY | APP_EVENT_INIT_DONE);
    }
    else
    {
        AppRtos_SetEvents(APP_EVENT_INIT_FAILED | APP_EVENT_ESTOP);
    }
}

bool AppRtosEvent_WaitForInitialization(void)
{
    uint32_t flags = osEventFlagsWait(appRtosSystemEvents,
                                      APP_EVENT_INIT_DONE | APP_EVENT_INIT_FAILED,
                                      osFlagsWaitAny | osFlagsNoClear, osWaitForever);
    return ((flags & osFlagsError) == 0U) && ((flags & APP_EVENT_INIT_DONE) != 0U);
}

bool AppRtos_SetEvents(uint32_t flags)
{
    return (appRtosSystemEvents != NULL)
        && ((osEventFlagsSet(appRtosSystemEvents, flags) & osFlagsError) == 0U);
}

bool AppRtos_ClearEvents(uint32_t flags)
{
    return (appRtosSystemEvents != NULL)
        && ((osEventFlagsClear(appRtosSystemEvents, flags) & osFlagsError) == 0U);
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
