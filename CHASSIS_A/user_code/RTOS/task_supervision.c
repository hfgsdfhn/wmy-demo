#include "app_rtos_internal.h"

#include "app_monitor.h"
#include "app_safety.h"
#include "bsp_board.h"
#include "control_config.h"

void AppTask_Safety(void *argument)
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
            (void)AppRtos_SetEvents(APP_EVENT_CAN_FAULT | APP_EVENT_ESTOP);
        }
        system_events = AppRtos_GetEvents();
        AppSafety_Step(system_events);

        next_wake += AppRtos_MsToTicks(APP_SAFETY_PERIOD_MS);
        (void)osDelayUntil(next_wake);
    }
}

void AppTask_Monitor(void *argument)
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
            (void)osMessageQueueGet(appRtosTelemetryQueue, &discarded,
                                    NULL, 0U);
            (void)osMessageQueuePut(appRtosTelemetryQueue, &stats, 0U, 0U);
        }
        AppMonitor_Step(&stats);

        next_wake += AppRtos_MsToTicks(APP_MONITOR_PERIOD_MS);
        (void)osDelayUntil(next_wake);
    }
}
