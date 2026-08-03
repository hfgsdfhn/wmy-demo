#include "app_rtos_internal.h"

#include "app_chassis.h"
#include "app_climb.h"
#include "control_config.h"

void AppTask_Control(void *argument)
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
        (void)osDelayUntil(next_wake);
    }
}

void AppTask_Mechanism(void *argument)
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
        (void)osDelayUntil(next_wake);
    }
}
