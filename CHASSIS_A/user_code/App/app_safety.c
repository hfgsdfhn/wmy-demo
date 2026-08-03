#include "app_safety.h"

#include "app_state.h"

static uint32_t app_safety_events;

bool AppSafety_Init(void)
{
    app_safety_events = 0U;
    return true;
}

void AppSafety_Step(uint32_t system_events)
{
    app_safety_events = system_events;
}

bool AppSafety_OutputAllowed(void)
{
    return (app_safety_events & (APP_EVENT_ESTOP | APP_EVENT_MOTOR_FAULT
                                 | APP_EVENT_CAN_FAULT)) == 0U;
}

uint32_t AppSafety_GetEvents(void)
{
    return app_safety_events;
}
