#include "app_rtos_internal.h"

#include "app_navigation.h"
#include "control_config.h"

void AppTask_Navigation(void *argument)
{
    app_trajectory_point_t point = {0};
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
        (void)osDelayUntil(next_wake);
    }
}
