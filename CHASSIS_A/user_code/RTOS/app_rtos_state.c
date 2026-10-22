#include "app_rtos_state.h"

#include <string.h>

#include "app_rtos_queue.h"

static app_robot_state_t appRtosRobotState;
static osMutexId_t appRtosRobotStateMutex;

static const osMutexAttr_t appRtosStateMutexAttributes =
{
    .name = "robotStateMutex",
    .attr_bits = osMutexPrioInherit
};

bool AppRtos_StateInit(void)
{
    memset(&appRtosRobotState, 0, sizeof(appRtosRobotState));
    appRtosRobotStateMutex = osMutexNew(&appRtosStateMutexAttributes);
    return appRtosRobotStateMutex != NULL;
}

bool AppRtos_PublishRobotState(const app_robot_state_t *state, uint32_t timeoutMs)
{
    if ((state == NULL) || (appRtosRobotStateMutex == NULL)
        || (osMutexAcquire(appRtosRobotStateMutex, AppRtos_MsToTicks(timeoutMs)) != osOK))
    {
        return false;
    }
    appRtosRobotState = *state;
    return osMutexRelease(appRtosRobotStateMutex) == osOK;
}

bool AppRtos_GetRobotState(app_robot_state_t *state, uint32_t timeoutMs)
{
    if ((state == NULL) || (appRtosRobotStateMutex == NULL)
        || (osMutexAcquire(appRtosRobotStateMutex, AppRtos_MsToTicks(timeoutMs)) != osOK))
    {
        return false;
    }
    *state = appRtosRobotState;
    return osMutexRelease(appRtosRobotStateMutex) == osOK;
}
