#include "app_rtos_state.h"

#include <string.h>


static app_real_world_t appRtosRobotState;
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

bool AppRtos_PublishRobotState(const app_real_world_t *state, uint32_t timeoutMs)
{
    if ((state == NULL) || (appRtosRobotStateMutex == NULL)
        || (osMutexAcquire(appRtosRobotStateMutex, AppRtos_MsToTicks(timeoutMs)) != osOK))
    {
        return false;
    }
    appRtosRobotState = *state;
    return osMutexRelease(appRtosRobotStateMutex) == osOK;
}

bool AppRtos_GetRobotState(app_real_world_t *state, uint32_t timeoutMs)
{
    if ((state == NULL) || (appRtosRobotStateMutex == NULL)
        || (osMutexAcquire(appRtosRobotStateMutex, AppRtos_MsToTicks(timeoutMs)) != osOK))
    {
        return false;
    }
    *state = appRtosRobotState;
    return osMutexRelease(appRtosRobotStateMutex) == osOK;
}

bool AppRtos_UpdateYaw(const app_real_world_t *state)
{
    if ((state == NULL) || (appRtosRobotStateMutex == NULL)
        || (osMutexAcquire(appRtosRobotStateMutex, 0U) != osOK))
    {
        return false;
    }
    appRtosRobotState.yaw_deg = state->yaw_deg;
    appRtosRobotState.imu_yaw_deg = state->imu_yaw_deg;
    appRtosRobotState.chassis_yaw_deg = state->chassis_yaw_deg;
    appRtosRobotState.fused_yaw_deg = state->fused_yaw_deg;
    appRtosRobotState.fused_yaw_rate_radps = state->fused_yaw_rate_radps;
    appRtosRobotState.yaw_rate_radps = state->yaw_rate_radps;
    appRtosRobotState.timestamp_ms = state->timestamp_ms;
    return osMutexRelease(appRtosRobotStateMutex) == osOK;
}

void AppRtos_UpdateYawFromISR(const app_real_world_t *state)
{
    if (state == NULL)
    {
        return;
    }
    appRtosRobotState.yaw_deg = state->yaw_deg;
    appRtosRobotState.imu_yaw_deg = state->imu_yaw_deg;
    appRtosRobotState.chassis_yaw_deg = state->chassis_yaw_deg;
    appRtosRobotState.fused_yaw_deg = state->fused_yaw_deg;
    appRtosRobotState.fused_yaw_rate_radps = state->fused_yaw_rate_radps;
    appRtosRobotState.yaw_rate_radps = state->yaw_rate_radps;
    appRtosRobotState.timestamp_ms = state->timestamp_ms;
}
