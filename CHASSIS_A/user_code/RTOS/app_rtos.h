#ifndef APP_RTOS_H
#define APP_RTOS_H

#include <stdbool.h>
#include <stdint.h>

#include "app_messages.h"
#include "app_state.h"

typedef enum
{
    APP_RTOS_FAULT_NONE = 0,
    APP_RTOS_FAULT_OBJECT_CREATE,
    APP_RTOS_FAULT_STACK_OVERFLOW,
    APP_RTOS_FAULT_MALLOC_FAILED,
    APP_RTOS_FAULT_ASSERT
} app_rtos_fault_t;

extern volatile app_rtos_fault_t appRtosFaultCode;
extern volatile uint32_t appRtosFaultLine;
extern const char * volatile appRtosFaultFile;

bool AppRtos_Init(void);
void AppRtos_Bootstrap(void);

bool AppRtos_SubmitChassisCommand(const app_chassis_command_t *command,
                                  uint32_t timeout_ms);
bool AppRtos_SubmitMechanismCommand(
    const app_mechanism_command_t *command, uint32_t timeout_ms);
bool AppRtos_SubmitTrajectoryPoint(const app_trajectory_point_t *point,
                                   uint32_t timeout_ms);
bool AppRtos_SubmitActionCommand(const app_action_command_t *command,
                                 uint32_t timeout_ms);
bool AppRtos_SubmitImuSample(const app_imu_sample_t *sample,
                             uint32_t timeout_ms);
bool AppRtos_SubmitDt35Sample(const app_dt35_sample_t *sample,
                              uint32_t timeout_ms);
bool AppRtos_SubmitCommunicationPacket(const app_comm_packet_t *packet,
                                       uint32_t timeout_ms);

bool AppRtos_PublishRobotState(const app_robot_state_t *state,
                               uint32_t timeout_ms);
bool AppRtos_GetRobotState(app_robot_state_t *state, uint32_t timeout_ms);
void AppRtos_GetRuntimeStats(app_runtime_stats_t *stats);

bool AppRtos_SetEvents(uint32_t flags);
bool AppRtos_ClearEvents(uint32_t flags);
uint32_t AppRtos_GetEvents(void);

void AppRtos_Assert(const char *file, uint32_t line);
void AppRtos_Panic(app_rtos_fault_t fault);

#endif /* APP_RTOS_H */
