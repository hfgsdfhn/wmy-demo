#ifndef APP_CHASSIS_H
#define APP_CHASSIS_H /* 底盘应用头文件保护宏 */

#include <stdbool.h>
#include <stdint.h>

#include "app_messages.h"
#include "app_monitor.h"
#include "bsp_can.h"

typedef struct
{
    uint32_t tx_ok_count;
    uint32_t tx_failed_count;
    float target_erpm[4];
    float measured_wheel_speed_mps[4];
    uint8_t feedback_valid_mask;
    uint32_t last_feedback_ms;
} app_chassis_diagnostics_t;

bool AppChassis_Init(bsp_can_t *can);
bool AppChassis_ProcessCan(void);
void AppChassis_ControlStep(const app_chassis_command_t *command,
                            uint32_t system_events);
bool AppChassis_RemoteControlToCommand(const app_dual_control_t *control,
                                       uint32_t timestamp_ms,
                                       app_chassis_command_t *command);
bool AppChassis_PowerBoardRemoteToCommand(const app_remote_control_t *control,
                                          uint32_t timestamp_ms,
                                          app_chassis_command_t *command);
bool AppChassis_OnCanFrame(const app_can_frame_t *frame);
bool AppChassis_GetCommand(app_chassis_command_t *command);
bool AppChassis_GetDiagnostics(app_chassis_diagnostics_t *diagnostics);
bool AppChassis_GetMeasuredVelocity(float *vx_mps, float *vy_mps,
                                    float *omega_radps, uint32_t timestamp_ms);
/* Converts motor eRPM to the calibrated motor angular rate in rad/s. */
float AppChassis_YawErpmToRadps(float wheel_erpm);
/* PID inputs and parameters use degrees; the returned chassis yaw rate is rad/s. */
bool AppChassis_ComputeYawRate(float target_yaw_deg, float current_yaw_deg,
                               const app_yaw_pid_params_t *params,
                               float *yaw_rate_radps);
void AppChassis_ResetYawPid(void);
void AppChassis_StopYawImmediately(void);

#endif /* APP_CHASSIS_H */
