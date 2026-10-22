#ifndef APP_MONITOR_H
#define APP_MONITOR_H /* 应用监控模块头文件保护宏 */

#include <stdbool.h>
#include <stdint.h>

#include "bsp_uart.h"

#define APP_EVENT_INIT_DONE               (1UL << 0)  /* 系统初始化完成事件 */
#define APP_EVENT_INIT_FAILED             (1UL << 1)  /* 系统初始化失败事件 */
#define APP_EVENT_ESTOP                   (1UL << 2)  /* 急停事件 */
#define APP_EVENT_MOTOR_FAULT             (1UL << 3)  /* 电机故障事件 */
#define APP_EVENT_CAN1_READY              (1UL << 4)  /* CAN1 就绪事件 */
#define APP_EVENT_DT35_ONLINE             (1UL << 7)  /* DT35 在线事件 */
#define APP_EVENT_MAIN_CTRL_ONLINE        (1UL << 8)  /* 主控在线事件 */
#define APP_EVENT_USB_ONLINE              (1UL << 9)  /* USB 在线事件 */
#define APP_EVENT_IMU_ONLINE              (1UL << 10) /* IMU 在线事件 */
#define APP_EVENT_CAN_FAULT               (1UL << 11) /* CAN 故障事件 */
#define APP_EVENT_DUAL_CONTROL_ONLINE     (1UL << 12) /* 双 MCU 控制在线事件 */
#define APP_EVENT_ALL_CAN_READY           APP_EVENT_CAN1_READY /* 所有 CAN 就绪事件 */

typedef enum
{
    APP_PC_COMMAND_NONE = 0,
    APP_PC_COMMAND_START = 1,
    APP_PC_COMMAND_STOP = 2,
    APP_PC_COMMAND_RESET = 3,
    APP_PC_COMMAND_ESTOP = 4
} app_pc_command_t;

typedef struct
{
    float velocity_x_mps;
    float velocity_y_mps;
    float yaw_rate_radps;
    uint32_t timestamp_ms;
    app_pc_command_t command;
    uint8_t online;
} app_pc_control_t;

typedef struct
{
    float max_linear_speed_mps;
    float max_angular_speed_radps;
    float max_acceleration_mps2;
} app_motion_limits_t;

typedef struct
{
    bool enabled;                    /* 是否启用 YAW 角度外环 PID */
    float kp;                        /* 比例系数：输出 deg/s，输入角度误差 deg */
    float kd;                        /* 微分系数：输出 deg/s，输入角度变化率 deg/s */
    float ki;                        /* 积分系数：输出 deg/s，输入角度积分 deg*s */
    float integral_limit;            /* 积分项限幅，单位 deg*s */
    float integral_separation_deg;   /* 积分分离阈值，单位 deg */
    float max_output_degps;          /* PID 输出角速度限幅，单位 deg/s */
    float deadzone_deg;              /* 角度误差死区，单位 deg */
    float output_multiplier;         /* Final PID yaw-rate output scale */
} app_yaw_pid_params_t;

typedef enum
{
    APP_YAW_PID_PROFILE_90 = 0,
    APP_YAW_PID_PROFILE_QE = 1,
    APP_YAW_PID_PROFILE_180 = 2
} app_yaw_pid_profile_t;

typedef struct
{
    uint8_t action_id;
    uint8_t action_result;
} app_yaw_action_status_t;

typedef struct
{
    uint32_t timestamp_ms;
    uint32_t system_events;
    uint32_t can_rx_count;
    uint32_t can_rx_dropped;
    uint32_t comm_rx_count;
    uint32_t comm_rx_dropped;
    uint32_t imu_rx_count;
    uint32_t imu_rx_dropped;
    uint32_t dt35_rx_count;
    uint32_t dt35_rx_dropped;
    uint32_t control_cycle_count;
    uint32_t control_overrun_count;
    uint32_t can_error_mask;
} app_runtime_stats_t;

bool AppMonitor_Init(void);
void AppMonitor_Step(const app_runtime_stats_t *stats);
bool AppMonitor_GetStats(app_runtime_stats_t *stats);

bool AppSafety_Init(void);
void AppSafety_Step(uint32_t system_events);
bool AppSafety_OutputAllowed(void);
uint32_t AppSafety_GetEvents(void);

void AppMonitor_UpdatePcManual(float velocity_x_mps, float velocity_y_mps,
                               float yaw_rate_degps, uint32_t timestamp_ms);
void AppMonitor_UpdatePcCommand(app_pc_command_t command, uint32_t timestamp_ms);
void AppMonitor_CheckPcTimeout(uint32_t timestamp_ms);
bool AppMonitor_GetPcControl(app_pc_control_t *control);
bool AppMonitor_UpdateMotionLimits(const app_motion_limits_t *limits);
bool AppMonitor_GetMotionLimits(app_motion_limits_t *limits);
bool AppMonitor_SetYawTarget(float yaw_deg, uint32_t timestamp_ms);
bool AppMonitor_AlignYawTarget(float yaw_deg);
bool AppMonitor_GetYawTarget(float *yaw_deg);
bool AppMonitor_AdvanceYawTarget(float yaw_rate_radps, float dt_s);
bool AppMonitor_StepYawTarget(float delta_deg);
void AppMonitor_ClearYawTarget(void);
bool AppMonitor_UpdateYawPidParams(const app_yaw_pid_params_t *params);
bool AppMonitor_UpdateYawPidProfileParams(app_yaw_pid_profile_t profile,
                                          const app_yaw_pid_params_t *params);
bool AppMonitor_GetYawPidParams(app_yaw_pid_params_t *params);
bool AppMonitor_SelectYawPidProfile(app_yaw_pid_profile_t profile);
bool AppMonitor_SelectYawPidByError(float error_deg);
bool AppMonitor_UpdateYawSwitchThreshold(float error_deg);
bool AppMonitor_StartYawTurn(uint8_t action_id, float current_yaw_deg);
void AppMonitor_UpdateYawActionResult(float current_yaw_deg);
void AppMonitor_ClearYawAction(void);
bool AppMonitor_GetYawActionStatus(app_yaw_action_status_t *status);
void AppMonitor_OnPcRx(bsp_uart_t *uart, const uint8_t *data, uint16_t len);
void AppMonitor_CommunicationStep(uint32_t timestamp_ms);

#endif /* APP_MONITOR_H */
