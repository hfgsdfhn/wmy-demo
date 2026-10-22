#include "app_monitor.h"

#include <stddef.h>
#include <string.h>
#include <math.h>

#include "pid_config.h"
#include "app_chassis.h"
#include "remote_buttons.h"
#include "app_dt35_position.h"
#include "app_init.h"
#include "app_messages.h"
#include "app_rtos.h"
#include "bsp_uart.h"
#include "DM_Imu.h"

static app_runtime_stats_t app_monitor_stats;
static bool app_monitor_valid;
static uint32_t app_safety_events;
static app_pc_control_t app_pc_control;
static app_motion_limits_t app_motion_limits;
static app_yaw_pid_params_t app_yaw_pid_params;
static app_yaw_pid_params_t app_yaw_pid_profiles[3];
static float app_yaw_target_deg;
static bool app_yaw_target_valid;
static app_yaw_action_status_t app_yaw_action_status;
static float app_yaw_switch_threshold_deg = CHASSIS_YAW_PID_SWITCH_ERROR_DEG;
static float app_dt35_transition_lpf_alpha = CHASSIS_DT35_TRANSITION_LPF_ALPHA;
static app_yaw_pid_profile_t app_yaw_active_profile = (app_yaw_pid_profile_t)-1;

#define APP_MONITOR_PC_FRAME_MAX 64U              /* PC 通信帧最大字节数 */
#define APP_MONITOR_PC_STATE_ID 0x10U             /* PC 状态上报帧 ID */
#define APP_MONITOR_PC_HEALTH_ID 0x11U            /* PC 健康状态上报帧 ID */
#define APP_MONITOR_PC_MANUAL_ID 0x20U            /* PC 手动速度命令帧 ID */
#define APP_MONITOR_PC_COMMAND_ID 0x22U           /* PC 启停命令帧 ID */
#define APP_MONITOR_PC_LIMITS_ID 0x23U            /* PC 运动限幅设置帧 ID */
#define APP_MONITOR_PC_YAW_TARGET_ID 0x24U        /* PC YAW 目标角帧 ID */
#define APP_MONITOR_PC_YAW_PID_PARAMS_ID 0x25U    /* PC YAW PID 参数帧 ID */
#define APP_MONITOR_PC_YAW_PID_PROFILE_ID 0x26U   /* PC YAW PID 配置档帧 ID */
#define APP_MONITOR_PC_YAW_PID_MODE_ID 0x27U      /* PC YAW PID 启用状态帧 ID */
#define APP_MONITOR_PC_STR_PID_PARAMS_ID 0x2AU
#define APP_MONITOR_PC_DT35_TARGETS_ID 0x2BU
#define APP_MONITOR_PC_YAW_SWITCH_ID 0x2CU       /* payload: error_deg float */
#define APP_MONITOR_PC_YAW_PID_PROFILE_PARAMS_ID 0x2DU
#define APP_MONITOR_PC_DT35_TRANSITION_LPF_ID 0x2EU
#define APP_MONITOR_PC_STATE_PAYLOAD_SIZE 56U     /* PC 状态帧负载字节数 */
#define APP_MONITOR_PC_HEALTH_PAYLOAD_SIZE 59U    /* PC 健康状态帧负载字节数 */
#define APP_MONITOR_DEG_TO_RAD 0.01745329251994329577f
#define APP_MONITOR_RAD_TO_DEG 57.2957795130823208768f


static uint8_t app_monitor_pc_frame[APP_MONITOR_PC_FRAME_MAX];
static uint16_t app_monitor_pc_length;
static uint16_t app_monitor_pc_expected;

static uint16_t AppMonitor_ReadU16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

static void AppMonitor_WriteU16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8U);
}

static uint16_t AppMonitor_Crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFU;
    uint16_t index;
    uint8_t bit;

    for (index = 0U; index < len; ++index) {
        crc ^= (uint16_t)data[index] << 8U;
        for (bit = 0U; bit < 8U; ++bit) {
            crc = (crc & 0x8000U) ? (uint16_t)((crc << 1U) ^ 0x1021U)
                                  : (uint16_t)(crc << 1U);
        }
    }
    return crc;
}

static void AppMonitor_ResetPcParser(void)
{
    app_monitor_pc_length = 0U;
    app_monitor_pc_expected = 0U;
}

static float AppMonitor_NormalizeYawDeg(float yaw_deg)
{
    while (yaw_deg > 180.0f) yaw_deg -= 360.0f;
    while (yaw_deg < -180.0f) yaw_deg += 360.0f;
    return yaw_deg;
}

bool AppMonitor_SelectYawPidProfile(app_yaw_pid_profile_t profile)
{
    if ((profile != APP_YAW_PID_PROFILE_90) && (profile != APP_YAW_PID_PROFILE_QE)
        && (profile != APP_YAW_PID_PROFILE_180)) {
        return false;
    }
    if (profile == app_yaw_active_profile) return true;
    if (!AppMonitor_UpdateYawPidParams(&app_yaw_pid_profiles[profile])) return false;
    app_yaw_active_profile = profile;
    return true;
}

bool AppMonitor_SelectYawPidByError(float error_deg)
{
    float magnitude = fabsf(AppMonitor_NormalizeYawDeg(error_deg));
    if (magnitude > app_yaw_switch_threshold_deg) {
        return AppMonitor_SelectYawPidProfile(APP_YAW_PID_PROFILE_180);
    }
    return AppMonitor_SelectYawPidProfile(APP_YAW_PID_PROFILE_90);
}

bool AppMonitor_UpdateYawSwitchThreshold(float error_deg)
{
    if (!isfinite(error_deg) || (error_deg < 0.0f) || (error_deg > 180.0f)) {
        return false;
    }
    app_yaw_switch_threshold_deg = error_deg;
    return true;
}

bool AppMonitor_Init(void)
{
    memset(&app_monitor_stats, 0, sizeof(app_monitor_stats));
    memset(&app_pc_control, 0, sizeof(app_pc_control));
    app_dt35_transition_lpf_alpha = CHASSIS_DT35_TRANSITION_LPF_ALPHA;
    app_motion_limits.max_linear_speed_mps = 3.0f;
    app_motion_limits.max_angular_speed_radps = 10.0f;
    app_motion_limits.max_acceleration_mps2 = 3.0f;
    app_yaw_pid_profiles[APP_YAW_PID_PROFILE_90] = (app_yaw_pid_params_t){
        CHASSIS_YAW_PID_ENABLED != 0U, CHASSIS_YAW_90_KP, CHASSIS_YAW_90_KD,
        CHASSIS_YAW_90_KI, CHASSIS_YAW_90_INTEGRAL_LIMIT,
        CHASSIS_YAW_90_INTEGRAL_SEPARATION, CHASSIS_YAW_90_MAX_OUTPUT_DEGPS,
        CHASSIS_YAW_90_DEADZONE, CHASSIS_YAW_90_OUTPUT_MULTIPLIER };
    app_yaw_pid_profiles[APP_YAW_PID_PROFILE_QE] = (app_yaw_pid_params_t){
        CHASSIS_YAW_PID_ENABLED != 0U, CHASSIS_YAW_QE_KP, CHASSIS_YAW_QE_KD,
        CHASSIS_YAW_QE_KI, CHASSIS_YAW_QE_INTEGRAL_LIMIT,
        CHASSIS_YAW_QE_INTEGRAL_SEPARATION, CHASSIS_YAW_QE_MAX_OUTPUT_DEGPS,
        CHASSIS_YAW_QE_DEADZONE, CHASSIS_YAW_QE_OUTPUT_MULTIPLIER };
    app_yaw_pid_profiles[APP_YAW_PID_PROFILE_180] = (app_yaw_pid_params_t){
        CHASSIS_YAW_PID_ENABLED != 0U, CHASSIS_YAW_180_KP, CHASSIS_YAW_180_KD,
        CHASSIS_YAW_180_KI, CHASSIS_YAW_180_INTEGRAL_LIMIT,
        CHASSIS_YAW_180_INTEGRAL_SEPARATION, CHASSIS_YAW_180_MAX_OUTPUT_DEGPS,
        CHASSIS_YAW_180_DEADZONE, CHASSIS_YAW_180_OUTPUT_MULTIPLIER };
    app_yaw_active_profile = (app_yaw_pid_profile_t)-1;
    if (!AppMonitor_SelectYawPidProfile((app_yaw_pid_profile_t)CHASSIS_YAW_PID_DEFAULT_PROFILE)) {
        return false;
    }
    /* The control task aligns the initial target to the first valid feedback. */
    app_yaw_target_deg = 0.0f;
    app_yaw_target_valid = false;
    memset(&app_yaw_action_status, 0, sizeof(app_yaw_action_status));
    AppMonitor_ResetPcParser();
    app_monitor_valid = false;
    return true;
}

void AppMonitor_Step(const app_runtime_stats_t *stats)
{
    if (stats != NULL)
    {
        app_monitor_stats = *stats;
        app_monitor_valid = true;
    }
}

bool AppMonitor_GetStats(app_runtime_stats_t *stats)
{
    if (!app_monitor_valid || (stats == NULL))
    {
        return false;
    }
    *stats = app_monitor_stats;
    return true;
}

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

void AppMonitor_UpdatePcManual(float velocity_x_mps, float velocity_y_mps,
                               float yaw_rate_degps, uint32_t timestamp_ms)
{
    app_pc_control.velocity_x_mps = velocity_x_mps;
    app_pc_control.velocity_y_mps = velocity_y_mps;
    app_pc_control.yaw_rate_radps = yaw_rate_degps * APP_MONITOR_DEG_TO_RAD;
    app_pc_control.timestamp_ms = timestamp_ms;
    app_pc_control.online = 1U;
}

void AppMonitor_UpdatePcCommand(app_pc_command_t command, uint32_t timestamp_ms)
{
    app_pc_control.command = command;
    app_pc_control.timestamp_ms = timestamp_ms;
    app_pc_control.online = 1U;

    if ((command == APP_PC_COMMAND_STOP) || (command == APP_PC_COMMAND_RESET)
        || (command == APP_PC_COMMAND_ESTOP)) {
        app_pc_control.velocity_x_mps = 0.0f;
        app_pc_control.velocity_y_mps = 0.0f;
        app_pc_control.yaw_rate_radps = 0.0f;
    }
    if (command == APP_PC_COMMAND_ESTOP) {
        AppMonitor_ClearYawTarget();
        AppRtos_SetEventsFromISR(APP_EVENT_ESTOP);
    }
    if (command == APP_PC_COMMAND_RESET) {
        AppRtos_ClearCanErrors();
        (void)AppRtos_ClearEvents(APP_EVENT_ESTOP | APP_EVENT_CAN_FAULT
                                  | APP_EVENT_MOTOR_FAULT);
    }
}

void AppMonitor_CheckPcTimeout(uint32_t timestamp_ms)
{
    uint32_t elapsed = timestamp_ms - app_pc_control.timestamp_ms;

    if (!app_pc_control.online) {
        return;
    }
    if (elapsed >= 100U) {
        app_pc_control.velocity_x_mps = 0.0f;
        app_pc_control.velocity_y_mps = 0.0f;
        app_pc_control.yaw_rate_radps = 0.0f;
    }
    if (elapsed >= 300U) {
        app_pc_control.online = 0U;
    }
}

bool AppMonitor_GetPcControl(app_pc_control_t *control)
{
    if (control == NULL) {
        return false;
    }
    *control = app_pc_control;
    return app_pc_control.online != 0U;
}

bool AppMonitor_UpdateMotionLimits(const app_motion_limits_t *limits)
{
    if ((limits == NULL) || (limits->max_linear_speed_mps <= 0.0f)
        || (limits->max_angular_speed_radps <= 0.0f)
        || (limits->max_acceleration_mps2 <= 0.0f)) {
        return false;
    }
    app_motion_limits = *limits;
    return true;
}

bool AppMonitor_GetMotionLimits(app_motion_limits_t *limits)
{
    if (limits == NULL) {
        return false;
    }
    *limits = app_motion_limits;
    return true;
}

bool AppMonitor_SetYawTarget(float yaw_deg, uint32_t timestamp_ms)
{
    if (!isfinite(yaw_deg)) {
        return false;
    }
    app_yaw_target_deg = yaw_deg;
    app_yaw_target_valid = true;
    app_pc_control.timestamp_ms = timestamp_ms;
    app_pc_control.online = 1U;
    return true;
}

bool AppMonitor_AlignYawTarget(float yaw_deg)
{
    if (!isfinite(yaw_deg)) {
        return false;
    }
    app_yaw_target_deg = AppMonitor_NormalizeYawDeg(yaw_deg);
    app_yaw_target_valid = true;
    return true;
}

bool AppMonitor_GetYawTarget(float *yaw_deg)
{
    if ((yaw_deg == NULL) || !app_yaw_target_valid) {
        return false;
    }
    *yaw_deg = app_yaw_target_deg;
    return true;
}

bool AppMonitor_AdvanceYawTarget(float yaw_rate_radps, float dt_s)
{
    const float full_turn_deg = 360.0f;

    if (!isfinite(yaw_rate_radps) || !isfinite(dt_s) || (dt_s < 0.0f)) {
        return false;
    }
    if (!app_yaw_target_valid) {
        app_yaw_target_deg = 0.0f;
        app_yaw_target_valid = true;
    }
    app_yaw_target_deg += yaw_rate_radps * APP_MONITOR_RAD_TO_DEG * dt_s;
    while (app_yaw_target_deg > 180.0f) {
        app_yaw_target_deg -= full_turn_deg;
    }
    while (app_yaw_target_deg < -180.0f) {
        app_yaw_target_deg += full_turn_deg;
    }
    return true;
}

bool AppMonitor_StepYawTarget(float delta_deg)
{
    if (!isfinite(delta_deg)) {
        return false;
    }
    return AppMonitor_AdvanceYawTarget(delta_deg * APP_MONITOR_DEG_TO_RAD,
                                       1.0f);
}

void AppMonitor_ClearYawTarget(void)
{
    app_yaw_target_valid = false;
}

bool AppMonitor_UpdateYawPidParams(const app_yaw_pid_params_t *params)
{
    if ((params == NULL) || !isfinite(params->kp)
        || !isfinite(params->kd) || !isfinite(params->ki)
        || !isfinite(params->max_output_degps)
        || !isfinite(params->output_multiplier)
        || !isfinite(params->integral_limit) || !isfinite(params->integral_separation_deg)
        || !isfinite(params->deadzone_deg)
        || (params->kp < 0.0f) || (params->kd < 0.0f) || (params->ki < 0.0f)
        || (params->integral_limit < 0.0f) || (params->integral_separation_deg < 0.0f)
        || (params->max_output_degps <= 0.0f)
        || (params->output_multiplier < 0.0f)
        || (params->deadzone_deg < 0.0f)) {
        return false;
    }
    app_yaw_pid_params = *params;
    AppChassis_ResetYawPid();
    return true;
}

bool AppMonitor_UpdateYawPidProfileParams(app_yaw_pid_profile_t profile,
                                          const app_yaw_pid_params_t *params)
{
    app_yaw_pid_params_t active_params;

    if (((profile != APP_YAW_PID_PROFILE_90) && (profile != APP_YAW_PID_PROFILE_QE)
         && (profile != APP_YAW_PID_PROFILE_180)) || (params == NULL)) {
        return false;
    }
    active_params = app_yaw_pid_params;
    if (!AppMonitor_UpdateYawPidParams(params)) {
        return false;
    }
    app_yaw_pid_profiles[profile] = *params;
    if (profile != app_yaw_active_profile) {
        app_yaw_pid_params = active_params;
        AppChassis_ResetYawPid();
    }
    return true;
}

bool AppMonitor_GetYawPidParams(app_yaw_pid_params_t *params)
{
    if (params == NULL) {
        return false;
    }
    *params = app_yaw_pid_params;
    return true;
}

bool AppMonitor_StartYawTurn(uint8_t action_id, float current_yaw_deg)
{
    float delta_deg;

    if (!isfinite(current_yaw_deg)) {
        return false;
    }
    if (action_id == CHASSIS_YAW_ACTION_LEFT_90_ID) {
        delta_deg = 90.0f;
    } else if (action_id == CHASSIS_YAW_ACTION_RIGHT_90_ID) {
        delta_deg = -90.0f;
    } else if (action_id == CHASSIS_YAW_ACTION_180_ID) {
        delta_deg = 180.0f;
    } else {
        return false;
    }
    if (!AppMonitor_SelectYawPidProfile(
            (action_id == CHASSIS_YAW_ACTION_180_ID)
                ? APP_YAW_PID_PROFILE_180 : APP_YAW_PID_PROFILE_90)) {
        return false;
    }

    app_yaw_target_deg = AppMonitor_NormalizeYawDeg(current_yaw_deg + delta_deg);
    app_yaw_target_valid = true;
    app_yaw_action_status.action_id = action_id;
    app_yaw_action_status.action_result = 1U;
    return true;
}

void AppMonitor_UpdateYawActionResult(float current_yaw_deg)
{
    app_yaw_pid_params_t params;
    float error_deg;

    if ((app_yaw_action_status.action_result != 1U) || !isfinite(current_yaw_deg)
        || !AppMonitor_GetYawPidParams(&params) || !app_yaw_target_valid) {
        return;
    }
    error_deg = AppMonitor_NormalizeYawDeg(app_yaw_target_deg - current_yaw_deg);
    if (fabsf(error_deg) <= params.deadzone_deg) {
        app_yaw_action_status.action_result = 2U;
    }
}

void AppMonitor_ClearYawAction(void)
{
    memset(&app_yaw_action_status, 0, sizeof(app_yaw_action_status));
}

bool AppMonitor_GetYawActionStatus(app_yaw_action_status_t *status)
{
    if ((status == NULL) || (app_yaw_action_status.action_id == 0U)) {
        return false;
    }
    *status = app_yaw_action_status;
    return true;
}

static bool AppMonitor_ParsePcFrame(const uint8_t *frame, uint16_t length,
                                    uint32_t timestamp_ms)
{
    uint16_t payload_length;

    if ((frame == NULL) || (length < 7U) || (frame[0] != 0xAAU)
        || (frame[1] != 0x55U)) return false;
    payload_length = AppMonitor_ReadU16(&frame[3]);
    if ((length != payload_length + 7U)
        || (AppMonitor_ReadU16(&frame[5U + payload_length])
            != AppMonitor_Crc16(&frame[2], payload_length + 3U))) return false;

    if ((frame[2] == APP_MONITOR_PC_MANUAL_ID) && (payload_length == 12U)) {
        float vx;
        float vy;
        float yaw_rate_degps;
        memcpy(&vx, &frame[5], sizeof(vx));
        memcpy(&vy, &frame[9], sizeof(vy));
        memcpy(&yaw_rate_degps, &frame[13], sizeof(yaw_rate_degps));
        AppMonitor_UpdatePcManual(vx, vy, yaw_rate_degps, timestamp_ms);
        return true;
    }
    if ((frame[2] == APP_MONITOR_PC_COMMAND_ID) && (payload_length == 1U)
        && (frame[5] >= APP_PC_COMMAND_START) && (frame[5] <= APP_PC_COMMAND_ESTOP)) {
        AppMonitor_UpdatePcCommand((app_pc_command_t)frame[5], timestamp_ms);
        return true;
    }
    if ((frame[2] == APP_MONITOR_PC_LIMITS_ID) && (payload_length == 12U)) {
        app_motion_limits_t limits;
        memcpy(&limits.max_linear_speed_mps, &frame[5], sizeof(float));
        memcpy(&limits.max_angular_speed_radps, &frame[9], sizeof(float));
        limits.max_angular_speed_radps *= APP_MONITOR_DEG_TO_RAD;
        memcpy(&limits.max_acceleration_mps2, &frame[13], sizeof(float));
        return AppMonitor_UpdateMotionLimits(&limits);
    }
    if ((frame[2] == APP_MONITOR_PC_YAW_TARGET_ID) && (payload_length == 4U)) {
        float yaw_target_deg;
        memcpy(&yaw_target_deg, &frame[5], sizeof(yaw_target_deg));
        return AppMonitor_SetYawTarget(yaw_target_deg, timestamp_ms);
    }
    if ((frame[2] == APP_MONITOR_PC_YAW_PID_PARAMS_ID) && (payload_length == 28U)) {
        /* Keep the local enable selection because the legacy parameter frame
           contains only numeric gains and limits. */
        app_yaw_pid_params_t params = app_yaw_pid_params;
        memcpy(&params.kp, &frame[5], sizeof(float));
        memcpy(&params.ki, &frame[9], sizeof(float));
        memcpy(&params.kd, &frame[13], sizeof(float));
        memcpy(&params.max_output_degps, &frame[17], sizeof(float));
        memcpy(&params.integral_limit, &frame[21], sizeof(float));
        memcpy(&params.integral_separation_deg, &frame[25], sizeof(float));
        memcpy(&params.deadzone_deg, &frame[29], sizeof(float));
        if (AppMonitor_UpdateYawPidParams(&params)) {
            app_yaw_pid_profiles[app_yaw_active_profile] = params;
            return true;
        }
        return false;
    }
    if ((frame[2] == APP_MONITOR_PC_YAW_PID_PROFILE_PARAMS_ID) && (payload_length == 29U)) {
        app_yaw_pid_profile_t profile = (app_yaw_pid_profile_t)frame[5];
        app_yaw_pid_params_t params;
        if ((profile != APP_YAW_PID_PROFILE_90) && (profile != APP_YAW_PID_PROFILE_QE)
            && (profile != APP_YAW_PID_PROFILE_180)) return false;
        params = app_yaw_pid_profiles[profile];
        memcpy(&params.kp, &frame[6], sizeof(float));
        memcpy(&params.ki, &frame[10], sizeof(float));
        memcpy(&params.kd, &frame[14], sizeof(float));
        memcpy(&params.max_output_degps, &frame[18], sizeof(float));
        memcpy(&params.integral_limit, &frame[22], sizeof(float));
        memcpy(&params.integral_separation_deg, &frame[26], sizeof(float));
        memcpy(&params.deadzone_deg, &frame[30], sizeof(float));
        return AppMonitor_UpdateYawPidProfileParams(profile, &params);
    }
    if ((frame[2] == APP_MONITOR_PC_YAW_PID_PROFILE_ID) && (payload_length == 1U)) {
        return AppMonitor_SelectYawPidProfile((app_yaw_pid_profile_t)frame[5]);
    }
    if ((frame[2] == APP_MONITOR_PC_YAW_PID_MODE_ID) && (payload_length == 1U)) {
        app_yaw_pid_params_t params = app_yaw_pid_params;
        params.enabled = frame[5] != 0U;
        return AppMonitor_UpdateYawPidParams(&params);
    }
    if ((frame[2] == APP_MONITOR_PC_STR_PID_PARAMS_ID) && (payload_length == 28U)) {
        app_dt35_pid_params_t params;
        params.transition_lpf_alpha = app_dt35_transition_lpf_alpha;
        memcpy(&params.kp, &frame[5],  sizeof(float));
        memcpy(&params.ki, &frame[9],  sizeof(float));
        memcpy(&params.kd, &frame[13], sizeof(float));
        memcpy(&params.max_output, &frame[17], sizeof(float));
        memcpy(&params.integral_limit, &frame[21], sizeof(float));
        memcpy(&params.integral_separation, &frame[25], sizeof(float));
        memcpy(&params.deadzone, &frame[29], sizeof(float));
        return AppDt35Position_UpdatePidParams(&params);
    }
    if ((frame[2] == APP_MONITOR_PC_DT35_TRANSITION_LPF_ID) && (payload_length == 4U)) {
        float alpha;
        memcpy(&alpha, &frame[5], sizeof(alpha));
        if (AppDt35Position_UpdateTransitionLpfAlpha(alpha)) {
            app_dt35_transition_lpf_alpha = alpha;
            return true;
        }
        return false;
    }
    if ((frame[2] == APP_MONITOR_PC_DT35_TARGETS_ID) && (payload_length == 16U)) {
        app_dt35_targets_t targets;
        memcpy(&targets.back_to_front_m, &frame[5], sizeof(float));
        memcpy(&targets.front_to_back_m, &frame[9], sizeof(float));
        memcpy(&targets.right_to_left_m, &frame[13], sizeof(float));
        memcpy(&targets.left_to_right_m, &frame[17], sizeof(float));
        return AppDt35Position_UpdateTargets(&targets);
    }
    if ((frame[2] == APP_MONITOR_PC_YAW_SWITCH_ID) && (payload_length == 4U)) {
        float error_deg;
        memcpy(&error_deg, &frame[5], sizeof(float));
        return AppMonitor_UpdateYawSwitchThreshold(error_deg);
    }
    return false;
}

void AppMonitor_OnPcRx(bsp_uart_t *uart, const uint8_t *data, uint16_t len)
{
    uint16_t index;
    (void)uart;

    if (data == NULL) return;
    for (index = 0U; index < len; ++index) {
        if (app_monitor_pc_length == 0U) {
            if (data[index] == 0xAAU) app_monitor_pc_frame[app_monitor_pc_length++] = data[index];
            continue;
        }
        if ((app_monitor_pc_length == 1U) && (data[index] != 0x55U)) {
            app_monitor_pc_length = (data[index] == 0xAAU) ? 1U : 0U;
            if (app_monitor_pc_length != 0U) app_monitor_pc_frame[0] = data[index];
            continue;
        }
        if (app_monitor_pc_length >= APP_MONITOR_PC_FRAME_MAX) {
            AppMonitor_ResetPcParser();
            continue;
        }
        app_monitor_pc_frame[app_monitor_pc_length++] = data[index];
        if (app_monitor_pc_length == 5U) {
            app_monitor_pc_expected = AppMonitor_ReadU16(&app_monitor_pc_frame[3]) + 7U;
            if (app_monitor_pc_expected > APP_MONITOR_PC_FRAME_MAX) {
                AppMonitor_ResetPcParser();
                continue;
            }
        }
        if ((app_monitor_pc_expected != 0U)
            && (app_monitor_pc_length == app_monitor_pc_expected)) {
            (void)AppMonitor_ParsePcFrame(app_monitor_pc_frame,
                                          app_monitor_pc_length, HAL_GetTick());
            AppMonitor_ResetPcParser();
        }
    }
}

static bool AppMonitor_SendPcState(const app_real_world_t *state)
{
    uint8_t frame[APP_MONITOR_PC_STATE_PAYLOAD_SIZE + 7U];
    uint16_t crc;
    float yaw_deg;
    float yaw_rate_degps;

    if (state == NULL) return false;
    yaw_deg = state->yaw_deg;
    yaw_rate_degps = state->yaw_rate_radps * APP_MONITOR_RAD_TO_DEG;
    frame[0] = 0xAAU;
    frame[1] = 0x55U;
    frame[2] = APP_MONITOR_PC_STATE_ID;
    AppMonitor_WriteU16(&frame[3], APP_MONITOR_PC_STATE_PAYLOAD_SIZE);
    memcpy(&frame[5], &state->timestamp_ms, sizeof(state->timestamp_ms));
    memcpy(&frame[9], &state->position_x_m, sizeof(state->position_x_m));
    memcpy(&frame[13], &state->position_y_m, sizeof(state->position_y_m));
    memcpy(&frame[17], &yaw_deg, sizeof(yaw_deg));
    memcpy(&frame[21], &state->velocity_x_mps, sizeof(state->velocity_x_mps));
    memcpy(&frame[25], &state->velocity_y_mps, sizeof(state->velocity_y_mps));
    memcpy(&frame[29], &yaw_rate_degps, sizeof(yaw_rate_degps));
    memcpy(&frame[33], state->dt35_distance_m, sizeof(state->dt35_distance_m));
    memcpy(&frame[41], &state->imu_yaw_deg, sizeof(state->imu_yaw_deg));
    memcpy(&frame[45], &state->chassis_yaw_deg, sizeof(state->chassis_yaw_deg));
    memcpy(&frame[49], &state->status_flags, sizeof(state->status_flags));
    memcpy(&frame[53], &state->fused_yaw_deg, sizeof(state->fused_yaw_deg));
    memcpy(&frame[57], &state->fused_yaw_rate_radps,
           sizeof(state->fused_yaw_rate_radps));
    crc = AppMonitor_Crc16(&frame[2], APP_MONITOR_PC_STATE_PAYLOAD_SIZE + 3U);
    AppMonitor_WriteU16(&frame[61], crc);
    return Bsp_Uart_Send(&uart10, frame, sizeof(frame));
}

static bool AppMonitor_SendPcHealth(const app_pc_health_t *health)
{
    uint8_t frame[APP_MONITOR_PC_HEALTH_PAYLOAD_SIZE + 7U];
    uint16_t crc;

    if (health == NULL) return false;
    frame[0] = 0xAAU;
    frame[1] = 0x55U;
    frame[2] = APP_MONITOR_PC_HEALTH_ID;
    AppMonitor_WriteU16(&frame[3], APP_MONITOR_PC_HEALTH_PAYLOAD_SIZE);
    memcpy(&frame[5], &health->timestamp_ms, sizeof(health->timestamp_ms));
    frame[9] = health->online_mask;
    memcpy(&frame[10], &health->system_events, sizeof(health->system_events));
    memcpy(&frame[14], &health->can_error_mask, sizeof(health->can_error_mask));
    frame[18] = health->pc_command;
    frame[19] = health->motor_feedback_mask;
    memcpy(&frame[20], &health->motor_tx_ok_count, sizeof(health->motor_tx_ok_count));
    memcpy(&frame[24], &health->motor_tx_failed_count, sizeof(health->motor_tx_failed_count));
    memcpy(&frame[28], health->motor_target_erpm, sizeof(health->motor_target_erpm));
    memcpy(&frame[44], health->motor_measured_speed_mps,
           sizeof(health->motor_measured_speed_mps));
    memcpy(&frame[60], &health->motor_last_feedback_ms, sizeof(health->motor_last_feedback_ms));
    crc = AppMonitor_Crc16(&frame[2], APP_MONITOR_PC_HEALTH_PAYLOAD_SIZE + 3U);
    AppMonitor_WriteU16(&frame[64], crc);
    return Bsp_Uart_Send(&uart10, frame, sizeof(frame));
}

void AppMonitor_CommunicationStep(uint32_t timestamp_ms)
{
    app_real_world_t state;
    app_runtime_stats_t stats;
    app_pc_control_t pc_control;
    app_pc_health_t health = {0};
    app_chassis_diagnostics_t chassis_diagnostics;
    app_dt35_sample_t dt35_sample;

    AppMonitor_CheckPcTimeout(timestamp_ms);
    if (AppRtos_GetRobotState(&state, 0U)) {
        float wheel_omega_radps;
        if (!AppChassis_GetMeasuredVelocity(&state.velocity_x_mps,
                                            &state.velocity_y_mps,
                                            &wheel_omega_radps, timestamp_ms)) {
            state.velocity_x_mps = 0.0f;
            state.velocity_y_mps = 0.0f;
        }
        (void)wheel_omega_radps;
        if (AppMessages_GetDt35(&dt35_sample)) {
            memcpy(state.dt35_distance_m, dt35_sample.distance_m,
                   sizeof(state.dt35_distance_m));
        }
        (void)AppMonitor_SendPcState(&state);
    }

    health.timestamp_ms = timestamp_ms;
    if (AppMonitor_GetPcControl(&pc_control)) {
        health.online_mask |= 1U << 0;
        health.pc_command = (uint8_t)pc_control.command;
    }
    health.online_mask |= AppMessages_GetOnlineMask();
    health.system_events = AppRtos_GetEvents();
    if (AppMonitor_GetStats(&stats)) health.can_error_mask = stats.can_error_mask;
    if (AppChassis_GetDiagnostics(&chassis_diagnostics)) {
        health.motor_feedback_mask = chassis_diagnostics.feedback_valid_mask;
        health.motor_tx_ok_count = chassis_diagnostics.tx_ok_count;
        health.motor_tx_failed_count = chassis_diagnostics.tx_failed_count;
        memcpy(health.motor_target_erpm, chassis_diagnostics.target_erpm,
               sizeof(health.motor_target_erpm));
        memcpy(health.motor_measured_speed_mps,
               chassis_diagnostics.measured_wheel_speed_mps,
               sizeof(health.motor_measured_speed_mps));
        health.motor_last_feedback_ms = chassis_diagnostics.last_feedback_ms;
    }
    (void)AppMonitor_SendPcHealth(&health);
}
