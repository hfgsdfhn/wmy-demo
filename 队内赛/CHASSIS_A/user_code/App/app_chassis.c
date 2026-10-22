#include "app_chassis.h"

#include <stddef.h>
#include <string.h>
#include <math.h>

#include "ak_motor.h"
#include "app_monitor.h"
#include "omni_chassis.h"

#define APP_CHASSIS_CAN_INDEX   0U                 /* 底盘电机所在 CAN 总线索引 */
#define APP_CHASSIS_MOTOR_COUNT 4U                 /* 底盘电机数量 */
#define APP_CHASSIS_FEEDBACK_TIMEOUT_MS 100U       /* 电机反馈超时阈值，单位 ms */
#define APP_CHASSIS_RC_AXIS_MAX 255.0f             /* 遥控摇杆满量程 */
#define APP_CHASSIS_RC_MAX_LINEAR_SPEED_MPS 1.0f   /* 遥控对应的最大线速度，单位 m/s */
#define APP_CHASSIS_RC_MAX_ANGULAR_SPEED_RADPS 1.0f /* 遥控对应的最大角速度，单位 rad/s */
#define APP_CHASSIS_RC_SHOULDER_YAW_RATE_RADPS 0.6f /* 电源板遥控肩键的固定旋转速度 */
/* 电机标定换算比例。 */
#define APP_ERPM_TO_RADPS 0.0012467f  /* 1 eRPM -> 电机角速度 rad/s */
#define APP_ERPM_TO_MPS   0.00009682f /* 1 eRPM -> 轮缘线速度 m/s */
#define APP_MPS_TO_ERPM   (1.0f / APP_ERPM_TO_MPS)
#define APP_CONTROL_PERIOD_MS 2.0f                /* PID 计算周期，单位 ms */
#define APP_CHASSIS_DEG_TO_RAD 0.01745329f

/* 底盘速度缓启动参数；周期必须与 AppTask_Control 的执行周期保持一致。 */
#define APP_CHASSIS_RAMP_PERIOD_MS 2.0f /* 缓启动控制周期，单位 ms */
#define APP_CHASSIS_ACCEL_STEP_MPS 0.003f /* 加速时每个控制周期的速度增量，单位 m/s */
#define APP_CHASSIS_DECEL_STEP_MPS 0.02f /* 减速时每个控制周期的速度减量，单位 m/s */
#define APP_CHASSIS_ANGULAR_ACCEL_STEP_RADPS 0.015f /* 每周期角加速增量，单位 rad/s */
#define APP_CHASSIS_ANGULAR_DECEL_STEP_RADPS 0.0180f /* 每周期角减速增量，单位 rad/s */
#define APP_CHASSIS_ACCEL_RATE_MPS2 \
    (APP_CHASSIS_ACCEL_STEP_MPS * 1000.0f / APP_CHASSIS_RAMP_PERIOD_MS) /* 加速时每秒速度变化量，单位 m/s^2 */
#define APP_CHASSIS_DECEL_RATE_MPS2 \
    (APP_CHASSIS_DECEL_STEP_MPS * 1000.0f / APP_CHASSIS_RAMP_PERIOD_MS) /* 减速时每秒速度变化量，单位 m/s^2 */

static ak_motor_t app_chassis_motors[APP_CHASSIS_MOTOR_COUNT];
static app_chassis_command_t app_chassis_command;
static app_chassis_command_t app_chassis_limited_command;
static bsp_can_t *app_chassis_can;
static bool app_chassis_initialized;
static app_chassis_diagnostics_t app_chassis_diagnostics;
static uint32_t app_chassis_feedback_ms[APP_CHASSIS_MOTOR_COUNT];
static float app_chassis_yaw_previous_error;
static float app_chassis_yaw_integral_error;
static bool app_chassis_yaw_pid_initialized;

static float AppChassis_Clamp(float value, float limit)
{
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

static float AppChassis_MoveToward(float current, float target, float step)
{
    float delta = target - current;
    if (fabsf(delta) <= step) return target;
    return current + ((delta > 0.0f) ? step : -step);
}

static float AppChassis_RampAxis(float current, float target)
{
    bool reversing = (current != 0.0f) && (target != 0.0f)
                  && ((current > 0.0f) != (target > 0.0f));
    return AppChassis_MoveToward(current, reversing ? 0.0f : target,
        reversing ? APP_CHASSIS_DECEL_STEP_MPS
                  : ((fabsf(target) > fabsf(current)) ? APP_CHASSIS_ACCEL_STEP_MPS
                                                       : APP_CHASSIS_DECEL_STEP_MPS));
}

static float AppChassis_RampYawRate(float current, float target)
{
    bool reversing = (current != 0.0f) && (target != 0.0f)
                  && ((current > 0.0f) != (target > 0.0f));
    return AppChassis_MoveToward(current, reversing ? 0.0f : target,
        reversing ? APP_CHASSIS_ANGULAR_DECEL_STEP_RADPS
                  : ((fabsf(target) > fabsf(current)) ? APP_CHASSIS_ANGULAR_ACCEL_STEP_RADPS
                                                       : APP_CHASSIS_ANGULAR_DECEL_STEP_RADPS));
}

static float AppChassis_NormalizeAngleDeg(float angle_deg)
{
    const float two_pi = 360.0f;

    while (angle_deg > 180.0f) angle_deg -= two_pi;
    while (angle_deg < -180.0f) angle_deg += two_pi;
    return angle_deg;
}

float AppChassis_YawErpmToRadps(float wheel_erpm)
{
    return wheel_erpm * APP_ERPM_TO_RADPS;
}

bool AppChassis_RemoteControlToCommand(const app_dual_control_t *control,
                                       uint32_t timestamp_ms,
                                       app_chassis_command_t *command)
{
    if ((control == NULL) || (command == NULL))
    {
        return false;
    }

    memset(command, 0, sizeof(*command));
    command->sequence = control->sequence;
    command->timestamp_ms = timestamp_ms;

    if ((control->control_mode != 1U) || (control->action_id != 1U)
        || ((control->flags & ((1U << 0U) | (1U << 2U)))
            != ((1U << 0U) | (1U << 2U)))
        || ((control->flags & (1U << 1U)) != 0U)) {
        return true;
    }

    command->velocity_x_mps = ((float)control->rc_joy_left_y
                               / APP_CHASSIS_RC_AXIS_MAX)
                             * APP_CHASSIS_RC_MAX_LINEAR_SPEED_MPS;
    command->velocity_y_mps = -((float)control->rc_joy_left_x
                                 / APP_CHASSIS_RC_AXIS_MAX)
                              * APP_CHASSIS_RC_MAX_LINEAR_SPEED_MPS;
    command->yaw_rate_radps = AppChassis_Clamp(
        ((float)control->shoulder_left / 64.0f) * APP_CHASSIS_RC_MAX_ANGULAR_SPEED_RADPS
        - ((float)control->shoulder_right / 64.0f) * APP_CHASSIS_RC_MAX_ANGULAR_SPEED_RADPS,
        APP_CHASSIS_RC_MAX_ANGULAR_SPEED_RADPS);
    command->valid = 1U;
    return true;
}

bool AppChassis_PowerBoardRemoteToCommand(const app_remote_control_t *control,
                                          uint32_t timestamp_ms,
                                          app_chassis_command_t *command)
{
    if ((control == NULL) || (command == NULL))
    {
        return false;
    }

    memset(command, 0, sizeof(*command));
    command->sequence = control->sequence;
    command->timestamp_ms = timestamp_ms;
    command->velocity_x_mps = ((float)control->joy_y / APP_CHASSIS_RC_AXIS_MAX)
                            * APP_CHASSIS_RC_MAX_LINEAR_SPEED_MPS;
    command->velocity_y_mps = ((float)control->joy_x / APP_CHASSIS_RC_AXIS_MAX)
                            * APP_CHASSIS_RC_MAX_LINEAR_SPEED_MPS;
    command->valid = 1U;
    return true;
}

bool AppChassis_Init(bsp_can_t *can)
{
    static const uint8_t motor_ids[APP_CHASSIS_MOTOR_COUNT] =
    {
        1, 2, 3, 4
    };
    uint8_t index;

    if (can == NULL)
    {
        return false;
    }

    app_chassis_can = can;
    memset(&app_chassis_command, 0, sizeof(app_chassis_command));
    memset(&app_chassis_limited_command, 0, sizeof(app_chassis_limited_command));
    memset(&app_chassis_diagnostics, 0, sizeof(app_chassis_diagnostics));
    memset(app_chassis_feedback_ms, 0, sizeof(app_chassis_feedback_ms));
    AppChassis_ResetYawPid();
    for (index = 0U; index < APP_CHASSIS_MOTOR_COUNT; index++)
    {
        if (!AkMotor_Init(&app_chassis_motors[index], can, motor_ids[index]))
        {
            app_chassis_can = NULL;
            app_chassis_initialized = false;
            return false;
        }
    }

    app_chassis_initialized = true;
    return true;
}

bool AppChassis_ProcessCan(void)
{
    return app_chassis_initialized && (app_chassis_can != NULL)
        && BspCan_Process(app_chassis_can);
}

/**
 * @brief  控制底盘电机的目标速度，
 *         若系统处于紧急停止、CAN 总线故障或电机故障状态，则忽略该命令。
 * 
 * @param command 
 * @param system_events 
 */
void AppChassis_ControlStep(const app_chassis_command_t *command,
                            uint32_t system_events)
{
    omni_chassis_wheel_speeds_t wheel_speeds;

    if (!app_chassis_initialized || (command == NULL))
    {
        return;
    }

    app_chassis_command = *command;
    if ((system_events & (APP_EVENT_ESTOP | APP_EVENT_MOTOR_FAULT
                          | APP_EVENT_CAN_FAULT)) != 0U)
    {
        app_chassis_command.valid = 0U;
        app_chassis_command.velocity_x_mps = 0.0f;
        app_chassis_command.velocity_y_mps = 0.0f;
        app_chassis_command.yaw_rate_radps = 0.0f;
        app_chassis_limited_command.velocity_x_mps = 0.0f;
        app_chassis_limited_command.velocity_y_mps = 0.0f;
        app_chassis_limited_command.yaw_rate_radps = 0.0f;
    }
    else
    {
        app_chassis_limited_command.velocity_x_mps = AppChassis_RampAxis(
            app_chassis_limited_command.velocity_x_mps,
            app_chassis_command.velocity_x_mps);
        app_chassis_limited_command.velocity_y_mps = AppChassis_RampAxis(
            app_chassis_limited_command.velocity_y_mps,
            app_chassis_command.velocity_y_mps);
        app_chassis_limited_command.yaw_rate_radps = AppChassis_RampYawRate(
            app_chassis_limited_command.yaw_rate_radps,
            app_chassis_command.yaw_rate_radps);
    }
    app_chassis_limited_command.valid = app_chassis_command.valid;

    if (app_chassis_limited_command.valid != 0U)
    {
        OmniChassis_ForwardKinematics(app_chassis_limited_command.velocity_x_mps,
                                      app_chassis_limited_command.velocity_y_mps,
                                      app_chassis_limited_command.yaw_rate_radps,
                                      &wheel_speeds);
    }
    else
    {
        OmniChassis_ForwardKinematics(0.0f, 0.0f, 0.0f, &wheel_speeds);
    }

    app_chassis_diagnostics.target_erpm[3] = wheel_speeds.left_front_mps * APP_MPS_TO_ERPM;
    app_chassis_diagnostics.target_erpm[1] = wheel_speeds.right_rear_mps * APP_MPS_TO_ERPM;
    app_chassis_diagnostics.target_erpm[0] = wheel_speeds.right_front_mps * APP_MPS_TO_ERPM;
    app_chassis_diagnostics.target_erpm[2] = wheel_speeds.left_rear_mps * APP_MPS_TO_ERPM;

    for (uint8_t index = 0U; index < APP_CHASSIS_MOTOR_COUNT; ++index)
    {
        if (AkMotor_SetSpeed(&app_chassis_motors[index],
                             app_chassis_diagnostics.target_erpm[index]))
        {
            app_chassis_diagnostics.tx_ok_count++;
        }
        else
        {
            app_chassis_diagnostics.tx_failed_count++;
        }
    }

}

bool AppChassis_OnCanFrame(const app_can_frame_t *frame)    // 处理底盘电机的 CAN 帧反馈
{
    uint8_t index;

    if (!app_chassis_initialized || (frame == NULL)
        || (frame->bus != (app_can_bus_t)APP_CHASSIS_CAN_INDEX)
        || (frame->is_extended_id == 0U))
    {
        return false;
    }

    for (index = 0U; index < APP_CHASSIS_MOTOR_COUNT; index++)
    {
        if ((frame->identifier & 0xFFU) == app_chassis_motors[index].id)
        {
            bool parsed = AkMotor_ParseFeedback(&app_chassis_motors[index],
                                                frame->data, frame->length);
            if (parsed)
            {
                app_chassis_diagnostics.feedback_valid_mask |= (uint8_t)(1U << index);
                app_chassis_diagnostics.measured_wheel_speed_mps[index] =
                    app_chassis_motors[index].feedback.speed * APP_ERPM_TO_MPS;
                app_chassis_diagnostics.last_feedback_ms = frame->timestamp_ms;
                app_chassis_feedback_ms[index] = frame->timestamp_ms;
            }
            return parsed;
        }
    }
    return false;
}

bool AppChassis_GetCommand(app_chassis_command_t *command)
{
    if (!app_chassis_initialized || (command == NULL))
    {
        return false;
    }

    *command = app_chassis_command;
    
    return true;
}

bool AppChassis_GetDiagnostics(app_chassis_diagnostics_t *diagnostics)
{
    if (!app_chassis_initialized || (diagnostics == NULL))
    {
        return false;
    }
    *diagnostics = app_chassis_diagnostics;
    return true;
}

bool AppChassis_GetMeasuredVelocity(float *vx_mps, float *vy_mps,
                                    float *omega_radps, uint32_t timestamp_ms)
{
    omni_chassis_wheel_speeds_t wheel_speeds;
    uint8_t index;

    if (!app_chassis_initialized || (vx_mps == NULL) || (vy_mps == NULL)
        || (omega_radps == NULL))
    {
        return false;
    }

    for (index = 0U; index < APP_CHASSIS_MOTOR_COUNT; ++index)
    {
        if (!app_chassis_motors[index].feedback.valid
            || ((uint32_t)(timestamp_ms - app_chassis_feedback_ms[index])
                > APP_CHASSIS_FEEDBACK_TIMEOUT_MS))
        {
            app_chassis_diagnostics.feedback_valid_mask &=
                (uint8_t)~(uint8_t)(1U << index);
            return false;
        }
    }

    wheel_speeds.left_front_mps = app_chassis_motors[3].feedback.speed * APP_ERPM_TO_MPS;
    wheel_speeds.right_rear_mps = app_chassis_motors[1].feedback.speed * APP_ERPM_TO_MPS;
    wheel_speeds.right_front_mps = app_chassis_motors[0].feedback.speed * APP_ERPM_TO_MPS;
    wheel_speeds.left_rear_mps = app_chassis_motors[2].feedback.speed * APP_ERPM_TO_MPS;
    OmniChassis_InverseKinematics(&wheel_speeds, vx_mps, vy_mps, omega_radps);
    return true;
}

void AppChassis_ResetYawPid(void)
{
    app_chassis_yaw_previous_error = 0.0f;
    app_chassis_yaw_integral_error = 0.0f;
    app_chassis_yaw_pid_initialized = false;
}

void AppChassis_StopYawImmediately(void)
{
    app_chassis_command.yaw_rate_radps = 0.0f;
    app_chassis_limited_command.yaw_rate_radps = 0.0f;
    AppChassis_ResetYawPid();
}

bool AppChassis_ComputeYawRate(float target_yaw_deg, float current_yaw_deg,
                               const app_yaw_pid_params_t *params,
                               float *yaw_rate_radps)
{
    float error;
    float derivative = 0.0f;
    float output_degps;

    if ((params == NULL) || (yaw_rate_radps == NULL)
        || !isfinite(target_yaw_deg) || !isfinite(current_yaw_deg)) {
        return false;
    }

    error = AppChassis_NormalizeAngleDeg(target_yaw_deg - current_yaw_deg);
    if (fabsf(error) <= params->deadzone_deg) {
        AppChassis_ResetYawPid();
        *yaw_rate_radps = 0.0f;
        return true;
    }
    if ((params->integral_limit <= 0.0f)
        || ((params->integral_separation_deg > 0.0f)
            && (fabsf(error) > params->integral_separation_deg))) {
        app_chassis_yaw_integral_error = 0.0f;
    } else {
        app_chassis_yaw_integral_error += error * (APP_CONTROL_PERIOD_MS * 0.001f);
        app_chassis_yaw_integral_error = AppChassis_Clamp(
            app_chassis_yaw_integral_error, params->integral_limit);
    }
    if (app_chassis_yaw_pid_initialized) {
        derivative = (error - app_chassis_yaw_previous_error)
                   / (APP_CONTROL_PERIOD_MS * 0.001f);
    }
    output_degps = AppChassis_Clamp(params->kp * error
                                    + params->ki * app_chassis_yaw_integral_error
                                    + params->kd * derivative,
                                    params->max_output_degps)
                * params->output_multiplier;
    app_chassis_yaw_previous_error = error;
    app_chassis_yaw_pid_initialized = true;
    *yaw_rate_radps = output_degps * APP_CHASSIS_DEG_TO_RAD;
    return true;
}
