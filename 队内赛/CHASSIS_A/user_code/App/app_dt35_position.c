#include "app_dt35_position.h"

#include <math.h>
#include <string.h>

#include "PID.h"
#include "pid_config.h"

typedef struct
{
    float target_x_m;
    float target_y_m;
} app_dt35_position_target_t;

typedef struct
{
    pid_t x_pid;
    pid_t y_pid;
    app_dt35_position_status_t status;
    app_dt35_position_direction_t sequence[5];
    float sequence_target_m[5];
    uint8_t sequence_count;
    uint8_t sequence_index;
    bool sequence_has_segment_targets;
    bool relative_forward_pending;
    bool relative_forward_active;
    float relative_forward_start_m;
    bool initialized;
    bool active;
    bool transition_filter_pending;
    float filtered_velocity_x_mps;
    float filtered_velocity_y_mps;
    app_dt35_pid_params_t pid_params;
    app_dt35_targets_t targets;
} app_dt35_position_controller_t;

static app_dt35_position_controller_t app_dt35_position;

static bool AppDt35Position_AreTargetsValid(const app_dt35_targets_t *targets)
{
    if ((targets == NULL) || !isfinite(targets->back_to_front_m)
        || !isfinite(targets->front_to_back_m) || !isfinite(targets->right_to_left_m)
        || !isfinite(targets->left_to_right_m) || (targets->back_to_front_m < 0.0f)
        || (targets->front_to_back_m < 0.0f) || (targets->right_to_left_m < 0.0f)
        || (targets->left_to_right_m < 0.0f)) return false;
    return true;
}

bool AppDt35Position_UpdateTargets(const app_dt35_targets_t *targets)
{
    if (!AppDt35Position_AreTargetsValid(targets)) return false;
    app_dt35_position.targets = *targets;
    return true;
}

bool AppDt35Position_UpdatePidParams(const app_dt35_pid_params_t *params)
{
    if ((params == NULL) || !isfinite(params->kp) || !isfinite(params->ki)
        || !isfinite(params->kd) || !isfinite(params->max_output)
        || !isfinite(params->integral_limit)
        || !isfinite(params->integral_separation)
        || !isfinite(params->deadzone) || !isfinite(params->transition_lpf_alpha)
        || params->kp < 0.0f
        || params->ki < 0.0f || params->kd < 0.0f || params->max_output <= 0.0f
        || params->integral_limit < 0.0f || params->integral_separation < 0.0f
        || params->deadzone < 0.0f || params->transition_lpf_alpha < 0.0f
        || params->transition_lpf_alpha > 1.0f) return false;
    app_dt35_position.pid_params = *params;
    PID_Init(&app_dt35_position.x_pid, params->kp, params->ki, params->kd,
             10.0f, params->max_output, params->integral_limit);
    PID_Init(&app_dt35_position.y_pid, params->kp, params->ki, params->kd,
             10.0f, params->max_output, params->integral_limit);
    PID_SetIntegralSeparation(&app_dt35_position.x_pid, params->integral_separation);
    PID_SetIntegralSeparation(&app_dt35_position.y_pid, params->integral_separation);
    return true;
}

bool AppDt35Position_UpdateTransitionLpfAlpha(float alpha)
{
    if (!isfinite(alpha) || (alpha < 0.0f) || (alpha > 1.0f)) return false;
    app_dt35_position.pid_params.transition_lpf_alpha = alpha;
    return true;
}

static bool AppDt35Position_IsLongitudinal(
    app_dt35_position_direction_t direction)
{
    return (direction == APP_DT35_POSITION_BACK_TO_FRONT)
           || (direction == APP_DT35_POSITION_FRONT_TO_BACK);
}

static bool AppDt35Position_ShouldInvert(
    app_dt35_position_direction_t direction)
{
    switch (direction) {
    case APP_DT35_POSITION_BACK_TO_FRONT:
        return CHASSIS_DT35_INVERT_BACK_TO_FRONT != 0U;
    case APP_DT35_POSITION_FRONT_TO_BACK:
        return CHASSIS_DT35_INVERT_FRONT_TO_BACK != 0U;
    case APP_DT35_POSITION_LEFT_TO_RIGHT:
        return CHASSIS_DT35_INVERT_LEFT_TO_RIGHT != 0U;
    case APP_DT35_POSITION_RIGHT_TO_LEFT:
        return CHASSIS_DT35_INVERT_RIGHT_TO_LEFT != 0U;
    default:
        return false;
    }
}

static bool AppDt35Position_AxisComplete(
    app_dt35_position_direction_t direction, float feedback_m, float target_m)
{
    if ((direction == APP_DT35_POSITION_BACK_TO_FRONT)
        || (direction == APP_DT35_POSITION_LEFT_TO_RIGHT)) {
        return feedback_m >= (target_m - app_dt35_position.pid_params.deadzone);
    }
    return feedback_m <= (target_m + app_dt35_position.pid_params.deadzone);
}

static void AppDt35Position_WriteZero(uint32_t timestamp_ms,
                                      app_chassis_command_t *command)
{
    if (command != NULL) {
        memset(command, 0, sizeof(*command));
        command->timestamp_ms = timestamp_ms;
        command->valid = 1U;
    }
}

static void AppDt35Position_ResetTransitionFilter(void)
{
    app_dt35_position.transition_filter_pending = false;
    app_dt35_position.filtered_velocity_x_mps = 0.0f;
    app_dt35_position.filtered_velocity_y_mps = 0.0f;
}

static bool AppDt35Position_GetTarget(app_dt35_position_direction_t direction,
                                      app_dt35_position_target_t *target)
{
    if (target == NULL) return false;
    if (AppDt35Position_IsLongitudinal(direction)) {
        target->target_x_m = 0.0f;
        target->target_y_m = (direction == APP_DT35_POSITION_BACK_TO_FRONT)
                             ? app_dt35_position.targets.back_to_front_m
                             : app_dt35_position.targets.front_to_back_m;
    } else if ((direction == APP_DT35_POSITION_RIGHT_TO_LEFT)
               || (direction == APP_DT35_POSITION_LEFT_TO_RIGHT)) {
        target->target_x_m = (direction == APP_DT35_POSITION_LEFT_TO_RIGHT)
                             ? app_dt35_position.targets.left_to_right_m
                             : app_dt35_position.targets.right_to_left_m;
        target->target_y_m = 0.0f;
    } else {
        return false;
    }
    return true;
}

bool AppDt35Position_Init(void)
{
    memset(&app_dt35_position, 0, sizeof(app_dt35_position));
    if (!PID_Init(&app_dt35_position.x_pid, CHASSIS_STR_KP, CHASSIS_STR_KI,
                  CHASSIS_STR_KD, 10.0f, CHASSIS_STR_MAX_OUTPUT_DEGPS,
                  CHASSIS_STR_INTEGRAL_LIMIT)
        || !PID_Init(&app_dt35_position.y_pid, CHASSIS_STR_KP,
                     CHASSIS_STR_KI, CHASSIS_STR_KD, 10.0f,
                     CHASSIS_STR_MAX_OUTPUT_DEGPS,
                     CHASSIS_STR_INTEGRAL_LIMIT)) return false;
    PID_SetIntegralSeparation(&app_dt35_position.x_pid,
                              CHASSIS_STR_INTEGRAL_SEPARATION);
    PID_SetIntegralSeparation(&app_dt35_position.y_pid,
                              CHASSIS_STR_INTEGRAL_SEPARATION);
    app_dt35_position.pid_params.kp = CHASSIS_STR_KP;
    app_dt35_position.pid_params.ki = CHASSIS_STR_KI;
    app_dt35_position.pid_params.kd = CHASSIS_STR_KD;
    app_dt35_position.pid_params.max_output = CHASSIS_STR_MAX_OUTPUT_DEGPS;
    app_dt35_position.pid_params.integral_limit = CHASSIS_STR_INTEGRAL_LIMIT;
    app_dt35_position.pid_params.integral_separation = CHASSIS_STR_INTEGRAL_SEPARATION;
    app_dt35_position.pid_params.deadzone = CHASSIS_STR_DEADZONE;
    app_dt35_position.pid_params.transition_lpf_alpha = CHASSIS_DT35_TRANSITION_LPF_ALPHA;
    app_dt35_position.targets.back_to_front_m = CHASSIS_DT35_BLUE_GO_SEG1_TARGET_M;
    app_dt35_position.targets.front_to_back_m = CHASSIS_DT35_BLUE_REVERSE_SEG1_TARGET_M;
    app_dt35_position.targets.right_to_left_m = CHASSIS_DT35_BLUE_GO_SEG4_TARGET_M;
    app_dt35_position.targets.left_to_right_m = CHASSIS_DT35_BLUE_GO_SEG2_TARGET_M;
    app_dt35_position.status.code = APP_DT35_POSITION_IDLE;
    app_dt35_position.initialized = true;
    return true;
}

static bool AppDt35Position_StartDirection(app_dt35_position_direction_t direction,
                                           const float *target_m)
{
    app_dt35_position_target_t target;
    if ((!app_dt35_position.initialized)
        || (CHASSIS_DT35_TRANSFORM_ENABLED == 0U)) {
        app_dt35_position.status.code = APP_DT35_POSITION_REJECTED;
        return false;
    }
    if (target_m != NULL) {
        if (!isfinite(*target_m) || (*target_m < 0.0f)) {
            app_dt35_position.status.code = APP_DT35_POSITION_REJECTED;
            return false;
        }
        if (AppDt35Position_IsLongitudinal(direction)) {
            target.target_x_m = 0.0f;
            target.target_y_m = *target_m;
        } else if ((direction == APP_DT35_POSITION_RIGHT_TO_LEFT)
                   || (direction == APP_DT35_POSITION_LEFT_TO_RIGHT)) {
            target.target_x_m = *target_m;
            target.target_y_m = 0.0f;
        } else {
            app_dt35_position.status.code = APP_DT35_POSITION_REJECTED;
            return false;
        }
    } else if (!AppDt35Position_GetTarget(direction, &target)) {
        app_dt35_position.status.code = APP_DT35_POSITION_REJECTED;
        return false;
    }
    app_dt35_position.status.direction = direction;
    app_dt35_position.status.target_x_m = target.target_x_m;
    app_dt35_position.status.target_y_m = target.target_y_m;
    app_dt35_position.status.code = APP_DT35_POSITION_RUNNING;
    app_dt35_position.status.active = true;
    app_dt35_position.active = true;
    PID_Reset(&app_dt35_position.x_pid);
    PID_Reset(&app_dt35_position.y_pid);
    return true;
}

bool AppDt35Position_Start(app_dt35_position_direction_t direction)
{
    app_dt35_position.sequence_count = 0U;
    app_dt35_position.sequence_index = 0U;
    app_dt35_position.sequence_has_segment_targets = false;
    app_dt35_position.relative_forward_pending = false;
    app_dt35_position.relative_forward_active = false;
    AppDt35Position_ResetTransitionFilter();
    return AppDt35Position_StartDirection(direction, NULL);
}

bool AppDt35Position_StartSequence(const app_dt35_position_direction_t *directions,
                                   uint8_t count)
{
    uint8_t index;

    if ((directions == NULL) || (count == 0U) || (count > 5U)) {
        return false;
    }
    for (index = 0U; index < count; ++index) {
        app_dt35_position_target_t target;
        if (!AppDt35Position_GetTarget(directions[index], &target)) {
            return false;
        }
        app_dt35_position.sequence[index] = directions[index];
    }
    app_dt35_position.sequence_count = count;
    app_dt35_position.sequence_index = 0U;
    app_dt35_position.sequence_has_segment_targets = false;
    app_dt35_position.relative_forward_pending = false;
    app_dt35_position.relative_forward_active = false;
    AppDt35Position_ResetTransitionFilter();
    return AppDt35Position_StartDirection(app_dt35_position.sequence[0], NULL);
}

bool AppDt35Position_StartSequenceWithTargets(
    const app_dt35_position_direction_t *directions, uint8_t count,
    const app_dt35_targets_t *targets)
{
    app_dt35_targets_t previous_targets;

    if (!AppDt35Position_AreTargetsValid(targets)) return false;
    previous_targets = app_dt35_position.targets;
    app_dt35_position.targets = *targets;
    if (AppDt35Position_StartSequence(directions, count)) return true;
    app_dt35_position.targets = previous_targets;
    return false;
}

bool AppDt35Position_StartSequenceWithSegmentTargets(
    const app_dt35_position_direction_t *directions, const float *target_m,
    uint8_t count)
{
    uint8_t index;

    if ((directions == NULL) || (target_m == NULL) || (count == 0U)
        || (count > 5U)) return false;
    for (index = 0U; index < count; ++index) {
        app_dt35_position_target_t target;
        if (!AppDt35Position_GetTarget(directions[index], &target)
            || !isfinite(target_m[index]) || (target_m[index] < 0.0f)) return false;
        app_dt35_position.sequence[index] = directions[index];
        app_dt35_position.sequence_target_m[index] = target_m[index];
    }
    app_dt35_position.sequence_count = count;
    app_dt35_position.sequence_index = 0U;
    app_dt35_position.sequence_has_segment_targets = true;
    app_dt35_position.relative_forward_pending = false;
    app_dt35_position.relative_forward_active = false;
    AppDt35Position_ResetTransitionFilter();
    return AppDt35Position_StartDirection(app_dt35_position.sequence[0],
        &app_dt35_position.sequence_target_m[0]);
}

void AppDt35Position_Stop(void)
{
    PID_Reset(&app_dt35_position.x_pid);
    PID_Reset(&app_dt35_position.y_pid);
    app_dt35_position.active = false;
    app_dt35_position.sequence_count = 0U;
    app_dt35_position.sequence_index = 0U;
    app_dt35_position.sequence_has_segment_targets = false;
    app_dt35_position.relative_forward_pending = false;
    app_dt35_position.relative_forward_active = false;
    AppDt35Position_ResetTransitionFilter();
    app_dt35_position.status.active = false;
    if (app_dt35_position.status.code == APP_DT35_POSITION_RUNNING) {
        app_dt35_position.status.code = APP_DT35_POSITION_IDLE;
    }
}

bool AppDt35Position_GetStatus(app_dt35_position_status_t *status)
{
    if (status == NULL) return false;
    *status = app_dt35_position.status;
    status->active = app_dt35_position.active;
    return true;
}

bool AppDt35Position_TransformRawDistance(const app_dt35_sample_t *raw,
                                          float *x_feedback_m,
                                          float *y_feedback_m)
{
    if ((raw == NULL) || (x_feedback_m == NULL) || (y_feedback_m == NULL)) return false;
#if (CHASSIS_DT35_LEFT_SENSOR_INDEX == 0U)
    /* DT35-1 (frame X / distance_m[0]) faces left; DT35-2 faces rear. */
    *x_feedback_m = raw->distance_m[0];
    *y_feedback_m = raw->distance_m[1];
#elif (CHASSIS_DT35_LEFT_SENSOR_INDEX == 1U)
    /* DT35-2 (frame Y / distance_m[1]) faces left; DT35-1 faces rear. */
    *x_feedback_m = raw->distance_m[1];
    *y_feedback_m = raw->distance_m[0];
#else
#error "CHASSIS_DT35_LEFT_SENSOR_INDEX must be 0U or 1U"
#endif
    return true;
}

bool AppDt35Position_Step(const app_dt35_sample_t *raw, uint32_t timestamp_ms,
                          app_chassis_command_t *command)
{
    float x_feedback_m;
    float y_feedback_m;
    float x_output;
    float y_output;
    bool x_complete;
    bool y_complete;
    float target_x_m;
    float target_y_m;
    float y_pid_feedback_m;
    float y_pid_target_m;

    AppDt35Position_WriteZero(timestamp_ms, command);
    if (!app_dt35_position.active || (command == NULL)) return false;
    if ((raw == NULL) || (raw->valid == 0U)) {
        app_dt35_position.status.code = APP_DT35_POSITION_SENSOR_INVALID;
        AppDt35Position_Stop();
        return false;
    }
    if (!AppDt35Position_TransformRawDistance(raw, &x_feedback_m, &y_feedback_m)
        || !isfinite(x_feedback_m) || !isfinite(y_feedback_m)) {
        app_dt35_position.status.code = APP_DT35_POSITION_TRANSFORM_INVALID;
        AppDt35Position_Stop();
        return false;
    }

    target_x_m = app_dt35_position.status.target_x_m;
    target_y_m = app_dt35_position.status.target_y_m;
    y_pid_feedback_m = y_feedback_m;
    y_pid_target_m = target_y_m;
    if (app_dt35_position.relative_forward_pending
        && (app_dt35_position.status.direction == APP_DT35_POSITION_BACK_TO_FRONT)) {
        app_dt35_position.relative_forward_start_m = y_feedback_m;
        app_dt35_position.relative_forward_pending = false;
        app_dt35_position.relative_forward_active = true;
    }
    if (AppDt35Position_IsLongitudinal(app_dt35_position.status.direction)) {
        /* Front/rear motion uses only the rear DT35; do not correct sideways. */
        PID_Reset(&app_dt35_position.x_pid);
        x_complete = true;
        if (app_dt35_position.relative_forward_active) {
            y_pid_feedback_m -= app_dt35_position.relative_forward_start_m;
            y_complete = y_pid_feedback_m
                         >= (target_y_m - app_dt35_position.pid_params.deadzone);
        } else {
            y_complete = AppDt35Position_AxisComplete(
                app_dt35_position.status.direction, y_feedback_m, target_y_m);
        }
        if (!y_complete) {
            y_output = PID_Calc(&app_dt35_position.y_pid,
                                y_pid_target_m, y_pid_feedback_m);
            if (AppDt35Position_ShouldInvert(
                    app_dt35_position.status.direction)) {
                y_output = -y_output;
            }
            command->velocity_x_mps = y_output;
        } else {
            PID_Reset(&app_dt35_position.y_pid);
        }
    } else {
        /* Left/right motion uses only the left DT35; do not correct fore/aft. */
        PID_Reset(&app_dt35_position.y_pid);
        y_complete = true;
        x_complete = AppDt35Position_AxisComplete(
            app_dt35_position.status.direction, x_feedback_m, target_x_m);
        if (!x_complete) {
            x_output = PID_Calc(&app_dt35_position.x_pid,
                                target_x_m, x_feedback_m);
            if (AppDt35Position_ShouldInvert(
                    app_dt35_position.status.direction)) {
                x_output = -x_output;
            }
            command->velocity_y_mps = x_output;
        } else {
            PID_Reset(&app_dt35_position.x_pid);
        }
    }
    if (x_complete && y_complete) {
        if ((app_dt35_position.sequence_index + 1U)
            < app_dt35_position.sequence_count) {
            app_dt35_position.sequence_index++;
            app_dt35_position.relative_forward_active = false;
            app_dt35_position.relative_forward_pending =
                (app_dt35_position.sequence[app_dt35_position.sequence_index]
                 == APP_DT35_POSITION_BACK_TO_FRONT);
            AppDt35Position_ResetTransitionFilter();
            app_dt35_position.transition_filter_pending = true;
            if (!AppDt35Position_StartDirection(
                    app_dt35_position.sequence[app_dt35_position.sequence_index],
                    app_dt35_position.sequence_has_segment_targets
                    ? &app_dt35_position.sequence_target_m[app_dt35_position.sequence_index]
                    : NULL)) {
                AppDt35Position_ResetTransitionFilter();
                return false;
            }
            return true;
        }
        app_dt35_position.active = false;
        app_dt35_position.sequence_count = 0U;
        app_dt35_position.sequence_has_segment_targets = false;
        app_dt35_position.relative_forward_pending = false;
        app_dt35_position.relative_forward_active = false;
        app_dt35_position.status.active = false;
        app_dt35_position.status.code = APP_DT35_POSITION_COMPLETED;
        PID_Reset(&app_dt35_position.x_pid);
        PID_Reset(&app_dt35_position.y_pid);
        AppDt35Position_ResetTransitionFilter();
    } else if (app_dt35_position.transition_filter_pending) {
        float alpha = app_dt35_position.pid_params.transition_lpf_alpha;
        app_dt35_position.filtered_velocity_x_mps += alpha
            * (command->velocity_x_mps - app_dt35_position.filtered_velocity_x_mps);
        app_dt35_position.filtered_velocity_y_mps += alpha
            * (command->velocity_y_mps - app_dt35_position.filtered_velocity_y_mps);
        command->velocity_x_mps = app_dt35_position.filtered_velocity_x_mps;
        command->velocity_y_mps = app_dt35_position.filtered_velocity_y_mps;
        app_dt35_position.transition_filter_pending = false;
    } else if (app_dt35_position.sequence_count > 0U) {
        app_dt35_position.filtered_velocity_x_mps = command->velocity_x_mps;
        app_dt35_position.filtered_velocity_y_mps = command->velocity_y_mps;
    }
    return true;
}
