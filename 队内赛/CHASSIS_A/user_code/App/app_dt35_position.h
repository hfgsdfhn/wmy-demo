#ifndef APP_DT35_POSITION_H
#define APP_DT35_POSITION_H

#include <stdbool.h>
#include <stdint.h>

#include "app_messages.h"

typedef enum
{
    APP_DT35_POSITION_BACK_TO_FRONT = 0,
    APP_DT35_POSITION_FRONT_TO_BACK,
    APP_DT35_POSITION_RIGHT_TO_LEFT,
    APP_DT35_POSITION_LEFT_TO_RIGHT
} app_dt35_position_direction_t;

typedef enum
{
    APP_DT35_POSITION_IDLE = 0,
    APP_DT35_POSITION_RUNNING,
    APP_DT35_POSITION_COMPLETED,
    APP_DT35_POSITION_REJECTED,
    APP_DT35_POSITION_SENSOR_INVALID,
    APP_DT35_POSITION_TRANSFORM_INVALID
} app_dt35_position_status_code_t;

typedef struct
{
    float kp;
    float ki;
    float kd;
    float max_output;
    float integral_limit;
    float integral_separation;
    float deadzone;
    float transition_lpf_alpha;
} app_dt35_pid_params_t;

typedef struct
{
    float back_to_front_m;
    float front_to_back_m;
    float right_to_left_m;
    float left_to_right_m;
} app_dt35_targets_t;

typedef struct
{
    app_dt35_position_status_code_t code;
    app_dt35_position_direction_t direction;
    bool active;
    float target_x_m;
    float target_y_m;
} app_dt35_position_status_t;

bool AppDt35Position_Init(void);
bool AppDt35Position_UpdatePidParams(const app_dt35_pid_params_t *params);
bool AppDt35Position_UpdateTransitionLpfAlpha(float alpha);
bool AppDt35Position_UpdateTargets(const app_dt35_targets_t *targets);
bool AppDt35Position_Start(app_dt35_position_direction_t direction);
bool AppDt35Position_StartSequence(const app_dt35_position_direction_t *directions,
                                   uint8_t count);
bool AppDt35Position_StartSequenceWithTargets(
    const app_dt35_position_direction_t *directions, uint8_t count,
    const app_dt35_targets_t *targets);
bool AppDt35Position_StartSequenceWithSegmentTargets(
    const app_dt35_position_direction_t *directions, const float *target_m,
    uint8_t count);
void AppDt35Position_Stop(void);
bool AppDt35Position_GetStatus(app_dt35_position_status_t *status);
bool AppDt35Position_TransformRawDistance(const app_dt35_sample_t *raw,
                                          float *x_feedback_m,
                                          float *y_feedback_m);
bool AppDt35Position_Step(const app_dt35_sample_t *raw,
                          uint32_t timestamp_ms,
                          app_chassis_command_t *command);

#endif /* APP_DT35_POSITION_H */
