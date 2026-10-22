#include "yaw_ekf.h"

#include <math.h>
#include <string.h>

#define YAW_EKF_DEFAULT_WHEEL_RATE_VAR 0.3f
#define YAW_EKF_DEFAULT_GYRO_RATE_VAR 0.003f
#define YAW_EKF_DEFAULT_BIAS_WALK_VAR 0.000001f

static bool YawEkf_IsValidVariance(float value)
{
    return isfinite(value) && (value >= 1.0e-10f) && (value <= 10000.0f);
}

static float YawEkf_NormalizeAngle(float angle_rad)
{
    const float pi = 3.14159265358979323846f;
    const float two_pi = 6.28318530717958647692f;

    while (angle_rad >= pi) angle_rad -= two_pi;
    while (angle_rad < -pi) angle_rad += two_pi;
    return angle_rad;
}

void YawEkf_Reset(yaw_ekf_t *filter, float initial_yaw_rad)
{
    if (filter == NULL) return;

    memset(filter, 0, sizeof(*filter));
    filter->yaw_rad = isfinite(initial_yaw_rad)
                    ? YawEkf_NormalizeAngle(initial_yaw_rad) : 0.0f;
    filter->covariance[0][0] = 1.0f;
    filter->covariance[1][1] = 1.0f;
    filter->params.wheel_rate_var = YAW_EKF_DEFAULT_WHEEL_RATE_VAR;
    filter->params.gyro_rate_var = YAW_EKF_DEFAULT_GYRO_RATE_VAR;
    filter->params.bias_random_walk_var = YAW_EKF_DEFAULT_BIAS_WALK_VAR;
}

bool YawEkf_SetParameters(yaw_ekf_t *filter, const yaw_ekf_params_t *params)
{
    if ((filter == NULL) || (params == NULL)
        || !YawEkf_IsValidVariance(params->wheel_rate_var)
        || !YawEkf_IsValidVariance(params->gyro_rate_var)
        || !YawEkf_IsValidVariance(params->bias_random_walk_var)) return false;

    filter->params = *params;
    return true;
}

bool YawEkf_GetParameters(const yaw_ekf_t *filter, yaw_ekf_params_t *params)
{
    if ((filter == NULL) || (params == NULL)) return false;
    *params = filter->params;
    return true;
}

void YawEkf_Update(yaw_ekf_t *filter, float gyro_rate_radps,
                   float wheel_rate_radps, bool wheel_rate_valid, float dt_s)
{
    float gyro_corrected;
    float fused_rate;

    if ((filter == NULL) || !isfinite(gyro_rate_radps)
        || !isfinite(dt_s) || (dt_s <= 0.0f)) return;

    gyro_corrected = gyro_rate_radps - filter->gyro_bias_radps;
    fused_rate = gyro_corrected;
    if (wheel_rate_valid && isfinite(wheel_rate_radps)
        && (filter->params.gyro_rate_var > 0.0f)
        && (filter->params.wheel_rate_var > 0.0f)) {
        float gyro_weight = 1.0f / filter->params.gyro_rate_var;
        float wheel_weight = 1.0f / filter->params.wheel_rate_var;
        fused_rate = (gyro_weight * gyro_corrected
                      + wheel_weight * wheel_rate_radps)
                   / (gyro_weight + wheel_weight);
    }
    filter->fused_rate_radps = fused_rate;
    filter->yaw_rad = YawEkf_NormalizeAngle(filter->yaw_rad + fused_rate * dt_s);
}

float YawEkf_GetYaw(const yaw_ekf_t *filter)
{
    return (filter != NULL) ? filter->yaw_rad : 0.0f;
}

float YawEkf_GetYawRate(const yaw_ekf_t *filter, float gyro_rate_radps)
{
    (void)gyro_rate_radps;
    return (filter != NULL) ? filter->fused_rate_radps : 0.0f;
}
