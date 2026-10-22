#ifndef YAW_EKF_H
#define YAW_EKF_H

#include <stdbool.h>

typedef struct
{
    float wheel_rate_var;
    float gyro_rate_var;
    float bias_random_walk_var;
} yaw_ekf_params_t;

typedef struct
{
    float yaw_rad;
    float gyro_bias_radps;
    float covariance[2][2];
    yaw_ekf_params_t params;
    float fused_rate_radps;
} yaw_ekf_t;

void YawEkf_Reset(yaw_ekf_t *filter, float initial_yaw_rad);
bool YawEkf_SetParameters(yaw_ekf_t *filter, const yaw_ekf_params_t *params);
bool YawEkf_GetParameters(const yaw_ekf_t *filter, yaw_ekf_params_t *params);
void YawEkf_Update(yaw_ekf_t *filter, float gyro_rate_radps,
                   float wheel_rate_radps, bool wheel_rate_valid, float dt_s);
float YawEkf_GetYaw(const yaw_ekf_t *filter);
float YawEkf_GetYawRate(const yaw_ekf_t *filter, float gyro_rate_radps);

#endif /* YAW_EKF_H */
