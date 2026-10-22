#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "yaw_ekf.h"

int main(void)
{
    yaw_ekf_t filter;
    const float dt_s = 0.002f;
    const float actual_rate_radps = 0.6f;
    const float gyro_bias_radps = 0.12f;
    yaw_ekf_params_t params = {0.08f, 0.004f, 0.000002f};
    unsigned int i;

    YawEkf_Reset(&filter, 0.0f);
    assert(YawEkf_SetParameters(&filter, &params));

    /* The wheel rate is the reference angular rate; the gyro has fixed bias. */
    for (i = 0U; i < 15000U; ++i)
    {
        YawEkf_Update(&filter, actual_rate_radps + gyro_bias_radps,
                      actual_rate_radps, true, dt_s);
    }

    assert(fabsf(filter.gyro_bias_radps - gyro_bias_radps) < 0.003f);
    assert(fabsf(YawEkf_GetYawRate(&filter,
                                   actual_rate_radps + gyro_bias_radps)
                  - actual_rate_radps) < 0.003f);
    assert(YawEkf_GetParameters(&filter, &params));
    assert(fabsf(params.wheel_rate_var - 0.08f) < 0.000001f);

    YawEkf_Reset(&filter, 0.0f);
    for (i = 0U; i < 500U; ++i)
    {
        YawEkf_Update(&filter, actual_rate_radps, 0.0f, false, dt_s);
    }
    assert(fabsf(YawEkf_GetYaw(&filter) - actual_rate_radps) < 0.003f);
    assert(fabsf(filter.gyro_bias_radps) < 0.000001f);
    assert(!YawEkf_SetParameters(&filter,
                                  &(yaw_ekf_params_t){0.0f, 0.004f, 0.000002f}));
    puts("yaw_ekf_test passed");
    return 0;
}
