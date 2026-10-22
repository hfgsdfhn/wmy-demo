#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "pid_config.h"

static void AssertNear(float actual, float expected)
{
    assert(fabsf(actual - expected) < 0.000001f);
}

int main(void)
{
    assert(CHASSIS_YAW_ACTION_LEFT_90_ID == 7U);
    assert(CHASSIS_YAW_ACTION_RIGHT_90_ID == 8U);
    AssertNear(CHASSIS_YAW_90_KP, 1.8f);
    AssertNear(CHASSIS_YAW_90_KI, 8.0f);
    AssertNear(CHASSIS_YAW_90_KD, 0.002f);
    AssertNear(CHASSIS_YAW_90_INTEGRAL_LIMIT, 30.0f);
    AssertNear(CHASSIS_YAW_90_INTEGRAL_SEPARATION, 10.0f);
    AssertNear(CHASSIS_YAW_90_MAX_OUTPUT_DEGPS, 150.0f);
    AssertNear(CHASSIS_YAW_90_DEADZONE, 3.0f);
    AssertNear(CHASSIS_YAW_QE_KP, 6.0f);
    AssertNear(CHASSIS_YAW_QE_KI, 8.0f);
    AssertNear(CHASSIS_YAW_QE_KD, 0.008f);
    AssertNear(CHASSIS_YAW_QE_INTEGRAL_LIMIT, 30.0f);
    AssertNear(CHASSIS_YAW_QE_INTEGRAL_SEPARATION, 10.0f);
    AssertNear(CHASSIS_YAW_QE_MAX_OUTPUT_DEGPS, 150.0f);
    AssertNear(CHASSIS_YAW_QE_DEADZONE, 0.0f);
    AssertNear(CHASSIS_YAW_QE_STEP_DEG, 5.0f);

    puts("yaw_pid_config_test passed");
    return 0;
}
