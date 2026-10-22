#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "app_dt35_position.h"
#include "pid_config.h"

#define FLOAT_EPSILON 0.0001f

static app_dt35_sample_t MakeSample(float rear_m, float left_m)
{
#if (CHASSIS_DT35_LEFT_SENSOR_INDEX == 0U)
    app_dt35_sample_t sample = { 0U, { left_m, rear_m }, 1U };
#else
    app_dt35_sample_t sample = { 0U, { rear_m, left_m }, 1U };
#endif
    return sample;
}

static float ExpectedOutput(float target, float feedback)
{
    float output = CHASSIS_STR_KP * (target - feedback);

    if (output > CHASSIS_STR_MAX_OUTPUT_DEGPS) return CHASSIS_STR_MAX_OUTPUT_DEGPS;
    if (output < -CHASSIS_STR_MAX_OUTPUT_DEGPS) return -CHASSIS_STR_MAX_OUTPUT_DEGPS;
    return output;
}

static void TestSingleDirectionIsUnfiltered(void)
{
    app_chassis_command_t command;
    app_dt35_sample_t sample = MakeSample(0.1f, 0.1f);

    assert(AppDt35Position_Start(APP_DT35_POSITION_BACK_TO_FRONT));
    assert(AppDt35Position_Step(&sample, 100U, &command));
    assert(fabsf(command.velocity_x_mps
                  - ExpectedOutput(CHASSIS_DT35_BLUE_GO_SEG1_TARGET_M, 0.1f))
           < FLOAT_EPSILON);
    assert(fabsf(command.velocity_y_mps) < FLOAT_EPSILON);
    AppDt35Position_Stop();
}

static void TestSequenceTransitionFilter(void)
{
    const app_dt35_position_direction_t sequence[2] = {
        APP_DT35_POSITION_BACK_TO_FRONT,
        APP_DT35_POSITION_RIGHT_TO_LEFT
    };
    app_chassis_command_t command;
    app_dt35_sample_t sample;
    float next_vy;

    assert(AppDt35Position_UpdateTransitionLpfAlpha(0.2f));
    assert(AppDt35Position_StartSequence(sequence, 2U));
    sample = MakeSample(0.1f, 0.1f);
    assert(AppDt35Position_Step(&sample, 200U, &command));
    assert(fabsf(command.velocity_x_mps
                 - ExpectedOutput(CHASSIS_DT35_BLUE_GO_SEG1_TARGET_M, 0.1f))
           < FLOAT_EPSILON);

    next_vy = -ExpectedOutput(CHASSIS_DT35_BLUE_GO_SEG4_TARGET_M,
                              CHASSIS_DT35_BLUE_GO_SEG4_TARGET_M + 0.1f);
    sample = MakeSample(CHASSIS_DT35_BLUE_GO_SEG1_TARGET_M,
                        CHASSIS_DT35_BLUE_GO_SEG4_TARGET_M + 0.1f);
    assert(AppDt35Position_Step(&sample, 210U, &command));
    assert(fabsf(command.velocity_x_mps) < FLOAT_EPSILON);
    assert(fabsf(command.velocity_y_mps) < FLOAT_EPSILON);
    assert(AppDt35Position_Step(&sample, 220U, &command));
    assert(fabsf(command.velocity_x_mps) < FLOAT_EPSILON);
    assert(fabsf(command.velocity_y_mps - 0.2f * next_vy) < FLOAT_EPSILON);
    AppDt35Position_Stop();
}

static void TestAlphaBoundsAndReset(void)
{
    app_chassis_command_t command;
    app_dt35_sample_t sample = MakeSample(0.1f, 0.1f);

    assert(!AppDt35Position_UpdateTransitionLpfAlpha(-0.01f));
    assert(!AppDt35Position_UpdateTransitionLpfAlpha(1.01f));
    assert(!AppDt35Position_UpdateTransitionLpfAlpha(NAN));
    assert(AppDt35Position_UpdateTransitionLpfAlpha(0.0f));
    assert(AppDt35Position_Start(APP_DT35_POSITION_BACK_TO_FRONT));
    assert(AppDt35Position_Step(&sample, 300U, &command));
    assert(fabsf(command.velocity_x_mps
                 - ExpectedOutput(CHASSIS_DT35_BLUE_GO_SEG1_TARGET_M, 0.1f))
           < FLOAT_EPSILON);
    AppDt35Position_Stop();
}

static void TestRepeatedForwardLegUsesRelativeFeedback(void)
{
    const app_dt35_position_direction_t sequence[3] = {
        APP_DT35_POSITION_BACK_TO_FRONT,
        APP_DT35_POSITION_RIGHT_TO_LEFT,
        APP_DT35_POSITION_BACK_TO_FRONT
    };
    app_chassis_command_t command;
    app_dt35_position_status_t status;
    app_dt35_sample_t sample;

    assert(AppDt35Position_UpdateTransitionLpfAlpha(0.2f));
    assert(AppDt35Position_StartSequence(sequence, 3U));
    sample = MakeSample(0.1f, 0.1f);
    assert(AppDt35Position_Step(&sample, 400U, &command));
    sample = MakeSample(CHASSIS_DT35_BLUE_GO_SEG1_TARGET_M, 0.1f);
    assert(AppDt35Position_Step(&sample, 410U, &command));
    sample = MakeSample(CHASSIS_DT35_BLUE_GO_SEG1_TARGET_M,
                          CHASSIS_DT35_BLUE_GO_SEG4_TARGET_M);
    assert(AppDt35Position_Step(&sample, 420U, &command));
    assert(AppDt35Position_GetStatus(&status));
    assert(status.direction == APP_DT35_POSITION_BACK_TO_FRONT);
    assert(fabsf(status.target_y_m - CHASSIS_DT35_BLUE_GO_SEG1_TARGET_M)
           < FLOAT_EPSILON);
    assert(AppDt35Position_Step(&sample, 430U, &command));
    assert(command.velocity_x_mps > 0.0f);
    AppDt35Position_Stop();
}

int main(void)
{
    assert(AppDt35Position_Init());
    TestSingleDirectionIsUnfiltered();
    TestSequenceTransitionFilter();
    TestAlphaBoundsAndReset();
    TestRepeatedForwardLegUsesRelativeFeedback();
    puts("dt35_position_test passed");
    return 0;
}
