#include <assert.h>
#include <stdio.h>

#include "remote_buttons.h"
#include "pid_config.h"

static uint32_t test_tick_ms;

uint32_t HAL_GetTick(void)
{
    return test_tick_ms;
}

static app_dt35_sample_t MakeSample(float rear_m, float left_m)
{
#if (CHASSIS_DT35_LEFT_SENSOR_INDEX == 0U)
    app_dt35_sample_t sample = { 0U, { left_m, rear_m }, 1U };
#else
    app_dt35_sample_t sample = { 0U, { rear_m, left_m }, 1U };
#endif
    return sample;
}

static void StartRemoteKey(uint32_t key, uint16_t session_id)
{
    app_remote_control_t control = { 0 };

    control.session_id = session_id;
    control.switches = APP_REMOTE_SWITCH_PC0_REGION_ENABLE;
    test_tick_ms += 1U;
    AppDt35Buttons_Scan(&control);

    control.keys = key;
    test_tick_ms += 1U;
    AppDt35Buttons_Scan(&control);
    test_tick_ms += CHASSIS_DT35_BUTTON_DEBOUNCE_MS;
    AppDt35Buttons_Scan(&control);
}

static void CompleteDirection(app_dt35_position_direction_t expected_direction,
                               float target_m,
                               bool relative_longitudinal)
{
    app_dt35_position_status_t status;
    app_chassis_command_t command;
    app_dt35_sample_t sample;

    assert(AppDt35Position_GetStatus(&status));
    assert(status.active);
    assert(status.direction == expected_direction);
    if ((expected_direction == APP_DT35_POSITION_BACK_TO_FRONT)
        || (expected_direction == APP_DT35_POSITION_FRONT_TO_BACK)) {
        assert(status.target_x_m == 0.0f);
        assert(status.target_y_m == target_m);
    } else {
        assert(status.target_x_m == target_m);
        assert(status.target_y_m == 0.0f);
    }

    switch (expected_direction) {
    case APP_DT35_POSITION_BACK_TO_FRONT:
        sample = MakeSample(target_m, 0.0f);
        break;
    case APP_DT35_POSITION_FRONT_TO_BACK:
        sample = MakeSample(target_m, 0.0f);
        break;
    case APP_DT35_POSITION_RIGHT_TO_LEFT:
        sample = MakeSample(0.0f, target_m);
        break;
    case APP_DT35_POSITION_LEFT_TO_RIGHT:
        sample = MakeSample(0.0f, target_m);
        break;
    default:
        assert(0);
        return;
    }
    assert(AppDt35Position_Step(&sample, test_tick_ms, &command));

    assert(AppDt35Position_GetStatus(&status));
    if (relative_longitudinal) {
        assert(status.active);
        assert(status.direction == expected_direction);
        assert(command.velocity_x_mps != 0.0f);
        if (expected_direction == APP_DT35_POSITION_BACK_TO_FRONT) {
            sample = MakeSample(2.0f * target_m, 0.0f);
        } else {
            sample = MakeSample(0.0f, 0.0f);
        }
        assert(AppDt35Position_Step(&sample, test_tick_ms + 1U, &command));
    }
}

static void AssertSequence(uint32_t key, uint16_t session_id,
                            const app_dt35_position_direction_t *expected,
                            const float *target_m, uint8_t count)
{
    app_dt35_position_status_t status;
    uint8_t index;

    StartRemoteKey(key, session_id);
    for (index = 0U; index < count; ++index) {
        CompleteDirection(expected[index], target_m[index], (index > 0U)
                          && (expected[index] == APP_DT35_POSITION_BACK_TO_FRONT));
    }
    assert(AppDt35Position_GetStatus(&status));
    assert(!status.active);
    assert(status.code == APP_DT35_POSITION_COMPLETED);
    AppDt35Buttons_CancelRemote();
}

static void TestRedSequences(void)
{
    static const float red_go_targets[] = {
        CHASSIS_DT35_RED_GO_SEG1_TARGET_M, CHASSIS_DT35_RED_GO_SEG2_TARGET_M,
        CHASSIS_DT35_RED_GO_SEG3_TARGET_M, CHASSIS_DT35_RED_GO_SEG4_TARGET_M,
        CHASSIS_DT35_RED_GO_SEG5_TARGET_M
    };
    static const float red_back_targets[] = {
        CHASSIS_DT35_RED_BACK_SEG1_TARGET_M, CHASSIS_DT35_RED_BACK_SEG2_TARGET_M,
        CHASSIS_DT35_RED_BACK_SEG3_TARGET_M, CHASSIS_DT35_RED_BACK_SEG4_TARGET_M,
        CHASSIS_DT35_RED_BACK_SEG5_TARGET_M
    };
    static const float red_reverse_targets[] = {
        CHASSIS_DT35_RED_REVERSE_SEG1_TARGET_M,
        CHASSIS_DT35_RED_REVERSE_SEG2_TARGET_M,
        CHASSIS_DT35_RED_REVERSE_SEG3_TARGET_M,
        CHASSIS_DT35_RED_REVERSE_SEG4_TARGET_M
    };
    static const app_dt35_position_direction_t pb8_expected[] = {
        APP_DT35_POSITION_FRONT_TO_BACK, APP_DT35_POSITION_LEFT_TO_RIGHT,
        APP_DT35_POSITION_FRONT_TO_BACK, APP_DT35_POSITION_RIGHT_TO_LEFT
    };
    static const app_dt35_position_direction_t pb5_expected[] = {
        APP_DT35_POSITION_BACK_TO_FRONT, APP_DT35_POSITION_RIGHT_TO_LEFT,
        APP_DT35_POSITION_BACK_TO_FRONT, APP_DT35_POSITION_LEFT_TO_RIGHT,
        APP_DT35_POSITION_BACK_TO_FRONT
    };
    static const app_dt35_position_direction_t pb6_expected[] = {
        APP_DT35_POSITION_BACK_TO_FRONT, APP_DT35_POSITION_LEFT_TO_RIGHT,
        APP_DT35_POSITION_BACK_TO_FRONT, APP_DT35_POSITION_RIGHT_TO_LEFT,
        APP_DT35_POSITION_BACK_TO_FRONT
    };

    AssertSequence(PB8, 1U, pb8_expected, red_reverse_targets, 4U);
    AssertSequence(PB5, 2U, pb5_expected, red_go_targets, 5U);
    AssertSequence(PB6, 3U, pb6_expected, red_back_targets, 5U);
}

static void TestBlueSequences(void)
{
    static const float blue_go_targets[] = {
        CHASSIS_DT35_BLUE_GO_SEG1_TARGET_M, CHASSIS_DT35_BLUE_GO_SEG2_TARGET_M,
        CHASSIS_DT35_BLUE_GO_SEG3_TARGET_M, CHASSIS_DT35_BLUE_GO_SEG4_TARGET_M,
        CHASSIS_DT35_BLUE_GO_SEG5_TARGET_M
    };
    static const float blue_back_targets[] = {
        CHASSIS_DT35_BLUE_BACK_SEG1_TARGET_M, CHASSIS_DT35_BLUE_BACK_SEG2_TARGET_M,
        CHASSIS_DT35_BLUE_BACK_SEG3_TARGET_M, CHASSIS_DT35_BLUE_BACK_SEG4_TARGET_M,
        CHASSIS_DT35_BLUE_BACK_SEG5_TARGET_M
    };
    static const float blue_reverse_targets[] = {
        CHASSIS_DT35_BLUE_REVERSE_SEG1_TARGET_M,
        CHASSIS_DT35_BLUE_REVERSE_SEG2_TARGET_M,
        CHASSIS_DT35_BLUE_REVERSE_SEG3_TARGET_M,
        CHASSIS_DT35_BLUE_REVERSE_SEG4_TARGET_M
    };
    static const app_dt35_position_direction_t pa11_expected[] = {
        APP_DT35_POSITION_BACK_TO_FRONT, APP_DT35_POSITION_LEFT_TO_RIGHT,
        APP_DT35_POSITION_BACK_TO_FRONT, APP_DT35_POSITION_RIGHT_TO_LEFT,
        APP_DT35_POSITION_BACK_TO_FRONT
    };
    static const app_dt35_position_direction_t pa12_expected[] = {
        APP_DT35_POSITION_BACK_TO_FRONT, APP_DT35_POSITION_RIGHT_TO_LEFT,
        APP_DT35_POSITION_BACK_TO_FRONT, APP_DT35_POSITION_LEFT_TO_RIGHT,
        APP_DT35_POSITION_BACK_TO_FRONT
    };
    static const app_dt35_position_direction_t pa8_expected[] = {
        APP_DT35_POSITION_FRONT_TO_BACK, APP_DT35_POSITION_RIGHT_TO_LEFT,
        APP_DT35_POSITION_FRONT_TO_BACK, APP_DT35_POSITION_LEFT_TO_RIGHT
    };

    AssertSequence(PA11, 4U, pa11_expected, blue_go_targets, 5U);
    AssertSequence(PA12, 5U, pa12_expected, blue_back_targets, 5U);
    AssertSequence(PA8, 6U, pa8_expected, blue_reverse_targets, 4U);
}

static void TestPc7YawZeroRequest(void)
{
    app_remote_control_t control = { 0 };

    control.session_id = 7U;
    control.switches = APP_REMOTE_SWITCH_PC0_REGION_ENABLE;
    test_tick_ms += 1U;
    AppDt35Buttons_Scan(&control);
    control.keys = PC7;
    test_tick_ms += 1U;
    AppDt35Buttons_Scan(&control);
    test_tick_ms += CHASSIS_DT35_BUTTON_DEBOUNCE_MS;
    AppDt35Buttons_Scan(&control);
    assert(AppDt35Buttons_TakePendingYawZeroRequest());
    assert(!AppDt35Buttons_TakePendingYawZeroRequest());
}

int main(void)
{
    assert(AppDt35Position_Init());
    TestRedSequences();
    TestBlueSequences();
    TestPc7YawZeroRequest();
    puts("dt35_buttons_test passed");
    return 0;
}
