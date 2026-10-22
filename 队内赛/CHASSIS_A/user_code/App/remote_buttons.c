#include "remote_buttons.h"

#include <stdint.h>

#include "app_dt35_position.h"
#include "pid_config.h"

static uint32_t app_dt35_candidate_keys;
static uint32_t app_dt35_stable_keys;
static uint32_t app_dt35_candidate_since_ms;
static bool app_dt35_pc_control_active;
static bool app_dt35_remote_sequence_active;
static uint8_t app_dt35_pending_yaw_action;
static bool app_dt35_pending_yaw_zero;
static uint16_t app_dt35_remote_session_id;
static bool app_dt35_remote_session_valid;

static const app_dt35_position_direction_t app_dt35_blue_go_sequence[] = { /* PA11 蓝区走 */
    APP_DT35_POSITION_BACK_TO_FRONT, APP_DT35_POSITION_LEFT_TO_RIGHT,
    APP_DT35_POSITION_BACK_TO_FRONT, APP_DT35_POSITION_RIGHT_TO_LEFT,
    APP_DT35_POSITION_BACK_TO_FRONT
};

static const app_dt35_position_direction_t app_dt35_blue_back_sequence[] = { /* PA12 蓝区回 */
    APP_DT35_POSITION_BACK_TO_FRONT, APP_DT35_POSITION_RIGHT_TO_LEFT,
    APP_DT35_POSITION_BACK_TO_FRONT, APP_DT35_POSITION_LEFT_TO_RIGHT,
    APP_DT35_POSITION_BACK_TO_FRONT
};

static const app_dt35_position_direction_t app_dt35_red_go_sequence[] = { /* PB5 红区走 */
    APP_DT35_POSITION_BACK_TO_FRONT, APP_DT35_POSITION_RIGHT_TO_LEFT,
    APP_DT35_POSITION_BACK_TO_FRONT, APP_DT35_POSITION_LEFT_TO_RIGHT,
    APP_DT35_POSITION_BACK_TO_FRONT
};
static const app_dt35_position_direction_t app_dt35_red_back_sequence[] = { /* PB6 红区回 */
    APP_DT35_POSITION_BACK_TO_FRONT, APP_DT35_POSITION_LEFT_TO_RIGHT,
    APP_DT35_POSITION_BACK_TO_FRONT, APP_DT35_POSITION_RIGHT_TO_LEFT,
    APP_DT35_POSITION_BACK_TO_FRONT
};
static const app_dt35_position_direction_t app_dt35_red_reverse_sequence[] = { /* PB8 红区倒着走 */
    APP_DT35_POSITION_FRONT_TO_BACK, APP_DT35_POSITION_LEFT_TO_RIGHT,
    APP_DT35_POSITION_FRONT_TO_BACK, APP_DT35_POSITION_RIGHT_TO_LEFT
};
static const app_dt35_position_direction_t app_dt35_blue_reverse_sequence[] = { /* PA8 蓝区倒着走 */
    APP_DT35_POSITION_FRONT_TO_BACK, APP_DT35_POSITION_RIGHT_TO_LEFT,
    APP_DT35_POSITION_FRONT_TO_BACK, APP_DT35_POSITION_LEFT_TO_RIGHT
};
static const float app_dt35_blue_go_targets[] = {
    CHASSIS_DT35_BLUE_GO_SEG1_TARGET_M, CHASSIS_DT35_BLUE_GO_SEG2_TARGET_M,
    CHASSIS_DT35_BLUE_GO_SEG3_TARGET_M, CHASSIS_DT35_BLUE_GO_SEG4_TARGET_M,
    CHASSIS_DT35_BLUE_GO_SEG5_TARGET_M
};
static const float app_dt35_blue_back_targets[] = {
    CHASSIS_DT35_BLUE_BACK_SEG1_TARGET_M, CHASSIS_DT35_BLUE_BACK_SEG2_TARGET_M,
    CHASSIS_DT35_BLUE_BACK_SEG3_TARGET_M, CHASSIS_DT35_BLUE_BACK_SEG4_TARGET_M,
    CHASSIS_DT35_BLUE_BACK_SEG5_TARGET_M
};
static const float app_dt35_blue_reverse_targets[] = {
    CHASSIS_DT35_BLUE_REVERSE_SEG1_TARGET_M,
    CHASSIS_DT35_BLUE_REVERSE_SEG2_TARGET_M,
    CHASSIS_DT35_BLUE_REVERSE_SEG3_TARGET_M,
    CHASSIS_DT35_BLUE_REVERSE_SEG4_TARGET_M
};
static const float app_dt35_red_go_targets[] = {
    CHASSIS_DT35_RED_GO_SEG1_TARGET_M, CHASSIS_DT35_RED_GO_SEG2_TARGET_M,
    CHASSIS_DT35_RED_GO_SEG3_TARGET_M, CHASSIS_DT35_RED_GO_SEG4_TARGET_M,
    CHASSIS_DT35_RED_GO_SEG5_TARGET_M
};
static const float app_dt35_red_back_targets[] = {
    CHASSIS_DT35_RED_BACK_SEG1_TARGET_M, CHASSIS_DT35_RED_BACK_SEG2_TARGET_M,
    CHASSIS_DT35_RED_BACK_SEG3_TARGET_M, CHASSIS_DT35_RED_BACK_SEG4_TARGET_M,
    CHASSIS_DT35_RED_BACK_SEG5_TARGET_M
};
static const float app_dt35_red_reverse_targets[] = {
    CHASSIS_DT35_RED_REVERSE_SEG1_TARGET_M,
    CHASSIS_DT35_RED_REVERSE_SEG2_TARGET_M,
    CHASSIS_DT35_RED_REVERSE_SEG3_TARGET_M,
    CHASSIS_DT35_RED_REVERSE_SEG4_TARGET_M
};

#define APP_DT35_SEQUENCE_COUNT(sequence) \
    ((uint8_t)(sizeof(sequence) / sizeof((sequence)[0])))

static void AppDt35Buttons_Reset(void)
{
    app_dt35_candidate_keys = 0U;
    app_dt35_stable_keys = 0U;
    app_dt35_candidate_since_ms = 0U;
    app_dt35_remote_sequence_active = false;
    app_dt35_pending_yaw_action = 0U;
    app_dt35_pending_yaw_zero = false;
    app_dt35_remote_session_valid = false;
}

void AppDt35Buttons_CancelRemote(void)
{
    AppDt35Buttons_Reset();
    if (!app_dt35_pc_control_active) AppDt35Position_Stop();
}

static bool AppDt35Buttons_StartRemoteSequence(
    const app_dt35_position_direction_t *sequence, uint8_t count,
    const float *target_m, uint8_t post_yaw_action)
{
    if (!AppDt35Position_StartSequenceWithSegmentTargets(sequence, target_m, count)) return false;
    app_dt35_remote_sequence_active = true;
    app_dt35_pending_yaw_action = post_yaw_action;
    return true;
}

bool AppDt35Buttons_TakePendingYawZeroRequest(void)
{
    if (!app_dt35_pending_yaw_zero) return false;
    app_dt35_pending_yaw_zero = false;
    return true;
}

bool AppDt35Buttons_TakePendingYawAction(uint8_t *action_id)
{
    if ((action_id == NULL) || (app_dt35_pending_yaw_action == 0U)
        || app_dt35_remote_sequence_active) return false;
    *action_id = app_dt35_pending_yaw_action;
    app_dt35_pending_yaw_action = 0U;
    return true;
}

void AppDt35Buttons_Scan(const app_remote_control_t *control)
{
    uint32_t now = HAL_GetTick();
    uint32_t keys;
    uint32_t pressed_keys;
    uint32_t previous_stable_keys;

    if (control == NULL) {
        AppDt35Buttons_CancelRemote();
        return;
    }

    app_dt35_pc_control_active = false;

    if (!app_dt35_remote_session_valid
        || (app_dt35_remote_session_id != control->session_id)) {
        AppDt35Buttons_CancelRemote();
        app_dt35_remote_session_id = control->session_id;
        app_dt35_remote_session_valid = true;
    }

    if (app_dt35_remote_sequence_active) {
        app_dt35_position_status_t status;
        if (AppDt35Position_GetStatus(&status) && !status.active) {
            app_dt35_remote_sequence_active = false;
            if (status.code != APP_DT35_POSITION_COMPLETED) {
                app_dt35_pending_yaw_action = 0U;
            }
        }
    }

    if ((control->switches & APP_REMOTE_SWITCH_PC0_REGION_ENABLE) == 0U) {
        AppDt35Position_Stop();
        app_dt35_remote_sequence_active = false;
        app_dt35_pending_yaw_action = 0U;
    }

    keys = control->keys & (PC7 | PA8 | PA11 | PA12 | PB8 | PB5 | PB6);
    if (keys != app_dt35_candidate_keys) {
        app_dt35_candidate_keys = keys;
        app_dt35_candidate_since_ms = now;
        return;
    }
    if ((keys == app_dt35_stable_keys)
        || ((uint32_t)(now - app_dt35_candidate_since_ms)
            < CHASSIS_DT35_BUTTON_DEBOUNCE_MS)) {
        return;
    }

    previous_stable_keys = app_dt35_stable_keys;
    app_dt35_stable_keys = keys;
    pressed_keys = keys & ~previous_stable_keys;
    if (pressed_keys != 0U) 
    {
        if ((control->switches & APP_REMOTE_SWITCH_PC0_REGION_ENABLE) != 0U
            && (pressed_keys & (pressed_keys - 1U)) == 0U
            && (keys & (keys - 1U)) == 0U) 
        {
            if (pressed_keys == PB8) 
            {
                AppDt35Buttons_StartRemoteSequence(app_dt35_red_reverse_sequence,
                    APP_DT35_SEQUENCE_COUNT(app_dt35_red_reverse_sequence),
                    app_dt35_red_reverse_targets, 0U);
            }
            else if (pressed_keys == PB6)
            {
                AppDt35Buttons_StartRemoteSequence(app_dt35_red_back_sequence,
                    APP_DT35_SEQUENCE_COUNT(app_dt35_red_back_sequence),
                    app_dt35_red_back_targets, 0U);
            }
            else if (pressed_keys == PB5)
            {
                AppDt35Buttons_StartRemoteSequence(app_dt35_red_go_sequence,
                    APP_DT35_SEQUENCE_COUNT(app_dt35_red_go_sequence),
                    app_dt35_red_go_targets, 0U);
            }
            else if (pressed_keys == PA11)
            {
                AppDt35Buttons_StartRemoteSequence(app_dt35_blue_go_sequence,
                    APP_DT35_SEQUENCE_COUNT(app_dt35_blue_go_sequence),
                    app_dt35_blue_go_targets, 0U);
            }
            else if (pressed_keys == PA12)
            {
                AppDt35Buttons_StartRemoteSequence(app_dt35_blue_back_sequence,
                    APP_DT35_SEQUENCE_COUNT(app_dt35_blue_back_sequence),
                    app_dt35_blue_back_targets, 0U);
            }
            else if (pressed_keys == PA8)
            {
                AppDt35Buttons_StartRemoteSequence(app_dt35_blue_reverse_sequence,
                    APP_DT35_SEQUENCE_COUNT(app_dt35_blue_reverse_sequence),
                    app_dt35_blue_reverse_targets, 0U);
            }
            else if (pressed_keys == PC7)
            {
                app_dt35_pending_yaw_zero = true;
            }
            else if (pressed_keys == PB9)
            {

            }
            else if (pressed_keys == PC13)
            {

            }
        }
    }
}
