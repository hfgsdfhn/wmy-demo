#include "app_climb.h"

#include <stddef.h>
#include <string.h>

#include "board_config.h"
#include "dji_motor.h"
#include "rs01_motor.h"
#include "app_state.h"

static rs01_motor_t app_climb_rs01[APP_CLIMB_RS01_COUNT];
static dji_motor_t app_climb_m2006[APP_CLIMB_M2006_COUNT];
static app_mechanism_command_t app_climb_command;
static app_action_command_t app_climb_action;
static bool app_climb_initialized;

bool AppClimb_Init(bsp_can_t *rs01_can, bsp_can_t *dji_can)
{
    static const uint8_t rs01_ids[APP_CLIMB_RS01_COUNT] =
    {
        APP_CLIMB_RS01_ID_1,
        APP_CLIMB_RS01_ID_2,
        APP_CLIMB_RS01_ID_3,
        APP_CLIMB_RS01_ID_4
    };
    static const uint8_t dji_ids[APP_CLIMB_M2006_COUNT] =
    {
        APP_CLIMB_M2006_ID_1,
        APP_CLIMB_M2006_ID_2
    };
    uint8_t index;

    if ((rs01_can == NULL) || (dji_can == NULL))
    {
        return false;
    }

    memset(&app_climb_command, 0, sizeof(app_climb_command));
    memset(&app_climb_action, 0, sizeof(app_climb_action));
    for (index = 0U; index < APP_CLIMB_RS01_COUNT; index++)
    {
        if (!Rs01Motor_Init(&app_climb_rs01[index], rs01_can,
                            rs01_ids[index]))
        {
            app_climb_initialized = false;
            return false;
        }
    }
    for (index = 0U; index < APP_CLIMB_M2006_COUNT; index++)
    {
        DjiMotor_Init(&app_climb_m2006[index], dji_can, dji_ids[index],
                      DJI_MOTOR_TYPE_M2006);
    }

    app_climb_initialized = true;
    return true;
}

void AppClimb_ControlStep(const app_mechanism_command_t *command,
                          uint32_t system_events)
{
    if (!app_climb_initialized || (command == NULL))
    {
        return;
    }

    app_climb_command = *command;
    if ((system_events & (APP_EVENT_ESTOP | APP_EVENT_MOTOR_FAULT
                          | APP_EVENT_CAN_FAULT)) != 0U)
    {
        app_climb_command.valid = 0U;
    }

    /* Closed-loop rod and rear-wheel output are added in the control phase. */
}

void AppClimb_ActionStep(const app_action_command_t *command,
                         bool command_available, uint32_t system_events)
{
    if (!app_climb_initialized || !command_available || (command == NULL)
        || ((system_events & (APP_EVENT_ESTOP | APP_EVENT_MOTOR_FAULT
                              | APP_EVENT_CAN_FAULT)) != 0U))
    {
        return;
    }

    app_climb_action = *command;
    /* The climb state machine is added after action definitions are fixed. */
}

bool AppClimb_OnCanFrame(const app_can_frame_t *frame)
{
    uint8_t index;

    if (!app_climb_initialized || (frame == NULL))
    {
        return false;
    }

    if ((frame->bus == (app_can_bus_t)APP_CLIMB_RS01_CAN_INDEX)
        && (frame->is_extended_id != 0U))
    {
        for (index = 0U; index < APP_CLIMB_RS01_COUNT; index++)
        {
            if (Rs01Motor_ProcessData(&app_climb_rs01[index],
                                      frame->identifier, frame->data,
                                      frame->length))
            {
                return true;
            }
        }
    }
    else if ((frame->bus == (app_can_bus_t)APP_CLIMB_DJI_CAN_INDEX)
             && (frame->is_extended_id == 0U)
             && (frame->length == APP_CAN_DATA_MAX_SIZE))
    {
        for (index = 0U; index < APP_CLIMB_M2006_COUNT; index++)
        {
            if (frame->identifier
                == (DJI_MOTOR_FEEDBACK_ID_BASE + app_climb_m2006[index].id
                    - 1U))
            {
                DjiMotor_ParseFeedback(&app_climb_m2006[index], frame->data);
                return true;
            }
        }
    }
    return false;
}
