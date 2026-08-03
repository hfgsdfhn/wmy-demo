#include "app_chassis.h"

#include <stddef.h>
#include <string.h>

#include "ak_motor.h"
#include "app_state.h"
#include "board_config.h"

static ak_motor_t app_chassis_motors[APP_CHASSIS_MOTOR_COUNT];
static app_chassis_command_t app_chassis_command;
static bool app_chassis_initialized;

bool AppChassis_Init(bsp_can_t *can)
{
    static const uint8_t motor_ids[APP_CHASSIS_MOTOR_COUNT] =
    {
        APP_CHASSIS_AK_ID_1,
        APP_CHASSIS_AK_ID_2,
        APP_CHASSIS_AK_ID_3,
        APP_CHASSIS_AK_ID_4
    };
    uint8_t index;

    if (can == NULL)
    {
        return false;
    }

    memset(&app_chassis_command, 0, sizeof(app_chassis_command));
    for (index = 0U; index < APP_CHASSIS_MOTOR_COUNT; index++)
    {
        if (!AkMotor_Init(&app_chassis_motors[index], can, motor_ids[index]))
        {
            app_chassis_initialized = false;
            return false;
        }
    }

    app_chassis_initialized = true;
    return true;
}

/**
 * @brief  控制底盘电机的目标速度，
 *         若系统处于紧急停止、CAN 总线故障或电机故障状态，则忽略该命令。
 * 
 * @param command 
 * @param system_events 
 */
void AppChassis_ControlStep(const app_chassis_command_t *command,
                            uint32_t system_events)
{
    if (!app_chassis_initialized || (command == NULL))
    {
        return;
    }

    app_chassis_command = *command;
    if ((system_events & (APP_EVENT_ESTOP | APP_EVENT_MOTOR_FAULT
                          | APP_EVENT_CAN_FAULT)) != 0U)
    {
        app_chassis_command.valid = 0U;
    }
}

bool AppChassis_OnCanFrame(const app_can_frame_t *frame)    // 处理底盘电机的 CAN 帧反馈
{
    uint8_t index;

    if (!app_chassis_initialized || (frame == NULL)
        || (frame->bus != (app_can_bus_t)APP_CHASSIS_CAN_INDEX)
        || (frame->is_extended_id == 0U))
    {
        return false;
    }

    for (index = 0U; index < APP_CHASSIS_MOTOR_COUNT; index++)
    {
        if ((frame->identifier & 0xFFU) == app_chassis_motors[index].id)
        {
            return AkMotor_ParseFeedback(&app_chassis_motors[index],
                                         frame->data, frame->length);
        }
    }
    return false;
}

bool AppChassis_GetCommand(app_chassis_command_t *command)
{
    if (!app_chassis_initialized || (command == NULL))
    {
        return false;
    }

    *command = app_chassis_command;
    
    return true;
}
