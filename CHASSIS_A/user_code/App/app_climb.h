#ifndef APP_CLIMB_H
#define APP_CLIMB_H

#include <stdbool.h>
#include <stdint.h>

#include "app_messages.h"
#include "bsp_can.h"

bool AppClimb_Init(bsp_can_t *rs01_can, bsp_can_t *dji_can);
void AppClimb_ControlStep(const app_mechanism_command_t *command,
                          uint32_t system_events);
void AppClimb_ActionStep(const app_action_command_t *command,
                         bool command_available, uint32_t system_events);
bool AppClimb_OnCanFrame(const app_can_frame_t *frame);

#endif /* APP_CLIMB_H */
