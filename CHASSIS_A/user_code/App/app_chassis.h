#ifndef APP_CHASSIS_H
#define APP_CHASSIS_H

#include <stdbool.h>
#include <stdint.h>

#include "app_messages.h"
#include "bsp_can.h"

bool AppChassis_Init(bsp_can_t *can);
void AppChassis_ControlStep(const app_chassis_command_t *command,
                            uint32_t system_events);
bool AppChassis_OnCanFrame(const app_can_frame_t *frame);
bool AppChassis_GetCommand(app_chassis_command_t *command);

#endif /* APP_CHASSIS_H */
