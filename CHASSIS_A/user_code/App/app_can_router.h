#ifndef APP_CAN_ROUTER_H
#define APP_CAN_ROUTER_H

#include <stdbool.h>

#include "app_messages.h"

bool AppCanRouter_Init(void);
bool AppCanRouter_Dispatch(const app_can_frame_t *frame);

#endif /* APP_CAN_ROUTER_H */
