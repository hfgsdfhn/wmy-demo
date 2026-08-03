#ifndef APP_INIT_H
#define APP_INIT_H

#include <stdbool.h>

#include "bsp_can.h"

bool AppInit_Run(bsp_can_rx_callback_t can_rx_callback,
                 bsp_can_error_callback_t can_error_callback);

#endif /* APP_INIT_H */
