#include "app_init.h"

#include "app_can_router.h"
#include "app_chassis.h"
#include "app_climb.h"
#include "app_communication.h"
#include "app_monitor.h"
#include "app_navigation.h"
#include "app_safety.h"
#include "app_sensors.h"
#include "board_config.h"
#include "bsp_board.h"

static bool app_init_complete;

bool AppInit_Run(bsp_can_rx_callback_t can_rx_callback,
                 bsp_can_error_callback_t can_error_callback)
{
    if (app_init_complete)
    {
        return true;
    }
    if (!BspBoardCan_Init(can_rx_callback, can_error_callback))
    {
        return false;
    }
    if (!AppChassis_Init(BspBoardCan_Get(APP_CHASSIS_CAN_INDEX)))
    {
        return false;
    }
    if (!AppClimb_Init(BspBoardCan_Get(APP_CLIMB_RS01_CAN_INDEX),
                      BspBoardCan_Get(APP_CLIMB_DJI_CAN_INDEX)))
    {
        return false;
    }
    if (!AppNavigation_Init())
    {
        return false;
    }
    if (!AppSensors_Init())
    {
        return false;
    }
    if (!AppCommunication_Init())
    {
        return false;
    }
    if (!AppCanRouter_Init())
    {
        return false;
    }
    if (!AppSafety_Init())
    {
        return false;
    }
    if (!AppMonitor_Init())
    {
        return false;
    }

    app_init_complete = true;
    return true;
}
