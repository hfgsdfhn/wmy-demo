#include "app_can_router.h"

#include <stddef.h>

#include "app_chassis.h"
#include "app_climb.h"

bool AppCanRouter_Init(void)
{
    return true;
}

bool AppCanRouter_Dispatch(const app_can_frame_t *frame)
{
    if (frame == NULL)
    {
        return false;
    }

    if (AppChassis_OnCanFrame(frame))
    {
        return true;
    }
    return AppClimb_OnCanFrame(frame);
}
