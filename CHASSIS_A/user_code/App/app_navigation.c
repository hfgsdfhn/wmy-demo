#include "app_navigation.h"

#include <stddef.h>
#include <string.h>

static app_trajectory_point_t app_navigation_target;
static bool app_navigation_target_valid;

bool AppNavigation_Init(void)
{
    memset(&app_navigation_target, 0, sizeof(app_navigation_target));
    app_navigation_target_valid = false;
    return true;
}

void AppNavigation_Step(const app_trajectory_point_t *point,
                        bool point_available)
{
    if (point_available && (point != NULL))
    {
        app_navigation_target = *point;
        app_navigation_target_valid = true;
    }

    /* B-spline generation and PathFollower integration are added later. */
}

bool AppNavigation_GetTarget(app_trajectory_point_t *point)
{
    if (!app_navigation_target_valid || (point == NULL))
    {
        return false;
    }
    *point = app_navigation_target;
    return true;
}
