#ifndef APP_NAVIGATION_H
#define APP_NAVIGATION_H

#include <stdbool.h>

#include "app_messages.h"

bool AppNavigation_Init(void);
void AppNavigation_Step(const app_trajectory_point_t *point,
                        bool point_available);
bool AppNavigation_GetTarget(app_trajectory_point_t *point);

#endif /* APP_NAVIGATION_H */
