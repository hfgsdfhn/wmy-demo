#ifndef APP_NAVIGATION_H
#define APP_NAVIGATION_H

#include <stdbool.h>

#include "path.h"

bool AppNavigation_Init(void);
void AppNavigation_Step(const PathPoint *point,
                        bool point_available);
bool AppNavigation_GetTarget(PathPoint *point);

#endif /* APP_NAVIGATION_H */
