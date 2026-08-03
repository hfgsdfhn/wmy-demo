#ifndef APP_MONITOR_H
#define APP_MONITOR_H

#include <stdbool.h>

#include "app_state.h"

bool AppMonitor_Init(void);
void AppMonitor_Step(const app_runtime_stats_t *stats);
bool AppMonitor_GetStats(app_runtime_stats_t *stats);

#endif /* APP_MONITOR_H */
