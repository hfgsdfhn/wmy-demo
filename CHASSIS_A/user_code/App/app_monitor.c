#include "app_monitor.h"

#include <stddef.h>
#include <string.h>

static app_runtime_stats_t app_monitor_stats;
static bool app_monitor_valid;

bool AppMonitor_Init(void)
{
    memset(&app_monitor_stats, 0, sizeof(app_monitor_stats));
    app_monitor_valid = false;
    return true;
}

void AppMonitor_Step(const app_runtime_stats_t *stats)
{
    if (stats != NULL)
    {
        app_monitor_stats = *stats;
        app_monitor_valid = true;
    }
}

bool AppMonitor_GetStats(app_runtime_stats_t *stats)
{
    if (!app_monitor_valid || (stats == NULL))
    {
        return false;
    }
    *stats = app_monitor_stats;
    return true;
}
