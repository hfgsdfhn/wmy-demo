#include "app_sensors.h"

#include <stddef.h>
#include <string.h>

static app_imu_sample_t app_sensors_imu;
static app_dt35_sample_t app_sensors_dt35;

bool AppSensors_Init(void)
{
    memset(&app_sensors_imu, 0, sizeof(app_sensors_imu));
    memset(&app_sensors_dt35, 0, sizeof(app_sensors_dt35));
    return true;
}

void AppSensors_UpdateImu(const app_imu_sample_t *sample)
{
    if (sample != NULL)
    {
        app_sensors_imu = *sample;
    }
}

void AppSensors_UpdateDt35(const app_dt35_sample_t *sample)
{
    if (sample != NULL)
    {
        app_sensors_dt35 = *sample;
    }
}

bool AppSensors_GetImu(app_imu_sample_t *sample)
{
    if ((sample == NULL) || (app_sensors_imu.valid == 0U))
    {
        return false;
    }
    *sample = app_sensors_imu;
    return true;
}

bool AppSensors_GetDt35(app_dt35_sample_t *sample)
{
    if ((sample == NULL) || (app_sensors_dt35.valid == 0U))
    {
        return false;
    }
    *sample = app_sensors_dt35;
    return true;
}
