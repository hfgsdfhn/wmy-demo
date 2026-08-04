#ifndef APP_SENSORS_H
#define APP_SENSORS_H

#include <stdbool.h>

#include "app_messages.h"

bool AppSensors_Init(void);
void AppSensors_UpdateImu(const app_imu_sample_t *sample);
void AppSensors_UpdateDt35(const app_dt35_sample_t *sample);
bool AppSensors_GetImu(app_imu_sample_t *sample);
bool AppSensors_GetDt35(app_dt35_sample_t *sample);

#endif /* APP_SENSORS_H */
