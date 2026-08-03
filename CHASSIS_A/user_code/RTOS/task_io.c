#include "app_rtos_internal.h"

#include "app_can_router.h"
#include "app_communication.h"
#include "app_sensors.h"

void AppTask_CanRx(void *argument)
{
    app_can_frame_t frame;

    (void)argument;
    if (!AppRtos_WaitForInitialization())
    {
        osThreadExit();
    }

    for (;;)
    {
        if (osMessageQueueGet(appRtosCanRxQueue, &frame, NULL,
                              osWaitForever) == osOK)
        {
            appRtosRuntimeStats.can_rx_count++;
            (void)AppCanRouter_Dispatch(&frame);
        }
    }
}

void AppTask_Sensor(void *argument)
{
    app_imu_sample_t imu_sample;
    app_dt35_sample_t dt35_sample;
    uint32_t flags;

    (void)argument;
    if (!AppRtos_WaitForInitialization())
    {
        osThreadExit();
    }

    for (;;)
    {
        flags = osThreadFlagsWait(APP_SENSOR_FLAG_IMU | APP_SENSOR_FLAG_DT35,
                                  osFlagsWaitAny, osWaitForever);
        if ((flags & osFlagsError) != 0U)
        {
            continue;
        }

        if ((flags & APP_SENSOR_FLAG_IMU) != 0U)
        {
            while (osMessageQueueGet(appRtosImuQueue, &imu_sample,
                                     NULL, 0U) == osOK)
            {
                appRtosRuntimeStats.imu_rx_count++;
                AppSensors_UpdateImu(&imu_sample);
            }
        }
        if ((flags & APP_SENSOR_FLAG_DT35) != 0U)
        {
            while (osMessageQueueGet(appRtosDt35Queue, &dt35_sample,
                                     NULL, 0U) == osOK)
            {
                appRtosRuntimeStats.dt35_rx_count++;
                AppSensors_UpdateDt35(&dt35_sample);
            }
        }
    }
}

void AppTask_Communication(void *argument)
{
    app_comm_packet_t packet;

    (void)argument;
    if (!AppRtos_WaitForInitialization())
    {
        osThreadExit();
    }

    for (;;)
    {
        if (osMessageQueueGet(appRtosCommunicationQueue, &packet, NULL,
                              osWaitForever) == osOK)
        {
            appRtosRuntimeStats.comm_rx_count++;
            AppCommunication_OnPacket(&packet);
        }
    }
}
