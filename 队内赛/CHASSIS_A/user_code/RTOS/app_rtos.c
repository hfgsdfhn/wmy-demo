#include "app_rtos.h"

#include <string.h>

#include "FreeRTOS.h"
#include "app_chassis.h"
#include "remote_buttons.h"
#include "app_dt35_position.h"
#include "app_init.h"
#include "app_rtos_event.h"
#include "app_rtos_state.h"
#include "pid_config.h"
#include "PathFollower.h"
#include "omni_chassis.h"
#include "task.h"
#include "DM_Imu.h"
#include "yaw_ekf.h"

#define APP_CONTROL_PERIOD_MS 2U             /* 控制任务周期，单位 ms */
#define APP_NAVIGATION_PERIOD_MS 10U         /* 导航任务周期，单位 ms */
#define APP_SAFETY_PERIOD_MS 10U             /* 安全任务周期，单位 ms */
#define APP_MONITOR_PERIOD_MS 100U           /* 监控任务周期，单位 ms */
#define APP_COMMUNICATION_WAIT_MS 20U        /* 双主控状态帧 50 Hz */
#define APP_CAN_RX_QUEUE_LENGTH 64U          /* CAN 接收消息队列长度 */
#define APP_COMMAND_QUEUE_LENGTH 4U          /* 底盘命令消息队列长度 */
#define APP_TELEMETRY_QUEUE_LENGTH 4U        /* 遥测消息队列长度 */
#define APP_TRAJECTORY_QUEUE_LENGTH 32U      /* 轨迹点消息队列长度 */
#define APP_RTOS_DEG_TO_RAD 0.01745329251994329577f
#define APP_RTOS_RAD_TO_DEG 57.2957795130823208768f
#define APP_DT35_MANUAL_CANCEL_AXIS 16

static osThreadId_t appRtosCanRxTask;
static osThreadId_t appRtosControlTask;
static osThreadId_t appRtosNavigationTask;
static osThreadId_t appRtosCommunicationTask;
static osThreadId_t appRtosSafetyTask;
static osThreadId_t appRtosMonitorTask;
static osThreadId_t appRtosSerialTask;
static app_runtime_stats_t appRtosRuntimeStats;
static osMessageQueueId_t appRtosCanRxQueue;
static osMessageQueueId_t appRtosChassisCommandQueue;
static osMessageQueueId_t appRtosTrajectoryQueue;
static osMessageQueueId_t appRtosTelemetryQueue;
static osMessageQueueId_t appRtosUartRxQueue;
static volatile uint32_t appRtosCanErrorMask;
/* Expand this single object in the debugger to inspect the YAW loop. */
typedef struct
{
    uint32_t enabled;
    uint32_t feedback_valid;
    float target_deg;
    float feedback_deg;
    float output_radps;
    float ekf_deg;
    float gyro_radps;
    uint32_t events;
} app_rtos_yaw_pid_debug_t;

volatile app_rtos_yaw_pid_debug_t appRtosYawPidDebug;
/* Integrated IMU yaw used by the position PID, in degrees. */
static float appRtosImuYawDeg;
/* Integrated chassis yaw rate reconstructed from wheel feedback, in degrees. */
static float appRtosChassisYawDeg;
static yaw_ekf_t appRtosYawEkf;

static float AppRtos_NormalizeYawDeg(float yaw_deg)
{
    const float two_pi = 360.0f;

    while (yaw_deg > 180.0f) yaw_deg -= two_pi;
    while (yaw_deg < -180.0f) yaw_deg += two_pi;
    return yaw_deg;
}

static bool AppRtos_GetYawPidFeedbackDeg(float *yaw_deg)
{
    if (yaw_deg == NULL) {
        return false;
    }

#if (CHASSIS_YAW_INPUT_SOURCE == 0U)
    if (!DM_IMU_IsGyroCalibrated()) {
        return false;
    }
    *yaw_deg = appRtosImuYawDeg;
    return true;
#elif (CHASSIS_YAW_INPUT_SOURCE == 1U)
    if (!DM_IMU_IsYawZeroed()) {
        return false;
    }
    *yaw_deg = DM_IMU_GetYaw() * APP_RTOS_RAD_TO_DEG;
    return true;
#elif (CHASSIS_YAW_INPUT_SOURCE == 2U)
    /* The EKF always has a valid bounded state after reset.  Do not bypass
     * the position loop merely because gyro bias calibration is still in
     * progress; the EKF will continue improving once gyro data is ready. */
    *yaw_deg = YawEkf_GetYaw(&appRtosYawEkf) * APP_RTOS_RAD_TO_DEG;
    return true;
#else
#error "CHASSIS_YAW_INPUT_SOURCE 配置无效"
#endif
}

static void AppRtos_ZeroYaw(void)
{
    (void)DM_IMU_ZeroYaw();
    appRtosImuYawDeg = 0.0f;
    appRtosChassisYawDeg = 0.0f;
    YawEkf_Reset(&appRtosYawEkf, 0.0f);
    AppMonitor_ClearYawAction();
    (void)AppMonitor_AlignYawTarget(0.0f);
    AppChassis_StopYawImmediately();
}

static bool AppRtos_IsYawTurnAction(uint8_t action_id)
{
    return (action_id == CHASSIS_YAW_ACTION_LEFT_90_ID)
        || (action_id == CHASSIS_YAW_ACTION_RIGHT_90_ID)
        || (action_id == CHASSIS_YAW_ACTION_180_ID);
}

typedef enum
{
    APP_RTOS_SHOULDER_RELEASED = 0,
    APP_RTOS_SHOULDER_DIRECTION_1,
    APP_RTOS_SHOULDER_PRESSED,
    APP_RTOS_SHOULDER_DIRECTION_2,
    APP_RTOS_SHOULDER_INVALID
} app_rtos_shoulder_state_t;

static app_rtos_shoulder_state_t AppRtos_DecodeShoulder(uint32_t keys, bool left)
{
    uint32_t direction_1 = left ? (1UL << 14U) : (1UL << 15U);
    uint32_t pressed = left ? (1UL << 16U) : (1UL << 17U);
    uint32_t direction_2 = left ? (1UL << 18U) : (1UL << 19U);
    uint32_t state = keys & (direction_1 | pressed | direction_2);

    if (state == 0U) return APP_RTOS_SHOULDER_RELEASED;
    if (state == direction_1) return APP_RTOS_SHOULDER_DIRECTION_1;
    if (state == pressed) return APP_RTOS_SHOULDER_PRESSED;
    if (state == direction_2) return APP_RTOS_SHOULDER_DIRECTION_2;
    return APP_RTOS_SHOULDER_INVALID;
}

#define TASK_ATTR(name_, stack_, priority_) { .name = name_, .stack_size = stack_, .priority = priority_ } /* 创建任务属性 */
static const osThreadAttr_t appRtosCanRxTaskAttributes = TASK_ATTR("canRxTask", 1536U, osPriorityHigh);
static const osThreadAttr_t appRtosControlTaskAttributes = TASK_ATTR("controlTask", 2048U, osPriorityRealtime);
static const osThreadAttr_t appRtosNavigationTaskAttributes = TASK_ATTR("navigationTask", 3072U, osPriorityAboveNormal);
static const osThreadAttr_t appRtosCommunicationTaskAttributes = TASK_ATTR("communicationTask", 2048U, osPriorityNormal);
static const osThreadAttr_t appRtosSafetyTaskAttributes = TASK_ATTR("safetyTask", 1024U, osPriorityHigh);
static const osThreadAttr_t appRtosMonitorTaskAttributes = TASK_ATTR("monitorTask", 1024U, osPriorityLow);
static const osThreadAttr_t appRtosSerialTaskAttributes = TASK_ATTR("serialTask", 1536U, osPriorityHigh);
#undef TASK_ATTR

#define QUEUE_ATTR(name_) { .name = name_ } /* 创建消息队列属性 */
static const osMessageQueueAttr_t appRtosCanRxQueueAttributes = QUEUE_ATTR("canRxQueue");
static const osMessageQueueAttr_t appRtosChassisQueueAttributes = QUEUE_ATTR("chassisCmdQueue");
static const osMessageQueueAttr_t appRtosTrajectoryQueueAttributes = QUEUE_ATTR("trajectoryQueue");
static const osMessageQueueAttr_t appRtosTelemetryQueueAttributes = QUEUE_ATTR("telemetryQueue");
static const osMessageQueueAttr_t appRtosUartRxQueueAttributes = QUEUE_ATTR("uartRxQueue");
#undef QUEUE_ATTR


static void AppTask_CanRx(void *argument);
static void AppTask_Control(void *argument);
static void AppTask_Navigation(void *argument);
static void AppTask_Communication(void *argument);
static void AppTask_Safety(void *argument);
static void AppTask_Monitor(void *argument);
static void AppTask_Serial(void *argument);

static void AppRtos_RecordCanRxDrop(void) { appRtosRuntimeStats.can_rx_dropped++; }
static void AppRtos_RecordCommunicationDrop(void) { appRtosRuntimeStats.comm_rx_dropped++; }

uint32_t AppRtos_MsToTicks(uint32_t milliseconds)
{
    uint64_t ticks = ((uint64_t)milliseconds * osKernelGetTickFreq() + 999U) / 1000U;
    return ((milliseconds > 0U) && (ticks == 0U)) ? 1U : (uint32_t)ticks;
}

static bool AppRtos_QueueInit(void)
{
    appRtosCanErrorMask = 0U;
    appRtosImuYawDeg = 0.0f;
    appRtosChassisYawDeg = 0.0f;
    YawEkf_Reset(&appRtosYawEkf, 0.0f);
    appRtosCanRxQueue = osMessageQueueNew(APP_CAN_RX_QUEUE_LENGTH, sizeof(app_can_frame_t), &appRtosCanRxQueueAttributes);
    appRtosChassisCommandQueue = osMessageQueueNew(APP_COMMAND_QUEUE_LENGTH, sizeof(app_chassis_command_t), &appRtosChassisQueueAttributes);
    appRtosTrajectoryQueue = osMessageQueueNew(APP_TRAJECTORY_QUEUE_LENGTH, sizeof(app_trajectory_point_t), &appRtosTrajectoryQueueAttributes);
    appRtosTelemetryQueue = osMessageQueueNew(APP_TELEMETRY_QUEUE_LENGTH, sizeof(app_runtime_stats_t), &appRtosTelemetryQueueAttributes);
    appRtosUartRxQueue = osMessageQueueNew(16U, sizeof(app_uart_rx_packet_t), &appRtosUartRxQueueAttributes);
    return appRtosCanRxQueue && appRtosChassisCommandQueue && appRtosTrajectoryQueue
        && appRtosTelemetryQueue && appRtosUartRxQueue;
}

static bool AppRtos_PutLatest(osMessageQueueId_t queue, const void *item, void *discarded, uint32_t timeoutMs, bool *discardedOldest)
{
    if (discardedOldest != NULL) *discardedOldest = false;
    if (osMessageQueuePut(queue, item, 0U, AppRtos_MsToTicks(timeoutMs)) == osOK) return true;
    if (osMessageQueueGet(queue, discarded, NULL, 0U) != osOK) return false;
    if (discardedOldest != NULL) *discardedOldest = true;
    return osMessageQueuePut(queue, item, 0U, 0U) == osOK;
}

bool AppRtos_SubmitChassisCommand(const app_chassis_command_t *command, uint32_t timeoutMs)
{
    app_chassis_command_t discarded;
    return command && appRtosChassisCommandQueue && AppRtos_PutLatest(appRtosChassisCommandQueue, command, &discarded, timeoutMs, NULL);
}

bool AppRtos_SubmitTrajectoryPoint(const app_trajectory_point_t *point, uint32_t timeoutMs)
{
    return point && appRtosTrajectoryQueue && osMessageQueuePut(appRtosTrajectoryQueue, point, 0U, AppRtos_MsToTicks(timeoutMs)) == osOK;
}

void AppRtos_CanRxFromISR(bsp_can_t *can, uint32_t canId, bool isExtendedId, const uint8_t *data, uint8_t length)
{
    app_can_frame_t frame;
    TickType_t tickCount;

    if (!appRtosCanRxQueue || !data || length > APP_CAN_DATA_MAX_SIZE || !can || can->instance_index >= (uint8_t)APP_CAN_BUS_COUNT)
        return;

    tickCount = xTaskGetTickCountFromISR();
    frame.timestamp_ms = (uint32_t)(((uint64_t)tickCount * 1000U) / configTICK_RATE_HZ);
    frame.identifier = canId;
    frame.bus = (app_can_bus_t)can->instance_index;
    frame.is_extended_id = isExtendedId ? 1U : 0U;
    frame.length = length;
    memset(frame.data, 0, sizeof(frame.data));
    memcpy(frame.data, data, length);

    if (osMessageQueuePut(appRtosCanRxQueue, &frame, 0U, 0U) != osOK)
        AppRtos_RecordCanRxDrop();
}

void AppRtos_CanErrorFromISR(bsp_can_t *can, uint32_t errorFlags)
{
    (void)errorFlags;
    if (can && can->instance_index < (uint8_t)APP_CAN_BUS_COUNT)
        appRtosCanErrorMask |= 1UL << can->instance_index;
}

void AppRtos_UartRxFromISR(app_uart_rx_source_t source, const uint8_t *data, uint16_t length)
{
    app_uart_rx_packet_t packet;
    if ((appRtosUartRxQueue == NULL) || (data == NULL) ||
        (length == 0U) || (length > APP_UART_RX_DATA_MAX)) return;
    packet.source = source;
    packet.length = length;
    memcpy(packet.data, data, length);
    (void)osMessageQueuePut(appRtosUartRxQueue, &packet, 0U, 0U);
}

void AppRtos_ClearCanErrors(void)
{
    appRtosCanErrorMask = 0U;
}

void AppRtos_SetEventsFromISR(uint32_t flags)
{
    if (appRtosSystemEvents != NULL) {
        (void)osEventFlagsSet(appRtosSystemEvents, flags);
    }
}

static bool AppRtos_TaskCreate(void)
{
    memset(&appRtosRuntimeStats, 0, sizeof(appRtosRuntimeStats));

    appRtosCanRxTask = osThreadNew(AppTask_CanRx, NULL, &appRtosCanRxTaskAttributes);
    appRtosControlTask = osThreadNew(AppTask_Control, NULL, &appRtosControlTaskAttributes);
    appRtosNavigationTask = osThreadNew(AppTask_Navigation, NULL, &appRtosNavigationTaskAttributes);
    appRtosCommunicationTask = osThreadNew(AppTask_Communication, NULL, &appRtosCommunicationTaskAttributes);
    appRtosSafetyTask = osThreadNew(AppTask_Safety, NULL, &appRtosSafetyTaskAttributes);
    appRtosMonitorTask = osThreadNew(AppTask_Monitor, NULL, &appRtosMonitorTaskAttributes);
    appRtosSerialTask = osThreadNew(AppTask_Serial, NULL, &appRtosSerialTaskAttributes);

    return appRtosCanRxTask && appRtosControlTask && appRtosNavigationTask && appRtosCommunicationTask && appRtosSafetyTask && appRtosMonitorTask && appRtosSerialTask;
}

bool AppRtos_Init(void)
{
    PathFollower_Init();
    PathFollower_SetDefaultPath();
    return AppDt35Position_Init()
        && AppRtos_QueueInit()
        && AppRtos_EventInit()
        && AppRtos_StateInit()
        && AppRtos_TaskCreate();
}

void AppRtos_GetRuntimeStats(app_runtime_stats_t *stats)
{
    if (stats != NULL) {
        *stats = appRtosRuntimeStats;
        stats->timestamp_ms = osKernelGetTickCount();
        stats->system_events = AppRtos_GetEvents();
        stats->can_error_mask = appRtosCanErrorMask;
    }
}

static void AppTask_CanRx(void *argument)
{
    app_can_frame_t frame;
    (void)argument;

    if (!AppRtosEvent_WaitForInitialization()) {
        osThreadExit();
    }

    for (;;) {
        if (osMessageQueueGet(appRtosCanRxQueue, &frame, NULL, osWaitForever) == osOK) {
            appRtosRuntimeStats.can_rx_count++;
            (void)AppChassis_OnCanFrame(&frame);
        }
             }
}

static void AppTask_Control(void *argument)
{
    app_chassis_command_t command = {0};
    app_dual_control_t dual_control;
    app_dual_control_freshness_t dual_control_freshness;
    app_remote_control_t remote_control;
    app_remote_control_freshness_t remote_control_freshness;
    app_pc_control_t pc_control;
    app_real_world_t state;
    app_yaw_pid_params_t yaw_pid_params;
    float yaw_target_deg = 0.0f;
    float gyro_z_radps;
    float gyro_z_degps;
    float chassis_vx_mps;
    float chassis_vy_mps;
    float chassis_yaw_rate_radps = 0.0f;
    float fused_yaw_rate_radps;
    bool chassis_yaw_rate_valid;
    float yaw_pid_feedback_deg = 0.0f;
    bool yaw_pid_feedback_valid;
    bool shoulder_yaw_active = false;
    uint32_t shoulder_yaw_step_elapsed_ms = 0U;
    uint8_t latched_yaw_turn_action_id = 0U;
    bool latched_yaw_turn_is_region = false;
    uint16_t remote_session_id = 0U;
    bool remote_session_valid = false;
    app_rtos_shoulder_state_t previous_right_shoulder = APP_RTOS_SHOULDER_RELEASED;
    app_rtos_shoulder_state_t left_shoulder;
    app_rtos_shoulder_state_t right_shoulder;
    uint8_t remote_yaw_action_id;
    bool remote_yaw_action_is_region;
    app_yaw_action_status_t remote_yaw_action_status;
    app_dt35_position_status_t dt35_position_status;
    bool pc_chassis_enabled;
    bool dt35_position_active;
    uint8_t pending_region_yaw_action;
    uint32_t events;
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(APP_CONTROL_PERIOD_MS);
    (void)argument;

    if (!AppRtosEvent_WaitForInitialization()) {
        osThreadExit();
    }

    xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        state.timestamp_ms = osKernelGetTickCount();
        gyro_z_radps = DM_IMU_GetGyroZ();
        appRtosYawPidDebug.gyro_radps = gyro_z_radps;
        gyro_z_degps = gyro_z_radps * APP_RTOS_RAD_TO_DEG;
        appRtosImuYawDeg = AppRtos_NormalizeYawDeg(
            appRtosImuYawDeg + gyro_z_degps * (APP_CONTROL_PERIOD_MS * 0.001f));
        state.imu_yaw_deg = DM_IMU_IsYawZeroed()
                              ? (DM_IMU_GetYaw() * APP_RTOS_RAD_TO_DEG)
                              : 0.0f;
        chassis_yaw_rate_valid = AppChassis_GetMeasuredVelocity(
            &chassis_vx_mps, &chassis_vy_mps, &chassis_yaw_rate_radps,
            state.timestamp_ms);
        if (chassis_yaw_rate_valid) {
            appRtosChassisYawDeg = AppRtos_NormalizeYawDeg(
                appRtosChassisYawDeg + chassis_yaw_rate_radps
                                       * (APP_CONTROL_PERIOD_MS * 0.001f)
                                       * APP_RTOS_RAD_TO_DEG);
        }
        YawEkf_Update(&appRtosYawEkf, gyro_z_radps, chassis_yaw_rate_radps,
                      chassis_yaw_rate_valid, APP_CONTROL_PERIOD_MS * 0.001f);
        fused_yaw_rate_radps = YawEkf_GetYawRate(&appRtosYawEkf, gyro_z_radps);
#if (CHASSIS_YAW_INPUT_SOURCE == 2U)
        state.yaw_deg = YawEkf_GetYaw(&appRtosYawEkf) * APP_RTOS_RAD_TO_DEG;
#else
        state.yaw_deg = appRtosImuYawDeg;
#endif
        state.chassis_yaw_deg = appRtosChassisYawDeg;
        state.fused_yaw_deg = YawEkf_GetYaw(&appRtosYawEkf) * APP_RTOS_RAD_TO_DEG;
        state.fused_yaw_rate_radps = fused_yaw_rate_radps;
        state.yaw_rate_radps = fused_yaw_rate_radps;
        appRtosYawPidDebug.ekf_deg = state.fused_yaw_deg;
        (void)AppRtos_UpdateYaw(&state);
        yaw_pid_feedback_valid = AppRtos_GetYawPidFeedbackDeg(&yaw_pid_feedback_deg);

        /* Establish heading hold at the first valid feedback after boot. */
        if (yaw_pid_feedback_valid) {
            if (AppMonitor_GetYawPidParams(&yaw_pid_params)
                && yaw_pid_params.enabled
                && !AppMonitor_GetYawTarget(&yaw_target_deg)) {
                (void)AppMonitor_AlignYawTarget(yaw_pid_feedback_deg);
            }
        }

        while (osMessageQueueGet(appRtosChassisCommandQueue, &command, NULL, 0U) == osOK) {
        }

        pc_chassis_enabled = AppMonitor_GetPcControl(&pc_control)
                             && (pc_control.command == APP_PC_COMMAND_START);
        dt35_position_active = AppDt35Position_GetStatus(&dt35_position_status)
                               && dt35_position_status.active;
        if (dt35_position_active) {
            /* DT35 position control owns all chassis axes until completion. */
            shoulder_yaw_active = false;
            shoulder_yaw_step_elapsed_ms = 0U;
            previous_right_shoulder = APP_RTOS_SHOULDER_RELEASED;
            latched_yaw_turn_action_id = 0U;
            latched_yaw_turn_is_region = false;
            AppMonitor_ClearYawAction();
            AppMonitor_ClearYawTarget();
            AppChassis_StopYawImmediately();
        } else if (pc_chassis_enabled) {
            /* PC manual vx/vy are world-frame commands.  YAW PID feedback
             * supplies only the frame angle; it is not rotated 90 degrees to
             * reuse one movement axis for the other. */
            command.timestamp_ms = osKernelGetTickCount();
            if (yaw_pid_feedback_valid) {
                OmniChassis_WorldToBody(
                    pc_control.velocity_x_mps, pc_control.velocity_y_mps,
                    yaw_pid_feedback_deg * APP_RTOS_DEG_TO_RAD,
                    &command.velocity_x_mps, &command.velocity_y_mps);
            } else {
                /* A world-frame command is unsafe without a valid heading. */
                command.velocity_x_mps = 0.0f;
                command.velocity_y_mps = 0.0f;
            }
            command.yaw_rate_radps = pc_control.yaw_rate_radps;
            command.valid = 1U;
        } else if (dt35_position_active) {
            /* The navigation task owns the command queue while a DT35 move
             * is active.  Do not overwrite its PID output with joystick data. */
            shoulder_yaw_active = false;
            shoulder_yaw_step_elapsed_ms = 0U;
            previous_right_shoulder = APP_RTOS_SHOULDER_RELEASED;
            latched_yaw_turn_action_id = 0U;
            latched_yaw_turn_is_region = false;
            AppMonitor_ClearYawAction();
        } else {
            remote_control_freshness = AppMessages_GetRemoteControlFreshness(
                osKernelGetTickCount());
            if ((remote_control_freshness == APP_REMOTE_CONTROL_ACTIVE)
                && AppMessages_GetRemoteControl(&remote_control)) {
                (void)AppChassis_PowerBoardRemoteToCommand(&remote_control,
                                                           osKernelGetTickCount(),
                                                           &command);
                /* The remote joystick is field-oriented.  Convert its
                 * world-frame translation into chassis-frame motion before
                 * combining it with any yaw command below. */
                if (yaw_pid_feedback_valid) {
                    OmniChassis_WorldToBody(
                        command.velocity_x_mps, command.velocity_y_mps,
                        yaw_pid_feedback_deg * APP_RTOS_DEG_TO_RAD,
                        &command.velocity_x_mps, &command.velocity_y_mps);
                } else {
                    command.velocity_x_mps = 0.0f;
                    command.velocity_y_mps = 0.0f;
                }
                if (!remote_session_valid || (remote_session_id != remote_control.session_id)) {
                    remote_session_id = remote_control.session_id;
                    remote_session_valid = true;
                    previous_right_shoulder = APP_RTOS_SHOULDER_RELEASED;
                    shoulder_yaw_active = false;
                    shoulder_yaw_step_elapsed_ms = 0U;
                    AppDt35Buttons_CancelRemote();
                    AppMonitor_ClearYawAction();
                    latched_yaw_turn_action_id = 0U;
                    latched_yaw_turn_is_region = false;
                }
                left_shoulder = AppRtos_DecodeShoulder(remote_control.keys, true);
                right_shoulder = AppRtos_DecodeShoulder(remote_control.keys, false);
                if (AppDt35Buttons_TakePendingYawZeroRequest()) {
                    AppRtos_ZeroYaw();
                    command.yaw_rate_radps = 0.0f;
                    latched_yaw_turn_action_id = 0U;
                    latched_yaw_turn_is_region = false;
                    shoulder_yaw_active = false;
                    shoulder_yaw_step_elapsed_ms = 0U;
                }
                remote_yaw_action_id = 0U;
                remote_yaw_action_is_region = false;
                if ((right_shoulder != previous_right_shoulder)
                    && (right_shoulder == APP_RTOS_SHOULDER_DIRECTION_1)) {
                    remote_yaw_action_id = CHASSIS_YAW_ACTION_LEFT_90_ID;
                } else if ((right_shoulder != previous_right_shoulder)
                           && (right_shoulder == APP_RTOS_SHOULDER_DIRECTION_2)) {
                    remote_yaw_action_id = CHASSIS_YAW_ACTION_RIGHT_90_ID;
                } else if ((right_shoulder != previous_right_shoulder)
                           && (right_shoulder == APP_RTOS_SHOULDER_PRESSED)) {
                    remote_yaw_action_id = CHASSIS_YAW_ACTION_180_ID;
                }
                previous_right_shoulder = right_shoulder;

                if (AppDt35Buttons_TakePendingYawAction(&pending_region_yaw_action)) {
                    remote_yaw_action_id = pending_region_yaw_action;
                    remote_yaw_action_is_region = true;
                }

                if (latched_yaw_turn_is_region
                    && ((remote_control.switches
                         & APP_REMOTE_SWITCH_PC0_REGION_ENABLE) == 0U)) {
                    latched_yaw_turn_action_id = 0U;
                    latched_yaw_turn_is_region = false;
                    AppMonitor_ClearYawAction();
                }

                if ((remote_yaw_action_id != 0U)
                    && (latched_yaw_turn_action_id == 0U)
                    && AppMonitor_GetYawPidParams(&yaw_pid_params)
                    && yaw_pid_params.enabled && yaw_pid_feedback_valid
                    && AppMonitor_StartYawTurn(remote_yaw_action_id,
                                               yaw_pid_feedback_deg)) {
                    latched_yaw_turn_action_id = remote_yaw_action_id;
                    latched_yaw_turn_is_region = remote_yaw_action_is_region;
                }

                if ((latched_yaw_turn_action_id != 0U)
                    && AppMonitor_GetYawPidParams(&yaw_pid_params)
                    && yaw_pid_params.enabled && yaw_pid_feedback_valid) {
                    command.yaw_rate_radps = 0.0f;
                    if (AppMonitor_GetYawTarget(&yaw_target_deg)) {
                        (void)AppMonitor_SelectYawPidByError(
                            yaw_target_deg - yaw_pid_feedback_deg);
                        (void)AppMonitor_GetYawPidParams(&yaw_pid_params);
                        AppChassis_ComputeYawRate(yaw_target_deg,
                                                  yaw_pid_feedback_deg,
                                                  &yaw_pid_params,
                                                  &command.yaw_rate_radps);
                        AppMonitor_UpdateYawActionResult(yaw_pid_feedback_deg);
                    }
                    if (AppMonitor_GetYawActionStatus(&remote_yaw_action_status)
                        && (remote_yaw_action_status.action_result == 2U)) {
                        latched_yaw_turn_action_id = 0U;
                        latched_yaw_turn_is_region = false;
                        AppMonitor_ClearYawAction();
                        AppMonitor_ClearYawTarget();
                        AppChassis_ResetYawPid();
                    }
                    shoulder_yaw_active = false;
                    shoulder_yaw_step_elapsed_ms = 0U;
                } else if (AppMonitor_GetYawPidParams(&yaw_pid_params)
                           && yaw_pid_params.enabled && yaw_pid_feedback_valid) {
                    bool shoulder_yaw_now = (left_shoulder == APP_RTOS_SHOULDER_DIRECTION_1)
                                         || (left_shoulder == APP_RTOS_SHOULDER_DIRECTION_2);
                    float shoulder_step_deg = (left_shoulder == APP_RTOS_SHOULDER_DIRECTION_1)
                                            ? CHASSIS_YAW_QE_STEP_DEG
                                            : -CHASSIS_YAW_QE_STEP_DEG;

                    if (shoulder_yaw_now && !shoulder_yaw_active) {
                        (void)AppMonitor_SelectYawPidProfile(APP_YAW_PID_PROFILE_QE);
                        (void)AppMonitor_StepYawTarget(
                            shoulder_step_deg);
                        shoulder_yaw_step_elapsed_ms = 0U;
                    } else if (shoulder_yaw_now) {
                        shoulder_yaw_step_elapsed_ms += APP_CONTROL_PERIOD_MS;
                        if (shoulder_yaw_step_elapsed_ms
                            >= CHASSIS_YAW_QE_STEP_PERIOD_MS) {
                            (void)AppMonitor_StepYawTarget(
                                shoulder_step_deg);
                            shoulder_yaw_step_elapsed_ms = 0U;
                        }
                    } else if (shoulder_yaw_active) {
                        (void)AppMonitor_SelectYawPidProfile(APP_YAW_PID_PROFILE_90);
                        shoulder_yaw_step_elapsed_ms = 0U;
                    }
                    shoulder_yaw_active = shoulder_yaw_now;
                } else {
                    shoulder_yaw_active = false;
                    shoulder_yaw_step_elapsed_ms = 0U;
                    AppChassis_ResetYawPid();
                }
            } else {
                previous_right_shoulder = APP_RTOS_SHOULDER_RELEASED;
                remote_session_valid = false;
                dual_control_freshness = AppMessages_GetDualControlFreshness(
                    osKernelGetTickCount());
                if (AppMessages_IsUpperControlEnabled()
                    && (dual_control_freshness == APP_DUAL_CONTROL_ACTIVE)
                    && AppMessages_GetDualControl(&dual_control)) {
                    (void)AppChassis_RemoteControlToCommand(&dual_control,
                                                            osKernelGetTickCount(),
                                                            &command);
                    if (AppRtos_IsYawTurnAction(dual_control.action_id)) {
                    shoulder_yaw_active = false;
                    shoulder_yaw_step_elapsed_ms = 0U;
                    command.velocity_x_mps = 0.0f;
                    command.velocity_y_mps = 0.0f;
                    command.yaw_rate_radps = 0.0f;
                    if (AppMonitor_GetYawPidParams(&yaw_pid_params) && yaw_pid_params.enabled
                        && yaw_pid_feedback_valid) {
                        if ((latched_yaw_turn_action_id != dual_control.action_id)
                            && AppMonitor_StartYawTurn(dual_control.action_id,
                                                       yaw_pid_feedback_deg)) {
                            latched_yaw_turn_action_id = dual_control.action_id;
                        }
                        if (AppMonitor_GetYawPidParams(&yaw_pid_params)
                            && AppMonitor_GetYawTarget(&yaw_target_deg)) {
                            (void)AppMonitor_SelectYawPidByError(
                                yaw_target_deg - yaw_pid_feedback_deg);
                            (void)AppMonitor_GetYawPidParams(&yaw_pid_params);
                            AppChassis_ComputeYawRate(yaw_target_deg,
                                                     yaw_pid_feedback_deg,
                                                     &yaw_pid_params,
                                                     &command.yaw_rate_radps);
                            AppMonitor_UpdateYawActionResult(yaw_pid_feedback_deg);
                        }
                    } else {
                        AppChassis_ResetYawPid();
                    }
                    } else {
                    if (latched_yaw_turn_action_id != 0U) {
                        latched_yaw_turn_action_id = 0U;
                        AppMonitor_ClearYawAction();
                    }
                    if (AppMonitor_GetYawPidParams(&yaw_pid_params) && yaw_pid_params.enabled
                        && yaw_pid_feedback_valid) {
                        bool shoulder_yaw_now = command.yaw_rate_radps != 0.0f;

                        if (shoulder_yaw_now && !shoulder_yaw_active) {
                            (void)AppMonitor_SelectYawPidProfile(APP_YAW_PID_PROFILE_QE);
                            (void)AppMonitor_StepYawTarget(
                                (command.yaw_rate_radps > 0.0f)
                                    ? CHASSIS_YAW_QE_STEP_DEG
                                    : -CHASSIS_YAW_QE_STEP_DEG);
                            shoulder_yaw_step_elapsed_ms = 0U;
                        } else if (shoulder_yaw_now) {
                            shoulder_yaw_step_elapsed_ms += APP_CONTROL_PERIOD_MS;
                            if (shoulder_yaw_step_elapsed_ms
                                >= CHASSIS_YAW_QE_STEP_PERIOD_MS) {
                                (void)AppMonitor_StepYawTarget(
                                    (command.yaw_rate_radps > 0.0f)
                                        ? CHASSIS_YAW_QE_STEP_DEG
                                        : -CHASSIS_YAW_QE_STEP_DEG);
                                shoulder_yaw_step_elapsed_ms = 0U;
                            }
                        } else if (!shoulder_yaw_now && shoulder_yaw_active) {
                            (void)AppMonitor_SelectYawPidProfile(APP_YAW_PID_PROFILE_90);
                            shoulder_yaw_step_elapsed_ms = 0U;
                        }
                        shoulder_yaw_active = shoulder_yaw_now;
                        if (AppMonitor_GetYawPidParams(&yaw_pid_params)
                            && AppMonitor_GetYawTarget(&yaw_target_deg)) {
                            AppChassis_ComputeYawRate(yaw_target_deg,
                                                     yaw_pid_feedback_deg,
                                                     &yaw_pid_params,
                                                     &command.yaw_rate_radps);
                        } else {
                            AppChassis_ResetYawPid();
                        }
                    } else {
                        shoulder_yaw_active = false;
                        shoulder_yaw_step_elapsed_ms = 0U;
                        AppChassis_ResetYawPid();
                    }
                    }
                } else if (AppMessages_IsUpperControlEnabled()) {
                /* A configured upper-control link owns this fallback path.
                 * It must not be replaced by YAW hold after 100 ms silence. */
                memset(&command, 0, sizeof(command));
                command.timestamp_ms = osKernelGetTickCount();
                shoulder_yaw_active = false;
                shoulder_yaw_step_elapsed_ms = 0U;
                latched_yaw_turn_action_id = 0U;
                AppMonitor_ClearYawAction();
                AppChassis_ResetYawPid();
                } else {
                /* No external motion owner: keep the chassis valid at zero
                 * linear speed so the firmware YAW hold loop can still act. */
                memset(&command, 0, sizeof(command));
                command.timestamp_ms = osKernelGetTickCount();
                command.valid = 1U;
                shoulder_yaw_active = false;
                shoulder_yaw_step_elapsed_ms = 0U;
                latched_yaw_turn_action_id = 0U;
                AppMonitor_ClearYawAction();
                if (AppMonitor_GetYawPidParams(&yaw_pid_params)
                    && yaw_pid_params.enabled
                    && yaw_pid_feedback_valid
                    && AppMonitor_GetYawTarget(&yaw_target_deg)) {
                    AppChassis_ComputeYawRate(yaw_target_deg,
                                             yaw_pid_feedback_deg,
                                             &yaw_pid_params,
                                             &command.yaw_rate_radps);
                } else {
                    AppChassis_ResetYawPid();
                }
                }
            }
        }

        /* Apply the firmware-owned YAW position loop at the final command
         * boundary.  PC manual movement uses the same target and feedback. */
        if (command.valid != 0U && yaw_pid_feedback_valid
            && AppMonitor_GetYawTarget(&yaw_target_deg)) {
            (void)AppMonitor_SelectYawPidByError(
                yaw_target_deg - yaw_pid_feedback_deg);
        }
        if (command.valid != 0U
            && AppMonitor_GetYawPidParams(&yaw_pid_params)
            && yaw_pid_params.enabled
            && yaw_pid_feedback_valid
            && AppMonitor_GetYawTarget(&yaw_target_deg)) {
            (void)AppChassis_ComputeYawRate(yaw_target_deg,
                                             yaw_pid_feedback_deg,
                                             &yaw_pid_params,
                                             &command.yaw_rate_radps);
        } else if (pc_chassis_enabled) {
            AppChassis_ResetYawPid();
        }

        events = AppRtos_GetEvents();
        appRtosYawPidDebug.enabled = 0U;
        appRtosYawPidDebug.feedback_valid = yaw_pid_feedback_valid ? 1U : 0U;
        appRtosYawPidDebug.target_deg = yaw_target_deg;
        appRtosYawPidDebug.feedback_deg = yaw_pid_feedback_deg;
        appRtosYawPidDebug.output_radps = command.yaw_rate_radps;
        appRtosYawPidDebug.events = events;
        if (AppMonitor_GetYawPidParams(&yaw_pid_params)) {
            appRtosYawPidDebug.enabled = yaw_pid_params.enabled ? 1U : 0U;
        }
        AppChassis_ControlStep(&command, events);
        appRtosRuntimeStats.control_cycle_count++;

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        if ((int32_t)(xTaskGetTickCount() - xLastWakeTime) > 0)
            appRtosRuntimeStats.control_overrun_count++;
    }
}

static void AppTask_Navigation(void *argument)
{
    app_real_world_t state;
    app_chassis_command_t command;
    float vx;
    float vy;
    float omega;
    app_dt35_sample_t dt35_sample;
    app_remote_control_t remote_control;
    bool position_was_active;
    bool remote_control_available;
    uint32_t now_ms;
    uint32_t nextWake;
    (void)argument;

    if (!AppRtosEvent_WaitForInitialization()) {
        osThreadExit();
    }

    nextWake = osKernelGetTickCount();

    for (;;) {
        memset(&state, 0, sizeof(state));
        memset(&command, 0, sizeof(command));
        app_dt35_position_status_t position_status;
        now_ms = osKernelGetTickCount();
        remote_control_available =
            (AppMessages_GetRemoteControlFreshness(now_ms)
             == APP_REMOTE_CONTROL_ACTIVE)
            && AppMessages_GetRemoteControl(&remote_control);
        AppDt35Buttons_Scan(remote_control_available ? &remote_control : NULL);
        if (remote_control_available
            && ((remote_control.joy_x > APP_DT35_MANUAL_CANCEL_AXIS)
                || (remote_control.joy_x < -APP_DT35_MANUAL_CANCEL_AXIS)
                || (remote_control.joy_y > APP_DT35_MANUAL_CANCEL_AXIS)
                || (remote_control.joy_y < -APP_DT35_MANUAL_CANCEL_AXIS))) {
            AppDt35Buttons_CancelRemote();
        }
        position_was_active = AppDt35Position_GetStatus(&position_status) && position_status.active;
        if (position_was_active) {
            memset(&dt35_sample, 0, sizeof(dt35_sample));
            (void)AppMessages_GetDt35(&dt35_sample);
            (void)AppDt35Position_Step(&dt35_sample, osKernelGetTickCount(),
                                       &command);
            (void)AppRtos_SubmitChassisCommand(&command, 0U);
        } else if (AppRtos_GetRobotState(&state, 0U))
        {
#if (CHASSIS_YAW_INPUT_SOURCE == 2U)
            float navigation_yaw_deg = state.fused_yaw_deg;
#else
            float navigation_yaw_deg = state.yaw_deg;
#endif
            PathFollower_Update(state.position_x_m * 1000.0f,
                                state.position_y_m * 1000.0f,
                                navigation_yaw_deg * APP_RTOS_DEG_TO_RAD,
                                navigation_yaw_deg * APP_RTOS_DEG_TO_RAD,
                                &vx, &vy, &omega);
            command.timestamp_ms = osKernelGetTickCount();
            command.velocity_x_mps = vx;
            command.velocity_y_mps = vy;
            command.yaw_rate_radps = omega;
            command.valid = PathFollower_IsFinished() ? 0U : 1U;
            (void)AppRtos_SubmitChassisCommand(&command, 0U);
        }
        nextWake += AppRtos_MsToTicks(APP_NAVIGATION_PERIOD_MS);
        osDelayUntil(nextWake);
    }
}

static void AppTask_Communication(void *argument)
{
    app_dual_status_t dual_status;
    app_chassis_command_t chassis_command;
    app_dual_control_t dual_control;
    app_dual_control_freshness_t dual_freshness;
    app_yaw_action_status_t yaw_action_status;
    uint32_t events;
    (void)argument;

    if (!AppRtosEvent_WaitForInitialization()) {
        osThreadExit();
    }

    for (;;) {
        uint32_t now = osKernelGetTickCount();

        AppMessages_CheckTimeout(now);
        AppMonitor_CommunicationStep(now);
        memset(&dual_status, 0, sizeof(dual_status));
        memset(&dual_control, 0, sizeof(dual_control));
        dual_freshness = AppMessages_GetDualControlFreshness(now);
        events = AppRtos_GetEvents();
        if ((events & APP_EVENT_ESTOP) != 0U) {
            dual_status.chassis_state = 5U;
            dual_status.fault_code = 5U;
            dual_status.status_flags |= 1U << 6;
        } else if ((events & (APP_EVENT_MOTOR_FAULT | APP_EVENT_CAN_FAULT)) != 0U) {
            dual_status.chassis_state = 6U;
            dual_status.fault_code = ((events & APP_EVENT_MOTOR_FAULT) != 0U) ? 4U : 3U;
        } else if (AppMonitor_GetYawActionStatus(&yaw_action_status)) {
            dual_status.chassis_state = 2U;
            dual_status.current_action_id = yaw_action_status.action_id;
            dual_status.action_result = yaw_action_status.action_result;
            dual_status.status_flags |= 1U << 4;
        } else if (AppChassis_GetCommand(&chassis_command) && (chassis_command.valid != 0U)) {
            dual_status.chassis_state = 3U;
            dual_status.current_action_id = 1U;
            dual_status.action_result = 1U;
            dual_status.status_flags |= (1U << 1) | (1U << 5);
        } else {
            dual_status.chassis_state = 1U;
        }
        if ((dual_freshness == APP_DUAL_CONTROL_ACTIVE)
            && AppMessages_GetDualControl(&dual_control)) {
            if ((dual_control.flags & (1U << 0U)) != 0U) {
                dual_status.status_flags |= 1U << 1;
            }
            if ((dual_control.flags & (1U << 2U)) != 0U) {
                dual_status.status_flags |= 1U << 5;
            }
        } else if (AppMessages_IsUpperControlEnabled()) {
            dual_status.status_flags |= 1U << 7;
            if ((dual_freshness == APP_DUAL_CONTROL_STOP)
                || (dual_freshness == APP_DUAL_CONTROL_UNAVAILABLE)) {
                dual_status.fault_code = 1U;
            }
        }
        dual_status.status_flags |= 1U << 0;
        (void)AppMessages_SendDualStatus(&uart8, &dual_status);
        osDelay(AppRtos_MsToTicks(APP_COMMUNICATION_WAIT_MS));
    }
}

static void AppTask_Safety(void *argument)
{
    bool initialized;
    uint32_t nextWake;
    uint32_t flags;
    uint32_t events;
    (void)argument;

    flags = osEventFlagsWait(appRtosSystemEvents, APP_EVENT_INIT_DONE | APP_EVENT_INIT_FAILED, osFlagsWaitAny | osFlagsNoClear, osWaitForever);
    initialized = ((flags & osFlagsError) == 0U) && ((flags & APP_EVENT_INIT_DONE) != 0U);
    nextWake = osKernelGetTickCount();

    for (;;) {
        if (!initialized || !AppChassis_ProcessCan()) {
            AppRtos_SetEvents(APP_EVENT_CAN_FAULT);
        } else if (appRtosCanErrorMask != 0U) {
            /* BspCan_Process recovered the FDCAN peripheral.  Do not require
               a PC RESET before fresh upper-controller commands can move. */
            AppRtos_ClearCanErrors();
            (void)AppRtos_ClearEvents(APP_EVENT_CAN_FAULT);
        }

        events = AppRtos_GetEvents();
        AppSafety_Step(events);

        nextWake += AppRtos_MsToTicks(APP_SAFETY_PERIOD_MS);
        osDelayUntil(nextWake);
    }
}

static void AppTask_Serial(void *argument)
{
    app_uart_rx_packet_t packet;
    (void)argument;
    if (!AppRtosEvent_WaitForInitialization()) osThreadExit();
    for (;;) {
        if (osMessageQueueGet(appRtosUartRxQueue, &packet, NULL, osWaitForever) == osOK) {
            if (packet.source == APP_UART_RX_PC) {
                AppMonitor_OnPcRx(&uart10, packet.data, packet.length);
            } else if (packet.source == APP_UART_RX_REMOTE) {
                AppMessages_OnRemoteControlRx(&uart1, packet.data, packet.length);
            }
        }
    }
}

static void AppTask_Monitor(void *argument)
{
    app_runtime_stats_t stats;
    app_runtime_stats_t discarded;
    uint32_t nextWake;
    (void)argument;

    nextWake = osKernelGetTickCount();

    for (;;) {
        AppRtos_GetRuntimeStats(&stats);

        if (osMessageQueuePut(appRtosTelemetryQueue, &stats, 0U, 0U) != osOK) {
            osMessageQueueGet(appRtosTelemetryQueue, &discarded, NULL, 0U);
            osMessageQueuePut(appRtosTelemetryQueue, &stats, 0U, 0U);
        }

        AppMonitor_Step(&stats);
        nextWake += AppRtos_MsToTicks(APP_MONITOR_PERIOD_MS);
        osDelayUntil(nextWake);
    }
}
