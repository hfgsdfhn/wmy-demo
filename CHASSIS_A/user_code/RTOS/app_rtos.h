#ifndef APP_RTOS_H
#define APP_RTOS_H

#include <stdbool.h>
#include <stdint.h>

#include "app_messages.h"
#include "app_state.h"
#include "path.h"

typedef enum
{
    APP_RTOS_FAULT_NONE = 0,
    APP_RTOS_FAULT_OBJECT_CREATE,
    APP_RTOS_FAULT_STACK_OVERFLOW,
    APP_RTOS_FAULT_MALLOC_FAILED,
    APP_RTOS_FAULT_ASSERT
} app_rtos_fault_t;

extern volatile app_rtos_fault_t appRtosFaultCode;
extern volatile uint32_t appRtosFaultLine;
extern const char * volatile appRtosFaultFile;

/* 初始化 RTOS 子系统，创建队列、事件和任务。 */
bool AppRtos_Init(void);
/* 执行应用初始化流程，并发布系统启动结果。 */
void AppRtos_Bootstrap(void);

/* 提交底盘控制命令。 */
bool AppRtos_SubmitChassisCommand(const app_chassis_command_t *command,
                                  uint32_t timeout_ms);
/* 提交机构控制命令。 */
bool AppRtos_SubmitMechanismCommand(
    const app_mechanism_command_t *command, uint32_t timeout_ms);
/* 提交导航轨迹点。 */
bool AppRtos_SubmitTrajectoryPoint(const PathPoint *point,
                                   uint32_t timeout_ms);
/* 提交动作命令。 */
bool AppRtos_SubmitActionCommand(const app_action_command_t *command,
                                 uint32_t timeout_ms);
/* 提交 IMU 采样数据。 */
bool AppRtos_SubmitImuSample(const app_imu_sample_t *sample,
                             uint32_t timeout_ms);
/* 提交 DT35 采样数据。 */
bool AppRtos_SubmitDt35Sample(const app_dt35_sample_t *sample,
                              uint32_t timeout_ms);
/* 提交通信报文。 */
bool AppRtos_SubmitCommunicationPacket(const app_comm_packet_t *packet,
                                       uint32_t timeout_ms);

/* 发布机器人状态。 */
bool AppRtos_PublishRobotState(const app_robot_state_t *state,
                               uint32_t timeout_ms);
/* 获取机器人状态。 */
bool AppRtos_GetRobotState(app_robot_state_t *state, uint32_t timeout_ms);
/* 获取当前运行时统计信息。 */
void AppRtos_GetRuntimeStats(app_runtime_stats_t *stats);

/* 设置系统事件标志。 */
bool AppRtos_SetEvents(uint32_t flags);
/* 清除系统事件标志。 */
bool AppRtos_ClearEvents(uint32_t flags);
/* 读取当前系统事件标志。 */
uint32_t AppRtos_GetEvents(void);

/* 记录断言失败位置。 */
void AppRtos_Assert(const char *file, uint32_t line);
/* 触发不可恢复故障处理。 */
void AppRtos_Panic(app_rtos_fault_t fault);

#endif /* APP_RTOS_H */
