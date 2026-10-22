#ifndef APP_RTOS_H
#define APP_RTOS_H /* RTOS 应用头文件保护宏 */

#include <stdbool.h>
#include <stdint.h>

#include "app_messages.h"
#include "app_monitor.h"
#include "bsp_can.h"
#include "cmsis_os2.h"

#define APP_UART_RX_DATA_MAX 64U
typedef enum { APP_UART_RX_PC = 0, APP_UART_RX_REMOTE } app_uart_rx_source_t;
typedef struct { app_uart_rx_source_t source; uint16_t length; uint8_t data[APP_UART_RX_DATA_MAX]; } app_uart_rx_packet_t;

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

bool AppRtos_Init(void);
void AppRtos_Bootstrap(void);

bool AppRtos_SubmitChassisCommand(const app_chassis_command_t *command,
                                  uint32_t timeout_ms);
bool AppRtos_SubmitTrajectoryPoint(const app_trajectory_point_t *point,
                                   uint32_t timeout_ms);

bool AppRtos_PublishRobotState(const app_real_world_t *state,
                               uint32_t timeout_ms);
bool AppRtos_GetRobotState(app_real_world_t *state, uint32_t timeout_ms);
bool AppRtos_UpdateYaw(const app_real_world_t *state);
void AppRtos_UpdateYawFromISR(const app_real_world_t *state);
void AppRtos_SetEventsFromISR(uint32_t flags);
void AppRtos_GetRuntimeStats(app_runtime_stats_t *stats);

bool AppRtos_SetEvents(uint32_t flags);
bool AppRtos_ClearEvents(uint32_t flags);
uint32_t AppRtos_GetEvents(void);

void AppRtos_Assert(const char *file, uint32_t line);
void AppRtos_Panic(app_rtos_fault_t fault);

uint32_t AppRtos_MsToTicks(uint32_t milliseconds);
void AppRtos_CanRxFromISR(bsp_can_t *can, uint32_t canId, bool isExtendedId,
                          const uint8_t *data, uint8_t length);
void AppRtos_CanErrorFromISR(bsp_can_t *can, uint32_t errorFlags);
void AppRtos_ClearCanErrors(void);
void AppRtos_UartRxFromISR(app_uart_rx_source_t source, const uint8_t *data, uint16_t length);

#endif /* APP_RTOS_H */
