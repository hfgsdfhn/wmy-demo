#include "app_rtos_event.h"

#include "app_init.h"
#include "cmsis_compiler.h"

osEventFlagsId_t appRtosSystemEvents;
volatile app_rtos_fault_t appRtosFaultCode = APP_RTOS_FAULT_NONE;
volatile uint32_t appRtosFaultLine;
const char * volatile appRtosFaultFile;

static const osEventFlagsAttr_t appRtosEventAttributes = { .name = "systemEvents" };

/**
 * @brief 初始化系统事件标志组
 *
 * 创建一个用于 RTOS 事件通信的 event flags 对象，并保存句柄。
 *
 * @return true  初始化成功
 * @return false 初始化失败
 */
bool AppRtos_EventInit(void)
{
    appRtosSystemEvents = osEventFlagsNew(&appRtosEventAttributes);
    return appRtosSystemEvents != NULL;
}

/**
 * @brief 启动系统引导流程
 *
 * 运行应用初始化函数，并根据初始化结果设置相应的系统事件。
 * 初始化成功时发布 CAN 就绪和初始化完成事件，失败时发布初始化失败和急停事件。
 */
void AppRtos_Bootstrap(void)
{
    if (AppInit_Run())
    {
        AppRtos_SetEvents(APP_EVENT_ALL_CAN_READY | APP_EVENT_INIT_DONE);
    }
    else
    {
        AppRtos_SetEvents(APP_EVENT_INIT_FAILED | APP_EVENT_ESTOP);
    }
}

/**
 * @brief 等待系统初始化完成
 *
 * 阻塞等待 INIT_DONE 或 INIT_FAILED 事件，且不清除事件标志。
 *
 * @return true  初始化完成且成功
 * @return false 初始化失败或等待出错
 */
bool AppRtosEvent_WaitForInitialization(void)
{
    uint32_t flags = osEventFlagsWait(appRtosSystemEvents,
                                      APP_EVENT_INIT_DONE | APP_EVENT_INIT_FAILED,
                                      osFlagsWaitAny | osFlagsNoClear, osWaitForever);
    return ((flags & osFlagsError) == 0U) && ((flags & APP_EVENT_INIT_DONE) != 0U);
}

/**
 * @brief 设置系统事件标志
 *
 * 向 RTOS 事件组写入指定的事件位。
 *
 * @param flags 要设置的事件位掩码
 * @return true  设置成功
 * @return false 设置失败
 */
bool AppRtos_SetEvents(uint32_t flags)
{
    return (appRtosSystemEvents != NULL)
        && ((osEventFlagsSet(appRtosSystemEvents, flags) & osFlagsError) == 0U);
}

/**
 * @brief 清除系统事件标志
 *
 * 从 RTOS 事件组中清除指定的事件位。
 *
 * @param flags 要清除的事件位掩码
 * @return true  清除成功
 * @return false 清除失败
 */
bool AppRtos_ClearEvents(uint32_t flags)
{
    return (appRtosSystemEvents != NULL)
        && ((osEventFlagsClear(appRtosSystemEvents, flags) & osFlagsError) == 0U);
}

/**
 * @brief 获取当前系统事件标志
 *
 * 从 RTOS 事件组读取当前事件位掩码。
 *
 * @return uint32_t 当前事件标志，读取失败时返回 0
 */
uint32_t AppRtos_GetEvents(void)
{
    uint32_t flags;

    if (appRtosSystemEvents == NULL)
    {
        return 0U;
    }
    flags = osEventFlagsGet(appRtosSystemEvents);
    return ((flags & osFlagsError) == 0U) ? flags : 0U;
}

/**
 * @brief 断言失败处理
 *
 * 记录断言发生的文件和行号，并进入系统崩溃处理。
 *
 * @param file  断言发生的源文件名称
 * @param line  断言发生的行号
 */
void AppRtos_Assert(const char *file, uint32_t line)
{
    appRtosFaultFile = file;
    appRtosFaultLine = line;
    AppRtos_Panic(APP_RTOS_FAULT_ASSERT);
}

/**
 * @brief 系统异常进入死循环
 *
 * 记录 RTOS 故障码，禁止中断，然后进入无法返回的循环，便于现场调试。
 *
 * @param fault 当前的 RTOS 故障类型
 */
void AppRtos_Panic(app_rtos_fault_t fault)
{
    appRtosFaultCode = fault;
    __disable_irq();
    for (;;)
    {
        __NOP();
    }
}
