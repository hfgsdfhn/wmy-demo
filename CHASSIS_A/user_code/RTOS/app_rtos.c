#include "app_rtos.h"

#include "app_rtos_event.h"
#include "app_rtos_queue.h"
#include "app_rtos_state.h"
#include "app_rtos_task.h"

bool AppRtos_Init(void)
{
    return AppRtos_QueueInit()
        && AppRtos_EventInit()
        && AppRtos_StateInit()
        && AppRtos_TaskCreate();
}
